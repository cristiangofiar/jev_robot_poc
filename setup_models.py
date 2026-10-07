"""Download pinned public HF artifacts and the official macOS/Windows decision runtime."""

import argparse
import hashlib
import json
import platform
import ssl
import shutil
import zipfile
from urllib.request import urlopen
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MODEL_DIR = ROOT / ".local_models"
RUNTIME_TAG = "b11391"  # System One was introduced in b11361.
REVISIONS = {
    "jaredpalmer/kev-4b": "6cfce5c2fa4b4bd64026336ab649c5ca78857d52",
    "convaiinnovations/laya": "7b928d828b7b0e022f929d9bd2e44165aa270148",
    "ggml-org/Kev-4B-GGUF": "d924f2e2c3872da8b8aaf3eb4453b4126deceb79",
    "ggml-org/Laya-GGUF": "22265007700297ba9e128297e82540cf28c5d7d4",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def download(url: str, target: Path) -> None:
    if target.is_file():
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + ".partial")
    context = ssl.create_default_context()
    if Path('/etc/ssl/cert.pem').is_file():
        context.load_verify_locations(cafile='/etc/ssl/cert.pem')
    with urlopen(url, context=context, timeout=120) as source, partial.open('wb') as output:
        shutil.copyfileobj(source, output)
    partial.rename(target)


def setup(originals: bool = True, brains=("laya",)) -> dict:
    system, machine = platform.system(), platform.machine().lower()
    if system == 'Darwin':
        bundle_name = f'llama-{RUNTIME_TAG}-bin-macos-{"arm64" if machine == "arm64" else "x64"}.tar.gz'
        executable_name = 'llama-server'
    elif system == 'Windows':
        bundle_name = f'llama-{RUNTIME_TAG}-bin-win-cpu-{"arm64" if machine == "arm64" else "x64"}.zip'
        executable_name = 'llama-server.exe'
    else:
        raise RuntimeError('Setup bundles support macOS and Windows; on Linux provide LLAMA_SERVER')
    MODEL_DIR.mkdir(exist_ok=True)
    artifacts = {}
    files = {
        "ggml-org/Kev-4B-GGUF": ["Kev-4B-Q4_K_M.gguf"],
        "ggml-org/Laya-GGUF": ["Laya-BF16.gguf"],
    }
    if originals:
        files.update({
            "jaredpalmer/kev-4b": ["adapter_model.safetensors", "adapter_config.json", "head.pt", "provenance.json", "tokenizer.json", "tokenizer_config.json", "README.md"],
            "convaiinnovations/laya": ["model.safetensors", "rl_agent_config.json", "encoder/config.json", "tokenizer/tokenizer.json", "tokenizer/tokenizer_config.json", "README.md"],
        })
    files = {repo: names for repo, names in files.items() if ("kev" in repo.lower() and "kev" in brains) or ("laya" in repo.lower() and "laya" in brains)}
    for repo, names in files.items():
        revision = REVISIONS[repo]
        for name in names:
            target = MODEL_DIR / repo.replace("/", "--") / name
            print(f"Downloading {repo}@{revision[:8]}/{name}", flush=True)
            download(f"https://huggingface.co/{repo}/resolve/{revision}/{name}", target)
            artifacts[target.relative_to(ROOT).as_posix()] = {
                "repository": repo, "revision": revision, "sha256": sha256(target),
                "bytes": target.stat().st_size,
            }
    runtime = MODEL_DIR / "runtime" / (RUNTIME_TAG + "_" + system + "_" + machine)
    archive = MODEL_DIR / "runtime" / bundle_name
    download(f"https://github.com/ggml-org/llama.cpp/releases/download/{RUNTIME_TAG}/{archive.name}", archive)
    if not runtime.exists():
        runtime.mkdir()
        if archive.suffix == '.zip':
            with zipfile.ZipFile(archive) as bundle:
                for name in bundle.namelist():
                    destination = (runtime / name).resolve()
                    if not destination.is_relative_to(runtime.resolve()):
                        raise ValueError('Unsafe runtime archive path')
                bundle.extractall(runtime)
        else:
            with tarfile.open(archive) as bundle:
                bundle.extractall(runtime, filter="data")
    servers = list(runtime.rglob(executable_name))
    if len(servers) != 1:
        raise RuntimeError("Expected exactly one llama-server in the official runtime bundle")
    server = servers[0]
    manifest = {
        "artifacts": artifacts,
        "runtime": {"tag": RUNTIME_TAG, "archive_sha256": sha256(archive),
                    "executable": server.relative_to(ROOT).as_posix()},
        "models": {
            "kev": {"source_repository": "jaredpalmer/kev-4b", "representation": "GGUF Q4_K_M",
                    "path": ".local_models/ggml-org--Kev-4B-GGUF/Kev-4B-Q4_K_M.gguf"},
            "laya": {"source_repository": "convaiinnovations/laya", "representation": "GGUF BF16",
                     "path": ".local_models/ggml-org--Laya-GGUF/Laya-BF16.gguf"},
        },
    }
    existing = MODEL_DIR / 'manifest.json'
    if existing.is_file():
        previous = json.loads(existing.read_text())
        manifest['artifacts'] = {**previous.get('artifacts', {}), **manifest['artifacts']}
    (MODEL_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("Ready: .local_models/manifest.json", flush=True)
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-originals", action="store_true", help="Download only inference representations")
    parser.add_argument("--brain", choices=("laya", "kev", "both"), default="laya")
    args = parser.parse_args()
    setup(originals=not args.skip_originals, brains=("laya", "kev") if args.brain == "both" else (args.brain,))
