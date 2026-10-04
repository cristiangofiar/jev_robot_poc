"""Serve one downloaded decision model on loopback with the native Metal runtime."""

import argparse
import json
import os
import subprocess
import socket
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Lock
from time import perf_counter
from datetime import datetime
from pathlib import Path
from urllib.parse import urlsplit

from experiment.environment import load_env

ROOT = Path(__file__).resolve().parent


def command(brain: str) -> list[str]:
    manifest = json.loads((ROOT / ".local_models/manifest.json").read_text())
    url = urlsplit(os.environ.get(brain.upper() + "_BASE_URL", f"http://127.0.0.1:{8008 if brain == 'kev' else 8009}"))
    if (url.scheme != "http" or url.hostname != "127.0.0.1" or url.port is None
            or url.path not in ("", "/") or url.username or url.password or url.query or url.fragment):
        raise ValueError("Local serving requires http://127.0.0.1:PORT")
    args = [str(ROOT / manifest["runtime"]["executable"]),
            "--model", str(ROOT / manifest["models"][brain]["path"]),
            "--alias", "kev-4b" if brain == "kev" else "laya",
            "--host", "127.0.0.1", "--port", str(url.port),
            "--ctx-size", "1024" if brain == "kev" else "512",
            "--batch-size", "512", "--ubatch-size", "512", "--parallel", "1",
            "--gpu-layers", "99"]
    if os.environ.get("LOCAL_MODEL_API_KEY"):
        # llama.cpp also reads this key from the environment, so it stays out of argv.
        os.environ["LLAMA_API_KEY"] = os.environ["LOCAL_MODEL_API_KEY"]
    return args


def serve(brain: str) -> None:
    """Keep native inference unchanged; display responses through a loopback proxy."""
    args = command(brain)
    port_index = args.index("--port") + 1
    public_port = int(args[port_index])
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        backend_port = listener.getsockname()[1]
    args[port_index] = str(backend_port)
    output_lock = Lock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def forward(self):
            started = perf_counter()
            body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
            connection = HTTPConnection("127.0.0.1", backend_port, timeout=120)
            try:
                # Forward authentication to the runtime, without logging headers.
                headers = {key: self.headers[key] for key in ("Content-Type", "Authorization") if key in self.headers}
                connection.request(self.command, self.path, body=body, headers=headers)
                response = connection.getresponse()
                status, data = response.status, response.read()
                content_type = response.getheader("Content-Type", "application/json")
            except (OSError, TimeoutError):
                status, data, content_type = 503, b'{"error":"Local runtime unavailable or still loading"}', "application/json"
            finally:
                connection.close()
            if self.command == "POST" and self.path.split("?")[0] == "/v1/systemone":
                with output_lock:
                    print("\n" + "=" * 72, flush=True)
                    print(f"[{datetime.now().astimezone().strftime('%H:%M:%S')}] {brain.upper()} OUTPUT | HTTP {status} | {(perf_counter() - started) * 1000:.1f} ms", flush=True)
                    try:
                        raw = json.loads(data)
                        for name, answer in raw.get("answers", {}).items():
                            selected = answer.get("choice", answer.get("score", answer.get("noul")))
                            print(f"  >>> {name}: {selected} | confidence={answer.get('confidence')}", flush=True)
                        print(json.dumps(raw, indent=2, ensure_ascii=False), flush=True)
                    except (ValueError, AttributeError, TypeError):
                        print(f"Non-JSON response ({len(data)} bytes)", flush=True)
                    print("=" * 72, flush=True)
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass  # Client timed out; the inference output was still logged.

        do_GET = forward
        do_POST = forward

    # Bind first, so a busy public port cannot leave an orphan model process.
    with ThreadingHTTPServer(("127.0.0.1", public_port), Handler) as proxy:
        log_path = ROOT / ".local_models" / f"{brain}-runtime.log"
        with log_path.open("w") as runtime_log:
            process = subprocess.Popen(args, cwd=ROOT, stdout=runtime_log, stderr=subprocess.STDOUT)
            try:
                print(f"{brain.upper()}: http://127.0.0.1:{public_port} | loading model; runtime log: {log_path}", flush=True)
                proxy.timeout = 0.5
                while process.poll() is None:
                    proxy.handle_request()
                raise RuntimeError(f"Model runtime exited ({process.returncode}); see {log_path}")
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--brain", choices=("kev", "laya"), required=True)
    args = parser.parse_args()
    load_env(ROOT / ".env")
    try:
        serve(args.brain)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
