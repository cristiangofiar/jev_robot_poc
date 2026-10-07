# Misión de muestras en Marte · Webots R2025a

Demo de Sojourner: detecta tres marcadores de muestra, los inspecciona y recoge mediante una parada, y entrega el inventario en una base. Los modelos eligen misiones; Python y Webots ejecutan navegación, motores y seguridad localmente. Se conserva la mecánica del [Sojourner oficial R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/nasa/protos/Sojourner.proto). El mundo adapta el ejemplo NASA instalado, su iluminación marciana, gravedad de 3,73 m/s², timestep de 16 ms y parámetros de contacto. Se ha sustituido el terreno enorme por un heightfield pequeño con colisiones y pendientes inferiores a 2°.

## Arranque

Requisitos: Webots R2025a y Python 3.12 o posterior en PATH/configurado en las preferencias de Webots. El controlador y sus scripts usan únicamente la biblioteca estándar y la API de Webots. Los PROTO/texturas remotos requieren conexión en la primera carga o la caché de Webots.

Configura `WEBOTS_EXECUTABLE` en `.env` o pasa `--webots`. Se preservan las credenciales existentes; copia `.env.example` solo si todavía no tienes `.env`.

```text
macOS:   WEBOTS_EXECUTABLE=/Applications/Webots.app/Contents/MacOS/webots
Windows: WEBOTS_EXECUTABLE=C:/Program Files/Webots/msys64/mingw64/bin/webots.exe
```

Comprueba la ruta de tu instalación Windows; el launcher también busca `WEBOTS_HOME`, `ProgramFiles` y PATH. Si Webots necesita otro Python, selecciona su ejecutable en sus preferencias. En macOS usa `python3`; en Windows puedes sustituirlo por `py -3.12`.

```sh
python3 run_experiment.py --brain rules
```

Arranca una instancia nueva con física y rendering; el modo determinista usa `fast`. Para observar la demo a velocidad real añade `--mode realtime`. El HUD muestra objetivo, inventario, entregas, decisión solicitada, movimiento aplicado, latencia, confianza si existe y seguridad. La vista de cámara nativa aparece en la esquina superior izquierda. También existe un Display vinculado a la cámara en la ventana genérica del rover. Puedes abrir `worlds/mars.wbt` directamente; en ese caso la misión usa `MARS_BRAIN` de `.env` (por defecto `rules`) y Webots permanece abierto al terminar. Recarga el mundo antes de empezar otra misión.

## Laya y otros modelos

Los modelos descargados existentes y `.local_models/manifest.json` se conservan. En una máquina nueva:

```sh
python3 setup_models.py --brain laya --skip-originals
python3 serve_brain.py --brain laya
```

Mantén el servidor en una terminal. En otra:

```sh
python3 probe_brain.py --brain laya
python3 run_experiment.py --brain laya
```

Laya se sirve en loopback, con el runtime oficial llama.cpp `b11391`, GGUF BF16 del checkpoint inglés fijado y `/v1/systemone`. Las revisiones y hashes de los pesos se guardan en el manifiesto. El servidor permite una única inferencia simultánea. El launcher hace un calentamiento antes de arrancar la simulación y guarda su respuesta aparte. Con DLM se usa `realtime`: acelerar la simulación puede caducar observaciones antes de que llegue la inferencia.

`setup_models.py` descarga paquetes nativos para macOS arm64/x64 y Windows arm64/x64; Windows utiliza CPU por defecto. Es posible configurar `LLAMA_SERVER`, `LAYA_MODEL_PATH`, `KEV_MODEL_PATH` y `LOCAL_GPU_LAYERS` para otro paquete compatible, por ejemplo uno con GPU. La implementación Windows está preparada pero debe validarse en el PC de destino; la validación física de esta entrega se realiza en macOS. Los caminos del manifiesto son relativos al repositorio.

Kev utiliza el mismo adapter y contrato, con GGUF Q4_K_M y alias `kev-4b`. En el PC de 32 GB:

```sh
python3 setup_models.py --brain kev --skip-originals
python3 serve_brain.py --brain kev
python3 run_experiment.py --brain kev
```

Detén un servidor antes de cargar otro en el Mac de 8 GB. Jev usa `OPENROUTER_API_KEY`, `JEV_BASE_URL` y `JEV_MODEL`; `--brain jev` hace llamadas a su API, incluido el calentamiento. Esta entrega valida exclusivamente Laya local: no carga Kev ni llama a Jev. No se presume que tengan las mismas decisiones o latencias. No se registran claves, cabeceras, contenido de `.env` ni mensajes completos de excepciones externas.

## Escena, sensores y significado de las acciones

La región operativa pública es `x ∈ [-1, 3.6]`, `y ∈ [-2.1, 2.1]` metros. La base está en `(0,0)`; se conoce como punto de lanzamiento. Las coordenadas de muestras y rocas permanecen en el mundo/Supervisor. El modelo recibe únicamente muestras observadas por la cámara y recordadas después de su detección, distancias derivadas localmente, inspecciones, inventario, batería estimada, indicador de obstáculo, sensores válidos e historial de tres decisiones.

| Acción | Efecto y condición |
| --- | --- |
| `EXPLORE` | Seguir puntos de exploración públicos, sin conocer posiciones de muestras ocultas. |
| `INSPECT_Sn` | Acercarse a un candidato observado y detenerse. Supervisor exige distancia ≤0,68 m y velocidad horizontal <0,006 m/s durante ≥1 s. Marca inspección. |
| `COLLECT_Sn` | Solo se ofrece tras inspeccionar. Acercamiento y misma comprobación física; añade inventario y oculta visualmente la muestra. |
| `RETURN` | Disponible con las tres muestras recogidas o batería estimada <20%. Volver a base, detenerse y entregar a ≤0,30 m. |
| `WAIT` | Pausa de tres segundos y nueva decisión. |

Detectar significa recibir un objeto de reconocimiento de cámara visible dentro de 3,2 m. El reconocimiento de Webots aporta ID y posición relativa perfectos: **es percepción idealizada**, con comprobación de oclusión, no clasificación visual aprendida. Inspeccionar es una parada simbólica, sin análisis científico. Recoger es una transferencia simbólica, sin brazo ni pinza, comprobada físicamente por el Supervisor. Entregar transfiere el inventario en la base. Éxito exige las tres muestras entregadas; regresar con batería baja puede producir entrega parcial.

El contrato System One contiene una pregunta `mission` de tipo `choice` con criterios dinámicos para los IDs candidatos. Se valida elección, modelo servido local, confianza y distribución completa de probabilidades. La confianza se muestra como salida del modelo y no como garantía de acierto. Las explicaciones de criterios describen acciones; no son razonamiento del modelo.

Todos los dispositivos se añaden por `extensionSlot`, sin modificar el PROTO:

| Dispositivo | Colocación y cobertura |
| --- | --- |
| RGB | `(-0.38,0,0.22)` m respecto al rover; mira a -X local, inclinación descendente 0,15 rad; 320×240, FOV horizontal 1,9 rad, muestreo 64 ms, reconocimiento 3,2 m. |
| Lidar | `(-0.38,0,0.17)` m; -X local, 360°, 360 puntos en una capa horizontal, 0,03–4 m, cada 16 ms. Se usan coordenadas de la nube de puntos real. |
| GPS | Origen del rover; ENU, metros y m/s, cada 16 ms. |
| IMU/brújula | Origen del rover; radianes para roll/pitch/yaw y vector del norte. IMU conduce; brújula se comprueba en la prueba de sensores. |

La cámara y el lidar están delante y encima del cuerpo. Se excluyen retornos visuales del propio vehículo a menos de 0,6 m del origen al construir el mapa; la seguridad usa los rangos actuales. La capa horizontal del lidar tiene limitaciones ante objetos bajos y cambios importantes de pendiente. No se han añadido encoders ni profundidad: GPS, IMU y reconocimiento bastan para esta primera escena.

La navegación usa A* sobre una cuadrícula de 0,15 m con obstáculos vistos por lidar e inflación de 0,43 m. La plataforma de aterrizaje de radio 0,65 m es una zona pública transitable. Solo conoce límites operativos, base y objetivos observados; no recibe el mapa de obstáculos del Supervisor. Recalcula rutas aproximadamente una vez por segundo, sigue puntos y gira con las cuatro ruedas exteriores. Avanza a la velocidad máxima oficial de 0,6 rad/s, aproximadamente 0,036 m/s. Los motores delanteros y traseros se dirigen; las ruedas centrales quedan pasivas durante giros, siguiendo el ejemplo oficial. La seguridad se evalúa cada 16 ms: sensores inválidos, inclinación >0,4 rad, límites y proximidad frontal/de giro detienen los motores.

La batería es un presupuesto sintético calculado localmente (`100 − 0.045 × segundos − 0.2 × metros recorridos`), no una medida eléctrica ni energía física de Webots. La demo no incorpora ROS, SLAM, un brazo, navegación en pendientes fuertes ni detección completa de colisiones con sensores del rover.

## Inferencia y registros

Una petición se ejecuta en un thread daemon; ningún thread de inferencia llama a Webots. El bucle físico nunca espera HTTP. Hay un máximo de una petición pendiente, incluso después de vencer el deadline y hasta que el thread termine. Se rechazan respuestas por deadline de reloj real (2 s), edad de observación en simulación (3 s), revisión semántica cambiada o elección ya no disponible. Estos valores se configuran con `--deadline`, `--max-age` o entorno.

Una misión aceptada puede seguir durante todo un trayecto; no se caduca una orden aceptada porque el rover avance. Las respuestas que terminan después del rechazo se guardan como `late_response` y no se aplican. La revisión cambia al descubrir, inspeccionar, recoger o entregar muestras; invalida respuestas calculadas sobre ese estado anterior. Se consulta al modelo al finalizar una acción/parada o explorar, no en cada timestep. Un error, timeout o respuesta obsoleta aplica la política determinista y lo registra como fallback. Un fallo de ruta detiene el movimiento, se registra y vuelve a solicitar una misión.

Cada ejecución crea un directorio exclusivo `results/FECHA_brain_ID/`:

- `steps.jsonl`: detecciones, petición/observación exacta, payload textual, respuesta, latencia, rechazo, acción solicitada/aplicada, fallback, comandos de motores y seguridad; sensores/estado cada 0,2 s.
- `frames/request_NNNN.png`: RGB real, asociado a ID de petición y timestamp simulado de la observación. Las peticiones se lanzan en ticks frescos de cámara. El modelo textual no recibe los PNG.
- `ground_truth.jsonl`: trayectorias físicas, validación de paradas/recogidas y contactos; nunca se transmite al modelo.
- `summary.json`: resultado físico, entregas, longitud recorrida, número de decisiones aceptadas, fallbacks, intervenciones y latencias. El éxito físico pertenece al sistema completo; no se atribuye navegación, seguridad o fallback al modelo.
- `snapshot/`: fuentes y mundo con hashes; manifiesto de modelos si existe. No incluye `.env` ni pesos.
- `launch.json`, `warmup.json` si aplica, `webots.log` y `world_final.png`.

Para respuestas terminadas, la latencia incluye request, transporte, inferencia, parseo y validación en reloj monotónico. Al vencer un deadline, `response.latency_ms` mide solo la espera hasta el rechazo; la duración real se conoce después en `late_response.latency_ms`. El resumen separa latencias aceptadas, esperas rechazadas y latencias tardías; el HUD deja la latencia desconocida mientras el thread sigue trabajando. Los tiempos de simulación, tiempos de pared y timestamps UTC se guardan separados. Las intervenciones cuentan transiciones a una parada de seguridad. `obstacle_contact_steps` cuenta muestras del Supervisor con contactos del rover por encima de z=0,09 m (umbral superior al terreno de esta escena); es una heurística de evaluación de esta escena, no atribución universal de colisiones. Un par de logs sin ambos eventos `end` queda inválido. El resumen conserva los fallbacks en lugar de mezclarlos con decisiones aceptadas.

## Verificación reproducible

```sh
python3 -m unittest discover -s tests -v
python3 run_experiment.py --brain rules --test sensors
python3 run_experiment.py --brain rules --test motion
python3 run_experiment.py --brain rules
python3 run_experiment.py --brain laya
python3 run_experiment.py --brain slow --delay 1 --deadline 0.1 --duration 40
python3 run_experiment.py --brain slow --delay 0.3 --max-age 0.05 --duration 15
```

Las pruebas `slow` simulan latencia sin cargar otro DLM. Las ejecuciones cortas terminan con `time_limit` y código no cero intencionadamente: comprueba los registros de física, timeout/rechazo y fallback. Puedes usar `--no-rendering` para desactivar solo la vista principal; los sensores siguen renderizando sus imágenes. Para capturas visuales conserva el rendering.

Las pruebas unitarias verifican elección de candidatos, validación de contrato, endpoints locales, errores sin secretos, deadline, revisión/edad de estado y rutas alrededor de obstáculos. La evidencia de pruebas físicas y de Laya se recoge en `VALIDATION.md`, con rutas a registros reales y limitaciones pendientes. Puedes auditar cualquier ejecución completa con `python3 tests/verify_physical.py results/ID`, y las pruebas de latencia añadiendo `--rejection deadline_exceeded` o `--rejection stale_observation`.
