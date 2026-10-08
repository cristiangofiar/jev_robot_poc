# Rover marciano con decisiones iterativas · Webots R2025a

Laya decide cada avance, giro, parada e interacción. Cada orden dura **0,8 s de simulación** por defecto. El rover usa una copia local del Sojourner oficial R2025a en `protos/Sojourner.proto`: seis ruedas, dirección en las cuatro exteriores y suspensión original, con el límite de las seis ruedas ampliado de 0,6 a 1,8 rad/s para permitir triple velocidad. El controlador aplica motores y seguridad cada 16 ms. No hay A*, ruta de exploración ni acercamiento automático en el control Laya.

## Ejecutar

Requisitos: Webots R2025a y Python ≥3.12 configurado en Webots. Se usa biblioteca estándar y la API nativa. Conserva tu `.env`; `.env.example` sirve para instalaciones nuevas. Configura `WEBOTS_EXECUTABLE` o pasa `--webots`:

```text
macOS:   /Applications/Webots.app/Contents/MacOS/webots
Windows: C:/Program Files/Webots/msys64/mingw64/bin/webots.exe
```

El launcher también busca `WEBOTS_HOME`, `ProgramFiles` y PATH. En Windows puede usarse `py -3.12` en lugar de `python3`. La primera carga puede necesitar Internet para PROTO y texturas.

```sh
# Terminal 1: exclusivamente Laya local
python3 serve_brain.py --brain laya
# Terminal 2
python3 run_experiment.py --brain laya
# Comparación determinista independiente
python3 run_experiment.py --brain rules
```

Los modelos y el runtime descargados se conservan. En otra máquina: `python3 setup_models.py --brain laya --skip-originals`. Laya utiliza el runtime llama.cpp del manifiesto, GGUF BF16 y `/v1/systemone` en loopback. El servidor tiene una ranura, contexto y batch físico de 4096: el batch anterior de 512 no admitía esta observación. Hay calentamiento previo, registrado en `warmup.json` y `webots.log`. El modo Laya usa `realtime`; acelerar la física puede caducar observaciones.

Kev local y Jev API mantienen el mismo adapter y contrato. `--brain kev` requiere su servidor; `--brain jev` utiliza la API configurada y puede generar cargos. **La validación de esta adaptación usa únicamente Laya local**, sin cargar Kev ni llamar a Jev. Windows tiene selección de EXE y rutas relativas comprobada mediante pruebas; falta ejecución física en Windows.

También puedes abrir `worlds/mars.wbt` directamente: usa las variables `MARS_*` y permanece abierto al terminar. Recarga para una misión nueva. El HUD distingue orden solicitada, acción, motores, seguridad, fallback, inventario y cobertura. La cámara mantiene su ventana nativa y Display.

Al reiniciar desde la UI, Webots puede conservar `MARS_RUN_DIR` del launcher. Si ese directorio ya contiene `steps.jsonl`, `ground_truth.jsonl` o `frames`, el rover crea un directorio nuevo junto al anterior y comunica esa ruta al Supervisor. Los logs y frames anteriores se conservan; el primer arranque del launcher utiliza su directorio preparado con normalidad.

A la izquierda de la vista 3D, debajo de la cámara, se muestran las seis últimas decisiones en orden cronológico. Cada entrada incluye número de petición, tiempo simulado, acción solicitada, acción aplicada y resultado, con el motivo de bloqueo o rechazo. Una decisión nueva aparece al final y desplaza las antiguas hacia arriba; el resultado actualiza su fila, sin duplicarla. Los bloqueos se muestran en naranja, los fallos en rojo y las ejecuciones completadas en verde. Esta lista es independiente del contexto compacto enviado al brain y funciona con todos los modelos y el comparador.

`results/FECHA_brain_ID/webots.log` muestra únicamente las respuestas del brain, con el mismo formato legible de `serve_brain.py`: separadores, hora/brain/HTTP/latencia, elección/confianza y JSON completo indentado con `model`, `answers` y `usage` cuando el servicio lo devuelve. Laya, Kev y Jev comparten el mismo formateador. Las respuestas aparecen también en la consola de Webots y en la terminal del launcher. El calentamiento se marca `warmup`; las respuestas rechazadas o tardías se identifican en la cabecera, manteniendo el JSON original.

Los prompts, sensores, acciones, resultados físicos, intervenciones y errores quedan en `steps.jsonl`; el resumen permanece en `summary.json`. Los diagnósticos de Webots se conservan en `webots-runtime.log` y los del servidor local en `.local_models/laya-runtime.log` o `kev-runtime.log`. Estos diagnósticos no se copian al registro de respuestas. Los fallos de calentamiento quedan en `warmup.json`, sin texto de excepción ni secretos. Para consultar las respuestas en vivo: `tail -f results/ID/webots.log`.

## Contrato de acciones

| Acción | Efecto |
| --- | --- |
| `FORWARD` | Avance recto, ruedas a 1,8 rad/s, aproximadamente 0,108 m/s; unos 0,086 m por intervalo de 0,8 s en terreno libre. |
| `REVERSE` | Retroceso recto a la misma velocidad, disponible solo tras el último movimiento bloqueado o interrumpido por la protección. La protección detiene por obstáculo trasero (`rear_obstacle`, lidar <0,8 m desde su montaje frontal) o límite detrás (`boundary_behind`). |
| `TURN_LEFT` / `TURN_RIGHT` | Giro en el sitio mediante dirección exterior y velocidades opuestas de 1,8 rad/s, triple velocidad anterior. |
| `STOP` | Parada durante un intervalo. |
| `INSPECT_<ID>` | Inspección del candidato indicado, sin mover ni acercar el rover. |
| `COLLECT_<ID>` | Recogida del candidato indicado, sin acercamiento. |
| `RETURN_TO_BASE` | Cambia el objetivo a la base. Los movimientos posteriores siguen siendo decisiones sucesivas. |

`SLOW_FORWARD` se eliminó del contrato de todos los brains. El avance y los giros usan triple velocidad; la duración de cada decisión y la espera de inferencia siguen siendo las mismas. El avance y los giros permanecen disponibles ante obstáculos: la protección únicamente puede parar. `REVERSE` es una opción de recuperación condicionada a un bloqueo registrado.

Las instrucciones de Laya, Kev y Jev indican usar `REVERSE` cuando se ofrezca tras un movimiento bloqueado, especialmente `front_obstacle` o `spin_clearance`, si hay espacio detrás y lo permiten los límites. Después deben observar de nuevo, recuperar espacio para girar y avanzar rodeando el obstáculo por un lado libre. El modelo decide cada paso; no hay recuperación automática. `REVERSE` requiere sensores válidos y que el último movimiento del historial reciente tenga resultado `blocked` o `interrupted` con motivo de seguridad. Un movimiento posterior completado elimina la opción; un bloqueo histórico, la falta de progreso o la mera cercanía de una roca no la habilitan por sí solos.

Las operaciones solo aparecen cuando son elegibles. El rover ofrece estas acciones al llegar a distancia estimada ≤0,63 m; inspeccionar requiere reconocimiento actual y dirección relativa dentro de ±18°. La acción frena, exige ≥1,2 s de parada y revalida llegada/visibilidad antes de pedir el ACK. Puede ocupar hasta 3 s para esa parada; no ejecuta avance ni giro. Recoger requiere inspección previa. El Supervisor verifica de forma independiente distancia física ≤0,68 m, velocidad horizontal <0,006 m/s y velocidad angular <0,04 rad/s durante ≥1 s. También verifica orientación ±20° y visibilidad comunicada por la cámara para inspeccionar. Solo un ACK válido añade inventario o inspección; solo el Supervisor oculta una muestra tras recogerla. Un pedido del modelo nunca basta.

La entrega es contabilidad local tras llegar físicamente a ≤0,27 m de la base y parar; el Supervisor exige ≤0,30 m y la parada física. El modelo conduce hasta allí. Los resultados del rover son `returned_and_delivered`, `returned_empty` o `time_limit`; regresar no certifica haber encontrado todas las muestras. El Supervisor evalúa aparte lo no descubierto, no recogido y no entregado.

## Lo que recibe Laya

La base `(0,0)` y región `x∈[-1,3.6]`, `y∈[-2.1,2.1]` son públicas. No se comunica el total de muestras, sus posiciones ocultas ni la geometría de rocas del Supervisor. Se descubre un ID únicamente al aparecer en el reconocimiento de la cámara. Se recuerda después, distinguiendo `visible`, antigüedad, distancia, dirección y estado de inspección. El objetivo durante exploración prioriza un candidato conocido; esto selecciona una referencia sensorial y nunca calcula su trayectoria.

La observación contiene cuatro distancias mínimas lidar (`front/left/right/rear`), velocidad, yaw, roll/pitch, objetivo y base relativos, candidatos elegibles, inventario recogido/entregado, presupuesto sintético, tiempo restante, acciones recientes, último modo aplicado a motores, seguridad actual y última intervención con timestamp, fallback, desplazamiento y giro en 8 s y repetición. La dirección se expresa además en texto breve para el clasificador; no se suministra una orden recomendada. Un filtro explícito del payload impide enviar datos del Supervisor o blancos exclusivos del controlador de comparación.

El prompt distingue proximidad de elegibilidad: una muestra cercana e invisible no se describe como lista para operar. Explica que pasar por encima no recoge la muestra y puede dejarla debajo o detrás de la cámara frontal; pide recuperar separación y orientación para verla de nuevo, observando tras cada movimiento. Aclara que girar en el sitio puede no bastar y que no existe marcha atrás: la recolocación debe usar los giros y avances disponibles por espacio libre. Recomienda aproximarse manteniendo la muestra delante, idealmente a 0,5–0,63 m, e inspeccionar cuando esa acción esté disponible antes de avanzar más. La recogida requiere inspección confirmada; si ya está inspeccionada, perder visibilidad no elimina por sí solo COLLECT. No se añade recuperación automática ni se filtran movimientos.

El historial distingue `requested` (decisión del brain), `accepted` (acción admitida tras validar la respuesta), `applied` (orden que llegó a ejecutarse o interacción confirmada) y `motor` (último modo y órdenes de velocidad de ruedas). Añade duración con motores activos, desplazamiento GPS, giro IMU y resultado `completed/blocked/interrupted/failed`. Una orden a los motores no prueba movimiento: las medidas físicas son independientes. Si FORWARD se bloquea antes de arrancar, queda `requested=FORWARD`, `accepted=FORWARD`, `applied=STOP`, motores a cero, desplazamiento cero y `safety=front_obstacle`. Si avanzó antes de la interrupción, conserva ese avance medido y termina con motores STOP. Las interacciones solo figuran aplicadas tras ACK válido. Cada petición recibe una copia compacta e independiente de las dos últimas ejecuciones finalizadas; el log conserva todos los resultados completos. La métrica de repetición sigue considerando las cuatro últimas decisiones.

El modo determinista usa las mismas operaciones y motores, con A* sobre obstáculos observados y celdas públicas sin visitar. `mission/baseline.py` se instancia exclusivamente con `rules/slow`; ni su ruta ni su referencia se entregan a Laya. Ningún fallback ejecuta esa política: el fallback aplica **STOP**.

La cámara RGB es 320×240, FOV horizontal 1,9 rad, muestreo/reconocimiento 64 ms y alcance de reconocimiento 3,2 m, con oclusión. La posición relativa proviene del [reconocimiento nativo de Webots](https://www.cyberbotics.com/doc/reference/camera?tab=python), una percepción idealizada de IDs, no clasificación visual aprendida. La proyección usa yaw y el tilt fijo; su precisión se limita al terreno suave de esta escena. El lidar aporta 360 rayos en una capa horizontal, alcance 4 m, cada 16 ms; se excluyen retornos del cuerpo antes de agrupar sectores. Una sola capa puede omitir objetos bajos.

GPS/IMU/brújula se mantienen activos. La IMU determina orientación; la prueba de sensores registra también la brújula. Cada petición guarda un PNG de su tick original de cámara; el modelo textual no recibe imágenes. INSPECT y COLLECT siguen siendo operaciones simbólicas sin brazo ni análisis científico.

## Exploración y regreso

La cobertura mide **centros de una cuadrícula pública de 0,4 m situados a ≤0,65 m del recorrido GPS**. El porcentaje y los centros sin visitar próximos por sector ayudan a explorar. Mide cercanía recorrida; no mide cobertura visual, área transitable exacta ni garantiza ausencia de muestras ocultas u ocluidas. No utiliza posiciones de muestras. Repetir una parada no aumenta esa cobertura.

Se comunica `return_due` al alcanzar el umbral de cobertura (65%), agotar el presupuesto de exploración (240 s), llegar a batería estimada ≤20% o consumir la reserva temporal de regreso. Esa reserva usa distancia pública a base: `2×distancia/0,078 + 35 s`; es una estimación conservadora ajustada a la nueva velocidad, sin resolver obstáculos. La batería es `100 − 0,045×segundos − 0,2×metros`, sin modelo eléctrico. Recoger todos los candidatos conocidos no dispara regreso ni éxito. Laya debe decidir iniciar y ejecutar el regreso; si ignora la condición puede acabar en `time_limit`.

```sh
python3 run_experiment.py --brain laya --action-duration 0.8 --explore-budget 240 --coverage-target 65 --duration 600
```

El intervalo configurable está acotado a `(0,1]` s. Los límites de entorno se validan también al abrir Webots directamente. Los parámetros temporales son segundos simulados salvo `--deadline` y `--timeout`, que son de reloj real.

## Asincronía y protección

Máximo una inferencia pendiente, incluso después de rechazarla y hasta que termine su thread. Ningún thread HTTP llama a Webots. Al expirar una acción los motores paran mientras llega otra decisión; la física continúa. No se rechazan respuestas por la edad simulada de la observación; `--max-age` y `MARS_MAX_AGE_S` fueron eliminados. Se rechazan deadline real (2 s), revisión cambiada, desplazamiento >0,06 m, cambio de yaw >0,12 rad, variación cercana del lidar >0,20 m y elección que perdió sus precondiciones. La revisión cambia al descubrir, inspeccionar, recoger, entregar o iniciar regreso. Las respuestas tardías que llegan durante la ejecución se registran y descartan. Una inferencia todavía en curso al terminar la misión se marca `mission_ended`; su duración final puede quedar sin medir.

La seguridad comprueba sensores, inclinación >0,4 rad, límites y proyección frontal del cuerpo, obstáculo frontal <0,32 m y espacio de giro <0,24 m. Interrumpe el intervalo y exige otra decisión para volver a moverse. La protección no calcula una maniobra de evasión. No se interpreta confianza como garantía de decisión correcta.

Las opciones de movimiento siguen disponibles ante obstáculos: el brain decide con los sensores y el resultado real de sus decisiones anteriores. La protección puede detener la acción escogida, sin seleccionar otra maniobra. El HUD muestra las órdenes de las ruedas y la última intervención con su timestamp, incluso durante la espera de otra decisión; «sin intervención activa» no significa que el frente esté libre.

## Evidencia y verificación

Cada ejecución guarda `results/FECHA_brain_ID/` con:

- `steps.jsonl`: observaciones/payloads originales, decisiones, rechazo, origen (`model/deterministic/fallback`), seguridad y sensores cada 0,2 s. `action` registra la aceptación; `action_result` registra la ejecución final, motores, desplazamiento/giro y protección. El auditor y resumen también admiten logs anteriores.
- `frames/request_NNNN.png`: RGB y timestamp de la petición original.
- `ground_truth.jsonl`: evaluación física y escena privada del Supervisor; nunca entra en el prompt.
- `summary.json`: descubrimientos, inspecciones, recogidas, entrega, contactos, intervalos sin progreso, repeticiones de decisiones, acciones bloqueadas/interrumpidas, fallbacks, fuentes y latencias.
- `snapshot/`: fuentes, PROTO local, mundo y manifiesto con hashes; excluye `.env` y pesos.
- `launch.json`, `warmup.json`, `webots.log` (respuestas legibles), `webots-runtime.log` (salida completa de Webots), `world_final.png`.

La latencia aceptada incluye transporte, inferencia, parseo y validación. La espera hasta timeout y la latencia tardía se guardan separadas. El reparto de distancia por fuente usa muestreo de pasos: es una aproximación que excluye desplazamiento de asentamiento parado. Los contactos se estiman con puntos físicos por encima de z=0,09 m, una heurística propia de este mundo; el contador cuenta pasos muestreados, no impactos únicos. Repetición incluye avances sucesivos normales; los intervalos sin progreso incluyen paradas y espera HTTP, no solo atascos. Los resultados siempre deben leerse junto con esas limitaciones.

```sh
python3 -m unittest discover -s tests -v
python3 run_experiment.py --brain rules --test sensors
python3 run_experiment.py --brain rules --test motion
python3 tests/interaction_world.py
python3 run_experiment.py --brain rules --world worlds/interaction.wbt --test preconditions
python3 run_experiment.py --brain rules
python3 run_experiment.py --brain laya
python3 run_experiment.py --brain slow --delay 1 --deadline 0.1 --duration 6
python3 run_experiment.py --brain slow --delay 0.3 --max-age 0.05 --duration 6
python3 tests/verify_physical.py results/ID
python3 tests/verify_physical.py results/ID --rejection deadline_exceeded
# Comprobar decisiones bloqueadas y su historial, sin reducir opciones de Laya
python3 tests/interaction_world.py --front-obstacle
python3 run_experiment.py --brain laya --world worlds/safety_feedback.wbt --duration 45
python3 tests/verify_physical.py results/ID --blocked
```

`tests/interaction_world.py` genera una escena controlada con una única muestra S17 a 0,58 m de la base; conserva la mecánica y el terreno. `--test preconditions` comprueba rechazo por parada insuficiente, recogida sin inspección y falta de visibilidad comunicada, después inspección/recogida/entrega válidas. Estas operaciones de fixture son control local de prueba, no decisiones de Laya. Puede ejecutarse Laya en esa escena con `--world worlds/interaction.wbt --explore-budget 8`. Esto prueba interacción cercana, sin demostrar acercamiento o regreso desde lejos.

Con `--front-obstacle`, el generador conserva las muestras originales y mueve una roca delante de la posición inicial para comprobar bloqueo inmediato. No inyecta decisiones: Laya sigue eligiendo entre todas sus opciones de movimiento. El auditor `--blocked` exige un FORWARD real del modelo bloqueado y una petición posterior que reciba ese resultado como STOP.

`slow` es un controlador sintético para fallos, sin otro modelo. Las ejecuciones truncadas salen con código no cero y conservan sus evidencias. `--skip-warmup` permite verificar física/fallback con el servicio indisponible. `--no-rendering` conserva los sensores renderizados. El auditor verifica ausencia de candidatos ocultos, frames, acciones acotadas, aplicación solicitada, STOP tras rechazos, operaciones físicas y parada final. Las comprobaciones reales y pendientes están en [VALIDATION.md](VALIDATION.md).
