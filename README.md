# Decision models for embodied agents

PoC experimental en Webots R2025a: los modelos seleccionan comportamientos
discretos; percepción, seguridad y actuadores permanecen fuera del modelo.

## Primera iteración (pasos 1–6)

El punto de partida tenía `cleaner.py` vacío y un apartamento de ejemplo. El
robot de estudio es el **iRobot Create**, ubicado en `(-4.65, -4.2, 0.0449)`;
el E-puck sobre la mesa conserva su controlador original. No hay todavía
Supervisor, obstáculos programados, navegación de limpieza ni métricas de
calidad. El baseline avanza con giros aleatorios periódicos. Ante un obstáculo
frontal o contacto, retrocede durante unos 0.5 s, gira a izquierda o derecha
durante 1–3 s y vuelve a avanzar cuando el frente está libre. La seed hace
reproducible la secuencia aleatoria.

Se renombró `world/` a `worlds/`, la [estructura nativa de proyectos Webots](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/src/webots/core/WbProject.cpp),
para que el mundo pertenezca al mismo proyecto que `controllers/`.

```text
Webots → Perception → Observation → Brain.decide → Decision
       → SafetyLayer → Actuators
                  ↘ JSONL (petición y acción aplicada por separado)
```

- `controllers/cleaner/brains/base.py`: `Action`, `Observation`, `Decision`,
  `Brain`; no importa Webots. `Observation.to_dict()` / `from_dict()` permite
  almacenar y reconstruir exactamente la entrada para un futuro replay.
- `brains/rules.py`: avance durante 3–6 s, giros aleatorios y recuperación
  con reversa ante distancia menor que `stop_distance_m` o contacto.
  Distancia inválida → `STOP`. No declara
  confidence ni probabilidades que no tiene.
- `perception.py`: lee sensores físicos y deriva una observación común.
  `object_detected` significa un retorno dentro del rango; `path_blocked` es
  una heurística del umbral frontal, no ground truth. No identifica tipo,
  movimiento ni trayectoria del objeto. La posición queda solo en el log.
- `safety.py`: fuerza `STOP` ante contacto, lectura inválida o distancia
  crítica; mantiene el stop hasta el umbral de liberación. Se evalúa cada
  timestep, incluso entre decisiones del brain. Con sensores válidos permite
  retroceder para liberar el contacto y girar cuando no hay contacto; la
  histéresis sigue bloqueando el avance hasta el umbral de liberación.
- `actuators.py`: convierte acciones en velocidades de ruedas idénticas
  para cualquier brain. `WAIT` y `REPLAN` detienen las ruedas; aún no existe
  planificador. Giros y marcha atrás no tienen sensores laterales/traseros;
  la reversa de recuperación es breve y los giros se detienen si hay contacto.
- `experiment/config.py`, `config.json`, `logger.py`: parámetros validados
  y registros reproducibles sin dependencias externas.

## Robot y sensores verificados

Se inspeccionó el [Create.proto de R2025a](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/irobot/create/protos/Create.proto):
`left wheel motor`, `right wheel motor`, `left wheel sensor`,
`right wheel sensor`, `bumper_left`, `bumper_right` y cuatro sensores
`cliff_*` orientados al suelo. Las ruedas tienen radio de 0.031 m y velocidad
máxima de 16.129 rad/s. Los sensores cliff **no** miden obstáculos delante.

El mundo añade mediante `bodySlot`:

- `front distance`: un rayo genérico frontal +X, a `(0.18, 0, 0.02)` respecto
  al robot, fuera del cuerpo, con alcance de 2 m. La [lookupTable](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/distancesensor.md)
  convierte distancia a metros con respuesta identidad y ruido cero.
  Se mide desde el origen del sensor, no desde el centro del robot.
  Un solo rayo no cubre el ancho completo del robot ni detecta objetos que
  aún caen por encima del rayo; el hard stop no garantiza evitar toda colisión.
- `benchmark gps`: [GPS sin ruido](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/gps.md)
  para posición y velocidad real en m/s; solo la velocidad entra al brain.

## Ejecutar y comprobar

1. Instalar Webots con soporte Python 3.10 o posterior.
2. Abrir `worlds/apartment.wbt` y **resetear/recargar el mundo** antes de cada
   ejecución. El Create ya utiliza `controller "cleaner"`.
3. Iniciar la simulación. El controlador deja el robot detenido al llegar a
   20 segundos simulados, sincroniza la orden final con `step(0)` sin avanzar
   la física y termina el controlador; no termina ni reinicia Webots.
4. Consultar `results/raw/<run_id>/steps.jsonl`.

La configuración está en `experiment/config.json`. Para usar otro archivo,
añadir `controllerArgs [ "--config" "/ruta/absoluta/config.json" ]` al Create.
Las rutas relativas de resultados se resuelven desde la raíz del proyecto.
El timestep y la seed deben coincidir con `WorldInfo` (16 ms y 0); el
controlador rechaza diferencias y arranques con tiempo simulado distinto de
cero. Cambiar `randomSeed` en el mundo al cambiar `seed` en la configuración.
No existe aún un runner que haga este cambio automáticamente.
`turn_wheel_speed_rad_s` es la velocidad de cada rueda durante un giro, no
la velocidad angular del cuerpo.

```sh
python3 -m unittest discover -s tests -v
```

Estas pruebas no requieren Webots. Incluyen ejecución del controlador con
dispositivos simulados, hard stop entre inferencias, fallback y lectura de
los registros. La validación física pendiente consiste en ejecutar el mundo
en Webots, mostrar los rayos con `View → Optional Rendering → Show Distance
Sensor Rays`, verificar su orientación y comprobar la parada ante un objeto
en la trayectoria. No se han obtenido todavía resultados físicos.

## Registro y tiempos

Cada ejecución crea un directorio nuevo y un JSONL con `run_start`, `step`
y `run_end`; cada línea incluye IDs, seed, modelo, versión y timestamp UTC.
Guarda la configuración efectiva y copias de los fuentes Python y del mundo,
con hashes SHA-256 en `run_start`. Los PROTO remotos están referenciados con
la etiqueta R2025a; no se archivan sus dependencias remotas. Conviene conservar
la versión instalada de Webots y su caché para reconstrucciones futuras.
`webots_version` queda `null` si el entorno no proporciona `WEBOTS_VERSION`;
`world_format_version` describe el formato del archivo, no la instalación.

En cada step se registran sensores, observación, decisión nueva si existe,
referencia a la última decisión, acción solicitada, acción aplicada, razón
de seguridad y velocidades ordenadas. `model_input` contiene **exactamente**
la observación recibida en esa inferencia; es `null` entre inferencias.

- `simulation_time_s` / `perception_time_s`: instante de muestreo en Webots.
- `*_wall_ms`: reloj monotónico en ms desde el inicio del controlador.
- `Decision.latency_ms`: tiempo de la llamada al adapter y validación de su
  salida; incluye overhead de Python. No representa solo el cómputo del modelo.
- `actuation_command_time_s` / `actuation_command_wall_ms`: emisión de la
  orden. El efecto físico se produce en los siguientes pasos de simulación.

El brain inicial es local y rápido; se llama de forma síncrona. Antes de
integrar una API habrá que ejecutar inferencia fuera del bucle físico con
timeouts y rechazo de decisiones obsoletas, para conservar el hard stop.
No comparar ms de simulación con ms de reloj real, ni inferir una reacción
física de la emisión de una orden.

`ground_truth`, evento, cámara, tokens, coste y `collision` quedan `null`
cuando no están disponibles. Un bumper positivo se registra como **contacto**;
no equivale a un detector completo de colisiones. `outcome="unscored"` no
afirma éxito de limpieza ni ausencia de colisión. Los errores del brain
generan una decisión `STOP` marcada como fallback; los errores del controlador
detienen motores, cierran el log y se propagan.
`stop_command_flushed` indica si se sincronizó la orden final con Webots;
es falso si el simulador ya había cerrado la conexión.

Resultados y `.env` están ignorados; no se guardan variables de entorno ni
credenciales. No se ha inicializado Git: el directorio original no era un
repositorio Git.

## Siguiente iteración

Pasos 7–8: Supervisor externo y primer escenario de caída con evento y
ground truth registrados por separado. Después: reset, repeticiones y
métricas, antes de integrar Jev/Clef o multimodal. No hay adapters ficticios
ni resultados comparativos inventados.
