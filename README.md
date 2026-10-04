# Decision models for embodied agents

PoC experimental en Webots R2025a: los modelos seleccionan comportamientos
discretos; percepción, seguridad y actuadores permanecen fuera del modelo.

## Primera iteración (pasos 1–6)

El punto de partida tenía `cleaner.py` vacío y un apartamento de ejemplo. El
robot de estudio es el **iRobot Create**, ubicado en `(-4.65, -4.2, 0.0449)`;
el E-puck sobre la mesa conserva su controlador original. El modo
`apartment_static` no programa obstáculos. No hay todavía navegación de
limpieza ni métricas agregadas de calidad. El baseline avanza recto hasta detectar contacto. Entonces retrocede durante unos 0.5 s, gira a izquierda o derecha
durante 1–3 s y vuelve a avanzar. La seed hace
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
- `brains/rules.py`: avance continuo; únicamente ante contacto hace una
  reversa breve seguida de un giro aleatorio.
  Distancia inválida → `STOP`. No declara
  confidence ni probabilidades que no tiene.
- `perception.py`: lee sensores físicos y deriva una observación común.
  `object_detected` significa un retorno dentro del rango; `path_blocked` es
  una heurística del umbral frontal, no ground truth. No identifica tipo,
  movimiento ni trayectoria del objeto. La posición queda solo en el log.
- `safety.py`: fuerza `STOP` ante contacto o lectura inválida. Se evalúa cada
  timestep, incluso entre decisiones del brain. Con sensores válidos permite
  retroceder para liberar el contacto y girar cuando no hay contacto; la
  parada por proximidad se controla mediante `stop_before_contact`: está
  desactivada en ambas configuraciones para permitir avanzar hasta el contacto
  y ejecutar la recuperación del RuleBasedBrain.
- `actuators.py`: convierte acciones en velocidades de ruedas idénticas
  para cualquier brain. `WAIT` y `REPLAN` detienen las ruedas; aún no existe
  planificador. Giros y marcha atrás no tienen sensores laterales/traseros;
  la reversa de recuperación es breve y los giros se detienen si hay contacto.
- `experiment/config.py`, `configs/*.json`, `logger.py`: parámetros validados
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
3. Iniciar la simulación. El mundo selecciona ahora `falling_object_close`
   para ambos controladores. El cleaner deja el robot detenido al llegar a
   6 segundos simulados, sincroniza la orden final con `step(0)` sin avanzar
   la física y termina el controlador; no termina ni reinicia Webots.
4. Consultar `results/raw/YYYY-MM-DD_HH-MM-SS/steps.jsonl`.

`experiment/configs/config.json` conserva la configuración estática (60 segundos,
0.4 m/s). `experiment/configs/falling_object_close.json` selecciona el nuevo escenario.
Para usar otro archivo, poner el **mismo**
`controllerArgs [ "--config" "experiment/configs/config.json" ]` en `DEF CLEANER`
y `DEF OBJECT_SPAWNER`. Se aceptan rutas absolutas o relativas a la raíz.
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
dispositivos simulados, hard stop entre inferencias, fallback, vinculación de
logs y eventos del Supervisor. La validación física pendiente consiste en ejecutar el mundo
en Webots, mostrar los rayos con `View → Optional Rendering → Show Distance
Sensor Rays`, verificar su orientación y comprobar la parada ante un objeto
en la trayectoria. No se han obtenido todavía resultados físicos.

## Registro y tiempos

Cada ejecución crea un directorio con fecha y hora local hasta los segundos
(`YYYY-MM-DD_HH-MM-SS`). Si ya existe, añade `_1`, `_2`, etc., sin sobrescribir
resultados anteriores. El `run_id` interno sigue siendo un UUID. Guarda un JSONL con `run_start`, `step`
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

El cleaner no recibe ground truth: sus campos `ground_truth` y evento siguen
en `null`, y una referencia apunta al archivo separado del Supervisor.
Cámara, tokens, coste y `collision` quedan `null` cuando no están disponibles.
Un bumper positivo se registra como **contacto**;
no equivale a un detector completo de colisiones. `outcome="unscored"` no
afirma éxito de limpieza ni ausencia de colisión. Los errores del brain
generan una decisión `STOP` marcada como fallback; los errores del controlador
detienen motores, cierran el log y se propagan.
`stop_command_flushed` indica si se sincronizó la orden final con Webots;
es falso si el simulador ya había cerrado la conexión.

Resultados y `.env` están ignorados; no se guardan variables de entorno ni
credenciales. Los snapshots ahora incluyen el Supervisor y las definiciones
JSON de escenarios, además del cleaner y la configuración efectiva.

## Segunda iteración: caída reproducible (pasos 7–8)

El mundo añade un `Robot` invisible con `supervisor TRUE`, controlador
`controllers/object_spawner/object_spawner.py` y un `Receiver` de radio en el
canal 42. El Create tiene un `Emitter` llamado `experiment lifecycle` en ese
canal. Solo envía IDs, configuración efectiva y nombre del directorio de
resultados; no recibe mensajes del Supervisor. No hay ground truth en
`Observation` ni acceso a nodos Webots desde el brain.

El [Supervisor de Webots](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/supervisor.md)
coloca el robot en la pose inicial del escenario antes del primer timestep,
crea una caja con `importMFNodeFromString`, observa sus poses/contactos y la
elimina al finalizar. Para `apartment_static`, termina sin manipular el mundo.

`experiment/scenarios/falling_object_close.json` define:

| Parámetro | Valor |
| --- | --- |
| Posición inicial del Create | (-4.65, -5.2, 0.0449) m |
| Orientación inicial | +X local hacia -X del mundo (yaw π) |
| Instante programado | 2.0 s simulados |
| Offset frontal de la caja | 0.7 m respecto al centro nominal del robot |
| Offset lateral | 0 m |
| Altura inicial del centro de la caja | 0.75 m sobre z=0 |
| Tamaño / masa | (0.2, 0.3, 0.2) m / 0.4 kg |
| Duración / velocidad nominal | 6 s / 0.4 m/s |
| Posición de liberación en el mundo | (-6.15, -5.2, 0.75) m |

El avance nominal a 2 s es 0.8 m. La posición de liberación se calcula una
sola vez desde la pose inicial, ese avance y el offset; **no** se calcula desde
la pose real del robot en el momento del evento. Así permanece igual si un
brain ya se detuvo o giró. El offset de 0.7 m es entre centros, no la distancia
que devolverá el sensor frontal. El evento no usa azar; la seed mantiene
controlada la física y las maniobras aleatorias del baseline.

La caja se importa con `Physics` y velocidad inicial nula. La liberación se
ordena en el primer límite de timestep igual o posterior al instante
programado: `scheduled_time_s`, `release_command_time_s` y
`first_physics_step_after_release_s` distinguen programación, orden y primer
paso físico. El mundo fija explícitamente `gravity 9.81`.

El Supervisor comprueba que su configuración coincida exactamente con la
del cleaner y que la definición del escenario coincida con su snapshot.
Escribe `ground_truth.jsonl` en el mismo directorio, con los mismos IDs:

- `scenario_start`: parámetros, posición nominal y contrafactual analítico.
- `event_released`: instante real de la orden, posición de la caja y pose
  real del robot. `ideal_action=STOP` es una etiqueta precautoria del evento,
  no una verdad universal por step.
- `trajectory`: pose y velocidad reales de robot/caja, distancia entre
  centros y puntos de contacto de ambos, desde la liberación hasta el cierre.
- `scenario_end`: estado, contacto acumulado con la caja, distancia mínima
  **entre centros** y confirmación de eliminación del objeto.

Para atribuir contactos a esta caja se buscan puntos de contacto comunes
entre el robot (incluidos sus descendientes) y la caja, con tolerancia de
1 µm; los contactos de la caja con el suelo por sí solos no cuentan como
colisión con el robot. Esta atribución debe validarse en Webots; se guardan
los puntos originales para auditarla. No se mide todavía la severidad de
colisión ni la distancia mínima entre superficies.

`nominal_collision_expected` es una estimación bajo caída libre seguida de
caja estática, robot vertical a velocidad constante y sin otros obstáculos;
no es un replay físico contrafactual. `no_object_contact_observed` significa
que no se observaron contactos compartidos con esta caja durante la ventana;
no afirma éxito de limpieza ni ausencia de otros contactos. Una interrupción
o error produce `outcome=incomplete`. Para considerar una ejecución válida
se necesitan tanto `run_end` del cleaner como `scenario_end` del Supervisor
completos y sin errores; un sidecar ausente tampoco demuestra éxito.

El baseline actual sigue intentando `CONTINUE` antes del contacto; la capa
de seguridad puede ser quien detenga el robot. Los logs mantienen esa
distinción: esta iteración no demuestra calidad semántica del brain.

Para la prueba física: recargar el mundo, comprobar la pose inicial, ver la
caja aparecer a los 2 s, revisar su caída y el hard stop, y consultar ambos
JSONL. No se ha ejecutado esta validación en un Webots instalado.

## Siguiente iteración

Pasos 9–11: reset automático, repeticiones y primeras métricas, antes de
integrar Jev/Clef o multimodal. No hay adapters ficticios ni resultados
comparativos inventados.
