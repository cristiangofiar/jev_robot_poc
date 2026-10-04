# Decision models for embodied agents

PoC experimental en Webots R2025a: los modelos seleccionan comportamientos
discretos; percepción, seguridad y actuadores permanecen fuera del modelo.

## Primera iteración (pasos 1–6)

El punto de partida tenía `cleaner.py` vacío y un apartamento de ejemplo. El
robot de estudio es el **iRobot Create**, ubicado en `(-4.65, -4.2, 0.0449)`;
el E-puck sobre la mesa conserva su controlador original. El modo
`apartment_static` no programa obstáculos. No hay todavía navegación de
limpieza. Las métricas iniciales del tercer incremento no incluyen calidad
semántica sin etiquetas por decisión. El baseline avanza recto hasta detectar contacto. Entonces retrocede durante unos 0.5 s, gira a izquierda o derecha
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
  Distancia inválida → `WAIT`. No declara
  confidence ni probabilidades que no tiene.
- `perception.py`: lee sensores físicos y deriva una observación común.
  `object_detected` significa un retorno dentro del rango; `path_blocked` es
  una heurística del umbral frontal, no ground truth. No identifica tipo,
  movimiento ni trayectoria del objeto. La posición queda solo en el log.
- `safety.py`: fuerza `WAIT` ante contacto o lectura inválida. Se evalúa cada
  timestep, incluso entre decisiones del brain. Con sensores válidos permite
  retroceder para liberar el contacto y girar cuando no hay contacto; la
  parada por proximidad se controla mediante `stop_before_contact`: está
  desactivada en ambas configuraciones para permitir avanzar hasta el contacto
  y ejecutar la recuperación del RuleBasedBrain.
- `actuators.py`: convierte acciones en velocidades de ruedas idénticas
  para cualquier brain. `WAIT` detiene las ruedas; aún no existe
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
   20 segundos simulados, sincroniza la orden final con `step(0)` sin avanzar
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
El runner del tercer incremento modifica seed y argumentos en copias aisladas.
`turn_wheel_speed_rad_s` es la velocidad de cada rueda durante un giro, no
la velocidad angular del cuerpo.

```sh
python3 -m unittest discover -s tests -v
```

Estas pruebas no requieren Webots. Incluyen ejecución del controlador con
dispositivos simulados, hard stop entre inferencias, fallback, vinculación de
logs y eventos del Supervisor. Ya se completaron seis ejecuciones físicas;
ver [registro de validación](experiment/validation_2026-10-04.md).
Queda inspeccionar visualmente los rayos con `View → Optional Rendering →
Show Distance Sensor Rays` y verificar su cobertura geométrica.

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

Los brains `rules` y `threshold` son síncronos. `kev`, `laya` y `jev` ejecutan
una única inferencia en un thread daemon mientras el bucle físico continúa.
Un deadline vencido produce WAIT; la petición que sigue ocupada no permite
crear más threads. Una respuesta obsoleta nunca se aplica. La capa de
seguridad sigue evaluándose en todos los pasos.
No comparar ms de simulación con ms de reloj real, ni inferir una reacción
física de la emisión de una orden.

El cleaner no recibe ground truth: sus campos `ground_truth` y evento siguen
en `null`, y una referencia apunta al archivo separado del Supervisor.
Cámara, tokens, coste y `collision` quedan `null` cuando no están disponibles.
Un bumper positivo se registra como **contacto**;
no equivale a un detector completo de colisiones. `outcome="unscored"` no
afirma éxito de limpieza ni ausencia de colisión. Los errores del brain
generan una decisión `WAIT` marcada como fallback; los errores del controlador
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
elimina al finalizar. Para `apartment_static`, no manipula el mundo; en modo batch espera al
cleaner y termina Webots.

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
| Duración / velocidad nominal | 20 s / 0.4 m/s |
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
  real del robot. `ideal_action=WAIT` es una etiqueta precautoria del evento,
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
JSONL. Las ejecuciones batch descritas abajo ya se probaron en Webots R2025a.

## Tercer incremento: repeticiones, métricas y tres adapters de texto

`run_experiment.py` crea un proyecto independiente por repetición y arranca
un proceso nuevo de Webots R2025a. Cada proyecto tiene fuentes, escenario,
configuración efectiva y un mundo con `seed + índice`; el original no se
reescribe. Esto reinicia física, controladores y objeto, en lugar de intentar
reutilizar estados vivos. El Supervisor cierra su log, espera el `run_end`
del cleaner con el WAIT final sincronizado y solo entonces llama
`simulationQuit`. Un proceso interrumpido o una pareja de logs incompleta
queda inválido.

Todos los modelos se ejecutan con `--mode=realtime`, timestep de 16 ms y
los mismos parámetros físicos. Usar modo fast con una API cambia la edad
simulada de sus observaciones y hace incomparables las latencias. El runner
realiza una inferencia de calentamiento fuera de la ventana puntuable y la
guarda en `warmup.json`.

```sh
# Preparar dos proyectos sin arrancar Webots ni llamar a una API.
python3 run_experiment.py --brain laya --scenario falling_object_close --runs 2 --dry-run

# Repeticiones reales: configurar WEBOTS_EXECUTABLE en .env.
python3 run_experiment.py --brain rules --scenario falling_object_close --runs 5 --seed 0
python3 run_experiment.py --brain threshold --scenario falling_object_close --runs 5 --seed 0
python3 analyze.py results/batches/ID_DEL_BATCH
```

`rules` conserva la recuperación por contacto. `threshold` añade el baseline
determinista WAIT si la distancia frontal es desconocida o menor al umbral,
y CONTINUE en otro caso. No declara confianza. Cada comparación debe usar
las mismas seeds, configuración, política de seguridad y ventana temporal.
Las dos configuraciones existentes conservan `stop_before_contact=false`;
los contactos e inputs inválidos siguen forzando la parada según la política
descrita arriba.

### Modelos locales en el M2 de 8 GB

[Kev-4B](https://huggingface.co/jaredpalmer/kev-4b) contiene un adapter y una
cabeza de decisión sobre Qwen3.5-4B; cargar el modelo original con el
[runtime del autor](https://github.com/jaredpalmer/kev) exige mucha más memoria
que la ruta aquí elegida. [Laya](https://huggingface.co/convaiinnovations/laya)
es un encoder de unos 421M parámetros con cabeza de decisión. Ninguno debe
tratarse como un chat que genera libremente el nombre de una acción.

`setup_models.py` descarga los artifacts originales de esos dos repositorios
y las conversiones públicas de ggml-org:

| Brain | Representación usada | Archivo de inferencia |
| --- | --- | --- |
| kev | GGUF Q4_K_M, adapter fusionado y cabeza conservada | ~3,0 GB |
| laya | GGUF BF16, checkpoint inglés raíz | ~0,84 GB |

El runtime nativo [llama.cpp b11391](https://github.com/ggml-org/llama.cpp/releases/tag/b11391)
incluye el endpoint de decisión `/v1/systemone` para ambas arquitecturas;
no se usa `/chat/completions`. Tiene Metal, un slot y contexto acotado.
Los repositorios, revisiones, representaciones, SHA-256 y versión del runtime
quedan en `.local_models/manifest.json`, y se copian al snapshot de cada run.
Los pesos y el runtime quedan ignorados por Git. Las revisiones del artifact
original y del GGUF se registran por separado; no se presume equivalencia
numérica entre la conversión cuantizada y el checkpoint original.

```sh
python3 setup_models.py  # macOS arm64; Python 3.12 o posterior

# Terminal 1: mantener SOLO UNO de los modelos cargado.
python3 serve_brain.py --brain laya

# Terminal 2: prueba del adapter y luego experimento.
python3 probe_brain.py --brain laya --output results/probes/laya.json
python3 run_experiment.py --brain laya --runs 5 --seed 0
```

Para Kev, detener Laya con Ctrl+C y sustituir `laya` por `kev` en ambos
comandos. El servidor escucha solo en loopback. No requiere PyTorch, MLX,
Transformers ni un entorno virtual: Python usa la biblioteca estándar y el
modelo se carga en un proceso nativo. Los artifacts descargados ocupan unos
5,5 GB en disco; RAM y disco son medidas diferentes.

Los dos modelos ya respondieron localmente a tres estados idénticos. Laya
eligió CONTINUE ante un obstáculo a 0,08 m con confianza ~0,9998; Kev eligió
WAIT. En esta prueba inicial Laya tardó ~163–576 ms y Kev ~1.203–2.138 ms.
Son pruebas de integración, no resultados de limpieza ni una evaluación
estadística. La confianza devuelta no implica calibración en este dominio.
La cuantización también puede alterar probabilidades y decisiones.

Se pudieron cargar y ejecutar por separado en este Mac. Esto no demuestra
que ambos más Webots y otras aplicaciones quepan a la vez en 8 GB. Kev
necesita especial atención a presión de memoria y latencia durante la
simulación. La carga original sin cuantizar no es la ruta recomendada aquí.

### Jev y prompts en inglés

El archivo `.env` está creado, con permisos 0600 e ignorado por Git. Completar:

```dotenv
OPENROUTER_API_KEY=tu_clave_de_openrouter
JEV_BASE_URL=https://openrouter.ai/api
JEV_MODEL=typesafe/jev-1.13
WEBOTS_EXECUTABLE=/ruta/a/Webots.app/Contents/MacOS/webots
```

`.env.example` contiene las demás variables: endpoint de Jev, URLs locales,
clave local opcional y precios opcionales por millón de tokens. El loader
no evalúa shell ni sustituye variables ya exportadas. El runner hereda las
credenciales a los controladores, sin copiar `.env` a proyectos ni snapshots.
Las peticiones registradas no llevan headers de autorización. Se verifica
TLS y se rechazan redirects que podrían reenviar la clave.

[JevBrain](controllers/cleaner/brains/jev.py) usa el contrato documentado de
[System One compatible de OpenRouter](https://openrouter.ai/blog/insights/what-is-jev/)
mediante `POST https://openrouter.ai/api/v1/systemone` y el modelo
`typesafe/jev-1.13`, con Bearer auth y elección tipada. Se registra el coste
devuelto en `usage.cost`; los precios de `.env` son solo un fallback si no
hay coste en la respuesta. No requiere instalar un SDK adicional.

```sh
python3 probe_brain.py --brain jev --output results/probes/jev.json
python3 run_experiment.py --brain jev --runs 5 --seed 0
```

Los prompts para Kev, Laya y Jev están en inglés en
[`behavior_selection.json`](controllers/cleaner/brains/prompts/behavior_selection.json).
Los tres reciben exactamente la misma pregunta, instrucciones y criterios,
con las ocho acciones del contrato, y el mismo JSON canónico de sensores.
Solo cambia el identificador de modelo enviado. Esto evita introducir
ventajas por instrucciones diferentes. Cada respuesta guarda la petición
exacta, hash del prompt, salida cruda, probabilidades, confianza, tokens y
modelo servido. `confidence` y probabilidad de la acción elegida se conservan
separadas: no se supone que sean la misma cantidad.

### Inferencia y métricas iniciales

Una inferencia pendiente mantiene la acción anterior mientras su observación
siga vigente; al inicio o si vence, se solicita WAIT. Solo hay una petición
en vuelo. `decision_timeout_s=3.0` mide reloj real y
`max_decision_age_s=2.0` mide edad en segundos simulados. Ambos son
configurables y quedan registrados; los valores iniciales permiten probar
Kev tras observar sus latencias. No hay retries en el bucle físico.

El log añade `decision_request` y `decision_result`, IDs de petición y de
la decisión activa, instante de la observación original y aceptación o
rechazo. La decisión que responde se vincula con su input original, aunque
el robot ya tenga sensores distintos. Un resultado rechazado se conserva
en metadata si ya llegó. Un request todavía pendiente al cierre se cuenta
como tal y no se inventa su respuesta ni su coste.

`analyze.py` genera `metrics.csv` y `aggregate.csv` por batch o árbol de runs:

- Colisión con la caja por ground truth, contacto por bumper y distancia
  mínima entre centros, manteniendo sus significados diferentes.
- Latencias del adapter, respuestas, errores, deadlines, obsolescencia y
  peticiones pendientes al cierre.
- Pasos con override de seguridad, cambios de acción, fracción de comandos
  de parada y distancia recorrida a partir del GPS.
- Tiempo desde liberación hasta una transición nueva a parada/desaceleración,
  atribuido a modelo, seguridad o fallback. Es una transición de comando,
  no un tiempo de frenado físico. La primera observación `path_blocked`
  tras el evento es heurística y no identifica inequívocamente a la caja.
- Coste estimado solo cuando hay precios y tokens de todas las respuestas;
  desconocido queda vacío, no cero.

Los agregados separan modelo, versión, escenario, parámetros controlados,
artifact del modelo y fuentes/geometría (normalizando solo la seed del mundo). Excluyen runs incompletos del denominador de colisiones
y señalan seeds planificadas sin un log. Comparar explícitamente el mismo
conjunto de seeds; la media de latencia agregada es una media por run.
Accuracy, ECE y Brier quedan vacíos porque aún faltan etiquetas por decisión:
la etiqueta WAIT del evento no se aplica artificialmente a toda la serie.

El siguiente incremento puede añadir datasets etiquetados y replay offline
para evaluar calidad/calibración, antes de cámara y multimodal.

### Arranque de Laya y diagnóstico de WAIT

El estado enviado a los modelos se describe en inglés, incluyendo el objetivo,
los sensores y la acción previa como descripción (por ejemplo, `stationary`),
sin presentar el enum `WAIT` como una instrucción que repetir. El prompt usa
criterios breves para avanzar con camino libre, detenerse ante bloqueo y
retroceder ante contacto. La observación original y la petición transformada
quedan registradas para poder inspeccionar ambas.

En una prueba local del checkpoint instalado, el prompt anterior eligió WAIT
con camino libre y acción previa WAIT; el nuevo eligió CONTINUE desde reposo,
WAIT ante bloqueo y BACK_UP ante contacto. Las probabilidades elegidas fueron
bajas (aproximadamente 0.25–0.35): es una comprobación de estos ejemplos, no
una validación de navegación ni calibración. `probe_brain.py` incluye dos
estados idénticos con diferente acción previa para comprobar regresiones.

Para Webots, arrancar primero `python3 serve_brain.py --brain laya`.
El servidor temporal del notebook no utiliza necesariamente el puerto que
espera el controlador. Los fallbacks registran `error_code` para distinguir
fallos de conexión y errores HTTP; el primer fallo también aparece en consola.

### Outputs visibles del servidor local

`python3 serve_brain.py --brain laya` (o `kev`) muestra cada respuesta de
`/v1/systemone` en un bloque con hora, estado HTTP, latencia, elección,
confianza y JSON completo con probabilidades. Las peticiones se retransmiten
al runtime en un puerto interno de loopback sin modificar sus respuestas.
Los mensajes de carga de Metal quedan en `.local_models/<brain>-runtime.log`.
Los headers de autorización no se imprimen. Reiniciar el servidor para
activar este formato; `Ctrl+C` libera el proceso del modelo.

### Acciones disponibles

El contrato actual contiene seis acciones: CONTINUE, SLOW_DOWN, TURN_LEFT,
TURN_RIGHT, BACK_UP y WAIT. STOP se ha eliminado del enum, de los
prompts y de los comandos de la simulación. WAIT mantiene ruedas a cero
para sensores inválidos, fallbacks, contacto que impide avanzar, espera de
inferencia y fin de ejecución. Eliminar STOP no implica movimiento continuo:
WAIT también es una acción estacionaria. Los resultados históricos
no se modifican y el analizador sigue reconociendo sus etiquetas antiguas.

### Historial de decisiones y WAIT

Cada observación incluye las últimas diez decisiones completadas, en orden
cronológico, con tiempo simulado, acción solicitada, acción aplicada y si
la respuesta fue aceptada. Se excluyen inferencias pendientes y no se añaden
entradas por cada timestep. También incluye el tiempo continuo en WAIT,
contado sobre la acción aplicada, incluso si fue impuesta por seguridad.
Cada inferencia recibe una copia del historial; se reinicia en cada ejecución.

WAIT representa una pausa breve ante peligro temporal o sensores desconocidos.
El prompt pide reevaluar en la próxima decisión y recuperar el movimiento
cuando los sensores lo permitan. Esto aporta contexto al modelo; no fuerza
un movimiento ni impone una duración máxima a las paradas de seguridad.
