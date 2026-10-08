# Validación de control iterativo · 8 de octubre de 2026

Webots R2025a, macOS Apple Silicon, Python 3.14. Trabajo en `botdriver`. Se investigó antes de editar mediante Codegraph (`status`, `query Pending`, `callers criteria`, `impact Navigator`) y se sincronizó después. `.git`, `.env` y los modelos descargados se conservaron. No se cargó Kev ni se llamó a Jev. Las inferencias reales utilizan Laya BF16 local a través de `/v1/systemone`.

**El control y sus comprobaciones físicas están implementados; Laya todavía no demuestra una misión autónoma de acercamiento, inspección, recogida y regreso.** Los fallos se conservan y no se sustituyen por navegación determinista oculta. La versión anterior que entregó todas las muestras escogía objetivos y utilizaba A*; sus resultados no validan este nuevo problema de control.

## Evidencia física

Todas las rutas siguientes son directorios reales bajo `results/`, con `steps.jsonl`, `ground_truth.jsonl`, snapshot, configuración, frames cuando hay peticiones y captura final. El auditor `tests/verify_physical.py` verifica inventario contra operaciones del Supervisor, frames originales, ausencia de candidatos ocultos, límites de acción, STOP de fallback y parada final.

| Prueba | Directorio | Comprobación real |
| --- | --- | --- |
| Sensores | `20261008_003441_rules_61e8df` | GPS/IMU/brújula válidos, nube lidar y RGB de 320×240, estabilidad y reconocimiento. S1/S2 aparecen a 0,064 s; S3 permanece sin descubrir. |
| Avance, dirección y giro | `20261008_005657_rules_b2da76` | Avance hacia +X >0,20 m, dirección exterior a izquierda, giro izquierdo y derecho >0,4 rad, velocidad final <0,006 m/s. Protección frontal interrumpe parte del tramo de dirección; cero contactos muestreados. Auditor pasó. |
| Precondiciones negativas/positivas | `20261008_005934_rules_f91057` | Escena generada con una única S17 a 0,58 m. Rechaza parada insuficiente, COLLECT sin inspección y INSPECT sin visibilidad comunicada. Después valida inspección, recogida y entrega. Secuencia de ACKs: `false,false,false,true,true,true`. Auditor pasó. Estas son operaciones del fixture local, sin decisiones del modelo. |
| Laya iterativa, 420 s | `20261008_003619_laya_6bb787` | 422 decisiones aceptadas; avances y giros. S3 se descubre a 43,648 s, después de la observación inicial. Sin inspecciones, recogidas ni regreso; termina en `time_limit`. 31 fallbacks STOP, 326 intervenciones, cero contactos muestreados. 454 PNGs auditados. |
| Laya cercana, parada como requisito previo | `20261008_004654_laya_61b647` | 45 s, una muestra, 53 decisiones aceptadas; repite FORWARD y no se detiene para inspeccionar. Sin operaciones ni entrega. Auditor pasó. |
| Laya cercana, INSPECT/COLLECT incluyen frenado y dwell | `20261008_004832_laya_688aa0` | 45 s, 48 decisiones aceptadas, incluidas paradas y avances. El modelo no elige INSPECT ni COLLECT pese a su disponibilidad al inicio. Sin operaciones ni entrega; cero fallbacks y contactos. Auditor pasó. |
| Comparador con contrato final | `20261008_005002_rules_12561d` | Inspección física a 33,872 s, recogida a 33,968 s, entrega a 131,760 s. Una muestra entregada, otras dos pendientes según evaluación privada. Tres descubrimientos, 158 decisiones deterministas, sin contactos/intervenciones/fallbacks. Auditor pasó. |

La prueba larga Laya conserva su snapshot anterior a los ajustes finales de STOP, disponibilidad de RETURN y frenado de operaciones. También hubo sondas diagnósticas contra el mismo servidor durante esa ejecución: sus latencias son exploratorias y pueden incluir cola de esas sondas. Mediana aceptada 621,6 ms, superior a la referencia anterior de 178–304 ms; contexto, criterios y frecuencia cambiaron. No se considera benchmark controlado ni garantía temporal.

## Comparación con el contrato final

Presupuestos iguales: máximo 180 s simulados, exploración 100 s, intervalo 0,8 s y misma escena. `rules` usa `fast`; Laya usa `realtime`. Son dos ejecuciones individuales, no estimaciones estadísticas.

| Métrica | Determinista `20261008_005002_rules_12561d` | Laya `20261008_005059_laya_2ffd11` |
| --- | --- | --- |
| Resultado | Regresa y entrega inventario parcial a 131,760 s | `time_limit`, 180 s, no regresa |
| Descubrimientos / inspecciones / recogidas / entregas | 3 / 1 / 1 / 1 | 3 / 0 / 0 / 0 |
| Contactos muestreados | 0 | 0 |
| Intervalos sin progreso de la métrica | 0 | 14 |
| Acciones sucesivas repetidas | 107 | 231 |
| Intervenciones de seguridad | 0 | 217 |
| Fallbacks | 0 | 1, STOP por `state_changed` al cambiar percepción |
| Decisiones aceptadas | 158 deterministas | 244 del modelo |
| Mediana de decisión/inferencia aceptada | 0,1 ms, cálculo local | 601,0 ms, Laya/HTTP/validación |
| Distancia aproximada atribuida al origen de órdenes | 2,774 m deterministas | 0,575 m modelo; 0 m fallback |
| Cobertura de proximidad final | 17,3% | 10,9% |

S3 se descubre con Laya a 56,000 s y aparece en sus siguientes observaciones; una respuesta basada en la revisión anterior se descarta. Se auditaron 246 frames originales. El Supervisor confirma por separado que las muestras pendientes permanecen sin recoger. La menor cobertura y repetición de Laya muestran un fallo de conducción; cero contactos no implica que el modelo evite obstáculos correctamente, porque la protección intervino 217 veces.

Las sondas diagnósticas adicionales se realizaron en la ejecución larga anterior; no se enviaron durante esta comparación final. Cada directorio conserva la fuente exacta de su ejecución. Tras las comparaciones se conservó también la última intervención de seguridad con timestamp en la observación, junto al modo aplicado a motores, para no perder ese contexto durante la espera parada; falta repetir una misión larga Laya con ese campo adicional. El calentamiento de la versión final del launcher usa un estado sintético sin candidatos; no aporta información de escena.

## Asincronía y fallos comprobados físicamente

| Caso | Directorio | Evidencia |
| --- | --- | --- |
| Deadline real 0,1 s, retraso sintético 1 s | `20261008_005426_slow_57acf2` | Cinco rechazos por timeout, cuatro respuestas tardías descartadas de 1000,6–1010,3 ms; esperas hasta rechazo 100,7–108,4 ms. La ranura sigue ocupada y la física avanza, con todos los motores parados. El último worker seguía pendiente al terminar la ventana. Auditor con `--rejection deadline_exceeded` pasó. |
| Edad máxima simulada 0,05 s, retraso 0,3 s | `20261008_005534_slow_508cce` | Cinco `stale_observation`, latencias 300,4–310,2 ms y cinco fallbacks STOP. Física, cámara y logs continúan. Auditor con `--rejection stale_observation` pasó. |
| Laya local detenido, `--skip-warmup` | `20261008_005605_laya_c44d48` | Seis `URLError` sin texto de excepción ni secretos; seis STOP. Cero navegación y cero operaciones, con física durante 6 s. No se inicia otro modelo. Auditor con `--rejection URLError` pasó. |

Estas ventanas terminan intencionadamente en `time_limit` y el launcher devuelve código no cero; ambas fuentes de logs terminan y el auditor pasa. El movimiento final de ~0,0027 m es asentamiento inicial del rover, con órdenes STOP, no avance por fallback. Las respuestas muy rápidas pueden caer entre pasos registrados cada 0,208 s; sus timestamps de simulación prueban avance del timestep aunque no haya una fila de sensor intermedia.

Las pruebas unitarias añaden cambios de posición y yaw sin cambio de inventario y cambios cercanos de clearance. La ejecución larga rechazó una respuesta por clearance; la comparación final rechazó una por nueva detección. El diseño normalmente consulta tras acabar el intervalo y frena durante HTTP, reduciendo desplazamiento de la observación; aun así comprueba pose, revisión y sensores antes de aplicar.

## Qué atribuir a cada componente

Laya solicita movimientos. Cada movimiento aceptado se aplica durante el intervalo acotado o hasta una interrupción de seguridad. La espera HTTP y los fallbacks paran. La protección decide exclusivamente detener; no elige rutas ni maniobras. INSPECT/COLLECT pueden frenar y mantener una parada una vez alcanzada la proximidad, pero nunca acercan el rover. El Supervisor valida las operaciones e incorpora su resultado físico al inventario. La entrega automática es contabilidad después de llegar y parar en base.

El comparador instancia A* únicamente en `rules/slow`, con obstáculos de lidar y muestras ya reconocidas; no recibe posiciones ocultas. Sus resultados son control determinista independiente. Las operaciones del fixture se marcan `physical_fixture`. La escena de una muestra comprueba independencia respecto al total y validación local; empezar cerca de base y muestra no valida navegación general de Laya.

Cobertura significa centros de cuadrícula pública próximos al GPS, no cobertura visual ni exploración completa. Ninguna ejecución termina por haber recogido los candidatos conocidos. Los totales, muestras no descubiertas y no recogidas se guardan únicamente en la evaluación del Supervisor. Regresar y entregar inventario puede ser un resultado parcial.

## Comprobaciones automatizadas y límites

`python3 -m unittest discover -s tests -v`: **13 pruebas pasaron**. Verifican acciones/candidatos, contrato System One y probabilidades, endpoints locales, adapters compartidos sin llamadas, independencia del inventario para regresar, errores sin texto secreto, deadline con worker aún vivo, revisión/edad, cambios de pose y clearance sin cambiar inventario, sectores lidar, cobertura, seguridad, A* de comparación y selección Windows/EXE/rutas relativas. Añaden bloqueo sin restringir opciones, interrupción tras movimiento parcial, órdenes de motores sin movimiento medido, resultados de operaciones/fallback y conservación de una copia independiente del historial ante lecturas asíncronas. La comprobación Windows utiliza archivos simulados; no es ejecución física en Windows.

El RGB y reconocimiento son sensores reales del simulador, pero los IDs/posiciones del reconocimiento son idealizados. La proyección yaw/tilt se limita a pendientes suaves. El lidar horizontal puede omitir objetos bajos; su filtrado del cuerpo y los umbrales son específicos de este rover. Contactos por encima de z=0,09 m son una heurística de escena. Repeticiones incluyen avances normales; intervalos sin progreso incluyen espera y paradas. La batería y reserva de regreso son estimaciones, sin modelo eléctrico ni prueba de que exista una ruta.

Se corrigieron en pruebas rayos sin retorno que contienen componentes no finitas, batch físico insuficiente para la nueva observación, filtrado que debía conservar incluso los retornos frontales inmediatos, el momento de actualización del plan exclusivo del comparador y su llegada a distancia suficiente para la interacción. Las ejecuciones iniciales fallidas o intermedias se conservan para diagnóstico. Después del último ajuste lidar se repitió movimiento (`20261008_005657_rules_b2da76`) y después del Supervisor final se repitieron precondiciones (`20261008_005934_rules_f91057`); ambos auditores pasaron. La compilación y `git diff --check` pasaron. El servidor Laya iniciado para las pruebas quedó detenido al terminar.

Queda pendiente conseguir conducción útil y consistente con Laya —incluidos acercamiento, elección de INSPECT/COLLECT y regreso desde lejos—, evaluar más escenas/semillas y ejecutar físicamente en Windows. Kev/Jev mantienen compatibilidad contractual sin validación real. No se afirma fiabilidad estadística, percepción aprendida ni exploración exhaustiva.

## Historial de ejecución real y decisiones libres · 8 de octubre de 2026

Se conservaron los criterios de acciones: los obstáculos no eliminan FORWARD, SLOW_FORWARD ni los giros. La protección sigue limitándose a parar. `action` registra la aceptación y `action_result` registra el resultado final, con orden solicitada/admitida/aplicada, últimas órdenes de ruedas, tiempo con motores comandados, desplazamiento GPS, giro IMU, protección y fallback. Una interrupción después de moverse conserva el movimiento parcial; un bloqueo inmediato registra STOP y cero movimiento. INSPECT/COLLECT solo se registran aplicadas tras ACK válido. El HUD conserva la última intervención con timestamp durante la espera de otra decisión.

| Prueba | Directorio | Evidencia |
| --- | --- | --- |
| Escena habitual, historial compacto | `20261008_212247_laya_5645ba` | 70 s, 42 decisiones aceptadas, tres descubrimientos, sin contactos ni intervenciones, 0,907 m de distancia aproximada atribuida a órdenes del modelo. No inspecciona ni recoge; termina por tiempo. Mediana aceptada 701,3 ms. El auditor verifica que cada petición recibe los resultados finalizados y que las opciones de movimiento siguen disponibles. |
| Obstáculo frontal desde el inicio | `20261008_212546_laya_a33e51` | Fixture que conserva las muestras y desplaza ROCK0 a x=0,8 m. Laya elige libremente FORWARD 67 veces durante 45 s. Las 67 acciones quedan `accepted=FORWARD`, `applied=STOP`, `result=blocked`, `safety=front_obstacle`, ruedas a cero y desplazamiento/giro cero. Las siguientes observaciones reciben esos resultados y mantienen las opciones de avance y giro. Todos los pasos tienen órdenes STOP; cero contactos y cero fallbacks. Mediana 626,1 ms. Auditor `--blocked` pasó y revisó 68 frames. El asentamiento inicial de ~0,0029 m no se atribuye al modelo. |
| Interacciones y entrega del comparador | `20261008_212725_rules_e21c5f` | 158 decisiones deterministas, inspección a 33,872 s, recogida a 33,968 s y entrega a 131,760 s. Los resultados de INSPECT/COLLECT figuran completados/aplicados tras validación física, con motores STOP. Auditor pasó; cero contactos, intervenciones y fallbacks. |
| HUD y sensores finales | `20261008_213124_rules_781df2` | Prueba de sensores y auditor pasaron. Se revisó la captura final: HUD ampliado al lado de la cámara y mensaje de fin debajo de sus filas, sin tapar el historial. Compilación, comprobación de whitespace y sincronización de Codegraph completadas. |

El primer ensayo (`20261008_211912_laya_0a34de`) enviaba cuatro resultados completos: tres decisiones fueron aceptadas y 38 respuestas se rechazaron por antigüedad, con inferencias posteriores alrededor de 860–890 ms. El auditor pasó y comprobó STOP de fallback, pero ese contexto perjudicaba la latencia. Se compactó la observación a las dos últimas ejecuciones, conservando el historial completo en JSONL y la detección de repetición sobre cuatro decisiones. No se ampliaron deadlines ni edad máxima y no se filtraron acciones por obstáculos. La latencia sigue sin ser una garantía.

Estas pruebas reutilizaron el servidor Laya local del usuario; se mantuvo activo. No se cargó Kev ni se llamó a Jev. La prueba de bloqueo demuestra feedback correcto, pero también que Laya puede insistir en decisiones ineficaces aun recibiendo ese feedback. No se impuso una maniobra de recuperación ni se atribuyó movimiento a esas decisiones. Sigue pendiente una evaluación larga de inteligencia/navegación con el nuevo historial.

## Lista de decisiones, consola y triple velocidad · 8 de octubre de 2026

El Supervisor muestra las seis últimas ejecuciones a la izquierda, debajo de la cámara, con petición, timestamp simulado, decisión solicitada, acción aplicada, resultado y motivo de protección/rechazo. Los eventos de aceptación y finalización se envían individualmente, de modo que un bloqueo que empieza y termina entre refrescos del HUD también aparece. La finalización actualiza su fila; una decisión nueva desplaza las anteriores hacia arriba. El contexto del brain sigue limitado a dos ejecuciones y la detección de repetición a cuatro. Los rechazos/fallos aparecen en rojo, las interrupciones/bloqueos en naranja y las ejecuciones completadas en verde.

El rover imprime `response`, `late_response` y `action_result` para todos los brains. La respuesta muestra elección, probabilidades, confianza, modelo servido, latencia y rechazo; el prompt no se repite en consola. El launcher copia en vivo stdout/stderr de Webots a la terminal de `run_experiment.py` y a `webots.log`, conservando el timeout. Una prueba utiliza un proceso hijo que solo termina cuando su primera línea ya se mostró, verificando que la salida es realmente en vivo; otra comprobación dentro de esa prueba confirma conservación de la salida tras timeout. Jev se comprobó con una respuesta sintética, sin llamada a su API.

Se eliminó `SLOW_FORWARD` de criterios, comparador y controlador. Avances y giros pasan de 0,6 a 1,8 rad/s. El PROTO oficial limitaba las seis ruedas a 0,6 rad/s: ordenar más velocidad sin modificarlo no aceleraba el rover. `protos/Sojourner.proto` conserva la geometría/física R2025a y eleva únicamente esos seis límites; referencias a apariencia/texturas usan las URLs oficiales para seguir resolviéndose desde su nueva ubicación. El mundo principal y los fixtures utilizan este PROTO y los snapshots lo incluyen. Se ajustaron la estimación de regreso y el texto de movimiento comunicado al brain. La prueba de motores acorta sus tramos en proporción a la velocidad, manteniendo dos segundos físicos de frenado. No cambian las opciones por obstáculos ni se añade recuperación automática.

| Prueba | Directorio | Evidencia |
| --- | --- | --- |
| Sensores y cabecera de la lista | `20261008_214600_rules_eef7f5` | Sensores y auditor pasaron. Captura con cabecera a la izquierda, debajo de la cámara. |
| Avance, dirección, ambos giros y parada con PROTO final | `20261008_215034_rules_cde19a` | Auditor pasó. Mediana de avance 0,108410 m/s, frente a 0,035995 m/s del ensayo anterior `20261008_005657_rules_b2da76`: aproximadamente triple velocidad. Protección frontal interviene durante el fixture; cero contactos muestreados y cero errores/avisos de Webots sobre PROTO o límites de motores. |
| Comparador, lista completa y consola en vivo | `20261008_214901_rules_40b7f7` | 85 decisiones deterministas; inspección a 12,384 s, recogida a 12,464 s y entrega a 71,216 s, con presupuesto de exploración de 45 s. Auditor pasó, 85 frames, cero contactos/intervenciones/fallbacks. Captura revisada: filas #80–#85 en orden, sin duplicación, con resultados finales y colores. El PROTO local figura en el snapshot. No es un benchmark de duración de misión contra el ensayo anterior, que usó otro presupuesto. |

`python3 -m unittest discover -s tests -v`: **17 pruebas pasaron**. Las cuatro nuevas comprueban desplazamiento y actualización de la lista, bloqueos y color de rechazo, consola Jev sin payload, streaming antes de finalizar y logs tras timeout, órdenes de motores triplicadas, seis límites del PROTO, mundo enlazado y eliminación de SLOW_FORWARD. Compilación y `git diff --check` pasaron.

Se conservaron los ensayos intermedios: `20261008_214640_rules_800338` detectó el recorte a 0,6 rad/s; `20261008_214817_rules_c70476` comprobó el aumento real, pero mostró la referencia a SolarCell pendiente de adaptar. Ambos problemas quedaron corregidos y se repitió la prueba física final. El primer intento de sensores dentro del sandbox (`20261008_214453_rules_412e06`) no pudo ejecutar Webots; las verificaciones físicas posteriores se realizaron fuera del sandbox. El intento Laya `20261008_214936_laya_9f0a60` terminó durante el calentamiento por conexión rechazada al servidor configurado, antes de iniciar física; no aporta validación del modelo. No se inició ningún servidor ni se cargaron Kev/Jev. `.env`, pesos y cambios previos se conservaron; no se hizo commit.

## Logs de brains consolidados · 8 de octubre de 2026

`webots.log` se abre antes de calentar el brain y conserva inicio, calentamiento/respuesta/error, eventos del controlador (excepto muestreo continuo `step`), salida de Webots y resumen final. Los eventos estructurados incluyen brain y UTC. El adapter conserva la respuesta System One completa validada en `raw_response`, también para respuestas tardías y calentamiento; las cabeceras y el texto/cuerpo de errores siguen excluidos. El estado/payload de la petición queda en el evento `request`, sin duplicarse dentro de la respuesta. Los resultados físicos y la protección no cambian.

Para servidores locales iniciados con `serve_brain.py`, el launcher sigue las líneas nuevas de `.local_models/laya-runtime.log` o `kev-runtime.log` y las incorpora con un prefijo de origen al mismo `webots.log`. La captura comienza al iniciar el experimento y se drena al terminar, sin copiar actividad anterior ni iniciar/detener servidores. Los servidores externos que no escriben esos archivos mantienen respuestas consolidadas, pero sus diagnósticos internos no son accesibles. Jev utiliza los mismos eventos sin requerir un archivo de runtime local.

**19 pruebas automatizadas pasaron.** La nueva prueba de integración ejecuta el launcher y un proceso hijo Python real para cada brain (Laya, Kev, Jev), usando transporte HTTP simulado: comprueba calentamiento, respuesta completa, aceptación, resultado, fin y resumen en el mismo archivo; diagnósticos locales presentes solo para Laya/Kev; exclusión de actividad anterior, payload duplicado y clave de autenticación. Otra prueba comprueba que un fallo antes de iniciar Webots deja `warmup_error` en el archivo sin copiar el texto secreto de la excepción. No se hicieron llamadas reales a Jev ni se cargaron modelos para estas pruebas.

La prueba física de sensores `20261008_220208_rules_d0548e` y su auditor pasaron. Su `webots.log` comienza con `launch`, registra inicio/detecciones/fin del controlador y termina con `summary`, todos identificados como `rules`. La compilación y `git diff --check` pasaron. README actualizado; cambios sin commit.

## Corrección del formato de salida · 8 de octubre de 2026

La salida consolidada anterior era demasiado ruidosa y se sustituyó por el formato pedido del servidor local: separadores, cabecera breve con hora/brain/HTTP/latencia, elección/confianza y JSON original indentado. `serve_brain.py`, el controlador y el calentamiento del launcher utilizan `brain_output`, el mismo formateador. `webots.log` y la terminal reciben únicamente estos bloques; las respuestas rechazadas/tardías llevan una nota breve en la cabecera. Prompts, acciones, ejecución y protección siguen íntegros en `steps.jsonl`; resumen en `summary.json`, fallos de calentamiento en `warmup.json` y diagnósticos de Webots en `webots-runtime.log`. Se eliminó la captura de logs internos del runtime local. No se alteraron movimientos, opciones, HUD ni control de seguridad.

**19 pruebas pasaron.** La integración de Laya/Kev/Jev con transporte simulado y un proceso hijo real verifica dos bloques de JSON legible (calentamiento y respuesta), sin eventos, prompts, resumen ni diagnósticos internos mezclados. Comprueba que los registros de ejecución permanecen completos en JSONL. Las pruebas de salida verifican también `usage`, HTTP/latencia, respuesta tardía descartada, streaming antes de finalizar, diagnóstico separado y conservación del bloque tras timeout. Se compartió un ejemplo en `results/brain_output_preview.log` a partir de respuestas reales ya registradas. Compilación y whitespace pasaron. Había una simulación y servidor Laya activos: no se reiniciaron ni se enviaron nuevas inferencias; la simulación deberá recargarse para aplicar la salida nueva. Sin llamadas a Jev ni commit.

## Reinicios manuales desde Webots · 8 de octubre de 2026

El controlador heredaba `MARS_RUN_DIR` al reiniciarse desde la UI e intentaba crear otra vez `steps.jsonl` con apertura exclusiva. Se añadió `start_run_log`: utiliza el directorio preparado por el launcher en el primer arranque y, si ya hay resultados o frames de una ejecución, crea un directorio hermano con fecha, brain e ID nuevo. La creación exclusiva de `steps.jsonl` se mantiene y también detecta una colisión entre arranques. El rover usa la ruta realmente seleccionada para frames/snapshot y la transmite al Supervisor para `ground_truth.jsonl`; no se sobrescribe ni mezcla evidencia anterior.

**20 pruebas automatizadas pasaron.** La regresión comprueba la reutilización inicial de un directorio con metadatos de lanzamiento, tres reinicios consecutivos con la misma ruta heredada, directorios independientes, apertura correcta del log del Supervisor y conservación byte a byte de logs/frames/metadatos anteriores. Comprueba además colisiones con solo `steps.jsonl`, solo `ground_truth.jsonl` y solo `frames`. Compilación y `git diff --check` pasaron. No se reinició la simulación del usuario ni se enviaron inferencias. Cambios sin commit.

## Prompt para recuperar la vista de una muestra · 8 de octubre de 2026

El contexto anterior decía «objetivo alcanzado, permanecer parado» únicamente por distancia, incluso ante una muestra sin inspeccionar y fuera de la vista de la cámara. Ahora comunica elegibilidad de INSPECT/COLLECT y, para una muestra cercana no inspeccionada e invisible, explica que puede estar debajo/detrás de la cámara y que necesita recolocarse para recuperar la vista. Las instrucciones aclaran que pasar por encima no recoge, que conviene inspeccionar a distancia visible y que girar sobre la muestra puede no resolverlo: sin acción de retroceso, el modelo dispone de giros y avances por espacio libre para recuperar separación y orientación. Los datos recordados pueden estar desactualizados. No se cambian criterios, motores ni validación física; no se ejecuta una maniobra automática.

**21 pruebas automatizadas pasaron.** La nueva comprobación cubre para Laya/Kev/Jev la muestra cercana invisible del caso observado, inspección disponible, recogida tras inspección incluso sin visibilidad, muestra visible desalineada y llegada a base. Verifica que se elimina la instrucción contradictoria de permanecer parado y que todas las opciones de movimiento siguen disponibles. Compilación y whitespace pasaron. No se hicieron inferencias reales ni se reinició Webots; estas pruebas verifican el prompt, no que Laya vaya a seguirlo de forma fiable. Cambios sin commit.
### Eliminación del límite de antigüedad de observaciones (2026-10-08)

Se eliminó el rechazo `stale_observation` y su configuración (`--max-age`, `MARS_MAX_AGE_S`). El timeout HTTP real y la validación de revisión, pose, clearance y precondiciones continúan activos. El test de `Pending` comprueba que una observación 500 segundos simulados posterior se acepta si el estado permanece válido; también conserva las comprobaciones de timeout y revisión.

`python3 -m unittest discover -s tests -v`: **21 pruebas pasaron**. Compilación Python y `git diff --check` pasaron. No se lanzó una nueva simulación ni se invocó ningún modelo para esta verificación.

La petición 20 y su respuesta real de `20261008_223705_laya_f180af` se extrajeron a `results/examples/laya_request_0020/{payload.json,response.json,exchange.md}`. Los JSON exportados se verificaron iguales a los objetos originales registrados; la petición coincide además con el payload guardado en la respuesta. El intercambio histórico conserva el contexto que se envió entonces, incluido el fallback por antigüedad de acciones previas.


### REVERSE para reposicionarse tras bloqueos (2026-10-08)

La última ejecución de Jev, `20261008_231616_jev_d00eb9`, confirmó INSPECT S1 a 16,992 s y COLLECT S1 a 17,520 s. Desde 24,928 s se registraron 38 giros bloqueados por `spin_clearance` (29 TURN_LEFT y 9 TURN_RIGHT), todos con motores STOP, desplazamiento y giro cero. La última observación disponible indicaba lidar frontal 0,237 m y trasero 1,622 m. El registro de ground truth no contiene contactos con rocas; el bloqueo observado corresponde a la protección local. La ejecución quedó truncada, sin evento final.

Se añadió `REVERSE` al contrato compartido de los brains, con ruedas a +1,8 rad/s y dirección recta. Las instrucciones indican retroceder tras un bloqueo si hay espacio detrás, observar de nuevo, girar cuando se recupere clearance y rodear el obstáculo por un lado libre. También permite recuperar separación de una muestra invisible. No se ejecuta recuperación automáticamente. La protección solo detiene por obstáculo trasero o límite detrás; la opción sigue disponible con sensores válidos.

`python3 -m unittest discover -s tests -v`: **22 pruebas pasaron**. La regresión usa las distancias del bloqueo real y verifica que el giro se bloquea mientras el retroceso está permitido, la presencia y decodificación de REVERSE en Laya/Kev/Jev, motores invertidos, dirección recta, historial de retroceso y parada por obstáculo trasero y por límite en ambas orientaciones. Compilación y `git diff --check` pasaron. El auditor físico admite REVERSE y sigue admitiendo logs históricos. No se invocaron modelos ni se verificó todavía REVERSE en una nueva simulación física.


### REVERSE condicionado al último movimiento bloqueado (2026-10-08)

La ejecución `20261008_233143_jev_de12ea` registró 12 FORWARD y 9 REVERSE completados sin movimientos bloqueados. Se restringió la opción REVERSE a sensores válidos y al último movimiento del historial reciente con resultado blocked o interrupted y motivo de seguridad. STOP e interacciones no se consideran movimientos; un movimiento posterior completado elimina la opción aunque conserve un bloqueo anterior. La proximidad de una roca, no_progress, una respuesta rechazada o una intervención histórica por sí solos no la habilitan. El prompt deja de sugerir retroceso libre para recuperar visibilidad.

**23 pruebas pasaron**, incluyendo habilitación tras bloqueo o interrupción parcial por seguridad y deshabilitación tras cada tipo de movimiento completado. Compilación Python y git diff --check pasaron. Se reevaluaron las 26 observaciones reales de esa ejecución con el nuevo contrato: ninguna ofrece REVERSE. Es una reproducción offline; no se invocaron modelos ni se lanzó una nueva simulación.
