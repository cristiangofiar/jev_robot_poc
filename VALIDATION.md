# Validación ejecutada · 7 de octubre de 2026

Entorno real: Webots R2025a, macOS Apple Silicon, Python 3.14, Mac de 8 GB. Se inspeccionó el repositorio con Codegraph (`status`, `query`, `callers`, `impact`) antes de sustituir el código; se sincronizó el índice después. No se modificó la instalación de Webots. Se conservaron `.env`, `.git` y los pesos/runtimes locales.

## Resultados físicos

| Comprobación | Registro real en `results/` | Resultado |
| --- | --- | --- |
| Mundo estable, cámara, GPS, IMU, brújula y lidar | `20261007_234105_rules_c52453` | 2,016 s simulados; posición z≈0,236 m, roll −0,0041 rad, pitch −0,0009 rad; RGB 320×240, 307.200 bytes BGRA; 72 retornos lidar finitos; S1/S2 detectadas, S3 oculta. Se guardó y revisó la imagen de cámara. |
| Avance, dirección izquierda, giro y parada | `20261007_233832_rules_ae06f4` | 40,016 s; 0,618 m recorridos; yaw final cambió 1,864 rad; velocidad final ≈0,0000024 m/s. El auditor verificó +X de avance, cambio de yaw al dirigir, giro y parada. Una parada frontal de seguridad, ningún contacto con roca observado. |
| Misión determinista completa | `20261007_233207_rules_c7e065` | **3/3 entregadas**, 273,344 s simulados, 7,429 m, siete decisiones, siete operaciones físicas (tres inspecciones, tres recogidas, una entrega), sin fallbacks ni intervenciones. |
| Misión completa con Laya local | `20261007_233310_laya_0e16cb` | **3/3 entregadas**, 279,072 s simulados, 7,573 m, siete decisiones de Laya aceptadas, sin fallbacks ni intervenciones. Mediana de latencia 266,1 ms; rango 178,0–303,8 ms. |
| Inferencia lenta y deadline | `20261007_233847_slow_0e7f4a` | Delay artificial 1 s, deadline 0,1 s; tres peticiones rechazadas, cero decisiones aceptadas. El fallback local recorrió 0,769 m y recogió S1 en 35,008 s. Se registró movimiento mientras el thread ya rechazado seguía ocupado; las tres respuestas tardías se descartaron. |
| Observación obsoleta | `20261007_233937_slow_5de159` | Delay 0,3 s, edad máxima 0,05 s; rechazo `stale_observation`, cero decisiones aceptadas; física continuó y fallback recorrió 0,374 m en 15,008 s. |

Las dos pruebas de latencia terminan deliberadamente con `time_limit` y código de salida 1 del launcher: son ventanas cortas para comprobar integración, no misiones completas. Los dos logs finalizan con parada de motores sincronizada. `tests/verify_physical.py` pasó para todas las ejecuciones anteriores, incluidos `--rejection deadline_exceeded` y `--rejection stale_observation`.

Laya se ejecutó mediante el servidor local real, no mediante mocks. El adapter recibió estados originados en sensores físicos y devolvió elecciones de misión; la integración completa incluyó HTTP asíncrono, aceptación, conducción, cámara y frames, reconocimiento, validación física de operaciones y entrega. El calentamiento quedó fuera de las siete decisiones de misión. No se cargó Kev ni se hicieron llamadas a Jev.

En ambas misiones completas, el Supervisor registró cero muestras de contactos del rover por encima de z=0,09 m. Esto significa ausencia de contactos con obstáculos **observados por esa heurística en esta escena**, no una garantía universal de navegación segura. Las recogidas aceptadas respetaron distancia ≤0,68 m y parada ≥1 s; la entrega ocurrió a ≤0,30 m de la base. Se auditaron todos los PNG de petición: tamaño 320×240, timestamp del tick de cámara igual al de la observación, e IDs candidatos presentes solo después de una detección. S3 no aparece en la petición inicial.

Los resultados físicos pertenecen al sistema completo. El modelo selecciona objetivos; navegación, giros, paradas, estimación de batería y comprobación de recogida/entrega son locales. En la prueba de delay toda la misión aplicada procede del fallback, no de decisiones del modelo lento.

## Pruebas automáticas

`python3 -m unittest discover -s tests -v`: **7 pruebas pasaron**. Cubren candidatos/acciones disponibles, salida System One inválida, probabilidades/confianza, endpoints locales, contrato compartido Laya/Kev/Jev sin llamadas, deadline, revisión semántica cambiada, edad máxima, errores sin texto secreto, ruta alrededor de obstáculos, último punto de ruta y selección del paquete Windows/EXE/rutas relativas. El caso Windows prepara archivos de prueba; no es ejecución real en Windows.

`tests/verify_physical.py` audita registros reales: ambos cierres, stop sincronizado, sensores finitos, estabilidad, seguridad aplicada, PNG/timestamps, ausencia de IDs ocultos, requisitos físicos de operaciones y progreso durante inferencia/rechazo. Puedes repetirlo sobre las carpetas de la tabla. Los registros completos, frames, snapshots y capturas permanecen en el workspace e ignorados por Git.

## Límites comprobados y pendientes

- Windows no está disponible en este entorno. El launcher y la preparación de modelos utilizan rutas/stdlib portables, y el paquete CPU Windows de llama.cpp b11391 fue comprobado en el catálogo oficial; falta ejecutar físicamente la demo en el PC de 32 GB.
- Kev local y Jev API comparten el contrato y tienen adaptadores preparados. Su comportamiento, latencias y resultados físicos no se validaron, siguiendo la restricción de usar solo Laya en pruebas reales.
- La percepción de muestras es el reconocimiento idealizado de Webots con oclusión, no un modelo visual. Los frames están listos para pruebas multimodales posteriores; aún no existe ese adapter.
- Terreno estático con pendientes <2°, una capa lidar horizontal y proyección 2D aproximada. Se filtraron retornos del propio rover y la plataforma pública de aterrizaje. El mapa no elimina obstáculos dinámicos ni admite un entorno arbitrario.
- Recogida/inspección simbólicas; batería sintética. El éxito de una ejecución no establece una tasa de éxito estadística, calibración de confianza ni equivalencia entre modelos.
- La consola gráfica de Webots emitió un aviso de textura OpenGL durante las cargas en macOS. No impidió las imágenes, lecturas, física ni misiones; no se presenta como un fallo de controlador.

Las pruebas iniciales descubrieron y permitieron corregir el sentido de avance, retornos del propio rover en el mapa, llegada al último waypoint y bloqueo de la plataforma al regresar. Las carpetas intermedias incompletas se conservan para diagnóstico; solo las ejecuciones indicadas arriba se citan como comprobaciones válidas. Después de las misiones se ajustaron el HUD y la distinción entre espera de deadline y latencia tardía; la navegación no cambió. La prueba final `20261007_234358_slow_c55ca2` (5,008 s) comprobó esa presentación y registro: espera hasta rechazo de 101,9 ms frente a duración tardía real de 1005,7 ms; respuesta descartada, física y fallback continuaron. Su auditor físico también pasó.
