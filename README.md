ConveyorBelt – Color Sorting Conveyor System (ESP32 + FreeRTOS)

Proyecto académico desarrollado en la Universidad Nacional de La Matanza (UNLaM) para la materia Sistemas Operativos Avanzados / IoT & Sistemas Embebidos.
El sistema consiste en una cinta transportadora inteligente, capaz de clasificar objetos según su color utilizando un ESP32, sensores ultrasónicos y un servo para desvío.
La lógica está implementada como máquina de estados finitos, con tareas concurrentes manejadas con FreeRTOS.

Funcionalidades principales

Detección de objetos mediante sensores ultrasónicos (inicio y fin de cinta).

Clasificación automática por color (rojo / azul / desconocido).

Desvío de objetos usando un servo en función de su color.

Control del motor mediante PWM y driver L298N.

Botón de parada de emergencia (STOP).

Botón de reinicio (RESTART).

Indicadores LED según estado del sistema.

Manejo de errores y timeouts (detección color o fin de cinta).

Máquina de estados robusta: ST_IDLE, ST_MOVING, ST_COLOR_DETECTED, ST_ERROR, ST_MANUAL_STOP.

🧠 Arquitectura del sistema

El programa está basado en una máquina de estados que responde a eventos generados por sensores, botones y lógica temporal.

🟡 ST_IDLE (Espera)

Motor apagado

LED amarillo encendido

Servo en posición inicial

Espera objeto en sensor de entrada

🟢 ST_MOVING (Objeto en movimiento)

Motor encendido

LED verde

Se evalúa color o emergencia

🔵🔴 ST_COLOR_DETECTED (Clasificación)

Servo → 45° para rojo

Servo → 135° para azul

Espera que el objeto llegue al final

🔴 ST_ERROR (Error)

Motor apagado

LED rojo parpadeando

Causas: timeout, color desconocido o ausencia de objetos

🛑 ST_MANUAL_STOP (Parada de emergencia)

Motor apagado

LED rojo fijo

Se reanuda solo con botón RESTART

🔌 Hardware utilizado
Sensores

2× HC-SR04 (ultrasonido)

Sensor de entrada: detecta objeto inicial

Sensor de salida: confirma llegada al final

Sensor de color TCS230 (en prototipo simulado por botones)

Detecta rojo / azul / desconocido

Actuadores

Motor DC con driver L298N

Control de dirección y velocidad

Servo motor

Define el desvío según el color

Señalización

LED amarillo → Espera

LED verde → Movimiento

LED rojo fijo → Parada manual

LED rojo intermitente → Error

Controles

Botón STOP (parada inmediata)

Botón RESTART (reinicio del sistema)

🛠️ Tecnologías utilizadas

ESP32 DevKit

FreeRTOS (tareas concurrentes)

C/C++ (Arduino Framework)

Wokwi (simulación)

PWM, interrupciones, timers

Electrónica básica (servo, motor, resistencias, protoboard)
