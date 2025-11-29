# ConveyorBelt -- Sistema de Clasificación por Color con ESP32 + FreeRTOS

ConveyorBelt es un proyecto académico de IoT y sistemas embebidos
desarrollado en la Universidad Nacional de La Matanza (UNLaM). El
sistema implementa una cinta transportadora capaz de clasificar objetos
según su color, utilizando un ESP32, sensores ultrasónicos, un sensor de
color y un servo encargado del desvío.

El flujo de funcionamiento se organiza mediante una máquina de estados
finitos (FSM) y tareas concurrentes manejadas con FreeRTOS.

## Características principales

-   Detección automática de objetos mediante sensores ultrasónicos.
-   Clasificación según color (rojo, azul, desconocido).
-   Desvío mediante servo según color detectado.
-   Control de motor a través de PWM y driver L298N.
-   Botón de parada de emergencia (STOP).
-   Botón de reinicio (RESTART).
-   Indicadores LED según estado operativo.
-   Manejo de errores: timeout, color no detectado o fallas en
    detección.
-   Implementación completa basada en una máquina de estados finitos.

## Arquitectura del sistema

### ST_IDLE (Espera)

-   Motor apagado
-   LED amarillo encendido
-   Servo en posición inicial
-   Espera que el objeto sea colocado

### ST_MOVING (Objeto en movimiento)

-   Motor encendido
-   LED verde
-   Se evalúa si se detecta color o una parada manual

### ST_COLOR_DETECTED (Clasificación)

-   Rojo → Servo a 45°
-   Azul → Servo a 135°
-   Se espera la llegada del objeto al final de la cinta

### ST_ERROR (Error)

-   Motor apagado
-   LED rojo parpadeando
-   Se origina por timeout, color desconocido o detección anómala

### ST_MANUAL_STOP (Parada manual)

-   Motor apagado
-   LED rojo fijo
-   Sale del estado solo con el botón RESTART

## Componentes utilizados

### Sensores

-   2× HC-SR04 (ultrasonido)
-   Sensor de color TCS230 (simulado en prototipo)

### Actuadores

-   Motor DC controlado por L298N
-   Servo motor

### Señalización

-   LED amarillo → Espera
-   LED verde → Movimiento
-   LED rojo intermitente → Error
-   LED rojo fijo → Parada manual

## Tecnologías empleadas

-   ESP32 DevKit
-   FreeRTOS
-   C/C++ (Arduino)
-   Wokwi
-   PWM, timers, interrupciones

## Simulación en Wokwi

https://wokwi.com/projects/442535339004692481

## Comunicación MQTT y Aplicación Android

El sistema fue ampliado para incluir comunicación MQTT con una aplicación Android, permitiendo:
-   Monitoreo en tiempo real del estado de la máquina (estado actual, errores, colores detectados).
-   Modificación remota de la velocidad de la cinta transportadora desde el dispositivo móvil.
-   Para integrar estas funcionalidades se incorporó un módulo de comunicación basado en WiFi + MQTT, utilizando tópicos dedicados.

## Flujo de comunicación

-   El ESP32 se conecta a WiFi y al broker MQTT.
-   Subscripción a los tópicos de control definidos.
-   El ESP32 publica continuamente:
        -   Estado actual de la FSM (color detectado, detenciones, errores)
-   El ESP32 recibe comandos de control para variar la velocidad de la cinta y para detenerla.   
-   La app actualiza la UI en tiempo real.
