# Control de Gimbal mediante Head Tracking

**Autor:** Nicolás Corimayo

*"Control de Gimbal mediante Head Tracking"* es el título original del proyecto, tal como figura
en la primera página del documento de grado **"Proyecto de Grado Control Gimbal -Head Tracking NC"**.

La fundamentación técnica de la implementación (arquitectura, firmware, reconstrucción del
protocolo y desarrollo del proyecto) está alineada con la **Sección 6 — "Desarrollo del proyecto"**
de dicho documento.

El código específico del proyecto (software de head tracking de escritorio, integración de
MAVLink, comunicación serial, firmware de Arduino Mega, manejo del protocolo GM3, herramientas
de ingeniería inversa y documentación alineada con el contenido técnico de la Sección 6) fue
desarrollado por **Nicolás Corimayo**.

## Objetivo del proyecto

Convertir los movimientos naturales de la cabeza del usuario, capturados por un visor de
realidad virtual, en comandos de orientación para un gimbal **CADDX GM3** en sus ejes de
*pitch*, *yaw* y *roll*. El sistema demuestra, a nivel de prototipo funcional, la viabilidad
técnica de controlar la orientación de una cámara estabilizada mediante *head tracking*.

## Resumen de la arquitectura

```
Oculus Quest 2 (sensores inerciales)
        │  orientación (cuaterniones)
        ▼
Software de head tracking (PC / netbook, C++)
   · cuaternión → ángulos de Euler (yaw, pitch, roll)
   · corrección de continuidad angular
   · filtro de Kalman escalar por eje
   · empaquetado en mensaje MAVLink ATTITUDE
        │  MAVLink sobre enlace serial
        ▼
Arduino Mega 2560
   · decodifica MAVLink ATTITUDE
   · convierte radianes → grados
   · adapta al protocolo propietario GM3 (Tilt/Pan int16 LE, Roll por LUT)
   · calcula CRC16-CCITT y construye la trama
        │  protocolo propietario GM3 sobre Serial2
        ▼
Gimbal CADDX GM3 (actuador final)
```

Pixhawk 6C y Mission Planner se utilizaron **únicamente** durante la etapa de ingeniería
inversa y **no** forman parte de la arquitectura final del prototipo. Detalle completo en
[docs/architecture.md](docs/architecture.md).

## Relación con la Sección 6

| Sección 6 | Componente | Ubicación en el repositorio |
|-----------|-----------|-----------------------------|
| 6.1 Investigación | Selección de hardware/software | (narrativa del documento) |
| 6.2 Análisis y recursos | C++, MAVLink, Arduino IDE, Visual Studio | `oculusmonitor.sln`, `dev/sdk/` |
| 6.3 Ingeniería inversa | Bridge CSV, análisis de tramas, CRC, validación | `firmware/arduino/reverse-engineering/`, `tools/reverse-engineering/` |
| 6.4 Arquitectura | Cadena Visor → PC → Mega → GM3 | [docs/architecture.md](docs/architecture.md) |
| 6.5 Software de Head Tracking | App de escritorio C++ | `oculusmonitor/oculusmonitor.cpp` |
| 6.6 Programa Arduino Mega | Receptor MAVLink + traductor GM3 | `firmware/arduino/gm3_mavlink_receiver/` |
| 6.7 Integración | Prototipo completo end-to-end | (este README, sección "Cómo ejecutar") |

## Estructura del repositorio

```
Software de head tracking/
├─ README.md                     Este documento
├─ DEPENDENCIES.md               Librerías, SDKs y componentes externos
├─ .gitignore
├─ oculusmonitor.sln             Solución de Visual Studio 2022
├─ oculusmonitor/                Software de head tracking de escritorio (C++)
│   ├─ oculusmonitor.cpp         Lógica principal (Euler, Kalman, MAVLink, serial, UI)
│   ├─ vrstate.cpp / vrstate.h   Gestión del estado de tracking del visor
│   ├─ imgui*.{cpp,h}, stb_*.h   Componentes externos (ver DEPENDENCIES.md)
│   └─ oculusmonitor.vcxproj     Proyecto de Visual Studio
├─ firmware/arduino/
│   ├─ gm3_mavlink_receiver/     Firmware final del Arduino Mega (Sección 6.6)
│   └─ reverse-engineering/      Sketches de ingeniería inversa (Sección 6.3)
├─ tools/reverse-engineering/    Herramientas de análisis en C++ (Sección 6.3)
├─ docs/                         Documentación técnica
│   ├─ architecture.md           Arquitectura del sistema (Sección 6.4)
│   ├─ workflow.md               Flujo de trabajo en tiempo de ejecución
│   ├─ mavlink.md                Contrato de comunicación MAVLink
│   ├─ protocolo-gm3.md          Protocolo propietario reconstruido (Sección 6.3)
│   └─ codigo-obsoleto.md        Variantes de código no extraídas
└─ dev/sdk/                      Headers y librerías externas (MAVLink, Oculus, etc.)
```

## Compilación del software de escritorio

Requisitos:

- **Visual Studio 2022** (toolset MSVC, plataforma x64, Windows).
- SDK de Oculus / LibOVR y headers de MAVLink, incluidos en `dev/sdk/` (ver
  [DEPENDENCIES.md](DEPENDENCIES.md)).

Pasos:

1. Abrir `oculusmonitor.sln` en Visual Studio 2022.
2. Seleccionar la configuración **Debug | x64**.
3. Compilar la solución. Los directorios de inclusión ya apuntan a `dev/sdk/include` y
   `dev/sdk/include/oculus`; el proyecto enlaza `dev/sdk/lib/LibOVR.lib`.
4. El ejecutable se genera en `bin/` (directorio ignorado por control de versiones).

## Carga del firmware de Arduino

Requisitos:

- **Arduino IDE**, placa **Arduino Mega 2560**.
- Librería **MAVLink** disponible para el IDE (headers `mavlink/common/...`).

Firmware final (Sección 6.6):

1. Abrir `firmware/arduino/gm3_mavlink_receiver/gm3_mavlink_receiver.ino`.
2. Seleccionar *Board: Arduino Mega 2560* y el puerto correspondiente.
3. Compilar y cargar.

Los sketches de `firmware/arduino/reverse-engineering/` corresponden a la etapa de
ingeniería inversa (Sección 6.3) y se conservan como referencia.

## Configuración de la comunicación serial

| Enlace | Puerto | Velocidad | Formato |
|--------|--------|-----------|---------|
| PC → Arduino Mega (MAVLink ATTITUDE) | `Serial` (USB) | **115200** | 8N1 |
| Arduino Mega → Gimbal GM3 (protocolo GM3) | `Serial2` | **115200** | 8N1 |

En el software de escritorio, el puerto COM y la velocidad son configurables desde la interfaz
("COM Port" / "Baud Rate"); los valores por defecto son `COM5` y `115200`. Ambos extremos del
enlace deben usar la misma velocidad (115200).

- MAVLink se utiliza **únicamente** para transportar mensajes `ATTITUDE` de la PC al Arduino.
- El proyecto **no** utiliza mensajes de comando de MAVLink como `COMMAND_LONG` o `COMMAND_INT`.

El contrato completo de MAVLink se documenta en [docs/mavlink.md](docs/mavlink.md), y el flujo
de trabajo en tiempo de ejecución en [docs/workflow.md](docs/workflow.md).

## Integración de hardware

Componentes del prototipo final:

- **Visor Oculus Quest 2** — adquisición de la orientación de la cabeza.
- **PC / netbook** — ejecuta el software de head tracking.
- **Arduino Mega 2560** — traductor entre MAVLink y el protocolo propietario del gimbal.
- **Gimbal CADDX GM3** — actuador final de orientación.

Conexiones: el visor entrega la orientación al software en la PC; la PC envía mensajes MAVLink
al Arduino Mega por USB (`Serial`); el Arduino Mega envía las tramas propietarias al gimbal por
`Serial2`.

Notas de hardware/runtime:

- El **Oculus Quest 2** debe estar disponible a través del runtime de Oculus para PC / LibOVR
  (Link / Air Link o configuración equivalente) para que el software acceda a la pose del visor.
- El `Serial2` del **Arduino Mega** usa **TX2 = pin 16** y **RX2 = pin 17** hacia el lado del gimbal.
- La **masa (GND)** debe ser compartida entre el Arduino y el gimbal.

El cableado serial y la secuencia de ejecución se detallan en [docs/workflow.md](docs/workflow.md).

## Cómo ejecutar el sistema completo

1. Cargar el firmware `gm3_mavlink_receiver.ino` en el Arduino Mega.
2. Conectar el Arduino Mega: `Serial` (USB) a la PC y `Serial2` al gimbal GM3; alimentar el gimbal.
3. Encender el visor Oculus Quest 2 y habilitar el enlace con la PC.
4. Compilar y ejecutar el software de escritorio.
5. En la interfaz, indicar el puerto COM del Arduino y la velocidad (115200) y presionar
   **"Connect Arduino"**.
6. Mover la cabeza: la orientación procesada se envía como MAVLink ATTITUDE al Arduino, que
   genera las tramas GM3 y orienta el gimbal. La ventana de visualización muestra el cuaternión,
   los ángulos *yaw/pitch/roll* (crudos y filtrados) y el paquete MAVLink generado.

## Limitaciones y problemas conocidos

- **Factor de escala de Tilt:** el valor de control definitivo es **`2900.0f`** (Sección 6,
  p.84), usado por el firmware final. El sketch de validación
  `gm3_direct_control_validation.ino` conserva el valor experimental histórico `321.0f` como
  artefacto de ingeniería inversa (ver [docs/protocolo-gm3.md](docs/protocolo-gm3.md)).
- **Roll por LUT:** el eje Roll se codifica mediante una tabla de correspondencias discreta
  (5 puntos identificados experimentalmente); valores intermedios se aproximan al vecino más
  cercano.
- **Comunicación cableada:** la transmisión PC ↔ Arduino ↔ Gimbal es serial cableada; una
  mejora futura es reemplazarla por telemetría inalámbrica.
- **Prototipo experimental:** el sistema demuestra viabilidad técnica y no constituye una
  solución final lista para aplicación industrial.

## Dependencies and external components

This project uses external libraries and SDKs required for headset tracking, interface rendering, MAVLink communication, and hardware integration.

For details, see [DEPENDENCIES.md](DEPENDENCIES.md).
