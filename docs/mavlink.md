# MAVLink communication

> Documentación técnica del contrato MAVLink entre el software de escritorio y el Arduino Mega.
> El título *"Control de Gimbal mediante Head Tracking"* es el título original del proyecto, tal
> como figura en la primera página del documento de grado. La **Sección 6 — "Desarrollo del
> proyecto"** se utiliza únicamente como referencia técnica de arquitectura, firmware,
> reconstrucción del protocolo y alineación de la implementación. La implementación específica
> del proyecto fue desarrollada por **Nicolás Corimayo**.

## Purpose

MAVLink es el protocolo de comunicación entre el software de head tracking ejecutado en la PC y
el Arduino Mega.

- MAVLink se utiliza para **transportar los datos de actitud/orientación ya procesados** (roll,
  pitch y yaw) desde la PC hacia el Arduino.
- El proyecto utiliza el mensaje **MAVLink `ATTITUDE`**.
- El proyecto **no** utiliza mensajes de comando de MAVLink tales como `COMMAND_LONG` o
  `COMMAND_INT`.
- MAVLink se emplea exclusivamente como **protocolo de transporte/mensajería** para los valores
  de roll, pitch y yaw. No se usa como capa de comandos.

## Direction of communication

```
PC / software de escritorio  →  Arduino Mega
```

- La PC **envía** los paquetes MAVLink.
- El Arduino **recibe y parsea** dichos paquetes.
- El Arduino **no** necesita enviar respuestas MAVLink para el flujo de trabajo del prototipo
  final. La comunicación MAVLink es unidireccional (PC → Arduino).

## Message used

| Elemento | Valor |
|----------|-------|
| Mensaje MAVLink | `ATTITUDE` |
| Dialecto / header | `common` |
| Include en la PC | `mavlink/common/mavlink.h` |
| Include en el Arduino | `mavlink/common/mavlink.h` |
| Empaquetador (PC) | `mavlink_msg_attitude_pack` |
| Parser (Arduino) | `mavlink_parse_char` |
| Decodificador (Arduino) | `mavlink_msg_attitude_decode` |

## Fields used

Campos del mensaje `ATTITUDE` utilizados:

- `roll`
- `pitch`
- `yaw`

Notas:

- `rollspeed`, `pitchspeed` y `yawspeed` son fijados en `0` por la aplicación de escritorio y
  **no** son utilizados por el firmware del Arduino.
- `time_boot_ms` es completado por la aplicación de escritorio.

## Units

- La aplicación de escritorio procesa los ángulos internamente en **grados**.
- Antes de empaquetar el mensaje MAVLink `ATTITUDE`, los valores de roll, pitch y yaw se
  convierten a **radianes**.
- El mensaje MAVLink `ATTITUDE` transmite roll, pitch y yaw en **radianes**.
- El Arduino convierte los valores recibidos nuevamente de **radianes a grados** antes de
  adaptarlos al protocolo del gimbal.

## Axis mapping

El mapeo entre ángulos y ejes del gimbal se realiza en el **firmware del Arduino**:

| Ángulo recibido | Eje del gimbal |
|-----------------|----------------|
| `pitch` | **Tilt** |
| `yaw`   | **Pan** |
| `roll`  | **Roll** |

## Serial configuration

| Parámetro | Valor |
|-----------|-------|
| Velocidad (baud) | `115200` |
| Formato | 8N1 |
| Dirección | USB serial de la PC → `Serial` del Arduino |
| Puerto COM (PC) | configurable desde la interfaz de escritorio ("COM Port") |
| Puerto COM por defecto (PC) | `COM5` |
| Puerto del Arduino para MAVLink | `Serial` (USB) |

Ambos extremos deben usar la misma velocidad (115200). El puerto `Serial2` del Arduino se reserva
para la salida hacia el gimbal (ver [protocolo-gm3.md](protocolo-gm3.md) y
[workflow.md](workflow.md)).

## MAVLink IDs

- `system_id = 1`
- `component_id = 200`
- Ambos están **hardcodeados** en el software de escritorio.
- El Arduino **no** filtra por `system_id` / `component_id` en el prototipo actual: acepta el
  mensaje `ATTITUDE` recibido por el canal serial.

## Runtime cadence

- La aplicación de escritorio envía **un** mensaje `ATTITUDE` por cada ciclo activo de
  actualización/render mientras la conexión serial está establecida.
- El Arduino genera las tramas de salida GM3 **cada vez que decodifica** un mensaje `ATTITUDE`
  (una trama Tilt/Pan y una trama Roll por mensaje).

## Source mapping

| Función | Archivo |
|---------|---------|
| Empaquetado MAVLink en la PC (`mavlink_msg_attitude_pack`, conversión grados→radianes, envío serial) | [`oculusmonitor/oculusmonitor.cpp`](../oculusmonitor/oculusmonitor.cpp) |
| Parseo y decodificación en el Arduino (`mavlink_parse_char`, `mavlink_msg_attitude_decode`, conversión radianes→grados, mapeo a ejes) | [`firmware/arduino/gm3_mavlink_receiver/gm3_mavlink_receiver.ino`](../firmware/arduino/gm3_mavlink_receiver/gm3_mavlink_receiver.ino) |

Ver también: [workflow.md](workflow.md) (flujo de trabajo en tiempo de ejecución) y
[architecture.md](architecture.md) (arquitectura general del sistema).
