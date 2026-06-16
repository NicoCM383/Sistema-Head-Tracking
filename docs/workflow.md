# Runtime workflow

> Flujo de trabajo completo del prototipo en tiempo de ejecución.
> El título *"Control de Gimbal mediante Head Tracking"* es el título original del proyecto, tal
> como figura en la primera página del documento de grado. La **Sección 6 — "Desarrollo del
> proyecto"** se utiliza únicamente como referencia técnica de arquitectura, firmware,
> reconstrucción del protocolo y alineación de la implementación. La implementación específica
> del proyecto fue desarrollada por **Nicolás Corimayo**.

## Cadena completa

```
Oculus Quest 2
   → Software de head tracking (PC)
   → Procesamiento de la orientación
   → MAVLink ATTITUDE sobre serial
   → Arduino Mega (puente / traductor)
   → Tramas del protocolo propietario GM3
   → Gimbal CADDX GM3
```

### Oculus Quest 2

- Provee los datos de orientación de la cabeza a través del runtime de Oculus para PC / LibOVR.
- La orientación se obtiene en forma de **cuaterniones**.

### Software de head tracking (PC)

- Recibe la pose/orientación del visor.
- Extrae el cuaternión de orientación de la cabeza.
- Convierte el cuaternión a **ángulos de Euler** (yaw, pitch, roll).
- Maneja yaw / pitch / roll.
- Realiza la **corrección de continuidad angular**.
- Aplica un **filtro de Kalman escalar por eje**.
- Visualiza los datos crudos y filtrados.
- Crea paquetes **MAVLink `ATTITUDE`**.
- Los envía al Arduino Mega por enlace serial.

### Arduino Mega

- Actúa como **puente y traductor**.
- Recibe los mensajes MAVLink `ATTITUDE` por `Serial`.
- Decodifica roll, pitch y yaw.
- Convierte los valores de **radianes a grados**.
- Mapea **pitch → Tilt**, **yaw → Pan**, **roll → Roll**.
- Construye las tramas propietarias **GM3**.
- Calcula el **CRC16-CCITT**.
- Envía las tramas finales al gimbal por `Serial2`.

### Gimbal CADDX GM3

- Recibe las tramas del protocolo propietario GM3.
- Se mueve según los comandos de orientación traducidos.

El contrato MAVLink detallado se documenta en [mavlink.md](mavlink.md); el protocolo propietario
del gimbal en [protocolo-gm3.md](protocolo-gm3.md); la arquitectura general en
[architecture.md](architecture.md).

## Runtime prerequisites

- El **Oculus Quest 2** debe estar conectado a la PC a través del runtime de Oculus para PC /
  Link / Air Link (o configuración equivalente) que permita a LibOVR acceder a la pose del visor.
- El **Arduino Mega 2560** debe estar grabado con
  [`gm3_mavlink_receiver.ino`](../firmware/arduino/gm3_mavlink_receiver/gm3_mavlink_receiver.ino).
- El **software de escritorio** debe estar compilado y en ejecución.
- El **puerto COM y la velocidad (baud)** deben coincidir entre la PC y el Arduino.
- El **gimbal** debe estar correctamente alimentado.
- El **`Serial2`** del Arduino debe estar conectado a la entrada serial del GM3.

## Serial wiring

Puertos seriales del Arduino Mega:

- `Serial` = USB hacia la PC / entrada MAVLink.
- `Serial2` = UART de salida hacia el gimbal GM3.

Pines de `Serial2` en el Arduino Mega:

- **TX2 = pin 16**
- **RX2 = pin 17**

- La **masa (GND)** debe ser compartida entre el Arduino y el gimbal.
- El conector exacto y el cableado de alimentación dependen del montaje físico del GM3 y
  **deben verificarse antes de energizar el sistema**.

## Runtime sequence

1. Grabar el firmware del Arduino (`gm3_mavlink_receiver.ino`).
2. Conectar el Arduino a la PC por USB.
3. Conectar el `Serial2` del Arduino a la interfaz serial del gimbal.
4. Alimentar el gimbal.
5. Conectar el Oculus Quest 2 al runtime de la PC.
6. Iniciar el software de escritorio.
7. Seleccionar el puerto COM y la velocidad `115200`.
8. Conectar el Arduino desde la interfaz ("Connect Arduino").
9. Mover el visor.
10. Verificar la visualización y la respuesta del gimbal.

## Prototype validation status

- El **pipeline a nivel de código está implementado** (adquisición, procesamiento, MAVLink,
  recepción, traducción a GM3).
- El **movimiento físico debe validarse con el hardware conectado**.
- Si el gimbal no se mueve, verificar: puerto COM, velocidad (baud), cableado, alimentación,
  masa (GND) compartida y la conexión del protocolo GM3.
