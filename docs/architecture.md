# Arquitectura del sistema

> Referencia técnica alineada con la **Sección 6.4 "Diseño de la arquitectura del sistema"**
> (y secciones 6.5–6.7) del documento *"Proyecto de Grado Control Gimbal -Head Tracking NC"*.
> Autor del proyecto: **Nicolás Corimayo**.

## Cadena funcional

La arquitectura convierte el movimiento natural de la cabeza del usuario en comandos de
orientación para el gimbal, organizando el sistema en bloques especializados y desacoplados.

> El flujo de trabajo en tiempo de ejecución se documenta por separado en
> [workflow.md](workflow.md) y el contrato de comunicación MAVLink en [mavlink.md](mavlink.md).

```
┌─────────────────────────┐
│  Oculus Quest 2          │  Sensado inercial (acelerómetro + giroscopio)
│  (visor de RV)           │  Reporta la orientación de la cabeza
└───────────┬─────────────┘
            │  orientación (cuaterniones)
            ▼
┌─────────────────────────┐
│  Software de head        │  Ejecutado en PC / netbook (C++)
│  tracking                │  · cuaternión → ángulos de Euler (yaw, pitch, roll)
│                          │  · corrección de continuidad angular
│                          │  · filtro de Kalman escalar por eje
│                          │  · empaquetado en mensaje MAVLink ATTITUDE
└───────────┬─────────────┘
            │  MAVLink sobre enlace serial (USB, 115200 8N1)
            ▼
┌─────────────────────────┐
│  Arduino Mega 2560       │  Nodo de adaptación e interoperabilidad
│                          │  · decodifica MAVLink ATTITUDE
│                          │  · convierte radianes → grados
│                          │  · Tilt/Pan: entero de 16 bits little-endian
│                          │  · Roll: tabla de correspondencias (LUT)
│                          │  · calcula CRC16-CCITT y construye la trama GM3
└───────────┬─────────────┘
            │  protocolo propietario GM3 sobre Serial2 (115200 8N1)
            ▼
┌─────────────────────────┐
│  Gimbal CADDX GM3        │  Actuador final: ejecuta la orientación solicitada
└─────────────────────────┘
```

## Roles de cada bloque

| Bloque | Responsabilidad |
|--------|-----------------|
| Visor Oculus Quest 2 | Sensado del movimiento de la cabeza; entrega de la orientación. |
| Software de head tracking (PC) | Adquisición, conversión a Euler, continuidad, filtrado de Kalman y generación de mensajes MAVLink. |
| Arduino Mega 2560 | Recepción de MAVLink y traducción al protocolo propietario del gimbal (Tilt/Pan/Roll + CRC). |
| Gimbal CADDX GM3 | Actuación final sobre los ejes de orientación. |

Criterios de diseño: **separación funcional de responsabilidades**, **modularidad** (cada bloque
puede validarse por separado) e **interoperabilidad** entre tecnologías heterogéneas, con el
Arduino Mega como elemento de acoplamiento entre la capa MAVLink y el protocolo cerrado del
gimbal.

## Herramientas de ingeniería inversa (no forman parte del prototipo final)

Durante la etapa previa de ingeniería inversa (Sección 6.3) se utilizaron una controladora de
vuelo **Pixhawk 6C** y el software **Mission Planner** para generar comandos válidos hacia el
gimbal y descifrar su protocolo propietario, junto con el Arduino Mega como puente y PuTTY para
capturar la salida serial.

**Pixhawk 6C y Mission Planner se usaron únicamente como herramientas instrumentales durante la
ingeniería inversa y las pruebas. No forman parte de la arquitectura final del prototipo.**

La cadena de comunicación definitiva queda reducida a:

```
PC / software de head tracking  →  Arduino Mega  →  Gimbal CADDX GM3
```

Es decir, el prototipo final opera de forma autónoma con el visor, la PC, el Arduino Mega y el
gimbal, sin depender de Pixhawk ni de Mission Planner durante la operación.

## Referencias

- Flujo de trabajo en tiempo de ejecución: [workflow.md](workflow.md)
- Contrato de comunicación MAVLink: [mavlink.md](mavlink.md)
- Protocolo propietario reconstruido: [protocolo-gm3.md](protocolo-gm3.md)
- Variantes de código no extraídas: [codigo-obsoleto.md](codigo-obsoleto.md)
- Visión general y ejecución: [../README.md](../README.md)
