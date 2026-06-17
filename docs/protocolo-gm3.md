# Protocolo propietario reconstruido — Gimbal Caddx GM3

> Referencia técnica alineada con la **Sección 6.3 "Ingeniería Inversa"** del documento
> *"Proyecto de Grado Control Gimbal - Head Tracking"* (páginas 78–89).
> Resume el subconjunto funcional del protocolo necesario para controlar la orientación del gimbal.
> Autor del proyecto: **Nicolás Corimayo**.

## Estructura de la trama

Longitud fija de **10 bytes**:

```
[b0] [b1] [b2] [b3] [b4] [b5] [b6] [b7] [b8]   [b9]
 A5   5A   03   B3   B4   B5   B6   B7   CRC_H  CRC_L
```

| Campo | Valor / significado |
|-------|---------------------|
| `b0 b1` | Encabezado fijo `A5 5A` |
| `b2`    | Identificador de comando `03` |
| `b3..b7`| Campos de datos asociados a los ejes |
| `b8 b9` | CRC16-CCITT calculado sobre `b0..b7` (`b8` = CRC_H, `b9` = CRC_L) |

## Modos funcionales

El protocolo presenta **dos modos** dentro de la misma estructura:

### Modo Tilt / Pan (conjunto)
```
[A5][5A][03][00][TL][TH][PL][PH][CRC_H][CRC_L]
```
- `b3 = 00`
- **Tilt** → `b4-b5`, entero de 16 bits con signo, *little-endian* → `Tilt_raw = b4 + (b5 << 8)`
- **Pan**  → `b6-b7`, entero de 16 bits con signo, *little-endian* → `Pan_raw = b6 + (b7 << 8)`

### Modo Roll
```
[A5][5A][03][R1][R0][00][00][00][CRC_H][CRC_L]
```
- **Roll** → `b3-b4`, **no lineal**: codificación discreta mediante tabla de correspondencias (LUT).
- `b5 = b6 = b7 = 00`

## Verificación de integridad (CRC)

- Algoritmo: **CRC16-CCITT**
- Polinomio: `0x1021`
- Inicialización: `0x0000`
- Cálculo sobre: `b0..b7`
- Resultado: `b8 = CRC_H`, `b9 = CRC_L`

## Tabla de correspondencias de Roll (LUT)

| Roll (°) | b3 | b4 |
|----------|----|----|
| -30      | A0 | EA |
| -10      | E0 | F8 |
|   0      | 00 | 00 |
| +10      | 10 | 07 |
| +30      | 50 | 15 |

## Factores de escala

Codificación de los ejes lineales:

- **Tilt** → entero con signo de 16 bits (*little-endian*), factor `TILT_UNITS_PER_DEG = 321.0f`.
- **Pan**  → entero con signo de 16 bits (*little-endian*), factor `PAN_UNITS_PER_DEG = 182.0f`.

El firmware final de runtime
[`gm3_mavlink_receiver.ino`](../firmware/arduino/gm3_mavlink_receiver/gm3_mavlink_receiver.ino)
aplica estos factores y **satura** los valores crudos de Tilt y Pan al rango de `int16_t`
(`-32768` a `32767`) antes de empaquetarlos en los bytes de la trama. El eje Roll se codifica
mediante `ROLL_LUT`. Para una visión general del proyecto, ver el [README](../README.md).

## Relación con el firmware final

El firmware final de runtime es
[`gm3_mavlink_receiver.ino`](../firmware/arduino/gm3_mavlink_receiver/gm3_mavlink_receiver.ino).
Cada elemento del protocolo reconstruido se corresponde con el firmware de la siguiente manera:

| Elemento del protocolo | Implementación en el firmware final |
|------------------------|-------------------------------------|
| Encabezado de la trama (`A5 5A 03`) | `sendTiltPanFrame` / `sendRollFrame` |
| Trama Tilt/Pan | `sendTiltPanFrame` |
| Trama Roll | `sendRollFrame` |
| CRC16-CCITT (b0..b7 → b8/b9) | `crc16_ccitt` |
| Factor de escala de Tilt `321.0f` | `TILT_UNITS_PER_DEG` |
| Factor de escala de Pan `182.0f` | `PAN_UNITS_PER_DEG` |
| Tabla de correspondencias de Roll | `ROLL_LUT` |

## Relación con herramientas de ingeniería inversa

Las siguientes herramientas dieron soporte a la **reconstrucción y validación** del protocolo
propietario GM3 durante la etapa de ingeniería inversa (Sección 6.3):

| Herramienta | Rol en la reconstrucción/validación |
|-------------|-------------------------------------|
| [`gm3_structural_analysis.cpp`](../tools/reverse-engineering/gm3_structural_analysis.cpp) | Análisis estructural de las tramas (longitud, encabezado, bytes constantes). |
| [`analyze_tilt.cpp`](../tools/reverse-engineering/analyze_tilt.cpp) | Identificación del eje Tilt (b4-b5, int16 little-endian). |
| [`analyze_pan.cpp`](../tools/reverse-engineering/analyze_pan.cpp) | Identificación del eje Pan (b6-b7, int16 little-endian). |
| [`analyze_roll.cpp`](../tools/reverse-engineering/analyze_roll.cpp) | Identificación del eje Roll (b3-b4, codificación discreta / LUT). |
| [`verify_crc.cpp`](../tools/reverse-engineering/verify_crc.cpp) | Verificación del CRC16-CCITT sobre b0..b7. |
| [`gm3_common.hpp`](../tools/reverse-engineering/gm3_common.hpp) | Módulo auxiliar común (lectura de CSV, conversión int16 LE, CRC). |
| [`gm3_direct_control_validation.ino`](../firmware/arduino/reverse-engineering/gm3_direct_control_validation/gm3_direct_control_validation.ino) | Validación enviando tramas propias reconstruidas al gimbal. |

Estas herramientas pertenecen a la etapa de análisis y validación, mientras que
[`gm3_mavlink_receiver.ino`](../firmware/arduino/gm3_mavlink_receiver/gm3_mavlink_receiver.ino)
es el **firmware final de runtime**.

> **Nota:** el sketch de validación
> [`gm3_direct_control_validation.ino`](../firmware/arduino/reverse-engineering/gm3_direct_control_validation/gm3_direct_control_validation.ino)
> y el firmware final
> [`gm3_mavlink_receiver.ino`](../firmware/arduino/gm3_mavlink_receiver/gm3_mavlink_receiver.ino)
> usan el mismo factor de calibración de Tilt (`321.0f`).

## Aclaración de alcance

Esta reconstrucción **no** equivale a haber descifrado la totalidad del protocolo interno del
gimbal, sino únicamente el conjunto de mensajes necesarios para controlar los ejes de
orientación, suficiente para reemplazar a Pixhawk + Mission Planner durante la operación final.
