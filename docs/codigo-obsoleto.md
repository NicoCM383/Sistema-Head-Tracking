# Código de referencia / obsoleto

> Parte de la documentación del proyecto. Para una visión general, ver el
> [README](../README.md); para el protocolo reconstruido, ver
> [protocolo-gm3.md](protocolo-gm3.md).
> Autor del proyecto: **Nicolás Corimayo**.

Este documento registra fragmentos de código presentes en los documentos Word que **no**
fueron extraídos como archivos fuente del proyecto, por tratarse de variantes anteriores.
Las variantes descritas aquí fueron superadas por los archivos fuente modulares y vigentes
del repositorio; su exclusión es una decisión de organización del proyecto y no afecta la
autoría del código específico del proyecto.

## `analisis de tramas c++.docx` — variante monolítica de las herramientas de análisis

Contiene una versión **anterior y autónoma** de los programas de análisis estructural y de ejes
(`gm3_structural_analysis`, `analyze_tilt`, `analyze_pan`, `analyze_roll`), en la que cada `.cpp`
incluía su propia copia de las funciones de parseo de tramas (no usaba un módulo común).

**Estado: obsoleto / solo referencia.** La versión oficial adoptada en el repositorio es la
**modular**, basada en el módulo auxiliar
[`gm3_common.hpp`](../tools/reverse-engineering/gm3_common.hpp), que es la explícitamente
referenciada por la **Sección 6.3** del documento de grado ("Para la lectura de los archivos y
operaciones comunes se utilizó además el módulo auxiliar `gm3_common.hpp`").

No contiene lógica única ausente en la versión modular, por lo que **no se extrajo**. El archivo
`.docx` original se conserva sin cambios.

## `CODIGO DE IDLE ARDUINO ... PUENTE TRANSPARENTE ... .docx` — puente simple

Contiene un puente serial Pixhawk ↔ Gimbal **sin** captura/volcado CSV. Es un precursor del
sketch oficial de ingeniería inversa
[`gm3_bridge_csv_logger.ino`](../firmware/arduino/reverse-engineering/gm3_bridge_csv_logger/gm3_bridge_csv_logger.ino),
que es el nombrado por la Sección 6.3 (puente transparente **y** logger CSV). El sketch simple
**no se extrajo** como artefacto del proyecto; el `.docx` original se conserva sin cambios.
