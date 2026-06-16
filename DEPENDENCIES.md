# Dependencies and external components

This project was developed by Nicolás Corimayo. "Control de Gimbal mediante Head Tracking" is the
project title as shown on the first page of the project document
"Proyecto de Grado Control Gimbal -Head Tracking NC". The project-specific implementation belongs
to the project author. This file documents the external libraries and SDKs used by the project.

The purpose of this document is purely technical: to identify which third-party libraries and
SDKs the project depends on, where they live in the repository, and under which license they are
distributed. It does not qualify the authorship of the project-specific code.

---

## 1. Project-specific code

The project-specific implementation was authored by **Nicolás Corimayo**. This includes all of
the project's code, structure, integration logic, firmware, protocol handling, tools and
documentation:

- **Desktop head-tracking logic** — orientation acquisition and processing.
- **Desktop application support code** — application scaffolding, VR tracking state management,
  geometry/math helpers and timing utilities used by the desktop application.
- **Quaternion → Euler conversion integration.**
- **Kalman filtering integration** — scalar Kalman filter applied per axis.
- **MAVLink message generation / integration** in the desktop application (building and sending
  the `ATTITUDE` message).
- **Serial communication** — port configuration, baud rate and packet transmission.
- **Arduino Mega firmware** — MAVLink reception and radian-to-degree conversion.
- **GM3 protocol handling** — frame construction, Tilt/Pan encoding, Roll LUT and CRC16-CCITT.
- **Reverse-engineering tools** — structural and per-axis analysis, CRC verification.
- **Project documentation aligned with Section 6 technical content.**

Project-specific / support code paths:

- `oculusmonitor/oculusmonitor.cpp`
- `oculusmonitor/vrstate.cpp`
- `oculusmonitor/vrstate.h`
- `oculusmonitor/aabb.h`
- `dev/sdk/include/kf/kf_time.h`
- `firmware/arduino/`
- `tools/reverse-engineering/`
- `docs/`

These components are part of the project implementation authored by Nicolás Corimayo and are not
external dependencies.

---

## 2. External dependencies

The following third-party libraries and SDKs are used by the project. Their original
license/attribution headers are preserved in the corresponding files and must not be removed.

### Dear ImGui

- **Purpose:** Immediate-mode GUI for the desktop application's visualization window.
- **Repository paths:** `oculusmonitor/imgui.cpp`, `oculusmonitor/imgui.h`,
  `oculusmonitor/imgui_draw.cpp`, `oculusmonitor/imgui_demo.cpp`,
  `oculusmonitor/imgui_internal.h`, `oculusmonitor/imconfig.h`,
  `oculusmonitor/imgui_impl_win32.{cpp,h}`, `oculusmonitor/imgui_impl_dx11.{cpp,h}`
- **License / attribution:** MIT License (Omar Cornut and contributors),
  https://github.com/ocornut/imgui . Version marker in source: `v1.63 WIP`. Header attribution
  is preserved in the files.
- **Status:** Bundled (source).

### stb (stb_truetype, stb_textedit, stb_rect_pack)

- **Purpose:** Font rasterization, text editing and rectangle packing used by Dear ImGui.
- **Repository paths:** `oculusmonitor/stb_truetype.h`, `oculusmonitor/stb_textedit.h`,
  `oculusmonitor/stb_rect_pack.h`
- **License / attribution:** Public domain / MIT dual license (Sean Barrett and contributors).
  Public-domain notice and author list are preserved in the file headers.
- **Status:** Bundled (source).

### MAVLink

- **Purpose:** Communication protocol between the desktop software and the Arduino Mega; the
  `ATTITUDE` message transports the processed orientation.
- **Repository paths:** `dev/sdk/include/mavlink/` (generated C headers; the `common` dialect is
  the one used by the project).
- **License / attribution:** MIT License (MAVLink generated headers, produced from the MAVLink
  XML message definitions).
- **Status:** Bundled (headers).

### Oculus SDK / LibOVR

- **Purpose:** Access to the headset's tracking state and head pose (orientation as quaternion).
- **Repository paths:** `dev/sdk/include/oculus/` (e.g. `OVR_CAPI.h`, `Extras/OVR_Math.h`),
  `dev/sdk/lib/LibOVR.lib`
- **License / attribution:** Proprietary Oculus SDK. Source headers carry
  "Copyright 2014 Oculus VR, LLC. All rights reserved." This notice is preserved and must not be
  removed.
- **Status:** Bundled (headers + static library); required to build the desktop application.
