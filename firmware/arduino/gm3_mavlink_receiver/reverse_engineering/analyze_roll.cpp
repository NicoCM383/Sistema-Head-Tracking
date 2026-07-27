// ============================================================================
// analyze_roll.cpp
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// ETAPA: Primera aproximacion (vease seccion 6.3.3)
//
// Este programa analiza el campo asociado al eje Roll en los bytes b3-b4.
// Al disponer de un numero reducido de muestras y no observar a simple vista
// una progresion lineal entre las combinaciones de bytes, dio origen al
// modelo de tabla de correspondencias (LUT).
//
// El analisis posterior (gm3_hypothesis_search.cpp) demostro que dicho modelo
// era incorrecto: al interpretar b3-b4 como entero de 16 bits little-endian,
// los valores son perfectamente lineales, con un factor de 182,04 u/grado
// (16 bits). La "LUT" era, en realidad, el muestreo de cinco puntos sobre una
// misma recta.
//
// Se conserva SIN MODIFICACIONES como registro del procedimiento empleado.
// La implementacion vigente es gm3_hypothesis_search.cpp.
//
// COMPILACION
//     g++ -std=c++17 -O2 -o analyze_roll analyze_roll.cpp
//
// USO
//     analyze_roll <log_roll>
// ============================================================================

#include "gm3_common.hpp"

#include <iomanip>
#include <iostream>
#include <vector>

// Offset del eje Roll (correctamente identificado ya en esta etapa)
constexpr int ROLL_OFFSET = 3;   // b3-b4

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Uso: analyze_roll <log_roll>\n";
        return 1;
    }

    std::vector<gm3::Frame> frames;
    if (!gm3::loadLog(argv[1], frames) || frames.empty()) {
        std::cerr << "Error: no se pudieron leer tramas de '" << argv[1] << "'\n";
        return 1;
    }

    std::cout << "\n=== analyze_roll (primera aproximacion) ===\n";
    std::cout << "Offset analizado: b" << ROLL_OFFSET
              << "-b" << (ROLL_OFFSET + 1) << "\n";
    std::cout << "Tramas: " << frames.size() << "\n\n";

    // Registrar las combinaciones de bytes distintas observadas
    std::cout << "Combinaciones (b3, b4) observadas:\n";
    std::cout << "  b3    b4\n";
    std::cout << "  --------\n";

    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& b = frames[i].bytes;
        if (b.size() < (size_t)ROLL_OFFSET + 2) continue;

        if (i % 5 == 0) {
            std::cout << "  0x" << std::hex << std::uppercase
                      << std::setw(2) << std::setfill('0') << (int)b[ROLL_OFFSET]
                      << "  0x"
                      << std::setw(2) << std::setfill('0') << (int)b[ROLL_OFFSET + 1]
                      << std::dec << std::setfill(' ') << "\n";
        }
    }

    std::cout << "\nEn esta etapa no se observo una progresion lineal evidente\n";
    std::cout << "entre las combinaciones de bytes, por lo que se modelo el eje\n";
    std::cout << "mediante una tabla de correspondencias (LUT).\n";
    std::cout << "NOTA: este modelo resulto posteriormente incorrecto. El eje Roll\n";
    std::cout << "es lineal (int16 little-endian, 182,04 u/grado).\n\n";

    return 0;
}
