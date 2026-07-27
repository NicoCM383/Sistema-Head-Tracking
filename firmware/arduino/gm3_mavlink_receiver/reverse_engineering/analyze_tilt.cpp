// ============================================================================
// analyze_tilt.cpp
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// ETAPA: Primera aproximacion (vease seccion 6.3.3)
//
// Este programa reconstruye el campo asociado al eje Tilt bajo un
// desplazamiento FIJADO DE ANTEMANO (b4-b5). Su diseno presenta la limitacion
// metodologica descrita en 6.3.4: al recibir el offset como parametro fijo,
// produce una serie numerica de aspecto plausible aunque la hipotesis sea
// incorrecta.
//
// El analisis posterior mediante busqueda exhaustiva (gm3_hypothesis_search.cpp)
// determino que el campo Tilt se ubica en realidad en b5-b6, no en b4-b5, y
// que su escala es 11,38 u/grado (12 bits) y no la estimada en esta etapa.
//
// Se conserva SIN MODIFICACIONES como registro del procedimiento empleado.
// La implementacion vigente es gm3_hypothesis_search.cpp.
//
// COMPILACION
//     g++ -std=c++17 -O2 -o analyze_tilt analyze_tilt.cpp
//
// USO
//     analyze_tilt <log_tilt>
//
//   El log debe contener capturas moviendo unicamente el eje Tilt.
// ============================================================================

#include "gm3_common.hpp"

#include <cmath>
#include <iostream>
#include <vector>

// Offset fijado por hipotesis en esta etapa (posteriormente corregido)
constexpr int TILT_OFFSET = 4;   // b4-b5

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Uso: analyze_tilt <log_tilt>\n";
        return 1;
    }

    std::vector<gm3::Frame> frames;
    if (!gm3::loadLog(argv[1], frames) || frames.empty()) {
        std::cerr << "Error: no se pudieron leer tramas de '" << argv[1] << "'\n";
        return 1;
    }

    std::cout << "\n=== analyze_tilt (primera aproximacion) ===\n";
    std::cout << "Hipotesis de offset: b" << TILT_OFFSET
              << "-b" << (TILT_OFFSET + 1) << " (int16 little-endian)\n";
    std::cout << "Tramas: " << frames.size() << "\n\n";

    // Reconstruir Tilt_raw = b4 + (b5 << 8) para cada trama y observar evolucion
    std::cout << "  trama   Tilt_raw   variacion\n";
    std::cout << "  ------------------------------\n";

    int prev = 0;
    bool first = true;
    int minRaw = 0, maxRaw = 0;

    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& b = frames[i].bytes;
        if (b.size() < (size_t)TILT_OFFSET + 2) continue;

        int raw = gm3::le16(b[TILT_OFFSET], b[TILT_OFFSET + 1]);

        int delta = first ? 0 : (raw - prev);
        if (first) { minRaw = maxRaw = raw; first = false; }
        minRaw = std::min(minRaw, raw);
        maxRaw = std::max(maxRaw, raw);

        // Mostrar solo una de cada cierta cantidad para no saturar
        if (i % 5 == 0) {
            std::cout << "  " << i << "\t   " << raw
                      << "\t   " << (delta >= 0 ? "+" : "") << delta << "\n";
        }
        prev = raw;
    }

    std::cout << "\nRango observado: [" << minRaw << ", " << maxRaw << "]\n";
    std::cout << "El valor evoluciona de manera monotonica y aproximadamente\n";
    std::cout << "proporcional al angulo aplicado, lo que reforzo (erroneamente)\n";
    std::cout << "la hipotesis de que este era el offset correcto del eje Tilt.\n\n";

    return 0;
}
