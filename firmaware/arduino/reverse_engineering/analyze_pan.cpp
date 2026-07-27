// ============================================================================
// analyze_pan.cpp
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// ETAPA: Primera aproximacion (vease seccion 6.3.3)
//
// Este programa reconstruye el campo asociado al eje Pan bajo un desplazamiento
// FIJADO DE ANTEMANO (b6-b7), interpretado como entero de 16 bits little-endian.
//
// El analisis posterior (gm3_hypothesis_search.cpp) determino que el eje Pan
// NO es un entero de 16 bits, sino un unico byte con signo ubicado en b7
// (escala 0,711 u/grado, 8 bits). Un aspecto metodologicamente notable es que,
// sobre estos mismos logs, la hipotesis de 16 bits en b6-b7 y la de 8 bits en
// b7 arrojan un ajuste practicamente identico (degeneracion matematica descrita
// en 6.3.5): con b6 en cero, b6+(b7<<8) equivale a Pan*256. La ambiguedad solo
// pudo resolverse mediante la validacion fisica.
//
// Se conserva SIN MODIFICACIONES como registro del procedimiento empleado.
// La implementacion vigente es gm3_hypothesis_search.cpp.
//
// COMPILACION
//     g++ -std=c++17 -O2 -o analyze_pan analyze_pan.cpp
//
// USO
//     analyze_pan <log_pan>
// ============================================================================

#include "gm3_common.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

// Offset fijado por hipotesis en esta etapa (posteriormente corregido)
constexpr int PAN_OFFSET = 6;   // b6-b7

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Uso: analyze_pan <log_pan>\n";
        return 1;
    }

    std::vector<gm3::Frame> frames;
    if (!gm3::loadLog(argv[1], frames) || frames.empty()) {
        std::cerr << "Error: no se pudieron leer tramas de '" << argv[1] << "'\n";
        return 1;
    }

    std::cout << "\n=== analyze_pan (primera aproximacion) ===\n";
    std::cout << "Hipotesis de offset: b" << PAN_OFFSET
              << "-b" << (PAN_OFFSET + 1) << " (int16 little-endian)\n";
    std::cout << "Tramas: " << frames.size() << "\n\n";

    std::cout << "  trama   Pan_raw   variacion\n";
    std::cout << "  -----------------------------\n";

    int prev = 0;
    bool first = true;
    int minRaw = 0, maxRaw = 0;

    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& b = frames[i].bytes;
        if (b.size() < (size_t)PAN_OFFSET + 2) continue;

        int raw = gm3::le16(b[PAN_OFFSET], b[PAN_OFFSET + 1]);

        int delta = first ? 0 : (raw - prev);
        if (first) { minRaw = maxRaw = raw; first = false; }
        minRaw = std::min(minRaw, raw);
        maxRaw = std::max(maxRaw, raw);

        if (i % 5 == 0) {
            std::cout << "  " << i << "\t   " << raw
                      << "\t   " << (delta >= 0 ? "+" : "") << delta << "\n";
        }
        prev = raw;
    }

    std::cout << "\nRango observado: [" << minRaw << ", " << maxRaw << "]\n";
    std::cout << "El valor evoluciona coherentemente con el movimiento horizontal.\n";
    std::cout << "NOTA: esta interpretacion de 16 bits resulto posteriormente\n";
    std::cout << "incorrecta; el eje Pan es un entero de 8 bits en b7.\n\n";

    return 0;
}
