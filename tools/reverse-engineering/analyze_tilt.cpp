// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Seccion 6.3 - Ingenieria Inversa (analisis del eje Tilt, b4-b5)
// Fuente: "Codigos para la ingenieria inversa.docx"
// Extraccion fiel (sin refactor).
// Compilar: g++ -std=c++17 analyze_tilt.cpp -o analyze_tilt
// Ejecutar: ./analyze_tilt <log_tilt.csv>
// ============================================================
#include "gm3_common.hpp"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Uso: analyze_tilt.exe <log_tilt.csv>\n";
        return 1;
    }

    std::vector<Frame> frames = readFrames(argv[1]);

    if (frames.empty()) {
        std::cerr << "No se encontraron tramas validas.\n";
        return 1;
    }

    std::cout << "=== ANALISIS TILT ===\n\n";
    std::cout << "Tramas analizadas: " << frames.size() << "\n\n";

    printUniqueCount(frames);

    int16_t minTilt = 32767;
    int16_t maxTilt = -32768;

    for (const Frame& f : frames) {
        int16_t tiltRaw = int16LE(f.b[4], f.b[5]);

        if (tiltRaw < minTilt) minTilt = tiltRaw;
        if (tiltRaw > maxTilt) maxTilt = tiltRaw;
    }
    std::cout << "\nHipotesis evaluada:\n";
    std::cout << "Tilt_raw = int16 little-endian usando b4-b5\n";

    std::cout << "\nResultado:\n";
    std::cout << "Tilt raw minimo: " << minTilt << "\n";
    std::cout << "Tilt raw maximo: " << maxTilt << "\n";

    std::cout << "\nFormula usada:\n";
    std::cout << "Tilt_raw = b4 + (b5 << 8)\n";

    return 0;
}
