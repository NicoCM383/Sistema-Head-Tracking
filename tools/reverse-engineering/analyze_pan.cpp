// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Seccion 6.3 - Ingenieria Inversa (analisis del eje Pan, b6-b7)
// Fuente: "Codigos para la ingenieria inversa.docx"
// Extraccion fiel (sin refactor).
// Compilar: g++ -std=c++17 analyze_pan.cpp -o analyze_pan
// Ejecutar: ./analyze_pan <log_pan.csv>
// ============================================================
#include "gm3_common.hpp"
#include <numeric>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Uso: analyze_pan.exe <log_pan.csv>\n";
        return 1;
    }

    std::vector<Frame> frames = readFrames(argv[1]);

    if (frames.empty()) {
        std::cerr << "No se encontraron tramas validas.\n";
        return 1;
    }

    std::cout << "=== ANALISIS PAN ===\n\n";
    std::cout << "Tramas analizadas: " << frames.size() << "\n\n";

    printUniqueCount(frames);

    int16_t minPan = 32767;
    int16_t maxPan = -32768;
    std::set<int> panValues;

    for (const Frame& f : frames) {
        int16_t panRaw = int16LE(f.b[6], f.b[7]);

        if (panRaw < minPan) minPan = panRaw;
        if (panRaw > maxPan) maxPan = panRaw;

        panValues.insert(panRaw);
    }

    std::cout << "\nHipotesis evaluada:\n";
    std::cout << "Pan_raw = int16 little-endian usando b6-b7\n";

    std::cout << "\nResultado:\n";
    std::cout << "Pan raw minimo: " << minPan << "\n";
    std::cout << "Pan raw maximo: " << maxPan << "\n";
    std::cout << "Cantidad de valores Pan distintos: "
              << panValues.size() << "\n";

    std::cout << "\nFormula usada:\n";
    std::cout << "Pan_raw = b6 + (b7 << 8)\n";

    return 0;
}
