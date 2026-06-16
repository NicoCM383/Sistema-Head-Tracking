// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Seccion 6.3 - Ingenieria Inversa (analisis estructural inicial)
// Fuente: "Codigos para la ingenieria inversa.docx"
// Extraccion fiel (sin refactor).
// Compilar: g++ -std=c++17 gm3_structural_analysis.cpp -o gm3_structural_analysis
// Ejecutar: ./gm3_structural_analysis <log.csv>
// ============================================================
#include "gm3_common.hpp"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Uso: gm3_structural_analysis.exe <log.csv>\n";
        return 1;
    }

    std::vector<Frame> frames = readFrames(argv[1]);

    if (frames.empty()) {
        std::cerr << "No se encontraron tramas validas.\n";
        return 1;
    }

    std::cout << "=== ANALISIS ESTRUCTURAL INICIAL GM3 ===\n\n";
    std::cout << "Tramas validas leidas: " << frames.size() << "\n\n";

    std::map<int, int> lenCount;

    for (const Frame& f : frames) {
        lenCount[f.len]++;
    }

    std::cout << "Distribucion de longitudes:\n";
    for (const auto& item : lenCount) {
        std::cout << "Longitud " << item.first
                  << " bytes: " << item.second
                  << " tramas\n";
    }
    std::cout << "\nValores mas frecuentes por posicion:\n";
    for (int pos = 0; pos < 10; pos++) {
        std::map<int, int> freq;

        for (const Frame& f : frames) {
            freq[f.b[pos]]++;
        }

        int bestValue = -1;
        int bestFreq = -1;

        for (const auto& item : freq) {
            if (item.second > bestFreq) {
                bestValue = item.first;
                bestFreq = item.second;
            }
        }

        double percent = 100.0 * bestFreq / frames.size();

        std::cout << "b" << pos << ": "
                  << byteToHex(static_cast<uint8_t>(bestValue))
                  << " aparece " << bestFreq << " veces ("
                  << std::fixed << std::setprecision(2)
                  << percent << "%)";

        if (percent == 100.0) {
            std::cout << "  <-- constante";
        }

        std::cout << "\n";
    }

    std::cout << "\nPrimeras 5 tramas:\n";

    for (size_t i = 0; i < frames.size() && i < 5; i++) {
        for (int j = 0; j < 10; j++) {
            std::cout << byteToHex(frames[i].b[j]) << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
