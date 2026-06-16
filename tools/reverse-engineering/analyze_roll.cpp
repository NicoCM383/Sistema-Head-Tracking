// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Seccion 6.3 - Ingenieria Inversa (analisis del eje Roll, b3-b4 / LUT)
// Fuente: "Codigos para la ingenieria inversa.docx"
// Extraccion fiel (sin refactor).
// Compilar: g++ -std=c++17 analyze_roll.cpp -o analyze_roll
// Ejecutar: ./analyze_roll <log_roll.csv>
// ============================================================
#include "gm3_common.hpp"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Uso: analyze_roll.exe <log_roll.csv>\n";
        return 1;
    }

    std::vector<Frame> frames = readFrames(argv[1]);

    if (frames.empty()) {
        std::cerr << "No se encontraron tramas validas.\n";
        return 1;
    }

    std::cout << "=== ANALISIS ROLL ===\n\n";
    std::cout << "Tramas analizadas: " << frames.size() << "\n\n";

    printUniqueCount(frames);

    int rollModeCount = 0;
    std::map<std::string, int> states;

    for (const Frame& f : frames) {
        bool rollMode = (f.b[5] == 0x00 &&
                         f.b[6] == 0x00 &&
                         f.b[7] == 0x00);

        if (rollMode) {
            rollModeCount++;

            std::string key = byteToHex(f.b[3]) + " " + byteToHex(f.b[4]);
            states[key]++;
        }
    }

    std::cout << "\nHipotesis evaluada:\n";
    std::cout << "Roll = combinacion discreta usando b3-b4\n";
    std::cout << "b5, b6 y b7 permanecen en 00 durante modo Roll\n";
    std::cout << "\nTramas compatibles con modo Roll: "
              << rollModeCount << " / " << frames.size() << "\n";

    std::cout << "\nEstados Roll observados b3 b4:\n";

    int shown = 0;

    for (const auto& item : states) {
        std::cout << item.first
                  << " aparece "
                  << item.second
                  << " veces\n";

        shown++;

        if (shown >= 80) {
            std::cout << "...\n";
            break;
        }
    }

    std::cout << "\nConclusion esperada:\n";
    std::cout << "Roll no presenta una relacion lineal simple.\n";
    std::cout << "Debe modelarse mediante una LUT.\n";

    return 0;
}
