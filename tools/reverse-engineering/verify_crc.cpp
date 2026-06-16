// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Seccion 6.3 - Ingenieria Inversa (verificacion del CRC16-CCITT)
// Fuente: "Codigos para la ingenieria inversa.docx"
// Extraccion fiel (sin refactor).
// CRC16-CCITT, polinomio 0x1021, init 0x0000, sobre b0..b7; b8=CRC_H, b9=CRC_L.
// Compilar: g++ -std=c++17 verify_crc.cpp -o verify_crc
// Ejecutar: ./verify_crc <log.csv>
// ============================================================
#include "gm3_common.hpp"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Uso: verify_crc.exe <log.csv>\n";
        return 1;
    }

    std::vector<Frame> frames = readFrames(argv[1]);

    if (frames.empty()) {
        std::cerr << "No se encontraron tramas validas.\n";
        return 1;
    }

    int ok = 0;
    int fail = 0;

    for (const Frame& f : frames) {
        uint16_t crcCalc = crc16_ccitt_0(f.b.data(), 8);

        uint16_t crcFrame =
            (static_cast<uint16_t>(f.b[8]) << 8) |
             static_cast<uint16_t>(f.b[9]);

        if (crcCalc == crcFrame) {
            ok++;
        } else {
            fail++;
        }
    }

    std::cout << "=== VERIFICACION CRC ===\n\n";
    std::cout << "Tramas analizadas: " << frames.size() << "\n";
    std::cout << "CRC OK: " << ok << "\n";
    std::cout << "CRC FAIL: " << fail << "\n";

    std::cout << "\nHipotesis evaluada:\n";
    std::cout << "CRC16-CCITT, polinomio 0x1021, init 0x0000\n";
    std::cout << "Calculado sobre b0..b7\n";
    std::cout << "Almacenado como b8=CRC_H y b9=CRC_L\n";
    return 0;
}
