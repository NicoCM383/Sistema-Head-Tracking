// ============================================================================
// gm3_structural_analysis.cpp
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// ETAPA: Analisis estructural
//
// Procesa uno o mas archivos de log capturados y determina la estructura
// general de la trama del protocolo Caddx GM3:
//   - distribucion de longitudes de trama
//   - frecuencia de cada valor en cada posicion de byte (b0..bN)
//   - identificacion de los bytes constantes (encabezado, identificador
//     de comando) frente a los bytes variables (datos, verificacion)
//
// COMPILACION
//     g++ -std=c++17 -O2 -o gm3_structural_analysis gm3_structural_analysis.cpp
//
// USO
//     gm3_structural_analysis <log1> [log2 ...]
// ============================================================================

#include "gm3_common.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Uso: gm3_structural_analysis <log1> [log2 ...]\n";
        return 1;
    }

    std::vector<gm3::Frame> frames;
    for (int i = 1; i < argc; ++i) {
        std::vector<gm3::Frame> part;
        if (!gm3::loadLog(argv[i], part)) {
            std::cerr << "Aviso: no se pudo abrir '" << argv[i] << "'\n";
            continue;
        }
        frames.insert(frames.end(), part.begin(), part.end());
    }

    if (frames.empty()) {
        std::cerr << "Error: no se obtuvieron tramas validas.\n";
        return 1;
    }

    std::cout << "\n=== Analisis estructural del protocolo GM3 ===\n";
    std::cout << "Tramas analizadas: " << frames.size() << "\n\n";

    // --- Distribucion de longitudes ---
    std::map<size_t, size_t> lenCount;
    for (const auto& f : frames) lenCount[f.bytes.size()]++;

    std::cout << "Distribucion de longitudes de trama:\n";
    for (const auto& [len, count] : lenCount) {
        std::cout << "  " << len << " bytes: " << count << " tramas\n";
    }

    // Longitud predominante
    size_t frameLen = 0, maxCount = 0;
    for (const auto& [len, count] : lenCount) {
        if (count > maxCount) { maxCount = count; frameLen = len; }
    }
    std::cout << "\nLongitud predominante: " << frameLen << " bytes\n\n";

    // --- Frecuencia de valores por posicion ---
    std::cout << "Frecuencia de valores por posicion de byte:\n";
    std::cout << "(se muestran los valores mas frecuentes de cada posicion)\n\n";

    for (size_t pos = 0; pos < frameLen; ++pos) {
        std::map<uint8_t, size_t> valCount;
        size_t total = 0;
        for (const auto& f : frames) {
            if (f.bytes.size() != frameLen) continue;
            valCount[f.bytes[pos]]++;
            total++;
        }

        // Valor mas frecuente
        uint8_t topVal = 0;
        size_t  topCnt = 0;
        for (const auto& [v, c] : valCount) {
            if (c > topCnt) { topCnt = c; topVal = v; }
        }

        double ratio = total ? (100.0 * topCnt / total) : 0.0;
        bool constant = (valCount.size() == 1);

        std::cout << "  b" << pos << ": ";
        std::cout << valCount.size() << " valores distintos, ";
        std::cout << "mas frecuente = 0x" << std::hex << std::uppercase
                  << std::setw(2) << std::setfill('0') << (int)topVal
                  << std::dec << std::setfill(' ');
        std::cout << " (" << std::fixed << std::setprecision(1) << ratio << "%)";
        if (constant) std::cout << "  <- CONSTANTE";
        std::cout << "\n";
    }

    // --- Interpretacion ---
    std::cout << "\n=== Interpretacion ===\n";
    std::cout << "Los bytes constantes al inicio corresponden al encabezado\n";
    std::cout << "y al identificador de comando. Los bytes variables intermedios\n";
    std::cout << "corresponden a los campos de datos, y los finales al mecanismo\n";
    std::cout << "de verificacion de integridad (CRC).\n\n";

    return 0;
}
