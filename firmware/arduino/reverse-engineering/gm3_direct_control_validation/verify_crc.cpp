// ============================================================================
// verify_crc.cpp
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// ETAPA: Verificacion de integridad
//
// Calcula el CRC16-CCITT (polinomio 0x1021, inicializacion 0x0000) sobre los
// primeros ocho bytes de cada trama capturada (b0..b7) y lo compara con los
// dos ultimos bytes (b8-b9), transmitidos en formato big-endian:
//
//     CRC16-CCITT(b0..b7) == (b8 << 8) | b9
//
// La coincidencia confirma que b8-b9 corresponden al campo de verificacion de
// integridad del mensaje. Esta identificacion resulto correcta y se mantuvo
// sin cambios durante todo el proceso: fue la razon por la cual el gimbal
// aceptaba las tramas generadas incluso bajo interpretaciones erroneas de los
// campos de datos.
//
// COMPILACION
//     g++ -std=c++17 -O2 -o verify_crc verify_crc.cpp
//
// USO
//     verify_crc <log1> [log2 ...]
// ============================================================================

#include "gm3_common.hpp"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

// ----------------------------------------------------------------------------
// CRC16-CCITT (polinomio 0x1021, init 0x0000)
// ----------------------------------------------------------------------------
uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc <<= 1;
        }
    }
    return crc;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Uso: verify_crc <log1> [log2 ...]\n";
        return 1;
    }

    std::vector<gm3::Frame> frames;
    for (int i = 1; i < argc; ++i) {
        std::vector<gm3::Frame> part;
        if (gm3::loadLog(argv[i], part))
            frames.insert(frames.end(), part.begin(), part.end());
    }

    if (frames.empty()) {
        std::cerr << "Error: no se obtuvieron tramas validas.\n";
        return 1;
    }

    size_t ok = 0, fail = 0, skipped = 0;

    for (const auto& f : frames) {
        if (f.bytes.size() != 10) { skipped++; continue; }

        uint16_t calc = crc16_ccitt(f.bytes.data(), 8);
        uint16_t stored = (static_cast<uint16_t>(f.bytes[8]) << 8) | f.bytes[9];

        if (calc == stored) ok++;
        else {
            fail++;
            if (fail <= 5) {  // mostrar solo los primeros desajustes
                std::cout << "Desajuste: calculado=0x" << std::hex << std::uppercase
                          << std::setw(4) << std::setfill('0') << calc
                          << "  almacenado=0x"
                          << std::setw(4) << std::setfill('0') << stored
                          << std::dec << std::setfill(' ') << "\n";
            }
        }
    }

    std::cout << "\n=== Verificacion de CRC16-CCITT ===\n";
    std::cout << "Polinomio 0x1021, init 0x0000, sobre b0..b7\n\n";
    std::cout << "Tramas coincidentes : " << ok << "\n";
    std::cout << "Tramas con desajuste: " << fail << "\n";
    std::cout << "Tramas descartadas  : " << skipped << " (longitud != 10)\n\n";

    if (fail == 0 && ok > 0) {
        std::cout << "Todas las tramas validas satisfacen el CRC. Se confirma que\n";
        std::cout << "b8-b9 corresponden al campo de verificacion de integridad.\n\n";
    }

    return 0;
}
