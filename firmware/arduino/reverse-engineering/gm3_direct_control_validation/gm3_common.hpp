// ============================================================================
// gm3_common.hpp
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Modulo auxiliar comun a los programas de analisis del protocolo Caddx GM3.
//
// Provee la lectura de los archivos de log capturados con PuTTY y las
// operaciones de bytes comunes a los distintos analizadores.
//
// Formato de log esperado (una linea por trama capturada):
//
//     <tiempo_ms>,<dir>,<len>,<b0> <b1> ... <bN>
//
// Ejemplo:
//     20,<,10,A5 5A 03 00 00 00 00 00 19 6E
//
//   - tiempo_ms : milisegundos desde el inicio del registro
//   - dir       : direccion de la comunicacion ('<' hacia el gimbal)
//   - len       : cantidad de bytes de la trama
//   - bytes     : contenido en hexadecimal separado por espacios
// ============================================================================

#ifndef GM3_COMMON_HPP
#define GM3_COMMON_HPP

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace gm3 {

// ----------------------------------------------------------------------------
// Una trama capturada
// ----------------------------------------------------------------------------
struct Frame {
    long                 timestamp;   // milisegundos
    char                 direction;   // '<' o '>'
    std::vector<uint8_t> bytes;       // contenido de la trama
};

// ----------------------------------------------------------------------------
// Conversion de un token hexadecimal ("A5", "0x5A") a byte
// ----------------------------------------------------------------------------
inline bool parseHexByte(std::string tok, uint8_t& out) {
    if (tok.rfind("0x", 0) == 0 || tok.rfind("0X", 0) == 0)
        tok = tok.substr(2);
    if (tok.empty()) return false;
    try {
        out = static_cast<uint8_t>(std::stoul(tok, nullptr, 16));
    } catch (...) {
        return false;
    }
    return true;
}

// ----------------------------------------------------------------------------
// Lectura de un archivo de log completo.
// Devuelve true si se pudo abrir el archivo (aunque no haya tramas validas).
// ----------------------------------------------------------------------------
inline bool loadLog(const std::string& path, std::vector<Frame>& out) {
    std::ifstream in(path);
    if (!in) return false;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;

        // Separar por comas los tres primeros campos y el resto (hex)
        std::vector<std::string> parts;
        std::stringstream ss(line);
        std::string field;
        while (std::getline(ss, field, ',')) parts.push_back(field);
        if (parts.size() < 4) continue;

        Frame f;
        try {
            f.timestamp = std::stol(parts[0]);
        } catch (...) {
            f.timestamp = 0;
        }
        f.direction = parts[1].empty() ? '?' : parts[1][0];

        // El ultimo campo contiene los bytes en hexadecimal
        std::stringstream hexss(parts.back());
        std::string tok;
        while (hexss >> tok) {
            uint8_t b;
            if (parseHexByte(tok, b)) f.bytes.push_back(b);
        }

        if (!f.bytes.empty()) out.push_back(std::move(f));
    }
    return true;
}

// ----------------------------------------------------------------------------
// Reconstruccion de un entero de 16 bits little-endian a partir de dos bytes
// ----------------------------------------------------------------------------
inline int16_t le16(uint8_t low, uint8_t high) {
    return static_cast<int16_t>(static_cast<uint16_t>(low) |
                                (static_cast<uint16_t>(high) << 8));
}

// ----------------------------------------------------------------------------
// Reconstruccion de un entero de 16 bits big-endian
// ----------------------------------------------------------------------------
inline int16_t be16(uint8_t high, uint8_t low) {
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) |
                                static_cast<uint16_t>(low));
}

} // namespace gm3

#endif // GM3_COMMON_HPP
