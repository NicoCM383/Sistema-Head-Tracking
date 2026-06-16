// ============================================================
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// Seccion 6.3 - Ingenieria Inversa (modulo auxiliar comun)
// Fuente: "Codigos para la ingenieria inversa.docx"
// Extraccion fiel (sin refactor).
// Lectura de logs CSV, conversion int16 LE y CRC16-CCITT.
// ============================================================
#ifndef GM3_COMMON_HPP
#define GM3_COMMON_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <set>
#include <map>
#include <string>
#include <iomanip>
#include <cstdint>
#include <algorithm>
#include <cmath>

struct Frame {
    long long t_ms;
    char dir;
    int len;
    std::array<uint8_t, 10> b;
};

static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t z = s.find_last_not_of(" \t\r\n");
    return s.substr(a, z - a + 1);
}

static std::vector<std::string> splitComma(const std::string& line) {
    std::vector<std::string> parts;
    std::stringstream ss(line);
    std::string item;

    while (std::getline(ss, item, ',')) {
        parts.push_back(trim(item));
    }

    return parts;
}

static bool parseHexByte(const std::string& token, uint8_t& out) {
    try {
        int v = std::stoi(token, nullptr, 16);
        if (v < 0 || v > 255) return false;
        out = static_cast<uint8_t>(v);
        return true;
    } catch (...) {
        return false;
    }
}

static bool parseFrameCSV(const std::string& line, Frame& frame) {
    if (line.empty()) return false;
    if (line[0] == '#') return false;
    if (line[0] == '=') return false;
    if (line.rfind("t_ms", 0) == 0) return false;

    std::vector<std::string> parts = splitComma(line);

    // Formato nuevo:
    // t_ms,dir,len,b0,b1,b2,b3,b4,b5,b6,b7,b8,b9
    if (parts.size() == 13) {
        try {
            frame.t_ms = std::stoll(parts[0]);
            frame.dir = parts[1].empty() ? '?' : parts[1][0];
            frame.len = std::stoi(parts[2]);
        } catch (...) {
            return false;
        }

        for (int i = 0; i < 10; i++) {
            if (!parseHexByte(parts[3 + i], frame.b[i])) {
                return false;
            }
        }

        return frame.len == 10;
    }
    // Formato anterior compatible:
    // t_ms,dir,len,A5 5A 03 00 00 00 00 00 19 6E
    if (parts.size() == 4) {
        try {
            frame.t_ms = std::stoll(parts[0]);
            frame.dir = parts[1].empty() ? '?' : parts[1][0];
            frame.len = std::stoi(parts[2]);
        } catch (...) {
            return false;
        }

        std::stringstream hs(parts[3]);
        std::string token;
        int i = 0;

        while (hs >> token && i < 10) {
            if (!parseHexByte(token, frame.b[i])) {
                return false;
            }
            i++;
        }

        return i == 10;
    }

    return false;
}

static std::vector<Frame> readFrames(const std::string& path) {
    std::ifstream file(path);
    std::vector<Frame> frames;

    if (!file.is_open()) {
        std::cerr << "No se pudo abrir el archivo: " << path << "\n";
        return frames;
    }

    std::string line;

    while (std::getline(file, line)) {
        Frame f;
        if (parseFrameCSV(trim(line), f)) {
            frames.push_back(f);
        }
    }

    return frames;
}

static std::string byteToHex(uint8_t b) {
    std::stringstream ss;
    ss << std::uppercase << std::hex
       << std::setw(2) << std::setfill('0')
       << static_cast<int>(b);
    return ss.str();
}

static int16_t int16LE(uint8_t lo, uint8_t hi) {
    uint16_t value = static_cast<uint16_t>(lo) |
                     (static_cast<uint16_t>(hi) << 8);
    return static_cast<int16_t>(value);
}

static uint16_t crc16_ccitt_0(const uint8_t* data, size_t len) {
    uint16_t crc = 0x0000;

    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

static void printUniqueCount(const std::vector<Frame>& frames) {
    std::set<int> unique[10];

    for (const Frame& f : frames) {
        for (int i = 0; i < 10; i++) {
            unique[i].insert(f.b[i]);
        }
    }
    std::cout << "Valores distintos por byte:\n";
    for (int i = 0; i < 10; i++) {
        std::cout << "b" << i << ": "
                  << unique[i].size() << "\n";
    }
}

#endif
