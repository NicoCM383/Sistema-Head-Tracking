// ============================================================================
// gm3_hypothesis_search.cpp
// ----------------------------------------------------------------------------
// Proyecto de Grado - Control de Gimbal mediante Head Tracking
// ETAPA: Segunda aproximacion (vease seccion 6.3.5)
//
// PROPOSITO
// ---------
// Los programas de la primera aproximacion (analyze_tilt.cpp, analyze_pan.cpp,
// analyze_roll.cpp) partian de una hipotesis fijada de antemano sobre la
// ubicacion y el tipo de dato de cada eje, y se limitaban a reconstruir el
// valor bajo dicha hipotesis. Ese enfoque presenta una debilidad metodologica:
// si la hipotesis de partida es incorrecta, el programa produce igualmente una
// serie numerica de aspecto plausible, sin senal alguna de error.
//
// Este programa invierte el procedimiento. En lugar de asumir la estructura,
// evalua de manera exhaustiva el espacio completo de hipotesis compatibles con
// una trama de longitud fija:
//
//     posicion de inicio  x  ancho (1 o 2 bytes)  x  endianness  x  con/sin signo
//
// Para cada hipotesis reconstruye la serie de valores y la contrasta contra el
// angulo de referencia mediante regresion lineal por minimos cuadrados. Las
// hipotesis se ordenan por coeficiente de determinacion (R^2), de modo que la
// estructura correcta emerge de los datos y no de una suposicion previa.
//
// El programa reporta ademas, para cada hipotesis, la profundidad de bits
// implicita en la pendiente obtenida (360 * pendiente = rango de la palabra),
// lo que permite verificar si el campo responde a una codificacion angular
// binaria.
//
// FORMATO DE ENTRADA
// ------------------
// Archivo de texto con una linea por muestra:
//
//     <angulo_referencia> <byte0> <byte1> ... <byteN>
//
// Los bytes en hexadecimal, separados por espacios o comas. El angulo de
// referencia en grados (decimal, admite signo). Se ignoran las lineas vacias
// y las que comienzan con '#'.
//
// Ejemplo:
//     # angulo   trama capturada
//     -30.0      A5 5A 03 A0 EA 00 00 00 DF 4B
//     -10.0      A5 5A 03 E0 F8 00 00 00 38 EC
//       0.0      A5 5A 03 00 00 00 00 00 19 6E
//
// USO
// ---
//     gm3_hypothesis_search <archivo> [--min-r2 0.98] [--top 15]
//
// COMPILACION
// -----------
//     g++ -std=c++17 -O2 -o gm3_hypothesis_search gm3_hypothesis_search.cpp
// ============================================================================

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// ----------------------------------------------------------------------------
// Una muestra: angulo de referencia y trama capturada
// ----------------------------------------------------------------------------
struct Sample {
    double               angle;
    std::vector<uint8_t> frame;
};

// ----------------------------------------------------------------------------
// Hipotesis sobre la codificacion de un campo
// ----------------------------------------------------------------------------
struct Hypothesis {
    int  offset;        // posicion del primer byte dentro de la trama
    int  width;         // 1 o 2 bytes
    bool littleEndian;  // solo aplica si width == 2
    bool isSigned;      // interpretacion con o sin signo

    // Resultados del ajuste
    double slope     = 0.0;   // unidades por grado
    double intercept = 0.0;
    double r2        = 0.0;
    double maxResid  = 0.0;
    double bits      = 0.0;   // profundidad implicita: log2(360 * |pendiente|)

    std::string describe() const {
        std::ostringstream os;
        os << "b" << offset;
        if (width == 2) {
            os << "-b" << (offset + 1)
               << (littleEndian ? " LE" : " BE");
        } else {
            os << "   ";
        }
        os << (isSigned ? " int" : " uint") << (width * 8);
        return os.str();
    }
};

// ----------------------------------------------------------------------------
// Extraccion del valor de un campo segun la hipotesis
// ----------------------------------------------------------------------------
double extractValue(const std::vector<uint8_t>& frame, const Hypothesis& h) {
    if (h.width == 1) {
        uint8_t raw = frame[h.offset];
        return h.isSigned ? static_cast<double>(static_cast<int8_t>(raw))
                          : static_cast<double>(raw);
    }

    uint16_t raw;
    if (h.littleEndian) {
        raw = static_cast<uint16_t>(frame[h.offset]) |
              (static_cast<uint16_t>(frame[h.offset + 1]) << 8);
    } else {
        raw = (static_cast<uint16_t>(frame[h.offset]) << 8) |
              static_cast<uint16_t>(frame[h.offset + 1]);
    }
    return h.isSigned ? static_cast<double>(static_cast<int16_t>(raw))
                      : static_cast<double>(raw);
}

// ----------------------------------------------------------------------------
// Regresion lineal por minimos cuadrados
//   modelo:  valor = pendiente * angulo + ordenada
// ----------------------------------------------------------------------------
bool fitLinear(const std::vector<Sample>& samples, Hypothesis& h) {
    const size_t n = samples.size();
    if (n < 3) return false;

    double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
    for (const auto& s : samples) {
        const double x = s.angle;
        const double y = extractValue(s.frame, h);
        sx += x;  sy += y;  sxx += x * x;  sxy += x * y;  syy += y * y;
    }

    const double N     = static_cast<double>(n);
    const double denom = N * sxx - sx * sx;
    if (std::fabs(denom) < 1e-12) return false;   // sin variacion en el angulo

    h.slope     = (N * sxy - sx * sy) / denom;
    h.intercept = (sy - h.slope * sx) / N;

    const double ssTot = syy - (sy * sy) / N;
    if (std::fabs(ssTot) < 1e-12) return false;   // campo constante: no informativo

    double ssRes = 0.0;
    h.maxResid   = 0.0;
    for (const auto& s : samples) {
        const double pred = h.slope * s.angle + h.intercept;
        const double d    = extractValue(s.frame, h) - pred;
        ssRes += d * d;
        h.maxResid = std::max(h.maxResid, std::fabs(d));
    }

    h.r2 = 1.0 - ssRes / ssTot;

    const double wordRange = 360.0 * std::fabs(h.slope);
    h.bits = (wordRange > 1.0) ? std::log2(wordRange) : 0.0;

    return true;
}

// ----------------------------------------------------------------------------
// Lectura del archivo de capturas
// ----------------------------------------------------------------------------
bool loadSamples(const std::string& path, std::vector<Sample>& out,
                 size_t& frameLen) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "Error: no se pudo abrir '" << path << "'\n";
        return false;
    }

    std::string line;
    size_t lineNo = 0;
    frameLen = 0;

    while (std::getline(in, line)) {
        ++lineNo;

        for (char& c : line) if (c == ',' || c == ';' || c == '\t') c = ' ';

        std::istringstream ls(line);
        std::string tok;
        if (!(ls >> tok)) continue;                 // linea vacia
        if (tok.empty() || tok[0] == '#') continue; // comentario

        Sample s;
        try {
            s.angle = std::stod(tok);
        } catch (...) {
            std::cerr << "Aviso: linea " << lineNo
                      << " ignorada (angulo no numerico)\n";
            continue;
        }

        while (ls >> tok) {
            if (tok.rfind("0x", 0) == 0 || tok.rfind("0X", 0) == 0)
                tok = tok.substr(2);
            try {
                s.frame.push_back(
                    static_cast<uint8_t>(std::stoul(tok, nullptr, 16)));
            } catch (...) {
                std::cerr << "Aviso: byte invalido en linea " << lineNo
                          << " ('" << tok << "')\n";
            }
        }

        if (s.frame.empty()) continue;

        if (frameLen == 0) {
            frameLen = s.frame.size();
        } else if (s.frame.size() != frameLen) {
            std::cerr << "Aviso: linea " << lineNo << " descartada (longitud "
                      << s.frame.size() << ", se esperaba " << frameLen << ")\n";
            continue;
        }

        out.push_back(std::move(s));
    }

    return !out.empty();
}

// ----------------------------------------------------------------------------
// Interpretacion de la profundidad de bits
// ----------------------------------------------------------------------------
std::string interpretBits(double bits) {
    const double tolerance = 0.15;
    const int candidates[] = {8, 10, 12, 14, 16};
    for (int b : candidates) {
        if (std::fabs(bits - b) < tolerance) {
            std::ostringstream os;
            os << "codif. angular de " << b << " bits";
            return os.str();
        }
    }
    return "-";
}

// ----------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr <<
            "Uso: gm3_hypothesis_search <archivo> [--min-r2 V] [--top N]\n\n"
            "  archivo    capturas: '<angulo> <byte0> <byte1> ...' por linea\n"
            "  --min-r2   umbral de coeficiente de determinacion (def. 0.90)\n"
            "  --top      cantidad de hipotesis a listar (def. 12)\n";
        return 1;
    }

    std::string path  = argv[1];
    double      minR2 = 0.90;
    size_t      top   = 12;

    for (int i = 2; i < argc - 1; ++i) {
        std::string a = argv[i];
        if (a == "--min-r2") minR2 = std::stod(argv[++i]);
        else if (a == "--top") top = std::stoul(argv[++i]);
    }

    std::vector<Sample> samples;
    size_t frameLen = 0;
    if (!loadSamples(path, samples, frameLen)) {
        std::cerr << "Error: no se obtuvieron muestras validas.\n";
        return 1;
    }

    std::cout << "\n"
              << "===========================================================\n"
              << " Analisis exhaustivo de hipotesis - protocolo Caddx GM3\n"
              << "===========================================================\n"
              << " Archivo            : " << path << "\n"
              << " Muestras validas   : " << samples.size() << "\n"
              << " Longitud de trama  : " << frameLen << " bytes\n";

    double aMin = samples.front().angle, aMax = aMin;
    for (const auto& s : samples) {
        aMin = std::min(aMin, s.angle);
        aMax = std::max(aMax, s.angle);
    }
    std::cout << " Rango angular      : " << std::fixed << std::setprecision(1)
              << aMin << " a " << aMax << " grados\n\n";

    // --- Generacion del espacio de hipotesis ---
    std::vector<Hypothesis> hyps;
    for (size_t off = 0; off < frameLen; ++off) {
        for (int width : {1, 2}) {
            if (off + static_cast<size_t>(width) > frameLen) continue;
            for (bool le : {true, false}) {
                if (width == 1 && !le) continue;   // endianness no aplica
                for (bool sg : {true, false}) {
                    Hypothesis h;
                    h.offset       = static_cast<int>(off);
                    h.width        = width;
                    h.littleEndian = le;
                    h.isSigned     = sg;
                    if (fitLinear(samples, h)) hyps.push_back(h);
                }
            }
        }
    }

    std::cout << " Hipotesis evaluadas: " << hyps.size() << "\n\n";

    std::sort(hyps.begin(), hyps.end(),
              [](const Hypothesis& a, const Hypothesis& b) { return a.r2 > b.r2; });

    // --- Tabla de resultados ---
    std::cout << "-----------------------------------------------------------"
                 "--------------------\n";
    std::cout << std::left
              << std::setw(18) << " Hipotesis"
              << std::right
              << std::setw(11) << "R2"
              << std::setw(14) << "u/grado"
              << std::setw(11) << "ordenada"
              << std::setw(9)  << "resid."
              << std::setw(8)  << "bits"
              << "   observacion\n";
    std::cout << "-----------------------------------------------------------"
                 "--------------------\n";

    size_t shown = 0;
    for (const auto& h : hyps) {
        if (shown >= top) break;
        std::cout << std::left << " " << std::setw(17) << h.describe()
                  << std::right << std::fixed
                  << std::setw(11) << std::setprecision(6) << h.r2
                  << std::setw(14) << std::setprecision(4) << h.slope
                  << std::setw(11) << std::setprecision(1) << h.intercept
                  << std::setw(9)  << std::setprecision(1) << h.maxResid
                  << std::setw(8)  << std::setprecision(2) << h.bits
                  << "   " << interpretBits(h.bits) << "\n";
        ++shown;
    }
    std::cout << "-----------------------------------------------------------"
                 "--------------------\n\n";

    // --- Conclusion ---
    if (!hyps.empty() && hyps.front().r2 >= minR2) {
        const Hypothesis& b = hyps.front();
        std::cout << "HIPOTESIS DE MEJOR AJUSTE\n"
                  << "  Campo              : " << b.describe() << "\n"
                  << "  Factor de escala   : " << std::setprecision(4) << b.slope
                  << " unidades/grado\n"
                  << "  Coef. determinacion: " << std::setprecision(6) << b.r2 << "\n"
                  << "  Residuo maximo     : " << std::setprecision(2)
                  << b.maxResid << " unidades\n"
                  << "  Profundidad        : " << std::setprecision(2) << b.bits
                  << " bits (" << interpretBits(b.bits) << ")\n\n";

        if (hyps.size() > 1) {
            const double margin = b.r2 - hyps[1].r2;
            std::cout << "  Margen sobre la hipotesis siguiente: "
                      << std::setprecision(6) << margin << "\n";
            if (margin < 1e-4) {
                std::cout << "  ADVERTENCIA: el margen es reducido. Las dos hipotesis\n"
                             "  resultan practicamente indistinguibles con este conjunto\n"
                             "  de datos; se recomienda ampliar el rango angular o el\n"
                             "  numero de muestras.\n";
            }
            std::cout << "\n";
        }
    } else {
        std::cout << "Ninguna hipotesis alcanzo el umbral R2 >= " << minR2 << ".\n"
                     "Posibles causas: el campo no varia linealmente con el angulo,\n"
                     "el eje analizado no esta presente en las capturas, o la trama\n"
                     "contiene los tres ejes acoplados (analizar un eje por vez).\n\n";
    }

    // --- Salida CSV de la mejor hipotesis, para el anexo ---
    if (!hyps.empty()) {
        const Hypothesis& b = hyps.front();
        std::cout << "--- Datos de la hipotesis de mejor ajuste (CSV) ---\n"
                  << "angulo_grados,valor_reconstruido,valor_predicho,residuo\n";
        for (const auto& s : samples) {
            const double v    = extractValue(s.frame, b);
            const double pred = b.slope * s.angle + b.intercept;
            std::cout << std::setprecision(2) << s.angle << ","
                      << std::setprecision(0) << v << ","
                      << std::setprecision(2) << pred << ","
                      << std::setprecision(2) << (v - pred) << "\n";
        }
        std::cout << "--- fin CSV ---\n\n";
    }

    return 0;
}
