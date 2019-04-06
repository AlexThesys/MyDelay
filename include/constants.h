#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <limits>
#include <cmath>
#include <emmintrin.h>

constexpr double PI = 3.14159265359;
constexpr double TWO_PI = 3.14159265359 * 2.0;

static const double epsilon = std::numeric_limits<double>::epsilon();

namespace audio_tools
{
static double SAMPLE_RATE = 44100.0;

static double BUFFER_SIZE = 512.0;

static constexpr double SMOOTHING = 0.4;
}

#endif // CONSTANTS_H
