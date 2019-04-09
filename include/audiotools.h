#ifndef AUDIOTOOLS_H
#define AUDIOTOOLS_H
#include <cmath>
#include "constants.h"

namespace audio_tools
{

inline void scaleVolume(double* buf, double volume) noexcept
{
    * buf    *= volume;
    *(buf+1) *= volume;

//    __m128d volpd = _mm_set1_pd(volume);
//    __m128d buffer = _mm_load_pd(buf);
//    buffer = _mm_mul_pd(buffer, volpd);
//    _mm_store_pd(buf, buffer);
}

inline void dbToVolume(double& volume) noexcept
{
    volume = pow(10.0, volume / 20.0);
}

//-------------------------------------------------------------

template <typename T>
inline T scaleRange(T newRangeMax, T newRangeMin,
    T originalRangeMax, T originalRangeMin, T originalValue) noexcept
{
    //returns newValue
    return (((newRangeMax - newRangeMin)
        * (originalValue - originalRangeMin)) /
        (originalRangeMax - originalRangeMin));
}

template <typename T = double>
inline T scaleRange(T newRangeMax, T newRangeMin, T originalValue) noexcept
{
    return (newRangeMin + (newRangeMax - newRangeMin) * originalValue);
}

//----------------------------------------------------------

template <typename T = double>
class ParamSmoothing {
    T a, b, currentValue;
    double sR;
    public:
    ParamSmoothing() = default;
    ParamSmoothing(double _sampleRate,
                   double smoothingMS, T cval);
    T smoothParam(const T) noexcept;
    void setAmount(const T) noexcept;
};

template <typename T>
ParamSmoothing<T>::ParamSmoothing(double _sampleRate,
                 double smoothingMS, T cval) : sR(_sampleRate) {
    a = exp(-TWO_PI / (smoothingMS * 0.001 * sR));
    b = 1.0 - a;
    currentValue = cval;
}

template <typename T>
void ParamSmoothing<T>::setAmount(const T smoothingMS) noexcept {
    a = exp(-TWO_PI / (smoothingMS * 0.001 * sR));
    b = 1.0 - a;
}

template <typename T>
inline T ParamSmoothing<T>::smoothParam(const T targetValue) noexcept {
    currentValue = (targetValue * b) + (currentValue * a);
    return currentValue;
}

//--------------------------------------------------------

inline double clamp4tan(double val)
{
    constexpr double maxVal = PI * 0.5 - 0.001;
    return (((val > 0.0) ? ((val < maxVal) ? val : maxVal) : 0.0));
}



}	// audio_tools


#endif // AUDIOTOOLS_H
