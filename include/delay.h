#ifndef DELAY_H
#define DELAY_H

#include <memory>
#include <vector>
#include "constants.h"
#include <cstring>

class Delay
{
    std:: unique_ptr<std::vector<float>> delayBuffer[2];
    typedef struct {__m128 mWet, mDry, mFb;} DCoeffs;
    alignas (16) DCoeffs dCoeffs;
    size_t mReadIndex[2], mWriteIndex[2];
    size_t delay_buff_size, delay_buff_mask;
    __m128 extFB;
    size_t ms2samples(const int) const noexcept;
    template<typename Width>
    Width findNextPow2(Width v) const noexcept;
    size_t multipleOfN(size_t, size_t);
public:
    Delay(const double);
    void updateDelay(float*, const int) noexcept;
    void updateDelayExtFB(float*, const int) noexcept;
    void setOffset(const int) noexcept;
    void setDryWet(const float) noexcept;
    void setFeedback(const float) noexcept;
    const __m128& getDelayedSample(const int ch) const noexcept;
    void setExternalFB(float fb) noexcept;
    void flushDelayBuffers() noexcept;
};

inline void Delay::setOffset(const int time) noexcept
{
//    mReadIndex[0] = ((mWriteIndex[0] - ms2samples(time)) & delay_buff_mask);
//    mReadIndex[1] = ((mWriteIndex[1] - ms2samples(time)) & delay_buff_mask);
    mReadIndex[0] = multipleOfN(((mWriteIndex[0] - ms2samples(time)) & delay_buff_mask), 4);
    mReadIndex[1] = multipleOfN(((mWriteIndex[1] - ms2samples(time)) & delay_buff_mask), 4);
}

inline void Delay::setDryWet(const float dw) noexcept
{
    dCoeffs.mWet = _mm_set1_ps(dw);
    dCoeffs.mDry = _mm_set1_ps(1 - dw);
}

inline void Delay::setFeedback(const float fb) noexcept
{
    dCoeffs.mFb = _mm_set1_ps(fb);
}

inline size_t Delay::ms2samples(const int ms) const noexcept
{
    float sr = static_cast<float>(audio_tools::SAMPLE_RATE) / 1000.0f;
    return static_cast<size_t>(static_cast<float>(ms) * sr);
}

template<typename Width>
Width Delay::findNextPow2(Width v) const noexcept
{
    --v;
    for (uint32_t i = 1, maxShift = sizeof (Width) << 3; i <= maxShift; i = i << 1){
        v |= v >> i;
    }
    ++v;
    return v;
}

inline size_t Delay::multipleOfN(size_t sRate, size_t N)
{
    size_t adjust = N - (sRate & (N - 1));
       return (sRate + adjust);
}

inline const __m128& Delay::getDelayedSample(const int ch) const noexcept
{
    return *(reinterpret_cast<__m128*>(&(delayBuffer[ch]->at(
                                             static_cast<size_t>(mReadIndex[ch])))));
}

inline void Delay::setExternalFB(float fb) noexcept
{
    extFB = _mm_set1_ps(fb);
}

inline void Delay::flushDelayBuffers() noexcept
{
    memset(&delayBuffer[0], 0, sizeof (float) * delay_buff_size);
    memset(&delayBuffer[1], 0, sizeof (float) * delay_buff_size);
}

#endif // DELAY_H


