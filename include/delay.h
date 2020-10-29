#ifndef DELAY_H
#define DELAY_H

#include <cstring>
#include <vector>
#include "constants.h"


class DelaySIMD
{
    typedef struct {
        alignas(16) float mWet[4];
        alignas(16) float mDry[4];
        alignas(16) float mFb[4];
    } DCoeffs;
    alignas(16) DCoeffs dCoeffs;
    std::vector<float> delayBuffer;
    size_t mReadIndex[2], mWriteIndex[2];
    size_t delay_buff_size, delay_buff_mask;

    static size_t ms2samples(const int) noexcept;
    template<typename Width>
    static Width findNextPow2(Width v) noexcept;
    static size_t multipleOfN(size_t, size_t);
public:
    DelaySIMD(const double);
    void updateDelay(float*, const int) noexcept;
    void setOffset(const double) noexcept;
    void setDryWet(const float) noexcept;
    void setFeedback(const float) noexcept;
    void flushDelayBuffers() noexcept;
};

inline void DelaySIMD::setOffset(const double time) noexcept
{
    const int tm = static_cast<int>(time);
    mReadIndex[0] = multipleOfN(((mWriteIndex[0] - ms2samples(tm)) & delay_buff_mask), 4);
    mReadIndex[1] = multipleOfN(((mWriteIndex[1] - ms2samples(tm)) & delay_buff_mask), 4);
}

inline void DelaySIMD::setDryWet(const float dw) noexcept
{
    const float neg = 1 - dw;
    for (int i = 0; i < 4; i++) {
        dCoeffs.mWet[i] = dw;
        dCoeffs.mDry[i] = neg;
    }
}

inline void DelaySIMD::setFeedback(const float fb) noexcept
{
    for (int i = 0; i < 4; i++)
        dCoeffs.mFb[i] = fb;
}

inline size_t DelaySIMD::ms2samples(const int ms) noexcept
{
    const double sr = (audio_tools::SAMPLE_RATE) / 1000.0;
    return static_cast<size_t>(static_cast<double>(ms) * sr);
}

template<typename Width>
Width DelaySIMD::findNextPow2(Width v) noexcept
{
    --v;
    for (uint32_t i = 1, maxShift = sizeof (Width) << 3; i <= maxShift; i = i << 1){
        v |= v >> i;
    }
    ++v;
    return v;
}

inline size_t DelaySIMD::multipleOfN(size_t sRate, size_t N)
{
    const size_t adjust = N - (sRate & (N - 1));
       return (sRate + adjust);
}

inline void DelaySIMD::flushDelayBuffers() noexcept
{
    memset(delayBuffer.data(), 0, sizeof (float) * delay_buff_size * 2);
}
#endif // DELAY_H


