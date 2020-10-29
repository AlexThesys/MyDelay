#include "../include/delay.h"

DelaySIMD::DelaySIMD(const double sr)
{
    delay_buff_size = findNextPow2(static_cast<size_t>(sr*2.0));	// sr*2 = 2 sec
    delay_buff_mask = delay_buff_size - 4;	// -1
    delayBuffer.resize(delay_buff_size * 2);
    memset(&dCoeffs, 0, sizeof(DCoeffs));
    memset(mWriteIndex, 0, sizeof (size_t)*2);
}

void DelaySIMD::updateDelay(float* buffer, const int ch) noexcept
{
    // y(n) = (1.0f-dryWet) * x(n) + dryWet*(x(n-D) + fb*y(n-D);

    __m128 input = _mm_load_ps(buffer);

    __m128 delayed = _mm_load_ps(&(delayBuffer[mReadIndex[ch]*(ch+1)]));

    __m128 xmm = _mm_load_ps(dCoeffs.mFb);
    __m128 yn = _mm_mul_ps(delayed, xmm);
    yn = _mm_add_ps(yn, input);
    _mm_store_ps(&(delayBuffer[mWriteIndex[ch]*(ch+1)]), yn);

    xmm = _mm_load_ps(dCoeffs.mWet);
    delayed = _mm_mul_ps(delayed, xmm);
    xmm = _mm_load_ps(dCoeffs.mDry);
    input = _mm_mul_ps(input, xmm);
    input = _mm_add_ps(delayed, input);
    _mm_store_ps(buffer, input);

    mReadIndex[ch] = (mReadIndex[ch] + 4) & delay_buff_mask;
    mWriteIndex[ch] = (mWriteIndex[ch] + 4) & delay_buff_mask;
}