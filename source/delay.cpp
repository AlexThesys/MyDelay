#include "../include/delay.h"


Delay::Delay(const double sr) : extFB(_mm_setzero_ps())
{
    delay_buff_size = findNextPow2(static_cast<size_t>(sr*2.0));	// sr*2 = 2 sec
    delay_buff_mask = delay_buff_size - 4;	// -1
    delayBuffer[0] = std::make_unique<std::vector<float>>(delay_buff_size);
    delayBuffer[1] = std::make_unique<std::vector<float>>(delay_buff_size);
    memset(&dCoeffs, 0, sizeof(DCoeffs));
    memset(mWriteIndex, 0, sizeof (size_t)*2);
}

void Delay::updateDelay(float* buffer, const int ch) noexcept
{
    // y(n) = (1.0f-dryWet) * x(n) + dryWet*(x(n-D) + fb*y(n-D);

    __m128* output = reinterpret_cast<__m128*>(buffer);
    __m128 input = *output;

    __m128 delayed = _mm_load_ps(&(delayBuffer[ch]->at(mReadIndex[ch])));
    // code duplication vs branchless
    __m128 yn = _mm_mul_ps(delayed, dCoeffs.mFb);
    yn = _mm_add_ps(yn, input);
    _mm_store_ps(&(delayBuffer[ch]->at(mWriteIndex[ch])), yn);
    // -------------------------------
    delayed = _mm_mul_ps(delayed, dCoeffs.mWet);
    input = _mm_mul_ps(input, dCoeffs.mDry);
    *output = _mm_add_ps(delayed, input);

    mReadIndex[ch] = (mReadIndex[ch] + 4) & delay_buff_mask;
    mWriteIndex[ch] = (mWriteIndex[ch] + 4) & delay_buff_mask;
}

void Delay::updateDelayExtFB(float* buffer, const int ch) noexcept
{

    __m128* output = reinterpret_cast<__m128*>(buffer);
    __m128 input = *output;

    __m128 delayed = _mm_load_ps(&(delayBuffer[ch]->at(mReadIndex[ch])));
    // code duplication vs branchless
    extFB = _mm_add_ps(extFB, input);
    _mm_store_ps(&(delayBuffer[ch]->at(mWriteIndex[ch])), extFB);
    // ----------------------------------
    delayed = _mm_mul_ps(delayed, dCoeffs.mWet);
    input = _mm_mul_ps(input, dCoeffs.mDry);
    *output = _mm_add_ps(delayed, input);

    mReadIndex[ch] = (mReadIndex[ch] + 4) & delay_buff_mask;
    mWriteIndex[ch] = (mWriteIndex[ch] + 4) & delay_buff_mask;
}
