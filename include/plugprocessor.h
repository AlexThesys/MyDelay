//------------------------------------------------------------------------
// Project     : VST SDK
//
// Category    : Examples
// Filename    : plugprocessor.h
// Created by  : Steinberg, 01/2018
// Description : HelloWorld Example for VST 3
//
//-----------------------------------------------------------------------------
// LICENSE
// (c) 2018, Steinberg Media Technologies GmbH, All Rights Reserved
//-----------------------------------------------------------------------------
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// 
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation 
//     and/or other materials provided with the distribution.
//   * Neither the name of the Steinberg Media Technologies nor the names of its
//     contributors may be used to endorse or promote products derived from this 
//     software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

#pragma once

#include "public.sdk/source/vst/vstaudioeffect.h"
#include "delay.h"
#include "audiotools.h"
#include <functional>
#include <cassert>
#include <memory>
#include "public.sdk/samples/vst/common/logscale.h"

namespace Steinberg {
namespace MyDelay {

//-----------------------------------------------------------------------------
class PlugProcessor : public Vst::AudioEffect
{
public:
	PlugProcessor ();

	tresult PLUGIN_API initialize (FUnknown* context) SMTG_OVERRIDE;
	tresult PLUGIN_API setBusArrangements (Vst::SpeakerArrangement* inputs, int32 numIns,
	                                       Vst::SpeakerArrangement* outputs, int32 numOuts) SMTG_OVERRIDE;
    tresult PLUGIN_API canProcessSampleSize(int32 symbolicSampleSize) SMTG_OVERRIDE;

	tresult PLUGIN_API setupProcessing (Vst::ProcessSetup& setup) SMTG_OVERRIDE;
	tresult PLUGIN_API setActive (TBool state) SMTG_OVERRIDE;
	tresult PLUGIN_API process (Vst::ProcessData& data) SMTG_OVERRIDE;

//------------------------------------------------------------------------
	tresult PLUGIN_API setState (IBStream* state) SMTG_OVERRIDE;
	tresult PLUGIN_API getState (IBStream* state) SMTG_OVERRIDE;

	static FUnknown* createInstance (void*)
	{
		return (Vst::IAudioProcessor*)new PlugProcessor ();
	}

    template<typename FloatType>
    void processAudio(FloatType* in, FloatType* out, int numSamples, int ch);

protected:
    typedef struct
    {
       audio_tools::ParamSmoothing<double> dryWetSmoother;
       audio_tools::ParamSmoothing<double> timeSmoother;
       audio_tools::ParamSmoothing<double> feedbackSmoother;
    } ParamSmoothers;
    ParamSmoothers paramSmoothers;
    std::unique_ptr<DelaySIMD> m_pDelay;

   //--------------------------
    Vst::ParamValue mDelayDryWet;
    Vst::ParamValue mDelayTime;
    Vst::ParamValue mDelayFB;
    bool mBypass;
    bool m_isSampleSize64;
    //----------------------------
private:
    char padding[6];
    using ProcFunc = void(*)(Vst::ProcessData& data, int32 numChannels, PlugProcessor* processor);
    typedef  void(*BypassFunc)(Vst::ProcessData& data, int32 numChannels);
    ProcFunc procFunc;
    BypassFunc bypassFunc;

//    void bypassed32(Vst::ProcessData& data, int32 numChannels);
//    void bypassed64(Vst::ProcessData& data, int32 numChannels);
};

inline void bypassed32(Vst::ProcessData& data, int32 numChannels)
{

    for (int32 channel = 0; channel < numChannels; channel++)
    {
        float* inputChannel = data.inputs[0].channelBuffers32[channel];
        float* outputChannel = data.outputs[0].channelBuffers32[channel];

        for (int32 sample = 0; sample < data.numSamples; sample++)
        {
            outputChannel[sample] = inputChannel[sample];
        }
    }
}

inline void bypassed64(Vst::ProcessData& data, int32 numChannels)
{

    for (int32 channel = 0; channel < numChannels; channel++)
    {
        double* inputChannel = data.inputs[0].channelBuffers64[channel];
        double* outputChannel = data.outputs[0].channelBuffers64[channel];

        for (int32 sample = 0; sample < data.numSamples; sample++)
        {
            outputChannel[sample] = inputChannel[sample];
        }
    }
}

//------------------------------------------------------------------------
} // namespace
} // namespace Steinberg
