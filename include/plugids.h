//------------------------------------------------------------------------
// Project     : VST SDK
//
// Category    : Examples
// Filename    : plugids.h
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

#include "public.sdk/source/vst/vstparameters.h"

namespace Steinberg {
namespace MyDelay {

// HERE are defined the parameter Ids which are exported to the host
enum MyDelayParams : Vst::ParamID
{
    kParamDelayDryWetID = 101,
    kParamDelayTimeID = 102,
    kParamDelayFeedbackID = 103,

    kBypassID = 104
};

namespace DelayConst
{
    static constexpr double DELAY_DRY_WET_MIN = 0.0;
    static constexpr double DELAY_DRY_WET_MAX = 1.0;
    static constexpr double DELAY_DRY_WET_DEFAULT = 0.5;
    static constexpr double DELAY_TIME_MS_MIN = 0.0;
    static constexpr double DELAY_TIME_MS_MAX = 2000.0;
    static constexpr double DELAY_TIME_MS_DEFAULT = 80.0;
    static constexpr double DELAY_FEEDBACK_MIN = 0.0;
    static constexpr double DELAY_FEEDBACK_MAX = 0.95;
    static constexpr double DELAY_FEEDBACK_DEFAULT = 0.4;
};


// HERE you have to define new unique class ids: for processor and for controller
// you can use GUID creator tools like https://www.guidgenerator.com/
static const FUID MyProcessorUID (0x316a13bd, 0x0fcb405d, 0xb4e7bd73, 0x7b7e8f9d);
static const FUID MyControllerUID (0x315e2651, 0x34784bcc, 0x8247f272, 0x1796e51a);

//------------------------------------------------------------------------
} // namespace HelloWorld
} // namespace Steinberg



