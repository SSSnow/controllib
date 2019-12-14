/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file blocks.h
 * @author snowpang
 * @em 18516614106@163.com
 * Controller library code
 */

#pragma once

#include "block/Block.hpp"
#include "block/BlockParam.hpp"

namespace control
{
/**
 * A sweep signal generator
 */
class __EXPORT BlockSweep : public Block
{
public:
// methods
	BlockSweep(SuperBlock *parent, const char *name) :
		Block(parent, name){}

    
    /* bandwith factor: k(t) = exp(freq_f*t/T - 1)/exp(freq_f)
     * amplitude factor: n(t) = exp(amp_f*t/T - 1)/exp(amp_f)   
     * bandwith: w(t) = bw_min + k(t) * (bw_max - bw_min)
     * angle: theta(t) = integrate(w(t))
     * sweep signal: sweep(t) = A * (1 - n(t))sin(theta(t)) 
     * /

// members
protected:
    float _max_bandwith;    //2.5*bandwith_180 the bw_180 is the bandwith which is the phase error at 180deg
    float _min_bandwith;    //0.5*bandwith_135 the bw_135 is the common bandwith which is the phase error at 135deg
    float _amplitude;       //sweep signal 
    float _freq_factor;     //the sin frequency exponential growth factor
    float _amp_factor;      //the amplitude exponential decline factor
    float _duration;        // the signal generate duration


}

}