/*
   Copyright 2012 Google Inc.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include <Arduino.h>

#include <math.h>
#include <cstdlib>

#include "../CLFM.h"

#include "synth.h"  
#include "wavetables.h"
#include "fm_op_kernel.h"

// wavefolders for the all operators are carriers algorithm 
int32_t calcfold(int32_t v, float foldamount) {
  const long thresh = 1 << 24;
  if (foldamount > 0)
  {
    float vf = (1.0 * v) / thresh;
    vf = (1.0 + foldamount) * vf;
    vf = (vf > 0 ? 1 : -1) * 2 * fabs(vf / 2 - round(vf / 2));
    return (int32_t)(vf * thresh);
  }
  return v;
}

#define MAXFOLD 8.0

int32_t getRaw(int32_t phase, wavetype wave, int32_t fold) {
  switch (wave) {
    case SIN:
    default:
      return Sin::lookup(phase);
    case TRI:
      return Tri::lookup(phase);
    case SQR:
      return Sqr::lookup(phase);
    case SINFOLD:
    {
      float foldamount = fold > 0 ? MAXFOLD * fold / 50.0 : 0;
      return calcfold(Sin::lookup(phase), foldamount);
    }
    case TRIFOLD:
    {
      float foldamount = fold > 0 ? MAXFOLD * fold / 50.0 : 0;
      return calcfold(Tri::lookup(phase), foldamount);
    }
  }
}

void FmOpKernel::compute(int32_t *output, const int32_t *input,
                         int32_t phase0, int32_t freq, wavetype wave,
                         int16_t fold, int32_t gain1, int32_t gain2, bool add) {
  int32_t dgain = (gain2 - gain1 + (_N_ >> 1)) >> LG_N;
  int32_t gain = gain1;
  int32_t phase = phase0;
  if (add) {
    for (int i = 0; i < _N_; i++) {
      gain += dgain;
      int32_t y = getRaw(phase + input[i], wave, fold);
      int32_t y1 = ((int64_t)y * (int64_t)gain) >> 24;
      output[i] += y1;
      phase += freq;
    }
  } else {
    for (int i = 0; i < _N_; i++) {
      gain += dgain;
      int32_t y = getRaw(phase + input[i], wave, fold);
      int32_t y1 = ((int64_t)y * (int64_t)gain) >> 24;
      output[i] = y1;
      phase += freq;
    }
  }
}

void FmOpKernel::compute_pure(int32_t *output, int32_t phase0, 
                              int32_t freq, wavetype wave,
                              int16_t fold, int32_t gain1, int32_t gain2, bool add) {
  int32_t dgain = (gain2 - gain1 + (_N_ >> 1)) >> LG_N;
  int32_t gain = gain1;
  int32_t phase = phase0;
  if (add) {
    for (int i = 0; i < _N_; i++) {
      gain += dgain;
      int32_t y = getRaw(phase, wave, fold);
      int32_t y1 = ((int64_t)y * (int64_t)gain) >> 24;
      output[i] += y1;
      phase += freq;
    }
  } else {
    for (int i = 0; i < _N_; i++) {
      gain += dgain;
      int32_t y = getRaw(phase, wave, fold);
      int32_t y1 = ((int64_t)y * (int64_t)gain) >> 24;
      output[i] = y1;
      phase += freq;
    }
  }
}

#define noDOUBLE_ACCURACY
#define HIGH_ACCURACY

void FmOpKernel::compute_fb(int32_t *output, int32_t phase0, int32_t freq, 
                            wavetype wave, int16_t fold, int32_t gain1, int32_t gain2,
                            int32_t *fb_buf, int fb_shift, bool add) {
  int32_t dgain = (gain2 - gain1 + (_N_ >> 1)) >> LG_N;
  int32_t gain = gain1;
  int32_t phase = phase0;
  int32_t y0 = fb_buf[0];
  int32_t y = fb_buf[1];
  if (add) {
    for (int i = 0; i < _N_; i++) {
      gain += dgain;
      int32_t scaled_fb = (y0 + y) >> (fb_shift + 1);
      y0 = y;
      y = getRaw(phase + scaled_fb, wave, fold);
      y = ((int64_t)y * (int64_t)gain) >> 24;
      output[i] += y;
      phase += freq;
    }
  } else {
    for (int i = 0; i < _N_; i++) {
      gain += dgain;
      int32_t scaled_fb = (y0 + y) >> (fb_shift + 1);
      y0 = y;
      y = getRaw(phase + scaled_fb, wave, fold);
      y = ((int64_t)y * (int64_t)gain) >> 24;
      output[i] = y;
      phase += freq;
    }
  }
  fb_buf[0] = y0;
  fb_buf[1] = y;
}