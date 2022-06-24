/*
   Copyright 2017 Pascal Gauthier.
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

#ifndef __FENV_H
#define __FENV_H

#include "synth.h"

// DX7 envelope generation

class FEnv {
  public:
    void setop(int op) { op_ = op; }

    // The rates and levels arrays are calibrated to match the Dx7 parameters
    // (ie, value 0..99). The outlevel parameter is calibrated in microsteps
    // (ie units of approx .023 dB), with 99 * 32 = nominal full scale. 
    void init(const int a, const int d, const int s, const int r, bool drone,
              int outlevel);

    void drone(bool set);
    void update(const int a, const int d, const int s, const int r, bool drone,
                int outlevel, bool refreshEnv);
    // Result is in Q24/doubling log format. Also, result is subsampled
    // for every N samples.
    // A couple more things need to happen for this to be used as a gain
    // value. First, the # of outputs scaling needs to be applied. Also,
    // modulation.
    // Then, of course, log to linear.
    int32_t getsample();

    void keydown(bool down);
    bool keyisdown() { return down_; }
    static int scaleoutlevel(int outlevel);
    void getPosition(char *step);
    bool isDroning();
    void silence(bool release);

    static void init_sr(double sample_rate);
    void transfer(FEnv &src);

    bool debugenv();

  private:

    int convertLevel(int level);
    float constrainlevel(float level);
    void calcCounts();
    void advance(int newix);

    // PG: This code is normalized to 44100, need to put a multiplier
    // if we are not using 44100.
    static uint32_t sr_multiplier;

    int op_;
    
    float a_, d_, s_, r_;
    bool drone_ = false;
    int outlevel_;
    // Level is stored so that 2^24 is one doubling, ie 16 more bits than
    // the DX7 itself (fraction is stored in level rather than separate
    // counter)
    float level_ = 0;
    float targetlevel_;
    int ix_ = 4;
    float inc_;
    int tempoutlevel_;
    int outleveldiff_ = 0;
    float outlevelfactor_;
    float outlevelfactordelta_ = 1.0f / 8;
    int count_ = 0;
    int counts_[4];
    float minlevel;
    float maxlevel; 

    bool down_ = false;
};

#endif
