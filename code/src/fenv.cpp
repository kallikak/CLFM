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

//using namespace std;

#include <Arduino.h>

#include <stdlib.h>
#include <math.h>

#include "synth.h"
#include "fenv.h"

// #define DEBUG

#define CHKOP(op) (op_ == op)

#define CHK0 CHKOP(0)
#define CHK1 CHKOP(1)
#define CHK2 CHKOP(2)
#define CHK3 CHKOP(3)
#define CHKALL(op) (1)

#define CHECK (CHK3 || CHK2)

uint32_t FEnv::sr_multiplier = 44100 >> 5;

const int levellut[] = {
  0, 5, 9, 13, 17, 20, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 42, 43, 45, 46
};

static const char *adsr[] = {"A", "D", "S", "R", "-"};

void FEnv::init_sr(double sampleRate) {
  sr_multiplier = (int)sampleRate >> 6; // converts it to seconds
}

// max durations in seconds
#define AMAX 10
#define DMAX 30
#define RMAX 30

void FEnv::calcCounts() {

  float attack = a_ * AMAX;  // seconds
  float decay = d_ * DMAX;   // seconds
  int sustain = s_; // level (0-99)
  float release = r_ * RMAX; // seconds
  // attack is 0 to 99 in "attack" millis => attack * sampleRate
  counts_[0] = attack * sr_multiplier;
  counts_[1] = sustain >= 0.9999 ? 0 : decay * sr_multiplier;
  counts_[2] = 2147483647 - 100;  // avoid overflow
  counts_[3] = drone_ ? counts_[2] : release * sr_multiplier;

  minlevel = 0;
  maxlevel = op_ < 4 ? 1.0 : 0;
#ifdef DEBUG
  if (CHECK) 
    Serial.printf("### calcCounts %d [outlevel=%d]: ADSR=%.3f %.3f %.3f %.3f => %.2f %.2f %d %.2f [%d %d - %d]\n", 
      op_, outlevel_, a_, d_, s_, r_, attack, decay, sustain, release, counts_[0], counts_[1], counts_[3]);
#endif      
}

void FEnv::init(const int a, const int d, const int s, const int r, bool drone, int ol) {
  a_ = a / 99.0;
  d_ = d / 99.0;
  s_ = s / 99.0;
  r_ = r / 99.0;
  drone_ = drone;
  
  outlevel_ = ol;
  outlevelfactor_ = 1;
  outleveldiff_ = 0;
  tempoutlevel_ = ol;

  calcCounts();
  
  if (drone_ || !down_ || ix_ == 4)
  {
    count_ = 0;
    ix_ = 4;
    advance(0);
  }

  down_ = true;
}

bool FEnv::debugenv() {
  if (count_ && count_ % 200 == 0) {
    if (ix_ >= 0 && ix_ < 4) {
      Serial.printf("%d.%s[%d/%d]\t", op_, adsr[ix_], count_, counts_[ix_]);
      return true;
    }
    else if (ix_ != 4)
      Serial.printf("op %d has index %d\n", op_, ix_);
  }
  return false;
}

int32_t FEnv::getsample() {
    if (drone_ || down_ || ix_ == 3)
    {
      count_++;
#ifdef DEBUG
      if (CHECK && count_ % 200 == 0)
        Serial.printf("%d %s: %d/%d\t%f/%f\n", op_, adsr[ix_], count_, counts_[ix_], level_, targetlevel_);
#endif        

      if (ix_ != 2 && count_ > counts_[ix_])
      {
        advance(ix_ + 1);
      }
      else if (ix_ == 2 && inc_ && abs(level_ - targetlevel_) < inc_) {
        level_ = targetlevel_;
        inc_ = 0;
      }
      else {
        level_ += inc_;
      }

      if (level_ >= maxlevel)
        level_ = maxlevel;
      else if (level_ <= minlevel) {
        level_ = minlevel;
        if (ix_ == 3) {
          advance(4);
          return 0;
        }
      }

      float flevel = level_ * 0.6 + 0.4;
      if (ix_ == 0)
        flevel = sqrt(flevel);
      if (flevel < 0.41)
        flevel = 0;
      float outflevel;

      if (outlevelfactor_ >= 1)
      {
        outlevel_ += outleveldiff_;
        outleveldiff_ = 0;
        outflevel = flevel * outlevel_;
      }
      else
      {
        outlevelfactor_ += outlevelfactordelta_;
        tempoutlevel_ = (int)(outlevel_ + outlevelfactor_ * outleveldiff_ + 0.5);
        outflevel = flevel * tempoutlevel_;
      }
        
      int32_t outlevel = (int32_t)(outflevel * (1 << 16));
      return outlevel;
    }
    else
    {
      return 0;
    }
}

void FEnv::keydown(bool d) {
  if (down_ != d) {
    down_ = d;
    if (!drone_)
      advance(d ? 0 : 3);
  }
}

void FEnv::silence(bool release) {
  advance(release ? 3 : 4);
}

bool FEnv::isDroning() {
  return drone_;
}

int FEnv::scaleoutlevel(int outlevel) {
  return outlevel >= 20 ? 28 + outlevel : levellut[outlevel];
}

int FEnv::convertLevel(int level)
{
  level = scaleoutlevel(level) >> 1;
  level = (level << 6) - 500;
  // level = (level << 6) + outlevel_ - 4256;
  level = level < 16 ? 16 : level;
  level <<= 16;
  return level;
}

float FEnv::constrainlevel(float level) {
  return level >= maxlevel ? maxlevel : (level < minlevel ? minlevel : level);
}

void FEnv::advance(int newix) {
  if (newix < 0 || newix > 4) {
    Serial.printf("Invalid index: %d [op %d - index %d]\n", newix, op_, ix_);
    newix = 4;
  }
  bool updateonly = ix_ == newix;
  if (updateonly && ix_ == 4)
    return; // nothing to do
  if (!updateonly)
  {
    ix_ = newix;
    if (ix_ < 4 && !counts_[ix_])
    {
      advance(ix_ + 1);
      return;
    }
    count_ = 0;
  }
  float startlevel = 0;
  switch (ix_)
  {
    case 0: // ATTACK
      startlevel = level_;
      targetlevel_ = maxlevel;
      break;
    case 1: // DECAY
      if (updateonly)
        startlevel = level_;
      else
        startlevel = maxlevel;
      targetlevel_ = s_;
      break;
    case 2: // SUSTAIN
      if (updateonly)
        startlevel = level_;
      else
        startlevel = s_;
      targetlevel_ = s_;
      break;
    case 3: // RELEASE
      startlevel = level_;
      targetlevel_ = minlevel;
      break;
    case 4:
      startlevel = 0;
      targetlevel_ = maxlevel; // inc_ will be zero so will never advance on its own
      break;
  }
  startlevel = constrainlevel(startlevel);
  targetlevel_ = constrainlevel(targetlevel_);

  level_ = startlevel;

  if (ix_ == 4 || (ix_ == 3 && drone_))
    inc_ = 0;
  else if (ix_ == 2)
    inc_ = (targetlevel_ - startlevel) / 64;
  else
    inc_ = (targetlevel_ - level_) / counts_[ix_];
#ifdef DEBUG
  if (CHECK) 
    Serial.printf("### Advancing %d to %s [%f => %f %f] %.3fs %d\n", 
      op_, adsr[ix_], level_, targetlevel_, inc_, millis() / 1000.0, outlevel_);
#endif      
}

void FEnv::drone(bool set) {
  drone_ = set;
  counts_[3] = drone_ ? counts_[2] : r_ * sr_multiplier;
  ix_ = -1;
  advance(drone_ || down_ ? 2 : 4);
}

void FEnv::update(const int a, const int d, const int s, const int r, bool drone, int ol, bool refreshEnv) {

  a_ = a / 99.0;
  d_ = d / 99.0;
  s_ = s / 99.0;
  bool savedrone = drone_;
  drone_ = drone;
  r_ = r / 99.0;

  if (!savedrone && drone_)
  {
    advance(2);
    return;
  }

  outleveldiff_ = ol - tempoutlevel_;

  if (abs(outleveldiff_) > 12)
  {
    outlevelfactordelta_ = 1.0f / max(1, min(8 * 64, ceil(abs(outleveldiff_) / 4)));
    outlevelfactor_ = 0;
    outlevel_ = tempoutlevel_;
  }
  else
  {
    outlevelfactor_ = 1.0f;
    outlevelfactordelta_ = 0;
    tempoutlevel_ = ol;
    outlevel_ = ol;
  }
  
  // if ((savedrone != drone_) || refreshEnv)
  if (refreshEnv)
  {
    calcCounts();
    advance(ix_);
  }
}

void FEnv::getPosition(char *step) {
  *step = ix_;
}

void FEnv::transfer(FEnv &src) {
  for (int i = 0; i < 4; i++) {
    counts_[i] = src.counts_[i];
  }
  a_ = src.a_;
  d_ = src.d_;
  s_ = src.s_;
  r_ = src.r_;
  drone_ = src.drone_;
  op_ = src.op_;
  outlevel_ = src.outlevel_;
  level_ = src.level_;
  targetlevel_ = src.targetlevel_;
  ix_ = src.ix_;
  down_ = src.down_;
  inc_ = src.inc_;
  outlevelfactor_ = src.outlevelfactor_;
  outlevelfactordelta_ = src.outlevelfactordelta_;
  outleveldiff_ = src.outleveldiff_;
  minlevel = src.minlevel;
  maxlevel = src.maxlevel;
}
