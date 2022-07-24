
/*
   Copyright 2016-2017 Pascal Gauthier.
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
#include <stdlib.h>

#include "../CLFM.h"

#include "synth.h"
#include "freqlut.h"
#include "exp2.h"
#include "dx7note.h"

// #define DEBUG

int32_t midinote_to_logfreq(int midinote) {
  //const int32_t base = 50857777;  // (1 << 24) * (log(440) / log(2) - 69/12)
  const int32_t base = 50857777;  // (1 << 24) * (LOG_FUNC(440) / LOG_FUNC(2) - 69/12)
  const int32_t step = (1 << 24) / 12;
  return base + step * midinote;
}

int32_t logfreq_round2semi(int freq) {
  const int base = 50857777;  // (1 << 24) * (log(440) / log(2) - 69/12)
  const int step = (1 << 24) / 12;
  const int rem = (freq - base) % step;
  return freq - rem;
}

// const int32_t coarsemul[] = {
//   -16777216, 0, 16777216, 26591258, 33554432, 38955489, 43368474, 47099600,
//   50331648, 53182516, 55732705, 58039632, 60145690, 62083076, 63876816,
//   65546747, 67108864, 68576247, 69959732, 71268397, 72509921, 73690858,
//   74816848, 75892776, 76922906, 77910978, 78860292, 79773775, 80654032,
//   81503396, 82323963, 83117622
// };

const int32_t coarsemul[] = {
  -117440512, -100663296, -83886080, -67108864, -50331648, -33554432, -16777216, 
  0, 16777216, 26591258, 33554432, 38955489, 43368474, 47099600,
  50331648, 53182516, 55732705, 58039632, 60145690, 62083076, 63876816,
  65546747, 67108864, 68576247, 69959732, 71268397, 72509921, 73690858,
  74816848, 75892776, 76922906, 77910978, 78860292, 79773775, 80654032,
  81503396, 82323963, 83117622
};

// ScaleLevel: break_pt = 33, left_depth = 0, right_depth = 0, left_curve = 0, right_curve = 0
// ScaleCurve: group = 17, depth = 0, curve = 0
// ScaleVelocity: velocity = 0, sensitivity = 0
#define BREAK_PT 33
#define DEF_DEPTH 0
#define SENSITIVITY 7

int32_t osc_freq(float midinote, int mode, int coarse, int fine, int detune) {
  // TODO: pitch randomization
  int32_t logfreq;
  int rounded = (int)midinote;
  if (mode == 0) {  // ratio mode
    logfreq = midinote_to_logfreq(rounded);
    float f = midinote - (int)midinote;
    if (f > 0) 
    {
      int32_t logfreq1 = midinote_to_logfreq(rounded + 1);
      logfreq += f * (logfreq1 - logfreq);
    }

    if (detune) // detune from -7 to 7
    {
      // could use more precision, closer enough for now. those numbers comes from my DX7
      //FRAC_NUM detuneRatio = 0.0209 * exp(-0.396 * (((float)logfreq) / (1 << 24))) / 7;
      FRAC_NUM detuneRatio = 0.0209 * EXP_FUNC(-0.396 * (((float)logfreq) / (1 << 24))) / 7;
      logfreq += detuneRatio * logfreq * detune;
    }

    logfreq += coarsemul[coarse & 31];
    if (fine) {
      // (1 << 24) / log(2)
      //logfreq += (int32_t)floor(24204406.323123 * log(1 + 0.01 * fine) + 0.5);
      logfreq += (int32_t)floor(24204406.323123 * LOG_FUNC(1 + 0.01 * fine) + 0.5);
    }

    // // This was measured at 7.213Hz per count at 9600Hz, but the exact
    // // value is somewhat dependent on midinote. Close enough for now.
    // //logfreq += 12606 * (detune -7);
  } else {  // fixed mode
    // ((1 << 24) * log(10) / log(2) * .01) << 3
    logfreq = (4458616 * ((coarse & 3) * 100 + fine)) >> 3;
    logfreq += detune > 7 ? 13457 * detune : 0;
  }
  return logfreq;
}

const uint8_t velocity_data[64] = {
  0, 70, 86, 97, 106, 114, 121, 126, 132, 138, 142, 148, 152, 156, 160, 163,
  166, 170, 173, 174, 178, 181, 184, 186, 189, 190, 194, 196, 198, 200, 202,
  205, 206, 209, 211, 214, 216, 218, 220, 222, 224, 225, 227, 229, 230, 232,
  233, 235, 237, 238, 240, 241, 242, 243, 244, 246, 246, 248, 249, 250, 251,
  252, 253, 254
};

// See "velocity" section of notes. Returns velocity delta in microsteps.
int ScaleVelocity(int velocity, int sensitivity) {
  int clamped_vel = max(0, min(127, velocity));
  int vel_value = velocity_data[clamped_vel >> 1] - 239;
  int scaled_vel = ((sensitivity * vel_value + 7) >> 3) << 4;
  return scaled_vel;
}

const uint8_t exp_scale_data[] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 14, 16, 19, 23, 27, 33, 39, 47, 56, 66,
  80, 94, 110, 126, 142, 158, 174, 190, 206, 222, 238, 250
};

int ScaleCurve(int group, int depth, int curve) {
  int scale;
  if (curve == 0 || curve == 3) {
    // linear
    scale = (group * depth * 329) >> 12;
  } else {
    // exponential
    int n_scale_data = sizeof(exp_scale_data);
    int raw_exp = exp_scale_data[min(group, n_scale_data - 1)];
    scale = (raw_exp * depth * 329) >> 15;
  }
  if (curve < 2) {
    scale = -scale;
  }
  return scale;
}

int ScaleLevel(int midinote, int break_pt, int left_depth, int right_depth,
               int left_curve, int right_curve) {
  int offset = midinote - break_pt - 17;
  if (offset >= 0) {
    return ScaleCurve((offset + 1) / 3, right_depth, right_curve);
  } else {
    return ScaleCurve(-(offset - 1) / 3, left_depth, left_curve);
  }
}

static const uint8_t pitchmodsenstab[] = {
  0, 10, 20, 33, 55, 92, 153, 255
};

// 0, 66, 109, 255
static const uint32_t ampmodsenstab[] = {
  0, 4342338, 7171437, 16777216
};

Dx7Note::Dx7Note() {
  for (int op = 0; op < 4; op++) {
    params_[op].phase = 0;
    params_[op].gain_out = 0;
    env_[op].setop(op);
  }
}

void Dx7Note::init(uint8_t algorithm, float midinote, int velocity) {
  for (int op = 0; op < 4; op++) {
    int a = config.env[op].a;
    int d = config.env[op].d;
    int s = config.env[op].s;
    int r = config.env[op].r;
    int outlevel = min(100, config.level[op] * config.scale[op]);
    outlevel = FEnv::scaleoutlevel(outlevel);
    int level_scaling = ScaleLevel(midinote, BREAK_PT, DEF_DEPTH, DEF_DEPTH, DEF_DEPTH, DEF_DEPTH);
    outlevel += level_scaling;
    outlevel = min(127, outlevel);
    outlevel = outlevel << 5;
    outlevel += ScaleVelocity(velocity, SENSITIVITY);
    outlevel = max(0, outlevel);

    env_[op].init(a, d, s, r, env_[op].isDroning(), outlevel);

    int mode = 0;
    int coarse = (int)(config.coarse[op]);
    int fine = config.fold ? 0 : config.fine[op];
    int detune = config.detune;
    int32_t freq = osc_freq(midinote, mode, coarse, fine, detune);
    opMode[op] = mode;
    basepitch_[op] = freq;
  }
  algorithm_ = algorithm;
  fb_factor_ = config.feedback < 95 ? config.feedback / 300.0 : 1.5;
}

void Dx7Note::setOPDrone(uint8_t op, bool set) {
  bool isdroning = env_[op].isDroning();
  env_[op].drone(set);
  if (isdroning && !set && !env_[op].keyisdown()) {
    env_[op].keydown(false);
    env_[op].silence(true);
  }
}

void Dx7Note::compute(int32_t *buf, FmCore* core) {
#ifdef DEBUG
    int sum = 0;
    bool debugout = false;
#endif
  // ==== OP RENDER ====
  for (int op = 0; op < 4; op++) {
    // if ( ctrls->opSwitch[op] == '0' )  {
    // if (!(ctrls->opSwitch & (1 << op)))  {
    //int32_t gain = pow(2, 10 + level * (1.0 / (1 << 24)));

    int32_t basepitch = basepitch_[op];

    // if ( opMode[op] )
    //   params_[op].freq = Freqlut::lookup(basepitch + pitch_base);
    // else
      params_[op].freq = Freqlut::lookup(basepitch);

    uint32_t level = env_[op].getsample();
#ifdef DEBUG
    debugout = env_[op].debugenv() || debugout;
#endif

    params_[op].level_in = level;
    params_[op].fold = config.fold ? config.fine[op] : 0;
#ifdef DEBUG
    sum += (level >> 16);
#endif    
  }
#ifdef DEBUG
  if (debugout)
    Serial.printf("\n");
#endif    

#ifdef DEBUG
  if (sum > 0) {
    const char *adsr[] = {"A", "D", "S", "R", "-"};
    static int count = 0;
    const int filtercount = 50;
    if (count++ % filtercount == 0)
    {
      Serial.print("### Levels ### ");
      for (int op = 0; op < 4; op++) {
        char step;
        env_[op].getPosition(&step);
        Serial.printf("%d: %s %4u ", op, adsr[(int)step], params_[op].level_in >> 16);
      }
      Serial.println();
    }
  }
#endif

  core->render(buf, params_, algorithm_, fb_buf_, fb_factor_);
}

void Dx7Note::keyup() {
  for (int op = 0; op < 4; op++) {
    if (!env_[op].isDroning()) {
      // Serial.printf("Op %d: keyup\n", op);
      env_[op].keydown(false);
    }
    // else
    //   Serial.printf("Op %d: droning\n", op);
  }
}

void Dx7Note::updateEnv(float midinote, int velocity)
{
  // Serial.println("In updateEnv");
  for (int op = 0; op < 4; op++) {
    int a = config.env[op].a;
    int d = config.env[op].d;
    int s = config.env[op].s;
    int r = config.env[op].r;
    int outlevel = min(100, config.level[op] * config.scale[op]);
    outlevel = FEnv::scaleoutlevel(outlevel);
    int level_scaling = ScaleLevel(midinote, BREAK_PT, DEF_DEPTH, DEF_DEPTH, DEF_DEPTH, DEF_DEPTH);
    outlevel += level_scaling;
    outlevel = min(127, outlevel);
    outlevel = outlevel << 5;
    outlevel += ScaleVelocity(velocity, SENSITIVITY);
    outlevel = max(0, outlevel);
#ifdef DEBUG    
    Serial.printf("Update env op %d: %d %d %d %d (drone %d)\n", op, a, d, s, r, env_[op].isDroning());
#endif    
    env_[op].update(a, d, s, r, env_[op].isDroning(), outlevel, true);
  }
}

void Dx7Note::updatePitchOnly(float pitch)
{
  for (int op = 0; op < 4; op++) {
    int coarse = (int)(config.coarse[op]);
    int fine = config.fold ? 0 : config.fine[op];
    basepitch_[op] = osc_freq(pitch, 0, coarse, fine, config.detune);
  }
}

void Dx7Note::update(uint8_t algorithm, float midinote, int velocity, bool refreshEnv) {
  // Serial.println("In update");
  for (int op = 0; op < 4; op++) {
    int a = config.env[op].a;
    int d = config.env[op].d;
    int s = config.env[op].s;
    int r = config.env[op].r;
    int outlevel = min(100, config.level[op] * config.scale[op]);
    int mode = 0;
    int coarse = (int)(config.coarse[op]);
    int fine = config.fold ? 0 : config.fine[op];
    int detune = config.detune;
    int32_t freq = osc_freq(midinote, mode, coarse, fine, detune);
    basepitch_[op] = freq;
    opMode[op] = mode;

    outlevel = FEnv::scaleoutlevel(outlevel);
    int level_scaling = ScaleLevel(midinote, BREAK_PT, DEF_DEPTH, DEF_DEPTH, DEF_DEPTH, DEF_DEPTH);
    outlevel += level_scaling;
    outlevel = min(127, outlevel);
    outlevel = outlevel << 5;
    outlevel += ScaleVelocity(velocity, SENSITIVITY);
    outlevel = max(0, outlevel);
#ifdef DEBUG    
    Serial.printf("Refresh op %d: %d %d %d %d\n", op, a, d, s, r, refreshEnv);
#endif    
    env_[op].update(a, d, s, r, env_[op].isDroning(), outlevel, refreshEnv);
  }
  algorithm_ = algorithm;
  fb_factor_ = config.feedback < 95 ? config.feedback / 300.0 : 1.5;
}

void Dx7Note::peekVoiceStatus(VoiceStatus &status) {
  for (int i = 0; i < 4; i++) {
    status.amp[i] = Exp2::lookup(params_[i].level_in - (14 * (1 << 24)));
    env_[i].getPosition(&status.ampStep[i]);
  }
}

/**
   Used in monophonic mode to transfer voice state from different notes
*/
void Dx7Note::transferState(Dx7Note &src) {
  for (int i = 0; i < 4; i++) {
    env_[i].transfer(src.env_[i]);
    params_[i].gain_out = src.params_[i].gain_out;
    params_[i].phase = src.params_[i].phase;
  }
}

void Dx7Note::transferSignal(Dx7Note &src) {
  for (int i = 0; i < 4; i++) {
    params_[i].gain_out = src.params_[i].gain_out;
    params_[i].phase = src.params_[i].phase;
  }
}

void Dx7Note::oscSync() {
  for (int i = 0; i < 4; i++) {
    params_[i].gain_out = 0;
    params_[i].phase = 0;
  }
}
