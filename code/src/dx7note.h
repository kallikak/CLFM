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

#ifndef SYNTH_DX7NOTE_H_
#define SYNTH_DX7NOTE_H_

// This is the logic to put together a note from the MIDI description
// and run the low-level modules.

// It will continue to evolve a bit, as note-stealing logic, scaling,
// and real-time control of parameters live here.

#include <stdint.h>
#include "fenv.h"
#include "fm_core.h"

struct VoiceStatus {
  uint32_t amp[4];
  char ampStep[4];
  char pitchStep;
};

class Dx7Note {
  public:
    Dx7Note();
    void init(uint8_t algorithm, float midinote, int velocity);

    // Note: this _adds_ to the buffer. Interesting question whether it's
    // worth it...
    void compute(int32_t *buf, FmCore* core);

    void keyup();

    // TODO: some way of indicating end-of-note. Maybe should be a return
    // value from the compute method? (Having a count return from keyup
    // is also tempting, but if there's a dynamic parameter change after
    // keyup, that won't work.

    // PG:add the update
    void update(uint8_t algorithm, float midinote, int velocity, bool refreshEnv);
    void setOPDrone(uint8_t op, bool set);
    void updatePitchOnly(float pitch);
    void updateEnv(float midinote, int velocity);
    void peekVoiceStatus(VoiceStatus &status);
    void transferState(Dx7Note& src);
    void transferSignal(Dx7Note &src);
    void oscSync();

  private:
    FEnv env_[4];
    FmOpParams params_[4];
    int32_t basepitch_[4];
    int32_t fb_buf_[2];
    int32_t fb_shift_;
    int32_t opMode[4];
    int algorithm_;
};

#endif
