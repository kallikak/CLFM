/*
   MicroDexed

   MicroDexed is a port of the Dexed sound engine
   (https://github.com/asb2m10/dexed) for the Teensy-3.5/3.6/4.x with audio shield.
   Dexed ist heavily based on https://github.com/google/music-synthesizer-for-android

   (c)2018-2021 H. Wirtz <wirtz@parasitstudio.de>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

*/

#ifndef DEXED_H_INCLUDED
#define DEXED_H_INCLUDED

#include <stdint.h>
#include <stdlib.h>
#if defined(TEENSYDUINO)
#include <Audio.h>
#endif
#include "fm_op_kernel.h"
#include "synth.h"
#include "fenv.h"
#include "aligned_buf.h"
#include "dx7note.h"

#define NUM_VOICE_PARAMETERS 156

struct ProcessorVoice {
  int16_t midi_note;
  uint8_t velocity;
  int16_t porta;
  bool keydown;
  bool live;
  uint32_t key_pressed_timer;
  Dx7Note *dx7_note;
};

enum ADSR {
  ATTACK,
  DECAY,
  SUSTAIN,
  RELEASE
};

enum OPERATORS {
  OP1,
  OP2,
  OP3,
  OP4,
  OP5,
  OP6
};

enum ON_OFF {
  OFF,
  ON
};

// GLOBALS

//==============================================================================

class Dexed
{
  public:
    Dexed(uint8_t maxnotes, int rate);
    ~Dexed();

    // Global methods
    void activate(void);
    void deactivate(void);
    bool getMonoMode(void);
    void setMonoMode(bool mode);
    void setRefreshMode(bool mode);
    void setMaxNotes(uint8_t n);
    uint8_t getMaxNotes(void);
    void doRefreshVoice(void);
    void doRefreshEnv();
    uint8_t getNumNotesPlaying(void);

    // Sound methods
    void keyup(int16_t pitch);
    void keydown(int16_t pitch, uint8_t velo);
    void freq(float fracpitch, uint8_t velo);
    void updatePitchOnly(float pitch);
    void panic(void);
    void notesOff(void);

    void setOPDrone(uint8_t op, bool set);
    void setAlgorithm(uint8_t algorithm);
    uint8_t getAlgorithm(void);
    uint8_t getCarrierCount(void);
    bool isIdle();
    bool isReleasing();

    ProcessorVoice voices[4];

  protected:
    uint8_t max_notes;
    int16_t currentNote;
    float vuSignal;
    bool monoMode;
    bool refreshMode;
    bool refreshVoice;
    bool refreshEnv;
    uint8_t engineType;
    uint8_t algorithm;
    uint32_t xrun;
    uint16_t render_time_max;
    VoiceStatus voiceStatus;
    FmCore* engineMsfa;
    void getSamples(uint16_t n_samples, int16_t* buffer);
};

#endif
