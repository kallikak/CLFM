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
#include "Arduino.h"

#include "synth.h"
#include "dexed.h"
#include "fm_core.h"
#include "exp2.h"
#include "sin.h"
#include "freqlut.h"
#include <unistd.h>
#include <limits.h>
//#include <dsp/support_functions.h>
#include <arm_math.h>

// FIXME - there's a memory overwrite bug connected to the voices
Dexed::Dexed(uint8_t maxnotes, int rate)
{
  Exp2::init();
  Tanh::init();
  Sin::init();

  Freqlut::init(rate);
  FEnv::init_sr(rate);
  
  engineMsfa = new FmCore;
  max_notes=maxnotes;
  currentNote = 0;
  vuSignal = 0.0;
  // voices=NULL;

  setMaxNotes(max_notes);
  setMonoMode(false);
  // loadInitVoice();

  xrun = 0;
  render_time_max = 0;
}

Dexed::~Dexed()
{
  currentNote = -1;

  for (uint8_t note = 0; note < max_notes; note++)
    delete voices[note].dx7_note;

  for (uint8_t note = 0; note < max_notes; note++)
    delete &voices[note];

  delete(engineMsfa);
}

void Dexed::setMaxNotes(uint8_t new_max_notes)
{
  uint8_t i=0;
  
  max_notes=constrain(max_notes,0,_MAX_NOTES);

#ifdef DEBUG
  Serial.print("Allocating memory for ");
  Serial.print(max_notes,DEC);
  Serial.println(" notes.");
  Serial.println();
#endif

  if(voices)
  {
    panic();
    for (i = 0; i < max_notes; i++)
    {
      if(voices[i].dx7_note)
    	delete voices[i].dx7_note;
    }
    // delete(voices);
  }

  max_notes=constrain(new_max_notes,0,_MAX_NOTES);

  if(max_notes>0)
  {

    // voices=new ProcessorVoice[max_notes]; // sizeof(ProcessorVoice) = 20
    for (i = 0; i < max_notes; i++)
    {
      voices[i].dx7_note = new Dx7Note; // sizeof(Dx7Note) = 692
      voices[i].keydown = false;
      voices[i].live = false;
      voices[i].key_pressed_timer = 0;
    }
  }
  // else
  //    voices=NULL;
}

void Dexed::activate(void)
{
  panic();
}

void Dexed::deactivate(void)
{
  panic();
}

void Dexed::getSamples(uint16_t n_samples, int16_t* buffer)
{
  uint16_t i, j;
  uint8_t note;
  float sumbuf[n_samples];
#ifdef USE_SIMPLE_COMPRESSOR
  float s;
  const double decayFactor = 0.99992;
#endif

  if (refreshVoice)
  {
    // Serial.println("### refreshing voice");
    for (i = 0; i < max_notes; i++)
    {
      if ( voices[i].live ) {
        // Serial.println("### voice is live");
        voices[i].dx7_note->update(algorithm, voices[i].midi_note, voices[i].velocity, refreshEnv);
      }
      else {
        // Serial.println("### voice isn't live");
        voices[i].dx7_note->updateEnv(voices[i].midi_note, voices[i].velocity);
      }
    }
    refreshVoice = false;
    refreshEnv = false;
  }
  else if (refreshEnv)
  {
    for (i = 0; i < max_notes; i++)
      voices[i].dx7_note->updateEnv(voices[i].midi_note, voices[i].velocity);
    refreshEnv = false;
  }

  for (i = 0; i < n_samples; i += _N_)
  {
    AlignedBuf<int32_t, _N_> audiobuf;

    for (uint8_t j = 0; j < _N_; ++j)
    {
      audiobuf.get()[j] = 0;
      sumbuf[i + j] = 0.0;
    }

    for (note = 0; note < max_notes; note++)
    {
      if (voices[note].live)
      {
        // Serial.printf("Voice for note %d is live\n", note);
        voices[note].dx7_note->compute(audiobuf.get(), engineMsfa);

        for (j = 0; j < _N_; ++j)
        {
          sumbuf[i + j] += signed_saturate_rshift(audiobuf.get()[j] >> 4, 24, 9) / 32768.0;
          audiobuf.get()[j] = 0;
          /*
                    int32_t val = audiobuf.get()[j];
                    val = val >> 4;
                    int32_t clip_val = val < -(1 << 24) ? 0x8000 : val >= (1 << 24) ? 0x7fff : val >> 9;
                    float f = ((float) clip_val) / (float) 0x8000;
                    if ( f > 1.0 ) f = 1.0;
                    if ( f < -1.0 ) f = -1.0;
                    sumbuf[j] += f;
                    audiobuf.get()[j] = 0;
          */
        }
      }
    }
  }

#ifdef USE_SIMPLE_COMPRESSOR
  // mild compression
  for (i = 0; i < n_samples; i++)
  {
    s = abs(sumbuf[i]);
    if (s > vuSignal)
      vuSignal = s;
    //else if (vuSignal > 0.001f)
    else if (vuSignal > 0.0005f)
      vuSignal *= decayFactor;
    else
      vuSignal = 0.0;
  }
#endif

  //arm_scale_f32(sumbuf, 0.00015, sumbuf, AUDIO_BLOCK_SAMPLES);
  arm_float_to_q15(sumbuf, buffer, n_samples);
}

bool Dexed::isIdle() {
  return getNumNotesPlaying() == 0;
}

bool Dexed::isReleasing() {
  uint8_t op_carrier = engineMsfa->get_carrier_operators(algorithm); // look for carriers
  uint8_t i;

  for (i = 0; i < max_notes; i++)
  {
    if (voices[i].live == true)
    {
      memset(&voiceStatus, 0, sizeof(VoiceStatus));
      voices[i].dx7_note->peekVoiceStatus(voiceStatus);

      for (uint8_t op = 0; op < 4; op++)
      {
        if ((op_carrier & (1 << op)))
        {
          // this voice is a carrier!
          if (voiceStatus.ampStep[op] == 3)
            return true;
        }
      }
    }
  }

  return false;
}

void Dexed::updatePitchOnly(float pitch)
{
  pitch -= TRANSPOSE_FIX;
  for (uint8_t i = 0; i < max_notes; i++)
  {
    if (voices[i].live)
    {
      voices[i].dx7_note->updatePitchOnly(pitch);
      voices[i].midi_note = (int)pitch;
    }
  }
}

void Dexed::freq(float pitch, uint8_t velo)
{
  pitch -= TRANSPOSE_FIX;
  bool foundvoice = false;
  for (uint8_t i = 0; i < max_notes; i++)
  {
    if (voices[i].keydown && voices[i].live)
    {
      voices[i].dx7_note->update(algorithm, pitch, velo, false);
      voices[i].midi_note = (int)pitch;
      voices[i].velocity = velo;
      foundvoice = true;
    }
  }
  if (!foundvoice) {
    keydown((int)pitch, velo);
    freq(pitch, velo);
  }
}

void Dexed::keydown(int16_t pitch, uint8_t velo) {
  if ( velo == 0 ) {
    keyup(pitch);
    return;
  }

  pitch -= TRANSPOSE_FIX;

  uint8_t note = currentNote;
  uint8_t keydown_counter = 0;

  if (!monoMode && refreshMode)
  {
    for (uint8_t i = 0; i < max_notes; i++)
    {
      if (voices[i].midi_note == pitch && voices[i].keydown == false && voices[i].live)
      {
        // retrigger or refresh note?
        voices[i].dx7_note->keyup();
        voices[i].midi_note = pitch;
        voices[i].velocity = velo;
        voices[i].keydown = true;
        voices[i].live = true;
        voices[i].dx7_note->init(algorithm, pitch, velo);
        voices[i].key_pressed_timer = millis();
        return;
      }
    }
  }

  for (uint8_t i = 0; i <= max_notes; i++)
  {
    if (i == max_notes)
    {
      uint32_t min_timer = 0xffff;

      if (monoMode)
        break;

      // no free sound slot found, so use the oldest note slot
      for (uint8_t n = 0; n < max_notes; n++)
      {
        if (voices[n].key_pressed_timer < min_timer)
        {
          min_timer = voices[n].key_pressed_timer;
          note = n;
        }
      }
      voices[note].keydown = false;
      voices[note].live = false;
      voices[note].key_pressed_timer = 0;
      keydown_counter--;
    }

    if (!voices[note].keydown)
    {
      currentNote = (note + 1) % max_notes;
      voices[note].midi_note = pitch;
      voices[note].velocity = velo;
      voices[note].keydown = true;
      voices[note].dx7_note->init(algorithm, pitch, velo);
      voices[i].key_pressed_timer = millis();
      keydown_counter++;
      break;
    }
    else
    {
      keydown_counter++;
    }
    note = (note + 1) % max_notes;
  }

  if ( monoMode ) {
    for (uint8_t i = 0; i < max_notes; i++) {
      if ( voices[i].live ) {
        // all keys are up, only transfer signal
        if ( ! voices[i].keydown ) {
          voices[i].live = false;
          voices[note].dx7_note->transferSignal(*voices[i].dx7_note);
          break;
        }
        if ( voices[i].midi_note < pitch ) {
          voices[i].live = false;
          voices[note].dx7_note->transferState(*voices[i].dx7_note);
          break;
        }
        return;
      }
    }
  }

  voices[note].live = true;
}

void Dexed::keyup(int16_t pitch) {
  uint8_t note;

  pitch = constrain(pitch, 0, 127);

  pitch -= TRANSPOSE_FIX;

  for (note = 0; note < max_notes; note++) {
    if ( voices[note].midi_note == pitch && voices[note].keydown ) {
      voices[note].keydown = false;
      voices[note].key_pressed_timer = 0;
      break;
    }
  }

  // note not found ?
  if ( note >= max_notes ) {
    return;
  }

  if ( monoMode ) {
    int16_t highNote = -1;
    uint8_t target = 0;
    for (int8_t i = 0; i < max_notes; i++) {
      if ( voices[i].keydown && voices[i].midi_note > highNote ) {
        target = i;
        highNote = voices[i].midi_note;
      }
    }

    if ( highNote != -1 && voices[note].live ) {
      voices[note].live = false;
      voices[note].key_pressed_timer = 0;
      voices[target].live = true;
      voices[target].dx7_note->transferState(*voices[note].dx7_note);
    }
  }

  voices[note].dx7_note->keyup();
}

void Dexed::doRefreshEnv(void)
{
  refreshEnv = true;
}

void Dexed::doRefreshVoice(void)
{
  refreshVoice = true;
}

bool Dexed::getMonoMode(void) {
  return monoMode;
}

void Dexed::setMonoMode(bool mode) {
  if (monoMode == mode)
    return;

  notesOff();
  monoMode = mode;
}

void Dexed::setRefreshMode(bool mode) {
  refreshMode = mode;
}

void Dexed::panic(void)
{
  for (uint8_t i = 0; i < max_notes; i++)
  {
    if (voices[i].live == true) {
      voices[i].keydown = false;
      voices[i].live = false;
      voices[i].key_pressed_timer = 0;
      if ( voices[i].dx7_note != NULL ) {
        voices[i].dx7_note->oscSync();
      }
    }
  }
}

void Dexed::notesOff(void) {
  for (uint8_t i = 0; i < max_notes; i++) {
    if (voices[i].live == true) {
      voices[i].keydown = false;
      voices[i].live = false;
    }
  }
}

uint8_t Dexed::getMaxNotes(void)
{
  return max_notes;
}

uint8_t Dexed::getNumNotesPlaying(void)
{
  uint8_t op_carrier = engineMsfa->get_carrier_operators(algorithm); // look for carriers
  uint8_t i;
  uint8_t count_playing_voices = 0;

  for (i = 0; i < max_notes; i++)
  {
    if (voices[i].live == true)
    {
      uint8_t op_amp = 0;
      uint8_t op_carrier_num = 0;

      memset(&voiceStatus, 0, sizeof(VoiceStatus));
      voices[i].dx7_note->peekVoiceStatus(voiceStatus);

      for (uint8_t op = 0; op < 4; op++)
      {
        if ((op_carrier & (1 << op)))
        {
          // this voice is a carrier!
          op_carrier_num++;
          if (voiceStatus.amp[op] <= VOICE_SILENCE_LEVEL && voiceStatus.ampStep[op] == 4)
          {
            // this voice produces no audio output
            op_amp++;
          }
        }
      }

      if (op_amp == op_carrier_num)
      {
        // all carrier-operators are silent -> disable the voice
        voices[i].live = false;
        voices[i].keydown = false;
        voices[i].dx7_note->keyup();
      }
      else
        count_playing_voices++;
    }
  }
  return (count_playing_voices);
}

void Dexed::setOPDrone(uint8_t op, bool set)
{
  op = constrain(op, 0, 3);
  // there's no dx7 sysex for this so just do it directly
  for (uint8_t i = 0; i < max_notes; i++) {
    voices[i].dx7_note->setOPDrone(op, set);
  }
}

uint8_t Dexed::getCarrierCount(void)
{
  uint8_t op_carrier = engineMsfa->get_carrier_operators(algorithm); // look for carriers
  uint8_t count =0 ;
  for (uint8_t op = 0; op < 4; op++)
  {
    if ((op_carrier & (1 << op)))
      count++;
  } 
  return count;
}

void Dexed::setAlgorithm(uint8_t alg)
{
  algorithm = alg;
#ifdef DEBUG
  Serial.print("Algorithm: ");
  Serial.println(algorithm + 1);

  uint8_t op_carrier = engineMsfa->get_carrier_operators(algorithm); // look for carriers
  for (uint8_t op = 0; op < 4; op++)
  {
    if ((op_carrier & (1 << op)))
      Serial.printf("\tOp %d is a carrier\n", 4 - op);
    else
      Serial.printf("\tOp %d is a modulator\n", 4 - op);
  }
#endif  
}

uint8_t Dexed::getAlgorithm(void)
{
  return algorithm;
}
