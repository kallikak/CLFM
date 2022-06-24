/*
   synth_dexed

   synth_dexed is a port of the Dexed sound engine (https://github.com/asb2m10/dexed)
   as library for the Teensy-3.5/3.6/4.x with an audio shield.
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

#include "synth_dexed.h"

#if defined(TEENSYDUINO)
void AudioSynthDexed::update(void)
{
  if (in_update == true)
  {
    xrun++;
    return;
  }
  else
    in_update = true;

  elapsedMicros render_time;
  audio_block_t *lblock;

  lblock = allocate();

  if (!lblock)
  {
    in_update = false;
    return;
  }

  getSamples(AUDIO_BLOCK_SAMPLES, lblock->data);

  if (render_time > audio_block_time_us) // everything greater audio_block_time_us (2.9ms for buffer size of 128) is a buffer underrun!
    xrun++;

  if (render_time > render_time_max)
    render_time_max = render_time;

  transmit(lblock, 0);
  release(lblock);

  in_update = false;
};
#endif
/*
  // https://www.musicdsp.org/en/latest/Effects/169-compressor.html#
  void compress
  (
  float*  wav_in,     // signal
  int     n,          // N samples
  double  threshold,  // threshold (percents)
  double  slope,      // slope angle (percents)
  int     sr,         // sample rate (smp/sec)
  double  tla,        // lookahead  (ms)
  double  twnd,       // window time (ms)
  double  tatt,       // attack time  (ms)
  double  trel        // release time (ms)
  )
  {
  typedef float   stereodata[2];
  stereodata*     wav = (stereodata*) wav_in; // our stereo signal
  threshold *= 0.01;          // threshold to unity (0...1)
  slope *= 0.01;              // slope to unity
  tla *= 1e-3;                // lookahead time to seconds
  twnd *= 1e-3;               // window time to seconds
  tatt *= 1e-3;               // attack time to seconds
  trel *= 1e-3;               // release time to seconds

  // attack and release "per sample decay"
  double  att = (tatt == 0.0) ? (0.0) : exp (-1.0 / (sr * tatt));
  double  rel = (trel == 0.0) ? (0.0) : exp (-1.0 / (sr * trel));

  // envelope
  double  env = 0.0;

  // sample offset to lookahead wnd start
  int     lhsmp = (int) (sr * tla);

  // samples count in lookahead window
  int     nrms = (int) (sr * twnd);

  // for each sample...
  for (int i = 0; i < n; ++i)
  {
    // now compute RMS
    double  summ = 0;

    // for each sample in window
    for (int j = 0; j < nrms; ++j)
    {
      int     lki = i + j + lhsmp;
      double  smp;

      // if we in bounds of signal?
      // if so, convert to mono
      if (lki < n)
        smp = 0.5 * wav[lki][0] + 0.5 * wav[lki][1];
      else
        smp = 0.0;      // if we out of bounds we just get zero in smp

      summ += smp * smp;  // square em..
    }

    double  rms = sqrt (summ / nrms);   // root-mean-square

    // dynamic selection: attack or release?
    double  theta = rms > env ? att : rel;

    // smoothing with capacitor, envelope extraction...
    // here be aware of pIV denormal numbers glitch
    env = (1.0 - theta) * rms + theta * env;

    // the very easy hard knee 1:N compressor
    double  gain = 1.0;
    if (env > threshold)
      gain = gain - (env - threshold) * slope;

    // result - two hard kneed compressed channels...
    float  leftchannel = wav[i][0] * gain;
    float  rightchannel = wav[i][1] * gain;
  }
  }
*/
