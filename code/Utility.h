#pragma once

typedef enum
{
  CHROMATIC,
  DIATONIC_MAJOR,
  DIATONIC_MINOR,
  PENTATONIC_MAJOR,
  PENTATONIC_MINOR,
  TRIAD_MAJOR,
  TRIAD_MINOR,
  MAJOR_SIXTH,
  MINOR_SIXTH,
  MAJOR_SEVENTH,
  MINOR_SEVENTH,
  BLUES,
  WHOLE_TONE
} quantisation;

int quantise_note(int n, quantisation q, int transpose);
float freqToNote(float f);
float noteToFreq(float n);

int handleCVBuffer(int v, int thresh);
