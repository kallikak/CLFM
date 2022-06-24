#include <Arduino.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Utility.h"

int drops[][12] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},       // CHROMATIC
  {0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0},       // DIATONIC_MAJOR   0 2 4 5 7 9 11
  {0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 2, 0},       // DIATONIC_MINOR   0 2 3 5 7 8 11
  {0, 1, 0, 1, 2, 0, 1, 0, 1, 0, 1, 2},       // PENTATONIC_MAJOR 0 2 5 7 9
  {0, 1, 2, 0, 1, 0, 1, 0, 1, 2, 0, 1},       // PENTATONIC_MINOR 0 3 5 7 10
  {0, 1, 2, 3, 0, 1, 2, 0, 1, 2, 3, 4},       // TRIAD_MAJOR      0 4 7
  {0, 1, 2, 0, 1, 2, 3, 0, 1, 2, 3, 4},       // TRIAD_MINOR      0 3 7
  {0, 1, 2, 3, 0, 1, 2, 0, 1, 0, 1, 2},       // MAJOR_SIXTH      0 4 7 9
  {0, 1, 2, 0, 1, 2, 3, 0, 1, 0, 1, 2},       // MINOR_SIXTH      0 3 7 9
  {0, 1, 2, 3, 0, 1, 2, 0, 1, 2, 0, 1},       // MAJOR_SEVENTH    0 4 7 10
  {0, 1, 2, 0, 1, 2, 3, 0, 1, 2, 0, 1},       // MINOR_SEVENTH    0 3 7 10
  {0, 1, 2, 0, 1, 0, 0, 0, 1, 2, 0, 1},       // BLUES            0 3 5 6 7 10
  {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},       // WHOLE_TONE       0 2 4 6 8 10
};

int quantise_note(int n, quantisation q, int transpose)
{
  // C C# D D# E F F# G G# A A# B 
  // 0 1  2 3  4 5 6  7 8  9 10 11
  // diatonic:     0 2 4 5 7 9 11
  // diatonic_m:   0 2 3 5 7 8 11
  // pentatonic:   0 2 5 7 9
  // pentatonic_m: 0 3 5 7 10
  // triad:        0 4 7
  // triad_m:      0 3 7
  // blues:        0 3 5 6 7 10
  // whole_tone:   0 2 4 6 8 10

  int k = (144 + n) % 12;
  n -= drops[(int)q][k];
  return transpose + n;
}

float freqToNote(float f)
{
  return 12 * log2(f / 440.0) + 49;
}

float noteToFreq(float n)
{
  //  f = 440 * 2 ^ ((n - 49) / 12)
  return pow(2, (n - 49) / 12.0) * 440;
}

#define BUFFERSIZE 11 // must be odd (since I don't average below)
#define MIDBUFFER (BUFFERSIZE >> 1)

static int buffer[BUFFERSIZE] = { 0 };
static int sortbuffer[BUFFERSIZE];
static int bufferindex = 0;
static int samples = 0;

int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  return b - a;
}

int median() 
{
  memcpy(sortbuffer, buffer, BUFFERSIZE * sizeof(int));
  qsort(sortbuffer, BUFFERSIZE, sizeof(sortbuffer[0]), sort_desc);
  return sortbuffer[MIDBUFFER];
}

int handleCVBuffer(int v, int thresh)
{
  int m = 0;
  buffer[bufferindex] = v;
  bufferindex = (bufferindex + 1) % BUFFERSIZE;
  if (++samples >= BUFFERSIZE)
  {
    m = median();
//    Serial.printf("Added %d, median = %d\n", v, m);
    // catch rapid change and just trust the new value
    if (abs(v - m) >= thresh)
      m = v;
  }

  return m;
}
