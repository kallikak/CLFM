#pragma once

#define N_ALGS 10

#define MAXFOLD 8.0
#define MAXFOLDPARAM 256

typedef enum {SIN, TRI, SQR, SINFOLD, TRIFOLD} wavetype;

typedef enum { DIV_128, DIV_64, DIV_32, DIV_16, DIV_8, DIV_4, DIV_2, UNITY, MUL_2, MUL_3, MUL_4, MUL_5, MUL_6, MUL_7, MUL_8 } coarseAdj;

typedef struct
{
  int a, d, s, r;
  int scaled_s;
  bool drone;
} envvals;

typedef struct configStruct
{
  int algorithm;
  coarseAdj coarse[4];
  int fine[4];
  int detune;
  bool sync;
  bool fold;
  wavetype wave[4];
  envvals env[4];
  int level[4];
  float scale[4];
  int feedback;
} configStruct;

extern configStruct config;
