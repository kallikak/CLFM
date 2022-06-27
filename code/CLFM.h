#pragma once

#define PROTOTYPE 1
#define RELEASE_1 2

//#define CLFM_VERSION PROTOTYPE
#define CLFM_VERSION RELEASE_1

#if CLFM_VERSION == PROTOTYPE
#define N_ALGS 8
#else
#define N_ALGS 10
#endif

typedef enum { DIV_128, DIV_64, DIV_32, DIV_16, DIV_8, DIV_4, DIV_2, UNITY, MUL_2, MUL_3, MUL_4, MUL_5, MUL_6, MUL_7, MUL_8 } coarseAdj;
extern const char *coarseFactors[];

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
  envvals env[4];
  int level[4];
  int feedback;
} configStruct;

extern configStruct config;
