#include "CLFM.h"
  
#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SerialFlash.h>

#include <USBHost_t36.h>

USBHost myusb;
USBHub hub1(myusb);
MIDIDevice midi1(myusb);

bool midimode = false;
#define POLYPHONY 16

bool updateall = false;
int loopcount = 0;

extern "C" uint32_t set_arm_clock(uint32_t frequency);

#include <ADC.h>

ADC *adc = new ADC();

#include "Bounce2.h"
Bounce *resetsw = new Bounce();

#define NO_AVG 0

#define ANALOG_BITS 12
#define ANALOG_SHIFT 5
#define ANALOG_MAX ((0x01 << ANALOG_BITS) - 1)
#define ANALOG_MID (0x01 << (ANALOG_BITS - 1))

#include "src/synth_dexed.h"
#include "Utility.h"

bool idle = true;
bool quantise = true;
bool aftertouch = false;

#define PITCH_BEND_FACTOR 7

bool resetpressed = false;

volatile bool feedback2 = false;

AudioSynthDexed         fm(midimode ? POLYPHONY : 1, SAMPLE_RATE);
AudioFilterStateVariable filter;
AudioAmplifier          amp;
AudioOutputI2S2         i2s2;
AudioOutputUSB          usb;
AudioConnection         patchCord0(fm, 0, filter, 0);
AudioConnection         patchCord1(filter, 0, amp, 0);
AudioConnection         patchCord2(amp, 0, i2s2, 0);
AudioConnection         patchCord3(amp, 0, i2s2, 1);
AudioConnection         patchCord4(amp, 0, usb, 0);
AudioConnection         patchCord5(amp, 0, usb, 1);

// operators are indexed in reverse in the dexed library
#define FINE_POT1 A7
#define FINE_POT2 A6
#define FINE_POT3 A5
#define FINE_POT4 A4

const int finePots[] = { FINE_POT1, FINE_POT2, FINE_POT3, FINE_POT4 };

#define COARSE_POT1 A3
#define COARSE_POT2 A2
#define COARSE_POT3 A1
#define COARSE_POT4 A0

const int coarsePots[] = { COARSE_POT1, COARSE_POT2, COARSE_POT3, COARSE_POT4 };

#define LEVEL1 A12
#define LEVEL1LED 34
#define LEVEL2 A11
#define LEVEL2LED 35
#define LEVEL3 A10
#define LEVEL3LED 36
#define LEVEL4 A9
#define LEVEL4LED 37

const int levelSliders[] = { LEVEL1, LEVEL2, LEVEL3, LEVEL4 };
const int levelLEDs[] = { LEVEL1LED, LEVEL2LED, LEVEL3LED, LEVEL4LED };

const float MAX_TIME = 30.0f;
  
#define ENV1 A15
#define ENV2 A14
#define ENV3 A13
#define ENV4 A8

const int envPots[] = { ENV1, ENV2, ENV3, ENV4 };
bool updateModulatorEnv = false;
bool updateCarrierEnv = false;
bool updateEnv = false;
int updateOneEnv = -1;

#define FEEDBACK_POT A16

#define QUANTISE_SW 0
#define FEEDBACK_SW 9

typedef enum { AD, ASR, ADSR, CTS } envCtrlMode;

#define ENV_MODE_ASR 29
#define ENV_MODE_ADSR 32
#define ENV_MODE_CTS 30

const int envModeSw[] = { ENV_MODE_CTS, ENV_MODE_ADSR, ENV_MODE_ASR };

#define CV_IN A17
#define GATE_IN 1

#define PITCH_OFFSET 60
#define MIDI_NOTE_OFFSET 24

volatile bool gate = LOW;
volatile bool gatetoggled = false;
volatile float note = -1;
 
#define RESET 31

const char *coarseFactors[] = {"1/128", "1/64", "1/32", "1/16", "1/8", "1/4", "1/2", "1", "2", "3", "4", "5", "6", "7", "8", "Overflow"};

typedef enum { CARRIER, MODULATOR } operatorType;

typedef struct
{
  int oldvalue = -1;
  int rawvalue = -1;
  int value = -1;
  unsigned long lastchange = 0; 
} potval;

typedef struct controlsStruct
{
  // raw pot values
  potval finepot[4];
  potval coarsepot[4];
  potval levelpot[4];
  potval envpot[4];
  potval feedbackpot;
  envCtrlMode envMode;
  byte modvalue;
} controlsStruct;

configStruct config;
controlsStruct controls;

bool showConfigOnChange = false;

const char *wavestr[] = {"sin", "tri", "sqr", "sinfld", "trifld"};
void printConfig()
{
  Serial.printf("Algorithm: %3d\n", config.algorithm + 1);
  Serial.printf("Coarse: %6s %6s %6s %6s\n", coarseFactors[config.coarse[3]], coarseFactors[config.coarse[2]], coarseFactors[config.coarse[1]], coarseFactors[config.coarse[0]]);  
  Serial.printf("Fine:   %6d %6d %6d %6d\n", config.fine[3], config.fine[2], config.fine[1], config.fine[0]);
  Serial.printf("Wave:   %6s %6s %6s %6s\n", 
    wavestr[config.wave[3]], wavestr[config.wave[2]], wavestr[config.wave[1]], wavestr[config.wave[0]]);
  for (int k = 3; k >= 0; --k)
  {
    if (config.env[k].drone)
      Serial.printf("Op %d drone\n", 4 - k);
    else
      Serial.printf("Op %d ADSR env: %3d %3d %3d %3d\n", 4 - k, config.env[k].a, config.env[k].d, config.env[k].s, config.env[k].r);
  }
  Serial.printf("Levels: %6d %6d %6d %6d\n", config.level[3], config.level[2], config.level[1], config.level[0]);
  Serial.printf("Feedback: %4d on operator %d\n", config.feedback, feedback2 ? 2 : 4);
  Serial.printf("Oscillator sync is %s\n", config.sync ? "on" : "off");
}

bool potchange(potval *v, unsigned long now, bool centre, bool jittery)
{
  bool force = false;
  if (updateall)
    return true;  // initial update
  // extra accepting near limits
#define LIMIT 1
  if (centre)
    force = abs(v->rawvalue - ANALOG_MID) < LIMIT && abs(v->oldvalue - ANALOG_MID) > LIMIT;
  else
    force = (v->rawvalue < LIMIT && v->oldvalue > LIMIT) || ((v->rawvalue > ANALOG_MAX - LIMIT) && (v->oldvalue < ANALOG_MAX - LIMIT));
  // small change in short time is ok, otherwise need bigger change
  int d = abs(v->rawvalue - v->oldvalue);
  if (force || d > (jittery ? 64 : 32))// || (d && (now - v->lastchange) < 20))
  {
//    Serial.printf("Raw %4d, Old %4d, Val %4d\n", v->rawvalue, v->oldvalue, v->value);
    v->oldvalue = v->rawvalue;
    v->lastchange = now;
    return true;
  }
  return false;
}

#define MID_RANGE 200  // middle 400 (10%) all return 0
#define SCALE (1.0f * ANALOG_MID / (ANALOG_MID - MID_RANGE))
void updatePot(potval *thepot, int pin, unsigned long t, int shift, bool invert, bool centre, int average)
{
  int v = adc->analogRead(pin);
  if (invert)
    v = ANALOG_MAX - v;
  if (centre)
  {
    int dv = v > ANALOG_MID ? (v - ANALOG_MID - MID_RANGE) : (ANALOG_MID - v - MID_RANGE);
    if (dv <= 0)
      v = ANALOG_MID;
    else if (v > ANALOG_MID)
      v = ANALOG_MID + round(dv * SCALE);
    else
      v = ANALOG_MID - round(dv * SCALE);
  }
  if (average)
  {
    if (centre && abs(v - ANALOG_MID) < 2)
      ;
    else
      v = (*thepot).rawvalue = (average - 1) * (*thepot).rawvalue / average + v  / average;
  }
  (*thepot).rawvalue = v;
  (*thepot).value = v >> shift;
}

// TODO consider accessing the fm object directly for this information
operatorType getOpType(int op, int algo)
{
  switch (op)
  {
    case 2:
      return algo >= 6 ? CARRIER : MODULATOR;
    case 1:
      return algo == 4 || algo >= 6 ? CARRIER : MODULATOR;
    case 0:
      return algo == 5 || algo == 9 ? CARRIER : MODULATOR;
    case 3:
    default:
      return CARRIER;
  }
}

void setAmpGain()
{
//  1 => 0.6
//  2 => 0.5
//  3 => 0.4
//  4 => 0.3

//  1 => 0.4
//  2 => 0.36
//  3 => 0.33
//  4 => 0.3
  float gain;
  if (midimode)
    gain = 0.3;
//    gain = 0.433 - fm.getCarrierCount() * 0.033;
  else
    gain = 0.7 - fm.getCarrierCount() * 0.1;
//  amp.gain(midimode ? gain / 2 : gain);
  amp.gain(gain);
}

envCtrlMode getEnvMode()
{
  if (digitalRead(ENV_MODE_CTS) == LOW)
    return CTS;
  else if (digitalRead(ENV_MODE_ASR) == LOW)
    return ASR;  
  else if (digitalRead(ENV_MODE_ADSR) == LOW)
    return ADSR;
  else
    return AD;  
}

int getAlgorithm()
{
  static int pins[] = {28, 13, 12, 11, 10, 8, 7, 6, 5};
  int a = 0;  // all high => algo 1
  int i;
  for (i = 0; i < 9; ++i)
  {
    if (digitalRead(pins[i]) == LOW)
    {
      a = i + 1;
      break;
    }
  }

  if (a == 0 && config.algorithm > 1)  // this is just break before make
  {
//    Serial.printf("%d %d\n", a, config.algorithm);
    a = config.algorithm;
  }
  
  return a;
}

void setAlgorithmLEDs(int algo)
{
  digitalWrite(levelLEDs[0], getOpType(0, algo) == CARRIER);
  digitalWrite(levelLEDs[1], getOpType(1, algo) == CARRIER);
  digitalWrite(levelLEDs[2], getOpType(2, algo) == CARRIER);
  digitalWrite(levelLEDs[3], getOpType(3, algo) == CARRIER);
}

void resetAllDrone()
{
  for (int i = 0; i < 4; ++i)
  {
    if (config.env[i].drone)
    {
      setDrone(i, false);
      config.env[i].drone = false;
    }
  }
}

void setDrone(int i, bool set)
{
  if (set && !config.env[i].drone)
  {
    config.env[i].drone = true;
    Serial.printf("Switching on drone %i\n", i);
    fm.setOPDrone(i, true);
  } 
  else if (config.env[i].drone)
  {
    Serial.printf("Switching off drone %i\n", i);
    config.env[i].drone = false;
    fm.setOPDrone(i, false);
  }
}

int rescaleSustain(int s)
{
  // for sustain, map 16 to 64, and linear either side 
  if (s <= 16)
    return 2 * s;
  else
    return 64 + 4 * (s - 16) / 7.0;      // 112 -> 64
}

void handleContinuousEnvelopes(int i, int v)
{
  static const int as[] = {127, 63, 0, 0, 32, 16, 0};
  static const int ds[] = {0, 63, 127, 127, 127, 127, 127};
  static const int ss[] = {0, 63, 127, 127, 127, 63, 0};
  static const int rs[] = {0, 32, 0, 32, 63, 32, 0};

  if (v > 123)  // drone
  {
    config.env[i].a = 0;
    config.env[i].d = 0;
    config.env[i].s = rescaleSustain(127);
    config.env[i].r = 0;
    if (!config.env[i].drone)
      setDrone(i, true);
  }
  else
  {
    if (config.env[i].drone)
      setDrone(i, false);
    const int scale = 21;    
    v = max(0, min(6 * scale, v));
    int k = floor(v / scale);
    int l = v % scale;
    int m = scale - l;
    config.env[i].a = (m * as[k] + l * as[k + 1]) / scale;
    config.env[i].d = (m * ds[k] + l * ds[k + 1]) / scale;
    config.env[i].s = rescaleSustain((m * ss[k] + l * ss[k + 1]) / scale);
    config.env[i].r = (m * rs[k] + l * rs[k + 1]) / scale;
  }
}

void handleAD_ASREnvelopes(int i, int v, envCtrlMode mode, int algo)
{
  int j;
  switch (i)
  {
    case 3: // carrier attack
      for (j = 0; j < 4; ++j)
      {
        if (getOpType(j, algo) == CARRIER)
        {
          config.env[j].a = v;
        }
      }
      break;
    case 2: // carrier decay or release
      for (j = 0; j < 4; ++j)
      {
        if (getOpType(j, algo) == CARRIER)
        {
          config.env[j].d = v;
          config.env[j].r = mode == AD ? 0 : v;
        }
      }
      break;
    case 1: // modulator attack
      for (j = 0; j < 4; ++j)
      {
        if (getOpType(j, algo) == MODULATOR)
        {
          config.env[j].a = v;
        }
      }
      break;
    case 0: // modulator decay or release
      for (j = 0; j < 4; ++j)
      {
        if (getOpType(j, algo) == MODULATOR)
        {
          config.env[j].d = v;
          config.env[j].r = mode == AD ? 0 : v;
        }
      }
      break; 
  }
}

void handleADSREnvelopes(int i, int v)
{
  for (int j = 0; j < 4; ++j)
  {
    switch (i)
    {
      case 3:
        config.env[j].a = v;
        break;
      case 2:
        config.env[j].d = v;
        break;
      case 1:
        config.env[j].s = rescaleSustain(v);
        break;
      case 0:
        config.env[j].r = v;
        break;
    }
  }
}

bool handleEnvelopes(int i, int v, envCtrlMode mode, int algo)
{
  int a = config.env[i].a;
  int d = config.env[i].d;
  int s = config.env[i].s;
  int r = config.env[i].r;
  switch (mode)
  {
    case AD:
    case ASR:
      updateEnv = true;
      updateCarrierEnv = i == 0 || i == 1;
      updateModulatorEnv = i == 2 || i == 3;
      handleAD_ASREnvelopes(i, v, mode, algo);
      resetAllDrone();
      break;
    case ADSR:
      updateEnv = true;
      handleADSREnvelopes(i, v);
      resetAllDrone();
      break;
    case CTS:
      updateOneEnv = i;
      handleContinuousEnvelopes(i, v);
      break;
  }
  return (a != config.env[i].a) || (d != config.env[i].d) || (s != config.env[i].s) || (r != config.env[i].r);
}

bool handleCoarseTuning(int i, int v)
{
  v = map(v, 0, 127, 0, 14);
  coarseAdj newcoarse = (coarseAdj)v;
  if (config.coarse[i] != newcoarse)
  {
    config.coarse[i] = newcoarse;
    return true;
  }
  return false;
}

bool handleFineTuning(int i, int v)
{
  int limit = config.fold ? MAXFOLDPARAM : 50;
  v = map(v, 0, config.fold ? 1023 : 127, -limit, limit);
  if (config.fine[i] != v)
  {
    config.fine[i] = v;
    return true;
  }
  return false;
}

void updateAllEnv(envCtrlMode mode, int algo)
{
  int i;
  for (i = 0; i < 4; ++i)
  {
    handleEnvelopes(i, controls.envpot[i].value, mode, algo);
  }
  
  fm.doRefreshEnv();
}

bool checkEnvMode()
{
  envCtrlMode newEnvMode = getEnvMode();
  if (newEnvMode != controls.envMode)
  {
    int i;
    controls.envMode = newEnvMode;
    if (controls.envMode == AD)
    {
      for (i = 0; i < 4; ++i)
      {
        config.env[i].s = 0;
        config.env[i].r = 0;
      }
    }
    else if (controls.envMode == ASR)
    {
      for (i = 0; i < 4; ++i)
      {
        config.env[i].s = rescaleSustain(127);
        config.env[i].r = config.env[i].d;  
      }
    }
    if (controls.envMode != CTS)
      resetAllDrone();
    return true;
  }
  return false;
}

void updateWavetypes()
{
  // set default and change where necessary
  config.fold = false;
  config.wave[0] = config.wave[1] = config.wave[2] = config.wave[3] = SIN;
  if (feedback2) 
  {
    switch (config.algorithm)
    {
      case 3:
        config.wave[0] = TRI;
        config.wave[1] = SQR;
        break;
      case 4:
        config.wave[0] = config.wave[1] = config.wave[3] = SQR;
        config.wave[2] = TRI;
        break;
      case 6:
        config.wave[1] = SQR;
        config.wave[3] = TRI;
        break;
      case 9:
        config.fold = true;
        config.wave[0] = config.wave[1] = TRIFOLD;
        config.wave[2] = config.wave[3] = SINFOLD;
        break;
    }
  }
}

void pin_reset() 
{
//  setMidiMode(!midimode);
  Serial.println("Resetting...");
  SCB_AIRCR = 0x05FA0004;
}

void handle_gate()
{
  if (midimode)
    return;
  gate = !digitalRead(GATE_IN);
//  Serial.printf("Gate in is %s\n", gate ? "HIGH" : "LOW");
  if (idle && gate) 
  {
    idle = false;
    set_arm_clock(600000000);  
  }
  if (!gate && note >= 0)
  {
    fm.keyup((int)note);
    note = -1;
    gatetoggled = true;
  }
}

bool checkswitches()
{
  bool update = false;
  if (quantise != digitalRead(QUANTISE_SW))
    quantise = !quantise;
  if (feedback2 != digitalRead(FEEDBACK_SW)) {
    feedback2 = !feedback2;
    update = true;
  }
  return update;
}

void checkSerialControl()
{
  if (Serial.available())
  {
    char c = Serial.read();
    switch (c)
    {
      case 'x':
        config.wave[3] = SQR;
        config.wave[1] = TRI;
        Serial.println("------------------------------------------");
        printConfig();
        Serial.println("------------------------------------------");
        break;
      case 'q':
        quantise = !quantise;
        Serial.println("==========================================");
        Serial.println(quantise ? "Quantise on" : "Quantise off");
        Serial.println("------------------------------------------");
        break;
      case 'c':
        Serial.println("==========================================");
        Serial.println("         Current Configuration");
        Serial.println("------------------------------------------");
        printConfig();
        Serial.println("------------------------------------------");
        break;
      case 'z':
        showConfigOnChange = !showConfigOnChange;
        Serial.println("==========================================");
        Serial.print(showConfigOnChange ? "Will show" : "Not showing");
        Serial.println(" configuration on change");
        Serial.println("------------------------------------------");
        printConfig();
        Serial.println("------------------------------------------");
        break;
      case 'p':
        Serial.println("==========================================");
        Serial.println("Panic! All notes off.");
        Serial.println("------------------------------------------");
        note = -1;
        fm.panic();
        break;
      case 't':
        Serial.println("=====================");
        Serial.print("Temperature: ");
        Serial.print(tempmonGetTemp());
        Serial.println("°C");
        Serial.println("---------------------");
        break;
      case 's':
        config.sync = !config.sync;
        Serial.println("=====================");
        Serial.print("Sync is now ");
        Serial.println(config.sync ? "on" : "off");
        Serial.println("---------------------");
        break;
      case 'h':
        Serial.println("==========================================");
        Serial.println("         Serial Comands");
        Serial.println("------------------------------------------");
        Serial.println("    c - print the current configuration");
        Serial.println("    z - toggle print configuration on change");
        Serial.println("    p - panic (all notes off)");
        Serial.println("    s - toggle oscillator sync");
        Serial.println("    t - show the temperature of the Teensy");
        Serial.println("    h - show the help (this message)");
        Serial.println("------------------------------------------");
        break;
    }
    Serial.flush();
  }
}

void setup() 
{ 
  for (int i = 0; i < 4; ++i)
  {
    pinMode(finePots[i], INPUT);
    pinMode(coarsePots[i], INPUT);
    pinMode(levelSliders[i], INPUT);
    pinMode(levelLEDs[i], OUTPUT);
    pinMode(envPots[i], INPUT);
    if (i < 3)
      pinMode(envModeSw[i], INPUT_PULLUP);
  }
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(28, INPUT_PULLUP);
  pinMode(QUANTISE_SW, INPUT_PULLUP);
  pinMode(FEEDBACK_SW, INPUT_PULLUP);
  pinMode(FEEDBACK_POT, INPUT);

  pinMode(GATE_IN, INPUT_PULLUP);
  pinMode(CV_IN, INPUT);
  
  pinMode(RESET, INPUT_PULLUP);
//  attachInterrupt(RESET, pin_reset, FALLING);
  resetsw->attach(RESET);
  resetsw->interval(50);
  
  AudioMemory(20);
  
  attachInterrupt(GATE_IN, handle_gate, CHANGE);
  
  analogReadResolution(ANALOG_BITS);
  
  adc->adc0->setAveraging(32); // set number of averages
  adc->adc0->setResolution(ANALOG_BITS); // set bits of resolution
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
  
  adc->adc1->setAveraging(32); // set number of averages
  adc->adc1->setResolution(ANALOG_BITS); // set bits of resolution
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
 
  filter.frequency(6000);
  setAmpGain();
  
  resetAllDrone();
  
  config.algorithm = -1; // invalid ensures initial update
  controls.envMode = (envCtrlMode)-1; // invalid ensures initial update
  controls.modvalue = 0;
  scaleModulators(0);
  config.detune = 0;
  config.sync = false;

  myusb.begin();

  midi1.setHandleNoteOn(handleNoteOn);
  midi1.setHandleNoteOff(handleNoteOff);
  midi1.setHandlePitchChange(handlePitchChange);
  midi1.setHandleControlChange(handleControlChange);
  midi1.setHandleAfterTouchChannel(handleAfterTouchChannel);

  usbMIDI.setHandleNoteOn(handleNoteOn);
  usbMIDI.setHandleNoteOff(handleNoteOff);
  usbMIDI.setHandlePitchChange(handlePitchChange);
  usbMIDI.setHandleControlChange(handleControlChange);
  usbMIDI.setHandleAfterTouchChannel(handleAfterTouchChannel);
  
  Serial.println("Setup complete");

  for (int i = 0; i < 7; ++i)
  {
    digitalWrite(levelLEDs[0], i < 4);
    digitalWrite(levelLEDs[1], i > 0 && i < 5);
    digitalWrite(levelLEDs[2], i > 1 && i < 6);
    digitalWrite(levelLEDs[3], i > 2);
    delay(100); 
  }
  delay(300); 

  idle = false;
  updateall = true;
}

void loop() 
{
  int i;
  unsigned long now = millis();
  bool needsUpdate = false;

  updateModulatorEnv = updateCarrierEnv = updateEnv = false;
  updateOneEnv = -1;

  loopcount++;
  for (i = 0; i < 4; ++i)
  {
    updatePot(&controls.finepot[i], finePots[i], now, config.fold ? 2 : ANALOG_SHIFT, false, true, config.fold ? 5 : NO_AVG); // smooth the folding a bit
    updatePot(&controls.coarsepot[i], coarsePots[i], now, ANALOG_SHIFT, true, true, NO_AVG);
    // slow down the modulator response to minimise stepping
    updatePot(&controls.levelpot[i], levelSliders[i], now, ANALOG_SHIFT, true, false, getOpType(i, config.algorithm) == CARRIER ? 4 : 10);
    updatePot(&controls.envpot[i], envPots[i], now, ANALOG_SHIFT, false, false, NO_AVG);
  }
  updatePot(&controls.feedbackpot, FEEDBACK_POT, now, ANALOG_SHIFT, false, false, NO_AVG);

  needsUpdate = checkswitches();  
  
  int alg = getAlgorithm();
  if (alg != config.algorithm)
  {
    config.algorithm = alg;
    setAlgorithmLEDs(alg);
  }

  if (checkEnvMode())
  {
    needsUpdate = true;
//    updateEnv = true;
    updateAllEnv(controls.envMode, config.algorithm);
  }

  // Use operators 3-6 with DX7 algorithms 1, 14, 8, 7, 5, 22, 31, 32.
//  static const int DX7ALGORITHMS[] = {1, 14, 8, 7, 5, 22, 31, 32};
  int dx7algo = config.algorithm;
  if (feedback2)
    dx7algo += N_ALGS;
  if (fm.getAlgorithm() != dx7algo)
  {
    fm.setAlgorithm(dx7algo);
    setAmpGain();
    needsUpdate = true;
    updateAllEnv(controls.envMode, config.algorithm);
    updateWavetypes();
  }

  for (i = 0; i < 4; ++i)
  {
    if (potchange(&controls.coarsepot[i], now, true, false))
    {
      if (handleCoarseTuning(i, controls.coarsepot[i].value))
      {
        needsUpdate = true;
      }
    }
    if (potchange(&controls.finepot[i], now, true, false))
    {
      if (handleFineTuning(i, controls.finepot[i].value))
      {
        needsUpdate = true;
      }
    }
    if (potchange(&controls.envpot[i], now, false, true))
    {
      bool envupdate = handleEnvelopes(i, controls.envpot[i].value, controls.envMode, config.algorithm);
      if (!needsUpdate) {
        needsUpdate = envupdate;
        updateEnv = true;
      }
    }
    if (potchange(&controls.levelpot[i], now, false, false))
    {
      long v = (uint8_t)(sqrt(controls.levelpot[i].value / 127.0) * 99);
//      config.level[i] = midimode && getOpType(i, config.algorithm) == CARRIER ? 0.75 * v : v;
//      config.level[i] = midimode ? 0.75 * v : v;
      config.level[i] = midimode ? 0.9 * v : v;
//      config.level[i] = v;
//      Serial.printf("Level: %d %3d %2d\n", i + 1, config.levelpot[i].value, v);
      needsUpdate = true;
    }
  }
  if (potchange(&controls.feedbackpot, now, false, false))
  {
    int newfb = round(controls.feedbackpot.value / 127.0 * 100);
    if (config.feedback != newfb)
    {
      config.feedback = newfb;
      needsUpdate = true;
    }
  }

  updateall = false;

  if (updateEnv)
    fm.doRefreshEnv();
    
  if (needsUpdate)
  {
    fm.doRefreshVoice();
    if (showConfigOnChange)
      printConfig();
  }
  
  myusb.Task();
  midi1.read();
  usbMIDI.read();
  if (!midimode)
  {
  // TODO move the pitch cv handling to a method
    bool releasing = fm.isReleasing();
    if (gate || releasing)
    {
  //    const float n = 10.0;
      static float saveraw = 0;
      
      long raw = ANALOG_MAX - adc->analogRead(CV_IN);
      int median = handleCVBuffer(raw, 1000); // maintain the median incase we are switched back
  //    if (quantise)
        saveraw = median;      
  //    else
  //      saveraw = (n - 1) * saveraw / n + 1.0 * raw  / n;
//    Serial.println(saveraw);
  /*
   * Odessa
   * 
     Calibration data
      C2 input is 33
      C7 input is 3489
      scaling factor f ~ 60 / (3489 - 33)
      pitch_cv = (raw - 33) * f;
   */
//      const float fbase = 149;
//      const float f = 60.0 / (3618 - fbase);

      const float fbase = 33;
      const float f = 60.0 / (3489 - fbase);
    
      float pitch_cv = (saveraw - fbase) * f;
      
      if (gate)
      {
//    Serial.printf("raw: %ld, median: %d, average: %f\n", raw, median, saveraw);
        bool newnote = false;
        if (quantise) 
        {
          pitch_cv = round(pitch_cv);
          newnote = abs(raw - saveraw) <= 5; // has it settled on a new value?
        }
        else
        {
          newnote = raw != saveraw;
        }
        if (newnote || gatetoggled)
        {
  //        Serial.printf("Pitch_cv/raw/saveraw/rawforlastnote = %f/%d/%.1f [%.1f] %d [%d] => %d\n", 
  //          pitch_cv + PITCH_OFFSET, raw, saveraw,  abs(raw - saveraw), note, gatetoggled, newnote);
          float notetoplay = PITCH_OFFSET + pitch_cv;
          if (gatetoggled || notetoplay != note)
          {
            if ((quantise && (gatetoggled && note >= 0)) || (!quantise && gatetoggled)) {
              fm.keyup((int)note);
            }
            note = PITCH_OFFSET + pitch_cv;
  //          Serial.printf("Note down: %f [%d]\n", note, (int)note);
            if (quantise)
              fm.keydown((int)note, 80);
            else
              fm.freq(note, 80);
          }
        }
        gatetoggled = false;
      }
      else 
      {
        if (releasing) {
  //        Serial.printf("Pitch_cv/raw/saveraw/rawforlastnote = %f/%d\n", pitch_cv + PITCH_OFFSET, round(PITCH_OFFSET + pitch_cv));
          fm.updatePitchOnly(quantise ? round(PITCH_OFFSET + pitch_cv) : PITCH_OFFSET + pitch_cv); 
        }
      }
    }

    if (!gate && note >= 0) // if a note somehow got stuck...
    {
      Serial.println("Emergency notes off");
      fm.keyup((int)note);
  //      fm.notesOff();
      note = -1;
    }
  }
  
  if (!idle && loopcount % 100 == 0) {
    idle = fm.isIdle();
    if (idle) {
      set_arm_clock(24000000);  
    }
  }

  checkSerialControl();
  handleResetButton();
}

typedef enum { NONE, PANIC, MIDI, CV, AFTERTOUCH } ResetState;

ResetState resetState = NONE;

void handleResetButton()
{
  if (loopcount < 10)
    return; // avoid initial transients
  resetsw->update();
  if (resetsw->read() == LOW)
  {
    long cd = resetsw->duration();
    if (cd > 4000)
    {
      pin_reset();
    }
    else if (cd > 3000)
    {
      if (resetState != AFTERTOUCH) resetState = AFTERTOUCH;
    }
    else if (cd > 2000)
    {
      if (resetState != CV) resetState = CV;
    }
    else if (cd > 1000)
    {
      if (resetState != MIDI) resetState = MIDI;
    }
    else if (cd > 100)
    {
      if (resetState != PANIC) resetState = PANIC;
    }
    handleResetLEDs();
  }
  else if (resetsw->rose()) 
  {
    Serial.printf("Reset clicked on state %d\n", resetState);
    switch (resetState)
    {
      case PANIC:
        fm.panic();
        break;
      case CV:
        setMidiMode(true);
        break;
      case MIDI:
        setMidiMode(false);
        break;
      case AFTERTOUCH:
        aftertouch = !aftertouch;
        Serial.printf("Aftertouch is %s\n", aftertouch ? "on" : "off");
        break;
      case NONE:
        break;
    }
    resetState = NONE;
    setAlgorithmLEDs(config.algorithm);
  }
}

void handleResetLEDs()
{
  digitalWrite(levelLEDs[3], resetState == PANIC);
  digitalWrite(levelLEDs[2], resetState == MIDI);
  digitalWrite(levelLEDs[1], resetState == CV);
  digitalWrite(levelLEDs[0], resetState == AFTERTOUCH);
}

void setMidiMode(bool set)
{
  if (set)
  {
    Serial.println("turning on midi mode");
    fm.setMaxNotes(POLYPHONY);
    midimode = true;
    note = -1;
    scaleModulators(controls.modvalue);
  }
  else
  {
    Serial.println("turning off midi mode");
    fm.setMaxNotes(1);
    midimode = false;
    scaleModulators(0);
  }
  updateall = true;
}

void handleNoteOn(byte channel, byte note, byte velocity) 
{
  if (midimode)
  {
    if (idle)
    {
      idle = false;
      set_arm_clock(600000000);  
    }
    fm.keydown((int16_t)note + MIDI_NOTE_OFFSET, (int8_t)velocity);
  }
}

void handleNoteOff(byte channel, byte note, byte velocity) 
{
  if (midimode)
    fm.keyup((int16_t)note + MIDI_NOTE_OFFSET);
}

void handlePitchChange(byte channel, int pitch) 
{
  if (midimode)
  {
    pitch = map(pitch, -8192, 8192, -7 * PITCH_BEND_FACTOR, 7 * PITCH_BEND_FACTOR);
    config.detune = pitch;
    fm.doRefreshVoice();
  }
}

void scaleModulators(byte amount)
{
  for (int i = 0; i < 4; ++i)
  {
    if (getOpType(i, config.algorithm) == MODULATOR)
    {
      float l = sqrt(config.level[i] / 100.0);
      float f = max(min((1 - l) + 0.1, 1), 0.1);
      config.scale[i] = 1 + f * (amount / 127.0);
//      int current = config.level[i];
//      int target = current * config.scale[i];
//      Serial.printf("%d => %.2f [%d => %d]\t", amount, config.scale[i], current, target);
    }
    else
      config.scale[i] = 1;
  }
  Serial.println();
  fm.doRefreshEnv();
}

void handleAfterTouchChannel(byte channel, byte pressure) 
{
  if (aftertouch)
    scaleModulators(controls.modvalue + pressure);
}

void handleControlChange(byte channel, byte control, byte value) 
{
  if (control == 1) // mod wheel
  {
    controls.modvalue = value;
    scaleModulators(value);
  }
}
