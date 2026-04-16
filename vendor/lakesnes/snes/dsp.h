
#ifndef DSP_H
#define DSP_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Dsp Dsp;

#include "apu.h"
#include "statehandler.h"

typedef struct DspChannel {
  // pitch
  uint16_t pitch;
  uint16_t pitchCounter;
  bool pitchModulation;
  // brr decoding
  int16_t decodeBuffer[12];
  uint8_t bufferOffset;
  uint8_t srcn;
  uint16_t decodeOffset;
  uint8_t blockOffset; // offset within brr block
  uint8_t brrHeader;
  bool useNoise;
  uint8_t startDelay;
  // adsr, envelope, gain
  uint8_t adsrRates[4]; // attack, decay, sustain, gain
  uint8_t adsrState; // 0: attack, 1: decay, 2: sustain, 3: release
  uint8_t sustainLevel;
  uint8_t gainSustainLevel;
  bool useGain;
  uint8_t gainMode;
  bool directGain;
  uint16_t gainValue; // for direct gain
  uint16_t preclampGain; // for bent increase
  uint16_t gain;
  // keyon/off
  bool keyOn;
  bool keyOff;
  // output
  int16_t sampleOut; // final sample, to be multiplied by channel volume
  int8_t volumeL;
  int8_t volumeR;
  bool echoEnable;
} DspChannel;

struct Dsp {
  Apu* apu;
  // mirror ram
  uint8_t ram[0x80];
  // 8 channels
  DspChannel channel[8];
  // overarching
  uint16_t counter;
  uint16_t dirPage;
  bool evenCycle;
  bool mute;
  bool reset;
  int8_t masterVolumeL;
  int8_t masterVolumeR;
  // accumulation
  int16_t sampleOutL;
  int16_t sampleOutR;
  int16_t echoOutL;
  int16_t echoOutR;
  // noise
  int16_t noiseSample;
  uint8_t noiseRate;
  // echo
  bool echoWrites;
  int8_t echoVolumeL;
  int8_t echoVolumeR;
  int8_t feedbackVolume;
  uint16_t echoBufferAdr;
  uint16_t echoDelay;
  uint16_t echoLength;
  uint16_t echoBufferIndex;
  uint8_t firBufferIndex;
  int8_t firValues[8];
  int16_t firBufferL[8];
  int16_t firBufferR[8];
  // sample ring buffer (1024 samples, *2 for stereo)
  int16_t sampleBuffer[0x400 * 2];
  uint16_t sampleOffset;      // producer-side (DSP write) cursor
  /* ThumbySNES: consumer-side (pull) cursor.
   *
   * Upstream dsp_getSamples always reads the LAST `wantedSamples`
   * (534 NTSC / 641 PAL, i.e. 1/60 sec worth) from the ring. That
   * assumes the caller pulls exactly once per emulated 60 Hz frame.
   *
   * On the Thumby Color we pull once per *rendered* frame, which
   * can be 7-25 fps when emulation is slow — between pulls the DSP
   * produces 1500-3000 samples but the original code only looks at
   * 534 of them. The rest get overwritten in the ring. Each pull's
   * resample ratio ends up different depending on how long the
   * previous frame took, which is exactly the "speeding up and
   * slowing down" audio wobble.
   *
   * `lastReadOffset` records where the consumer left off so
   * dsp_getSamples can resample the range [lastReadOffset,
   * sampleOffset] — the samples actually produced since the last
   * pull — into the caller's requested output length. Pitch stays
   * correct at any emulation framerate; tempo just slows gracefully
   * when SPC emulation is starved. */
  uint16_t lastReadOffset;
};

Dsp* dsp_init(Apu* apu);
void dsp_free(Dsp* dsp);
void dsp_reset(Dsp* dsp);
void dsp_handleState(Dsp* dsp, StateHandler* sh);
void dsp_cycle(Dsp* dsp);
uint8_t dsp_read(Dsp* dsp, uint8_t adr);
void dsp_write(Dsp* dsp, uint8_t adr, uint8_t val);
void dsp_getSamples(Dsp* dsp, int16_t* sampleData, int samplesPerFrame);

#endif
