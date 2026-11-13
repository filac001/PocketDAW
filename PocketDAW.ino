#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/i2s.h"

#define USE_BT 1                       // 1 = Bluetooth A2DP headphones, 0 = wired output only (GPIO25)
#if USE_BT
#include "BluetoothA2DPSource.h"
#include "esp_bt.h"
#include "esp_heap_caps.h"
#endif
#define BT_HEAP_RESERVE (USE_BT ? 70000 : 30000)   // v26: heap reserve for the BT stack when CONNECTING (without PSRAM); measurements showed BT stayed >20 kB free
#define BT_FREE_FLOOR   9000                       // v26: how much free heap remains once BT is connected and the bank is expanded
#define BT_LOAD_FLOOR   6000                       // v26: bankLoadLane never lowers the free heap below this while BT is running

// ---------------- PINS ----------------
// SD (VSPI): CS=5, SCK=18, MISO=19, MOSI=23 | OLED: SDA=21, SCL=22 | AUDIO: GPIO25
#define SD_CS 5
enum { B_UP, B_DN, B_LF, B_RT, B_CE, B_PL, B_MI, B_C1, B_C2, B_C3, B_C4, NB };
const uint8_t PIN_B[NB] = { 27, 12, 32, 33, 4, 16, 17, 13, 14, 15, 3 };

#define SR    22050
#define SAMPLE_SR 16000              // better sample quality within the 0.5 s limit
#define SAMPLE_MAX (SAMPLE_SR / 2)   // max samples per sample at 16 kHz (0.5 s)
#define DECIM_ERR 0.0005f            // allowed storage error at the lower rate (energy ratio, ~ -33 dB = below the 8-bit noise floor)
#define BUFN  128
#define NVMAX 6     // max voices per oscillator channel (CH4 = 6, CH2/CH3 = 4)
#define MAXE  64    // max entries in a folder that the browser can see

struct Sample { uint8_t* d = nullptr; uint32_t n = 0; uint16_t sr = SAMPLE_SR; };   // sr = sample rate of the stored sample
struct WavInfo { uint32_t n = 0; uint8_t div = 1; };       // n = length without silence (at 16 kHz), div = 1/2/4 -> 16000/8000/4000 Hz
typedef void (*EmitFn)(void*, float);                      // decoder callback (must be declared before the first function because of Arduino auto-prototypes)

inline int nvOf(int c) { return c == 3 ? 6 : 4; }   // number of voices of the channel
inline bool isDrum(int c) { return c == 0; }        // only CH1 is the drum/sampler

// ---------------- DATA ----------------
struct Pattern {
  union {
    struct {                       // CH1 (drum)
      uint8_t drum[128];            // bitmask of 8 lanes
      int8_t  dpitch[128][8];       // pitch of EVERY step in semitones (-12..+12)
    };
    struct {                       // CH2 / CH3 / CH4 (oscillators)
      uint8_t note[128][NVMAX];     // up to 4 (CH2/CH3) or 6 (CH4) notes per step, 0 = empty
      uint8_t len[128];             // step length in 1/16 notes (shared by all notes of the step)
    };
  };
};
struct Chan {
  Pattern pat[4];
  uint8_t song[128] = {0};            // pattern chain: 1..4, 0 = end
  uint8_t psteps[4] = { 16, 16, 16, 16 };   // number of steps of EACH pattern (P1..P4): 8/16/32/64/128
  uint8_t vol = 200;
  int8_t  pitch = 0;                 // pitch of the whole channel (FX menu), in semitones
  uint8_t lp = 255, hp = 0, crush = 8, wave = 0;
  uint8_t atk = 5, dec = 100, sus = 200, rel = 80;   // ADSR
  bool    mute = false;
  uint8_t songPos = 0, step = 0;
  float   lpS = 0, hpS = 0;
};
Chan ch[4];
// Micro timing is kept separate from the Pattern union for SM5-SM9 compatibility.
// A value of -16..+16 shifts the step by up to +/- 1/4 of a 1/16 note.
int8_t microTiming[4][4][128] = {{{0}}};
// Per-step velocity: 1..127 (127 = full volume = old behavior). Applies to all lanes/notes of that step.
uint8_t stepVel[4][4][128];                 // [channel][pattern][step]; filled with 127 in setup()
inline float velOf(int c, int p, int s) { return stepVel[c][p][s] / 127.0f; }
// Swing per channel (50 = straight .. 75 = heavy swing). Swing in the transport bar sets all channels at once.
volatile uint8_t swingCh[4] = { 50, 50, 50, 50 };

// Copy/paste of a single pattern (includes note/drum pitch, lengths and micro timing).
Pattern patternClipboard;
int8_t clipboardMicro[128];
uint8_t clipboardVel[128];
uint8_t clipboardSteps = 16;
bool clipboardValid = false;
const char* editMsg = "";
uint32_t editMsgUntil = 0;
bool microMode = false;

// Momentary performance FX: C1=beat repeat, C2=delay, C3=gradual high-pass, C4=low-pass; release = OFF.
volatile uint8_t performanceFx = 0;
int16_t perfDelay[4096] = {0};            // v26: int16 (Q15) instead of float -> 8 kB less
int16_t perfStutter[2048] = {0};          // v26: int16 (Q15) instead of float -> 4 kB less
inline int16_t toQ15(float x) { if (x > 1.0f) x = 1.0f; if (x < -1.0f) x = -1.0f; return (int16_t)(x * 32767.0f); }
#define Q15F (1.0f / 32767.0f)
uint16_t perfStutterPos = 0;
uint16_t perfDelayPos = 0;
float perfFilter = 0.0f, perfTapeGain = 1.0f;
uint32_t perfPhase = 0;
uint8_t perfLastFx = 0;

// Chain = 128 cells (1 cell = 16 steps). A pattern of 16/32/64/128 steps occupies 1/2/4/8 cells.
// song[i] is only valid at the START of a pattern; the cells it covers are empty (normalizeChain clears them).
inline int cellsOf(int steps) { return steps < 16 ? 1 : steps / 16; }
inline int spanOf(int c, int cell) { uint8_t p = ch[c].song[cell]; return p ? cellsOf(ch[c].psteps[p - 1]) : 1; }
int blockStart(int c, int cell) {              // start of the pattern that contains this cell
  if (cell < 0) cell = 0; if (cell > 127) cell = 127;
  int i = 0, p = 0;
  while (i <= cell && i < 128) { p = i; i += spanOf(c, i); }
  return p;
}
int prevBlock(int c, int cell) {               // start of the previous pattern/cell
  int i = 0, p = 0;
  while (i < cell && i < 128) { p = i; i += spanOf(c, i); }
  return p;
}
void normalizeChain(int c) {                   // a pattern REPLACES the cells it occupies
  int i = 0;
  while (i < 128) {
    int n = spanOf(c, i);
    for (int k = 1; k < n && i + k < 128; k++) ch[c].song[i + k] = 0;
    i += n;
  }
}

struct OscV { float ph = 0, freq = 440, env = 0; int32_t gate = 0; uint8_t st = 0; float vel = 1; };  // st: 0 idle, 1 A, 2 D, 3 S, 4 R
OscV ov[3][NVMAX];                    // [CH2/CH3/CH4][voice]

struct Voice  { float pos = 0, inc = 1, env = 1; bool on = false; float vel = 1; };   // inc = read speed (pitch of that hit)
struct SampleFx {
  uint8_t start = 0, end = 100;       // trim in percent
  int8_t pitch = 0, fine = 0;         // semitones and cents
  uint8_t speed = 100;                // playback speed in percent
  uint8_t attack = 0, decay = 0, sustain = 100, release = 0; // ADSR in ms / percent
  uint8_t fadeIn = 0, fadeOut = 8;    // percent of the duration
  uint8_t choke = 0;                  // 0 = off, 1..4 = group
  bool reverse = false, normalize = false, loop = false;
  float normGain = 1.0f;
};
SampleFx sampleFx[8];
Sample smp[8];                        // one sample bank (CH1)
Voice  vc[8];
uint8_t drumVol[8] = {255,255,255,255,255,255,255,255}; // volume of each sample in the drum rack
bool drumVolMode = false;
char kitPath[64];                     // current kit (folder)
uint32_t bankBudget = 200000;         // bytes available for samples (computed at power-on)
bool wavRamWarning = false;            // show a warning if some WAV was not loaded
// ---- v25: lazy bank ----
char     laneFile[8][32];             // WAV file name per lane (in the kitPath folder)
WavInfo  laneInfo[8];                 // analysis result (length, decimation)
bool     laneHas[8];                  // lane has a valid WAV in the kit
uint32_t laneAlloc[8];                // bytes the lane currently holds in RAM (0 = not loaded)
bool     kitSkipped = false;          // some WAV in the kit is unsupported/missing
bool     bankShort = false;           // v26: a used lane is not fully in RAM
int16_t  bankFailWant = -1;           // last lane mask for which loading failed (to avoid looping)

volatile bool playing = false;
bool ramOverlay = false;  // RAM monitor overlay on the main screen
volatile bool audioPauseReq = false, audioPaused = false;
bool audioTaskStarted = false;
volatile uint16_t bpm = 120;
volatile uint8_t swing = 50;          // 50 = straight, 75 = heavy swing
volatile float gsps = SR * 60.0f / 120 / 4;
float sinT[256];
float la[4], ha[4], qn[4];
float aR[3], dR[3], rR[3], sL[3];
bool sdOk = false;

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
const char* CN[4] = { "DRM", "BAS", "MEL", "CHD" };
const char* NN[12] = { "C","C#","D","D#","E","F","F#","G","G#","A","A#","B" };
const uint8_t STEPS_OPT[5] = { 8, 16, 32, 64, 128 };
bool validSteps(int v) { for (int i = 0; i < 5; i++) if (STEPS_OPT[i] == v) return true; return false; }

// ---------------- BUTTONS ----------------
struct Btn { bool down, press, rel, lng, lngDone; uint32_t t, last, rep; };
Btn btn[NB];

void scanButtons() {
  uint32_t now = millis();
  for (int i = 0; i < NB; i++) {
    Btn& b = btn[i];
    b.press = b.rel = b.lng = false;
    bool d = !digitalRead(PIN_B[i]);
    if (d != b.down && now - b.last > 20) {
      b.last = now;
      b.down = d;
      if (d) { b.t = now; b.rep = now + 400; b.lngDone = false; b.press = true; }
      else if (!b.lngDone) b.rel = true;
    }
    if (b.down) {
      if (!b.lngDone && now - b.t > 600) { b.lngDone = true; b.lng = true; }
      if (i != B_CE && now > b.rep) { b.press = true; b.rep = now + 70; }   // auto-repeat
    }
  }
}

// ---------------- AUDIO ----------------
inline float noteFreq(int n, int pitch) { return 440.0f * powf(2.0f, (n - 69 + pitch) / 12.0f); }

void triggerStep(int c) {
  Chan& C = ch[c];
  // Empty slots ('-') consume their time and stay silent.
  // Do not jump back to the start as soon as you hit an empty slot: chains such as --12
  // must pass through the first two silent slots and then play pattern 1, then 2.
  uint8_t pid = C.song[C.songPos];
  int plen = pid ? C.psteps[pid - 1] : 16;   // steps in this pattern
  if (C.step >= plen) C.step = 0;
  // A pattern only plays if it is actually selected (1..4) and the channel is not muted.
  if (pid && !C.mute) {
    Pattern& P = C.pat[pid - 1];
    float vel = stepVel[c][pid - 1][C.step] / 127.0f;   // velocity of this step
    if (isDrum(c)) {
      uint8_t m = P.drum[C.step];
      for (int l = 0; l < 8; l++) if (m & (1 << l)) {
        SampleFx& fx = sampleFx[l];
        if (fx.choke) for (int q = 0; q < 8; q++) if (q != l && sampleFx[q].choke == fx.choke) vc[q].on = false;
        float semis = C.pitch + P.dpitch[C.step][l] + fx.pitch + fx.fine / 100.0f;
        vc[l].inc = (smp[l].sr / (float)SR) * (fx.speed / 100.0f) * powf(2.0f, semis / 12.0f);
        uint32_t st = (uint32_t)((uint64_t)smp[l].n * fx.start / 100);
        uint32_t en = (uint32_t)((uint64_t)smp[l].n * fx.end / 100);
        if (en <= st + 1) { st = 0; en = smp[l].n; }
        vc[l].pos = fx.reverse ? (float)(en - 1) : (float)st;
        vc[l].env = 0.0f; vc[l].vel = vel; vc[l].on = smp[l].d && en > st;
      }
    } else {
      int k = c - 1, nv = nvOf(c);
      int len = P.len[C.step]; if (len < 1) len = 1;
      for (int v = 0; v < nv; v++) {
        uint8_t n = P.note[C.step][v];
        if (!n) continue;
        OscV& o = ov[k][v];
        o.freq = noteFreq(n, C.pitch);
        o.gate = (int32_t)(len * gsps);
        o.vel = vel;
        o.st = 1;                       // attack (the envelope continues from the current level)
      }
    }
  } else if (!pid) {
    // In an empty slot, stop any tail left over from the previous pattern.
    if (isDrum(c)) {
      for (int l = 0; l < 8; l++) vc[l].on = false;
    } else {
      int k = c - 1;
      for (int v = 0; v < NVMAX; v++) { ov[k][v].st = 0; ov[k][v].env = 0; ov[k][v].gate = 0; }
    }
  }
  if (++C.step >= plen) {
    C.step = 0;
    C.songPos += pid ? cellsOf(C.psteps[pid - 1]) : 1;   // skip the cells the pattern occupies
    // Empty slots between patterns stay silent but are still traversed.
    // We only wrap around at the end of the chain once no patterns remain after the current position.
    bool anyLater = false;
    for (int i = C.songPos; i < 128; i++) {
      if (C.song[i]) { anyLater = true; break; }
    }
    if (C.songPos >= 128 || !anyLater) C.songPos = 0;
  }
}

// Micro offset in samples; event scheduling is tracked separately for each channel.
int32_t stepMicroSamples(int c, float sps) {
  Chan& C = ch[c];
  uint8_t pid = C.song[C.songPos];
  if (!pid) return 0;
  int st = C.step;
  int v = microTiming[c][pid - 1][st];
  return (int32_t)(constrain(v, -16, 16) * (sps / 64.0f));
}

void resetPlayheads() { for (int c = 0; c < 4; c++) { ch[c].step = 0; ch[c].songPos = 0; } }
void togglePlay() { if (playing) playing = false; else { resetPlayheads(); playing = true; } }

void prepBuf() {
  for (int c = 0; c < 4; c++) {
    Chan& C = ch[c];
    float l = C.lp / 255.0f, h = C.hp / 255.0f;
    la[c] = 0.005f + l * l * 0.99f;
    ha[c] = C.hp ? 0.001f + h * h * 0.5f : 0.0f;
    qn[c] = (C.crush >= 8) ? 0 : (float)(1 << C.crush) / 2.0f;
  }
  for (int k = 0; k < 3; k++) {          // ADSR -> per-sample rates
    Chan& C = ch[k + 1];
    float x = C.atk / 255.0f; aR[k] = 1.0f / ((0.001f + x * x * 1.0f) * SR);   // attack 1 ms .. 1 s
    x = C.dec / 255.0f;       dR[k] = 1.0f / ((0.005f + x * x * 1.5f) * SR);   // decay 5 ms .. 1.5 s
    x = C.rel / 255.0f;       rR[k] = 1.0f / ((0.005f + x * x * 2.0f) * SR);   // release 5 ms .. 2 s
    sL[k] = C.sus / 255.0f;
  }
}

inline float waveOf(uint8_t w, float ph) {
  switch (w) {
    case 0: return 2 * ph - 1;                                   // saw
    case 1: return ph < 0.5f ? 1.0f : -1.0f;                     // square
    case 2: return ph < 0.5f ? 4 * ph - 1 : 3 - 4 * ph;          // triangle
    default: return sinT[(int)(ph * 256) & 255];                 // sine
  }
}

inline float renderCh(int c) {
  Chan& C = ch[c];
  float raw = 0;
  if (isDrum(c)) {
    for (int l = 0; l < 8; l++) {
      Voice& v = vc[l];
      if (!v.on) continue;
      SampleFx& fx = sampleFx[l];
      uint32_t st = (uint32_t)((uint64_t)smp[l].n * fx.start / 100);
      uint32_t en = (uint32_t)((uint64_t)smp[l].n * fx.end / 100);
      if (en <= st + 1) { st = 0; en = smp[l].n; }
      if (fx.reverse) {
        if (v.pos < (float)st) { if (fx.loop) v.pos = (float)(en - 1); else { v.on = false; continue; } }
      } else if (v.pos >= (float)en) {
        if (fx.loop) v.pos = (float)st; else { v.on = false; continue; }
      }
      uint32_t i = (uint32_t)v.pos;
      if (i >= smp[l].n) { v.on = false; continue; }
      uint32_t i1 = fx.reverse ? (i > st ? i - 1 : i) : ((i + 1 < en) ? i + 1 : i);
      float frac = v.pos - (float)i;
      float a = ((int)smp[l].d[i] - 128) * (1.0f / 128);
      float b = ((int)smp[l].d[i1] - 128) * (1.0f / 128);
      float x = a + (b - a) * fabsf(frac);
      float total = (float)(en - st);
      float relPos = fx.reverse ? (float)(en - 1) - v.pos : v.pos - st;
      float fadeInN = total * fx.fadeIn / 100.0f;
      float fadeOutN = total * fx.fadeOut / 100.0f;
      float fade = 1.0f;
      if (fadeInN > 1 && relPos < fadeInN) fade = relPos / fadeInN;
      if (fadeOutN > 1 && relPos > total - fadeOutN) fade = fminf(fade, (total - relPos) / fadeOutN);
      float atkN = fx.attack * SR / 1000.0f;
      float decN = fx.decay * SR / 1000.0f;
      float relN = fx.release * SR / 1000.0f;
      float env = 1.0f;
      if (atkN > 1 && relPos < atkN) env = relPos / atkN;
      else if (decN > 1 && relPos < atkN + decN) env = 1.0f - (1.0f - fx.sustain/100.0f) * ((relPos-atkN)/decN);
      else env = fx.sustain / 100.0f;
      if (relN > 1 && relPos > total - relN) env *= fmaxf(0.0f, (total-relPos)/relN);
      env *= fade;
      if (fx.normalize) x *= fx.normGain;
      raw += x * env * v.vel * (drumVol[l] / 255.0f);
      v.pos += fx.reverse ? -v.inc : v.inc;
      (void)relN;
    }
    raw *= 0.5f;
  } else {
    int k = c - 1, nv = nvOf(c);
    float sum = 0;
    for (int v = 0; v < nv; v++) {
      OscV& o = ov[k][v];
      if (!o.st) continue;
      o.ph += o.freq * (1.0f / SR); if (o.ph >= 1) o.ph -= 1;
      if (o.gate > 0) { if (--o.gate == 0 && o.st < 4) o.st = 4; }   // end of note -> release
      switch (o.st) {
        case 1: o.env += aR[k]; if (o.env >= 1) { o.env = 1; o.st = 2; } break;
        case 2: o.env -= dR[k]; if (o.env <= sL[k]) { o.env = sL[k]; o.st = 3; } break;
        case 3: break;
        case 4: o.env -= rR[k]; if (o.env <= 0) { o.env = 0; o.st = 0; } break;
      }
      sum += waveOf(C.wave, o.ph) * o.env * o.vel;
    }
    raw = sum * (1.4f / nv);            // 4 voices -> 0.35 (as before), 6 voices -> 0.23
  }
  if (C.mute) return 0;
  C.lpS += la[c] * (raw - C.lpS);          // lo-pass
  float y = C.lpS;
  C.hpS += ha[c] * (y - C.hpS);            // hi-pass
  y -= C.hpS;
  if (qn[c] > 0) y = roundf(y * qn[c]) / qn[c];   // bitcrush
  return y * (C.vol / 255.0f);
}

// ---------------- BLUETOOTH (A2DP source) ----------------
// audioTask still fills the DAC (GPIO25); in addition, each block is copied into a ring buffer,
// and the A2DP callback reads it, doubles it to 44100 Hz and sends it to the headphones.
#if USE_BT
#define BTRB       4096                 // ring buffer (mono 16-bit, 22050 Hz), power of two
#define BT_TARGET  1280                 // v26: target fill (~58 ms): the A2DP callback requests blocks of ~512 samples at once
#define BT_MAXFILL 3072                 // v26: above this the whole block is dropped (the callback has stalled)
#define BT_MAXDEV  8
BluetoothA2DPSource a2dp;
bool btA2dpStarted = false;
char btName[24]   = {0};                // saved headphones (connect automatically at power-on)
char btTarget[24] = {0};                // what we are currently connecting to
char btDev[BT_MAXDEV][24];              // discovered devices
volatile int btDevN = 0;
int btSel = 0, btTop = 0;
int16_t btRb[BTRB];
volatile uint32_t btW = 0, btR = 0;
volatile bool btPrebuf = true;          // true = waiting for the ring to fill up to BT_TARGET before playback starts
volatile uint32_t btCbCnt = 0, btMatchCnt = 0;   // diagnostics: how many devices were reported / how many times they matched btTarget
volatile bool btRestartReq = false;     // request for a rescan (executed in loop(), never from the BT callback)
uint32_t btScanT = 0;                   // when the last scan was started
#define BT_RETRY_MS 30000               // how long to wait before rescanning if the saved device was not found
#define BT_SCAN_MS  12000               // inquiry lasts ~10 s

bool btConnected() { return btA2dpStarted && a2dp.is_connected(); }

// Called by audioTask (core 1) every block. A block is either written entirely or dropped entirely (no partial writes).
void btPush(const int16_t* d, int n) {
  uint32_t w = btW;
  if ((int32_t)(w - btR) + n > BT_MAXFILL) return;
  for (int i = 0; i < n; i++) btRb[(w + i) & (BTRB - 1)] = d[i];
  __sync_synchronize();
  btW = w + n;
}

// Called by the BT stack: 22050 Hz mono -> 44100 Hz stereo.
// v26: cubic (Catmull-Rom) resampler with a variable step. The step is 0.5 +-0.5 % and is regulated by the
// ring fill level (P-controller), so the difference between the I2S and BT clocks does not lead to an empty/overfull buffer.
int32_t btData(Frame* f, int32_t n) {
  static float h0 = 0, h1 = 0, h2 = 0, h3 = 0, ph = 0, gain = 0;
  uint32_t r = btR, w = btW;
  __sync_synchronize();
  int32_t fill = (int32_t)(w - r);
  if (btPrebuf) {
    if (fill < BT_TARGET) {                                  // still building up a reserve -> silence
      for (int32_t i = 0; i < n; i++) { f[i].channel1 = 0; f[i].channel2 = 0; }
      return n;
    }
    btPrebuf = false;
    h0 = h1 = h2 = h3 = 0; ph = 0; gain = 0;                 // clean start + fade-in
  }
  float adj = (float)(fill - BT_TARGET) * 5.0e-6f;           // more reserve -> consume slightly faster, less -> slower
  if (adj > 0.005f) adj = 0.005f; else if (adj < -0.005f) adj = -0.005f;
  const float step = 0.5f * (1.0f + adj);
  float yl = 0;
  int32_t i = 0;
  bool under = false;
  for (; i < n && !under; i++) {
    float t = ph;
    float y = h1 + 0.5f * t * (h2 - h0 + t * (2.0f * h0 - 5.0f * h1 + 4.0f * h2 - h3 + t * (3.0f * (h1 - h2) + h3 - h0)));
    if (gain < 1.0f) { gain += 1.0f / 256.0f; if (gain > 1.0f) gain = 1.0f; y *= gain; }
    if (y > 32767.0f) y = 32767.0f; else if (y < -32768.0f) y = -32768.0f;
    yl = y;
    int16_t o = (int16_t)y;
    f[i].channel1 = o; f[i].channel2 = o;
    ph += step;
    while (ph >= 1.0f) {
      if (r == w) { w = btW; __sync_synchronize(); }         // the producer may have added more in the meantime
      if (r == w) { under = true; break; }
      ph -= 1.0f;
      h0 = h1; h1 = h2; h2 = h3; h3 = (float)btRb[r & (BTRB - 1)]; r++;
    }
  }
  btR = r;
  if (under) {                                               // real underrun: short fade-out, then prebuffering again
    btPrebuf = true;
    for (; i < n; i++) { yl *= 0.96f; f[i].channel1 = (int16_t)yl; f[i].channel2 = (int16_t)yl; }
  }
  return n;
}

// Called for every discovered device: stores the name in the list; true = connect to this one
bool btSsidCb(const char* ssid, esp_bd_addr_t addr, int rssi) {
  if (!ssid || !ssid[0]) return false;
  int n = btDevN, found = -1;
  for (int i = 0; i < n; i++) if (!strncmp(btDev[i], ssid, sizeof(btDev[i]) - 1)) { found = i; break; }
  if (found < 0 && n < BT_MAXDEV) {
    strncpy(btDev[n], ssid, sizeof(btDev[n]) - 1);
    btDev[n][sizeof(btDev[n]) - 1] = 0;
    btDevN = n + 1;
  }
  btCbCnt++;
  bool match = btTarget[0] && !strncmp(ssid, btTarget, sizeof(btTarget) - 1);
  if (match) btMatchCnt++;
  Serial.printf("[BT] found '%s' rssi=%d match=%d\n", ssid, rssi, (int)match);
  return match;
}

void btBegin() {
  if (btA2dpStarted) return;
  btA2dpStarted = true;
  a2dp.set_ssid_callback(btSsidCb);
  a2dp.set_data_callback_in_frames(btData);
  a2dp.set_auto_reconnect(true);
  a2dp.set_volume(90);                  // 0..127
  if (btTarget[0]) a2dp.start(btTarget);  // known name -> standard path, connect by name
  else             a2dp.start();          // no name -> scan + btSsidCb
  btScanT = millis();
}

// Scanning in the A2DP library lasts only ~10 s and the callback is called ONLY during scanning.
// So to (re)connect, the scan has to be started anew: end() + start().
void btTick() {
  if (!btA2dpStarted) return;
  uint32_t now = millis();
  bool connecting = (a2dp.get_connection_state() == ESP_A2D_CONNECTION_STATE_CONNECTING);
  if (!btRestartReq && btTarget[0] && !a2dp.is_connected() && !connecting && now - btScanT > BT_RETRY_MS) btRestartReq = true;
  if (btRestartReq && now - btScanT > 1500) {     // at least 1.5 s between restarts
    btRestartReq = false;
    Serial.printf("[BT] restart scan, target='%s'\n", btTarget);
    a2dp.end();
    delay(300);
    if (btTarget[0]) a2dp.start(btTarget); else a2dp.start();
    btScanT = millis();
  }
}
#endif

void audioTask(void*) {
  static uint16_t buf[BUFN * 2];
#if USE_BT
  static int16_t btBuf[BUFN];
#endif
  uint64_t sampleClock = 0;
  uint64_t nextDue[4] = {0,0,0,0};
  uint32_t stepClock[4] = {1,1,1,1};
  int32_t lastMicro[4] = {0,0,0,0};
  bool wasPlaying = false;
  for (;;) {
    if (audioPauseReq) {
      audioPaused = true;
      while (audioPauseReq) vTaskDelay(1);
      audioPaused = false;
    }
    prepBuf();
    float sps = SR * 60.0f / bpm / 4.0f;
    gsps = sps;
    if (playing && !wasPlaying) {
      resetPlayheads();
      sampleClock = 0;
      for (int c = 0; c < 4; c++) { nextDue[c] = 0; stepClock[c] = 1; lastMicro[c] = 0; }
    }
    wasPlaying = playing;
    for (int i = 0; i < BUFN; i++) {
      if (playing) {
        // Each channel has its own schedule: the micro offset can be earlier or later.
        for (int c = 0; c < 4; c++) if (sampleClock >= nextDue[c]) {
          triggerStep(c);
          int32_t nextMicro = stepMicroSamples(c, sps);
          float sw = swingCh[c] / 100.0f;                         // per-channel swing
          float interval = (stepClock[c] & 1) ? 2.0f * sps * sw : 2.0f * sps * (1.0f - sw);
          int32_t delta = (int32_t)interval + nextMicro - lastMicro[c];
          if (delta < 1) delta = 1;
          nextDue[c] += (uint32_t)delta;
          lastMicro[c] = nextMicro;
          stepClock[c]++;
        }
      }
      float mix = 0;
      for (int c = 0; c < 4; c++) mix += renderCh(c);
      mix *= 0.5f;

      // Performance FX: C1 beat-repeat, C2 delay, C3 slow high-pass, C4 low-pass.
      // The filters sweep over ~1.5 s while the button is held; releasing turns them off immediately.
      uint8_t fx = performanceFx;
      if (fx != perfLastFx) {
        perfPhase = 0;
        perfFilter = mix;
        perfLastFx = fx;
      }

      if (fx == 1) { // BEAT REPEAT: repeats a short rhythmic audio clip
        mix = perfStutter[perfStutterPos] * Q15F;
        perfStutterPos = (perfStutterPos + 1) & 2047;
      } else {
        perfStutter[perfStutterPos] = toQ15(mix);
        perfStutterPos = (perfStutterPos + 1) & 2047;
      }

      if (fx == 2) { // DELAY: the existing delay effect
        float d = perfDelay[perfDelayPos] * Q15F;
        perfDelay[perfDelayPos] = toQ15(mix + d * 0.38f);
        perfDelayPos = (perfDelayPos + 1) & 4095;
        mix = mix * 0.72f + d * 0.42f;
      } else {
        perfDelay[perfDelayPos] = toQ15(mix);
        perfDelayPos = (perfDelayPos + 1) & 4095;
      }

      if (fx == 3) { // HIGH-PASS: cutoff slowly rises from 100 Hz to 2 kHz
        float progress = perfPhase / (SR * 1.5f);
        if (progress > 1.0f) progress = 1.0f;
        float cutoff = 100.0f + progress * 1900.0f;
        float a = 6.2831853f * cutoff / SR;
        if (a > 0.95f) a = 0.95f;
        perfFilter += a * (mix - perfFilter);
        mix -= perfFilter;
        perfPhase++;
      } else if (fx == 4) { // LOW-PASS: cutoff slowly falls from 10 kHz to 250 Hz
        float progress = perfPhase / (SR * 1.5f);
        if (progress > 1.0f) progress = 1.0f;
        float cutoff = 10000.0f - progress * 9750.0f;
        float a = 6.2831853f * cutoff / SR;
        if (a > 0.95f) a = 0.95f;
        perfFilter += a * (mix - perfFilter);
        mix = perfFilter;
        perfPhase++;
      }

      perfTapeGain = 1.0f;
      mix *= perfTapeGain;
      if (mix > 1) mix = 1; if (mix < -1) mix = -1;
      uint16_t s = ((uint16_t)(mix * 127 + 128)) << 8;
      buf[2 * i] = s; buf[2 * i + 1] = s;
#if USE_BT
      btBuf[i] = (int16_t)(mix * 32000.0f);
#endif
      sampleClock++;
    }
#if USE_BT
    if (btConnected()) btPush(btBuf, BUFN);
    else { btR = btW; btPrebuf = true; }                       // not connected: empty ring, and prebuffering again when connecting
#endif
    size_t w;
    i2s_write(I2S_NUM_0, buf, sizeof(buf), &w, portMAX_DELAY);
  }
}

void initAudio() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SR,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFN,
    .use_apll = false
  };
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, NULL);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);   // RIGHT = GPIO25
  if (xTaskCreatePinnedToCore(audioTask, "audio", 4096, NULL, 3, NULL, USE_BT ? 1 : 0) == pdPASS)
    audioTaskStarted = true;  // the BT stack occupies core 0
}

// ---------------- SPLASH ----------------
void splashBar(const char* msg, int pct) {      // pct < 0 = no bar
  oled.clearDisplay(); oled.setTextColor(WHITE);
  oled.setTextSize(2); oled.setCursor(10, 8); oled.print("PocketDaw");
  oled.setTextSize(1);
  oled.setCursor((128 - (int)strlen(msg) * 6) / 2, 30); oled.print(msg);
  if (pct >= 0) {
    if (pct > 100) pct = 100;
    oled.drawRect(14, 46, 100, 10, WHITE);
    oled.fillRect(16, 48, (96 * pct) / 100, 6, WHITE);
  }
  oled.display();
}
void splash(const char* msg) { splashBar(msg, -1); }

// bar range while loading a kit (setup: 20..100 %, later 0..100 %)
int kitBarLo = 0, kitBarHi = 100;
int kitPct(int num, int den) { return kitBarLo + (kitBarHi - kitBarLo) * num / den; }

// ---------------- SD: WAV ----------------
// Reads the WAV header: finds the "fmt " and "data" chunks
bool parseWav(File& f, uint16_t& chn, uint32_t& rate, uint16_t& bits, uint32_t& dataLen) {
  uint8_t h[12];
  if (f.read(h, 12) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) return false;
  bool gotFmt = false;
  uint8_t ck[8];
  while (f.read(ck, 8) == 8) {
    uint32_t sz = ck[4] | (ck[5] << 8) | (ck[6] << 16) | ((uint32_t)ck[7] << 24);
    if (!memcmp(ck, "fmt ", 4)) {
      uint8_t b[16];
      if (sz < 16 || f.read(b, 16) != 16) return false;
      uint16_t fmt = b[0] | (b[1] << 8);
      chn  = b[2] | (b[3] << 8);
      rate = b[4] | (b[5] << 8) | (b[6] << 16) | ((uint32_t)b[7] << 24);
      bits = b[14] | (b[15] << 8);
      if (fmt != 1 && fmt != 0xFFFE) return false;          // PCM only
      f.seek(f.position() + (sz - 16) + (sz & 1));
      gotFmt = true;
    } else if (!memcmp(ck, "data", 4)) {
      dataLen = sz;
      return gotFmt;
    } else {
      f.seek(f.position() + sz + (sz & 1));                 // skip LIST and other chunks
    }
  }
  return false;
}

// Decoder: WAV (8/16 bit, mono/stereo, any sample rate) -> mono float (-1..1) at tgtRate.
// Averaging provides anti-aliasing. Calls emit() for every output sample, returns the number of samples (0 = error).
uint32_t decodeWav(File& f, uint32_t tgtRate, uint32_t maxOut, EmitFn emit, void* ctx) {
  f.seek(0);
  uint16_t chn = 0, bits = 0; uint32_t rate = 0, dataLen = 0;
  if (!parseWav(f, chn, rate, bits, dataLen)) return 0;
  if (chn < 1 || chn > 2 || (bits != 8 && bits != 16) || rate < 4000 || rate > 192000) return 0;
  uint32_t fb = chn * (bits / 8);                           // bytes per frame
  uint32_t avail = f.size() - f.position();
  if (dataLen > avail) dataLen = avail;
  uint32_t frames = dataLen / fb;
  float step = (float)rate / tgtRate;                       // source frames per output sample
  uint32_t outN = (uint32_t)(frames / step);
  if (outN > maxOut) outN = maxOut;
  if (outN < 16) return 0;

  uint8_t blk[512];
  uint32_t perBlk = (sizeof(blk) / fb) * fb;
  uint32_t remaining = frames * fb, outI = 0;
  float acc = 0, sum = 0, last = 0; int cnt = 0;
  while (outI < outN && remaining >= fb) {
    uint32_t want = remaining < perBlk ? remaining : perBlk;
    uint32_t got = f.read(blk, want);
    if (got < fb) break;
    remaining -= got;
    for (uint32_t o = 0; o + fb <= got && outI < outN; o += fb) {
      float x = 0;
      for (int c = 0; c < chn; c++) {
        if (bits == 16) x += (int16_t)(blk[o + c * 2] | (blk[o + c * 2 + 1] << 8)) / 32768.0f;
        else            x += ((int)blk[o + c] - 128) / 128.0f;
      }
      x /= chn;
      sum += x; cnt++; acc += 1;
      while (acc >= step && outI < outN) {
        float v = cnt ? sum / cnt : last;                    // averaging (anti-alias)
        last = v; sum = 0; cnt = 0; acc -= step;
        emit(ctx, v); outI++;
      }
    }
  }
  return outI;
}

// ---- analysis: where the sound really ends and how far the storage rate can be reduced without loss ----
struct AnaCtx { uint32_t i, lastActive; float x[8]; double e2, e4, tot; };
void emitAna(void* c, float v) {
  AnaCtx* a = (AnaCtx*)c;
  uint32_t i = a->i;
  a->x[i & 7] = v;
  a->tot += (double)v * v;
  if (fabsf(v) > 0.012f) a->lastActive = i + 1;             // above the 8-bit noise threshold (|q-128| > 1)
  if (i >= 2 && (i & 1) == 0) {                             // error if the sample is stored at 8 kHz (linear interpolation = same as during playback)
    float d = a->x[(i - 1) & 7] - 0.5f * (a->x[(i - 2) & 7] + v);
    a->e2 += (double)d * d;
  }
  if (i >= 4 && (i & 3) == 0) {                             // ... or at 4 kHz
    float x0 = a->x[(i - 4) & 7], dx = v - x0;
    for (int k = 1; k < 4; k++) { float d = a->x[(i - 4 + k) & 7] - (x0 + dx * k * 0.25f); a->e4 += (double)d * d; }
  }
  a->i = i + 1;
}
bool analyzeWav(File& f, WavInfo& wi) {
  AnaCtx a; memset(&a, 0, sizeof(a));
  if (!decodeWav(f, SAMPLE_SR, SAMPLE_MAX, emitAna, &a)) return false;
  if (a.lastActive < 16) { wi.n = 0; return true; }         // completely silent sample: nothing to load, but not an error
  uint32_t n = a.lastActive + 32;                           // small tail margin, the fade-out smooths it
  if (n > a.i) n = a.i;
  wi.n = n; wi.div = 1;
  if (n >= 256 && a.tot > 1e-6) {                           // only inspect samples that are long and loud enough
    double r2 = 2.0 * a.e2 / a.tot, r4 = 1.3333 * a.e4 / a.tot;
    if (r4 < DECIM_ERR) wi.div = 4;                         // sub/kick: 4 kHz, a quarter of the RAM
    else if (r2 < DECIM_ERR) wi.div = 2;                    // bass/tom: 8 kHz, half the RAM
  }
  return true;
}

// ---- loading: WAV -> 8-bit unsigned mono at tgtRate (SAMPLE_SR / div) ----
struct StoreCtx { uint8_t* m; uint32_t i; };
void emitStore(void* c, float v) {
  StoreCtx* s = (StoreCtx*)c;
  int q = (int)(v * 127.0f + 128.5f);
  s->m[s->i++] = (uint8_t)constrain(q, 0, 255);
}
bool loadWav(File& f, Sample& out, uint32_t maxS, uint32_t tgtRate) {
  if (maxS < 16) return false;
  uint8_t* m = (uint8_t*)(psramFound() ? ps_malloc(maxS) : malloc(maxS));
  if (!m) return false;
  StoreCtx sc = { m, 0 };
  uint32_t outI = decodeWav(f, tgtRate, maxS, emitStore, &sc);
  if (outI < 16) { free(m); return false; }
  uint32_t fade = (128UL * tgtRate) / SAMPLE_SR;            // ~8 ms fade-out to prevent clicks
  if (fade < 16) fade = 16;
  if (fade > outI) fade = outI;
  for (uint32_t i = 0; i < fade; i++) {
    uint32_t idx = outI - 1 - i;
    m[idx] = (uint8_t)(128 + ((int)m[idx] - 128) * (int)i / (int)fade);
  }
  out.d = m; out.n = outI; out.sr = (uint16_t)tgtRate;
  return true;
}

// ---------------- SD: FOLDERS AND KITS ----------------
int cmpName(const void* a, const void* b) { return strcasecmp((const char*)a, (const char*)b); }
bool isWav(const char* n) { size_t l = strlen(n); return l > 4 && !strcasecmp(n + l - 4, ".wav"); }
void joinPath(char* out, size_t sz, const char* base, const char* name) {
  if (!strcmp(base, "/")) snprintf(out, sz, "/%s", name); else snprintf(out, sz, "%s/%s", base, name);
}

// List of folders (dirs=true) or .wav files (dirs=false), sorted by name
int listDir(const char* path, char (*names)[32], int maxN, bool dirs) {
  File d = SD.open(path);
  if (!d || !d.isDirectory()) return 0;
  int n = 0;
  for (File e = d.openNextFile(); e && n < maxN; e = d.openNextFile()) {
    const char* nm = e.name();
    bool ok = nm[0] != '.' && nm[0] != '_' && strlen(nm) < 32;
    if (ok) {
      if (dirs) ok = e.isDirectory() && strcasecmp(nm, "songs") && strcasecmp(nm, "System Volume Information");
      else      ok = !e.isDirectory() && isWav(nm);
    }
    if (ok) strcpy(names[n++], nm);
    e.close();
  }
  d.close();
  qsort(names, n, 32, cmpName);
  return n;
}

int countWavs(const char* path) {
  File d = SD.open(path);
  if (!d || !d.isDirectory()) return 0;
  int n = 0;
  for (File e = d.openNextFile(); e; e = d.openNextFile()) {
    const char* nm = e.name();
    if (!e.isDirectory() && nm[0] != '.' && nm[0] != '_' && isWav(nm)) n++;
    e.close();
  }
  d.close();
  return n;
}

void saveCfg() {
  if (!sdOk) return;
  const char* tmp = "/pocketdaw.cfg.tmp";
  const char* bak = "/pocketdaw.cfg.bak";
  SD.remove(tmp);
  File f = SD.open(tmp, FILE_WRITE);
  if (!f) return;
  bool ok = f.write((uint8_t*)kitPath, sizeof(kitPath)) == sizeof(kitPath);
#if USE_BT
  if (ok) ok = f.write((uint8_t*)btName, sizeof(btName)) == sizeof(btName);
#endif
  f.flush(); f.close();
  size_t expected = sizeof(kitPath);
#if USE_BT
  expected += sizeof(btName);
#endif
  File check = SD.open(tmp);
  bool valid = ok && check && check.size() == expected;
  if (check) check.close();
  if (!valid) { SD.remove(tmp); return; }
  if (!SD.exists("/pocketdaw.cfg") && SD.exists(bak)) SD.rename(bak, "/pocketdaw.cfg");
  SD.remove(bak);
  bool hadOld = SD.exists("/pocketdaw.cfg");
  if (hadOld && !SD.rename("/pocketdaw.cfg", bak)) { SD.remove(tmp); return; }
  if (!SD.rename(tmp, "/pocketdaw.cfg")) {
    if (hadOld) SD.rename(bak, "/pocketdaw.cfg");
    SD.remove(tmp); return;
  }
  SD.remove(bak);
}

bool loadCfg() {
  if (!SD.exists("/pocketdaw.cfg") && SD.exists("/pocketdaw.cfg.bak")) SD.rename("/pocketdaw.cfg.bak", "/pocketdaw.cfg");
  File f = SD.open("/pocketdaw.cfg");
  if (!f) return false;
  if (f.size() < sizeof(kitPath)) { f.close(); return false; }   // the old cfg (2 kits) is larger -> read only the first (CH1)
  f.read((uint8_t*)kitPath, sizeof(kitPath));
#if USE_BT
  if (f.size() == sizeof(kitPath) + sizeof(btName)) { f.read((uint8_t*)btName, sizeof(btName)); btName[sizeof(btName) - 1] = 0; }
#endif
  f.close();
  kitPath[63] = 0;
  return true;
}

// Loads the first 8 .wav files from the folder (alphabetically) into lanes.
// v25: analysis of each WAV (end of sound without silence + whether it can be stored at 8/4 kHz without loss),
// and only the lanes that are in use are loaded into RAM (see bankSync).
bool loadKit(const char* path, const char* msg, bool save) {
  char pth[64]; strncpy(pth, path, 63); pth[63] = 0;
  static char fl[MAXE][32];
  int n = listDir(pth, fl, MAXE, false);
  if (n == 0) return false;
  if (n > 8) n = 8;
  playing = false;
  // Stop the audio task at a block boundary before freeing the sample memory.
  bool pausedHere = false;
  if (audioTaskStarted) {
    audioPauseReq = true;
    uint32_t pauseT = millis();
    while (!audioPaused && millis() - pauseT < 150) delay(1);
    if (!audioPaused) { audioPauseReq = false; return false; }
    pausedHere = true;
  }
  for (int l = 0; l < 8; l++) { vc[l].on = false; smp[l].n = 0; }
  for (int l = 0; l < 8; l++) if (smp[l].d) { free(smp[l].d); smp[l].d = nullptr; }
  for (int l = 0; l < 8; l++) smp[l].sr = SAMPLE_SR;
  if (pausedHere) audioPauseReq = false;
  splashBar(msg, kitBarLo);

  // v25: here each WAV is only ANALYZED (length + decimation) and the name is remembered;
  // the actual loading into RAM is done by bankSync(), and only for lanes that are in use.
  char full[96];
  bool skipped = false;
  for (int l = 0; l < 8; l++) { laneHas[l] = false; laneAlloc[l] = 0; laneInfo[l] = WavInfo(); laneFile[l][0] = 0; }
  for (int i = 0; i < n; i++) {
    joinPath(full, sizeof(full), pth, fl[i]);
    File f = SD.open(full);
    if (f) {
      WavInfo wi;
      if (analyzeWav(f, wi)) {
        uint32_t need = (wi.n + wi.div - 1) / wi.div;
        if (need >= 16) { laneInfo[i] = wi; laneHas[i] = true; strncpy(laneFile[i], fl[i], 31); laneFile[i][31] = 0; }
      } else skipped = true;                                // unsupported format
      f.close();
    } else skipped = true;
    splashBar(msg, kitPct(i + 1, n));
  }
  strncpy(kitPath, pth, 63); kitPath[63] = 0;
  kitSkipped = skipped;
  wavRamWarning = skipped;
  bankFailWant = -1;
  if (save) saveCfg();
  bankSync();                                               // load only the lanes the song actually uses
  splashBar(msg, kitBarHi);
  int cnt = 0; for (int l = 0; l < 8; l++) if (laneHas[l]) cnt++;
  return cnt > 0;
}

// ---------------- SD: SONGS ----------------
// /songs/s1.bin .. s16.bin : SM10 = SM9 + micro timing; writing via TMP/BAK with size verification
// SM7 format supports 128 steps; older SM5/SM6 songs are loaded as 64-step.
#define SONG_SLOTS 17                        // slots 1..16 + AUTO (autosave)
bool slotUsed[SONG_SLOTS];
void songBase(char* b, int s) { if (s < 16) snprintf(b, 24, "/songs/s%d", s + 1); else strcpy(b, "/songs/auto"); }
char songNames[16][9] = {{0}};
void loadSongNames() {
  if (!sdOk) return;
  File f = SD.open("/songs/names.bin");
  if (f) { if (f.size() == sizeof(songNames)) f.read((uint8_t*)songNames, sizeof(songNames)); f.close(); }
  for (int i=0;i<16;i++) songNames[i][8]=0;
}
void saveSongNames() {
  if (!sdOk) return;
  SD.mkdir("/songs"); SD.remove("/songs/names.tmp");
  File f=SD.open("/songs/names.tmp",FILE_WRITE);
  if (!f) return;
  bool ok=f.write((uint8_t*)songNames,sizeof(songNames))==sizeof(songNames);
  f.flush(); f.close();
  if (ok) { SD.remove("/songs/names.bin"); SD.rename("/songs/names.tmp","/songs/names.bin"); }
  else SD.remove("/songs/names.tmp");
}
void songPath(char* p, int s) { char b[24]; songBase(b, s); snprintf(p, 24, "%s.bin", b); }
void refreshSlots() {
  char p[24], bak[24], b[24];
  for (int i = 0; i < SONG_SLOTS; i++) {
    songBase(b, i);
    snprintf(p, sizeof(p), "%s.bin", b); snprintf(bak, sizeof(bak), "%s.bak", b);
    slotUsed[i] = sdOk && (SD.exists(p) || SD.exists(bak));
  }
}

// Autosave: digest (hash) of everything that goes into the song, compared with the last saved state.
uint32_t savedHash = 0;
uint32_t lastInputT = 0, autoLastT = 0, autoCheckT = 0, autoMsgUntil = 0;
uint32_t autoHash = 0;                       // state of the last AUTOSAVE (separate from manual saving)
#define AUTO_SLOT       16                    // slot 'A' = /songs/auto.bin
#define AUTO_IDLE_MS    3000UL               // saves only after 3 s without input
#define AUTO_MIN_GAP_MS 30000UL              // and at most every 30 s
uint32_t songHash() {
  uint32_t h = 2166136261u;
  auto feed = [&h](const uint8_t* d, size_t n) { for (size_t i = 0; i < n; i++) { h ^= d[i]; h *= 16777619u; } };
  for (int c = 0; c < 4; c++) {
    Chan& C = ch[c];
    feed((const uint8_t*)C.pat, sizeof(C.pat));
    feed(C.song, 128);
    feed(C.psteps, 4);
    uint8_t pr[12] = { C.vol, (uint8_t)C.pitch, C.lp, C.hp, C.crush, C.wave, C.atk, C.dec, C.sus, C.rel, swingCh[c], (uint8_t)C.mute };
    feed(pr, sizeof(pr));
  }
  feed((const uint8_t*)microTiming, sizeof(microTiming));
  feed((const uint8_t*)stepVel, sizeof(stepVel));
  feed(drumVol, sizeof(drumVol));
  uint16_t bp = bpm; feed((const uint8_t*)&bp, 2);
  uint8_t sw = swing; feed(&sw, 1);
  feed((const uint8_t*)kitPath, sizeof(kitPath));
  return h;
}
const size_t PR_SZ = 12;   // number of channel parameters in the file
// The old SM5/SM6 format had 64 steps; we keep supporting loading of old songs.
struct LegacyPattern64 {
  union {
    struct { uint8_t drum[64]; int8_t dpitch[64][8]; };
    struct { uint8_t note[64][NVMAX]; uint8_t len[64]; };
  };
};
const size_t SONG_BASE_SIZE = 3 + 2 + 1 + sizeof(kitPath);
const size_t LEGACY_SONG_SIZE = SONG_BASE_SIZE + 4 * (sizeof(LegacyPattern64) * 4 + 32 + PR_SZ);
const size_t LEGACY_SONG_SIZE_V6 = LEGACY_SONG_SIZE + 8;
const size_t SONG_SIZE_V7 = SONG_BASE_SIZE + 4 * (sizeof(Pattern) * 4 + 32 + PR_SZ) + 8;
const size_t SONG_SIZE_V9 = SONG_SIZE_V7 + 16 + 4 * (128 - 32);   // SM9: chain of 128 cells
const size_t SONG_SIZE_V10 = SONG_SIZE_V9 + 1 + sizeof(microTiming); // SM10: per-step micro timing
const size_t SONG_SIZE_V11 = SONG_SIZE_V10 + sizeof(stepVel) + 4;    // SM11: per-step velocity + per-channel swing
const size_t SONG_SIZE_V8 = SONG_SIZE_V7 + 16;   // + 4 channels x 4 patterns: number of steps

bool writeExact(File& f, const uint8_t* data, size_t n) {
  return f.write(data, n) == n;
}

bool saveSong(int s) {
  if (!sdOk || s < 0 || s >= SONG_SLOTS) return false;
  SD.mkdir("/songs");
  char b[24], p[24], tmp[24], bak[24]; songBase(b, s);
  snprintf(p, sizeof(p), "%s.bin", b);
  snprintf(tmp, sizeof(tmp), "%s.tmp", b);
  snprintf(bak, sizeof(bak), "%s.bak", b);
  if (!SD.exists(p) && SD.exists(bak)) SD.rename(bak, p);
  SD.remove(tmp);
  File f = SD.open(tmp, FILE_WRITE);
  if (!f) return false;
  bool ok = writeExact(f, (const uint8_t*)"SM11", 4);
  uint16_t bpmStored = bpm;
  ok = ok && writeExact(f, (uint8_t*)&bpmStored, 2);
  uint8_t sw = swing;
  ok = ok && writeExact(f, &sw, 1);
  ok = ok && writeExact(f, (uint8_t*)kitPath, sizeof(kitPath));
  for (int c = 0; c < 4 && ok; c++) {
    Chan& C = ch[c];
    ok = writeExact(f, (uint8_t*)C.pat, sizeof(C.pat));
    ok = ok && writeExact(f, C.song, 128);
    uint8_t pr[PR_SZ] = { C.psteps[0], C.vol, (uint8_t)C.pitch, C.lp, C.hp, C.crush, C.wave, C.atk, C.dec, C.sus, C.rel, (uint8_t)C.mute };
    ok = ok && writeExact(f, pr, PR_SZ);
  }
  ok = ok && writeExact(f, drumVol, sizeof(drumVol));
  for (int c = 0; c < 4 && ok; c++) ok = writeExact(f, ch[c].psteps, 4);
  ok = ok && writeExact(f, (const uint8_t*)microTiming, sizeof(microTiming));
  ok = ok && writeExact(f, (const uint8_t*)stepVel, sizeof(stepVel));
  uint8_t swc[4] = { swingCh[0], swingCh[1], swingCh[2], swingCh[3] };
  ok = ok && writeExact(f, swc, 4);
  f.flush();
  f.close();
  if (!ok) { SD.remove(tmp); return false; }

  // Check the actual size before replacing a valid song.
  File verify = SD.open(tmp);
  bool valid = verify && verify.size() == SONG_SIZE_V11;
  if (verify) verify.close();
  if (!valid) { SD.remove(tmp); return false; }

  SD.remove(bak);
  bool hadOld = SD.exists(p);
  if (hadOld && !SD.rename(p, bak)) { SD.remove(tmp); return false; }
  if (!SD.rename(tmp, p)) {
    if (hadOld) SD.rename(bak, p);
    SD.remove(tmp);
    return false;
  }
  SD.remove(bak);
  savedHash = songHash();                                   // state is saved -> autosave not needed
  refreshSlots();
  return true;
}

bool loadSong(int s) {
  if (!sdOk) return false;
  char b[24], p[24], bak[24]; songBase(b, s);
  snprintf(p, sizeof(p), "%s.bin", b);
  snprintf(bak, sizeof(bak), "%s.bak", b);
  if (!SD.exists(p) && SD.exists(bak)) SD.rename(bak, p); // recovery after an interruption during replacement
  File f = SD.open(p);
  if (!f) return false;
  uint8_t hd[4] = {0};
  if (f.read(hd, 3) != 3 || hd[0] != 'S' || hd[1] != 'M' || (hd[2] != '5' && hd[2] != '6' && hd[2] != '7' && hd[2] != '8' && hd[2] != '9' && hd[2] != '1')) { f.close(); return false; }
  bool v10 = hd[2] == '1';
  if (v10 && f.read(&hd[3], 1) != 1) { f.close(); return false; }
  if (v10 && hd[3] != '0' && hd[3] != '1') { f.close(); return false; }
  bool v11 = v10 && hd[3] == '1';
  // SM10 uses a four-digit magic; older versions stay three-digit.
  if (!v10) { f.seek(3); }
  bool isNew = v10 || hd[2] == '7' || hd[2] == '8' || hd[2] == '9';
  bool hasDrumVol = v10 || hd[2] == '6' || hd[2] == '7' || hd[2] == '8' || hd[2] == '9';
  size_t expectedSize = v11 ? SONG_SIZE_V11 : v10 ? SONG_SIZE_V10 : hd[2] == '9' ? SONG_SIZE_V9 : hd[2] == '8' ? SONG_SIZE_V8 : isNew ? SONG_SIZE_V7 : (hasDrumVol ? LEGACY_SONG_SIZE_V6 : LEGACY_SONG_SIZE);
  if (f.size() != expectedSize) { f.close(); return false; }
  if (!v10) f.seek(3);
  playing = false;
  uint16_t bpmLoaded; f.read((uint8_t*)&bpmLoaded, 2); bpm = constrain((int)bpmLoaded, 40, 240);
  uint8_t sw; f.read(&sw, 1); swing = constrain((int)sw, 50, 75);
  char kp[64]; f.read((uint8_t*)kp, sizeof(kp)); kp[63] = 0;
  uint8_t oldSong[4][32] = {{0}};                          // SM5-SM8: chain of 32 slots (converted to cells)
  for (int c = 0; c < 4; c++) {
    Chan& C = ch[c];
    if (isNew) {
      f.read((uint8_t*)C.pat, sizeof(C.pat));
    } else {
      // Old 64-step songs are copied into the first 64 places; the rest stays empty.
      memset(C.pat, 0, sizeof(C.pat));
      LegacyPattern64 oldPat[4];
      f.read((uint8_t*)oldPat, sizeof(oldPat));
      for (int pi = 0; pi < 4; pi++) {
        if (isDrum(c)) {
          memcpy(C.pat[pi].drum, oldPat[pi].drum, sizeof(oldPat[pi].drum));
          memcpy(C.pat[pi].dpitch, oldPat[pi].dpitch, sizeof(oldPat[pi].dpitch));
        } else {
          memcpy(C.pat[pi].note, oldPat[pi].note, sizeof(oldPat[pi].note));
          memcpy(C.pat[pi].len, oldPat[pi].len, sizeof(oldPat[pi].len));
        }
      }
    }
    for (int pi = 0; pi < 4; pi++) {                        // pattern sanitization
      Pattern& P = C.pat[pi];
      if (isDrum(c)) {
        for (int st = 0; st < 128; st++) for (int l = 0; l < 8; l++) P.dpitch[st][l] = constrain((int)P.dpitch[st][l], -12, 12);
      } else {
        for (int st = 0; st < 128; st++) {
          P.len[st] = constrain((int)P.len[st], 0, 16);
          for (int v = 0; v < NVMAX; v++) if (P.note[st][v] > 127) P.note[st][v] = 0;
        }
      }
    }
    if (hd[2] == '9' || v10) { f.read(C.song, 128); for (int i = 0; i < 128; i++) if (C.song[i] > 4) C.song[i] = 0; }
    else { f.read(oldSong[c], 32); for (int i = 0; i < 32; i++) if (oldSong[c][i] > 4) oldSong[c][i] = 0; }
    uint8_t pr[PR_SZ]; f.read(pr, PR_SZ);
    uint8_t s0 = validSteps(pr[0]) ? pr[0] : 16;            // old formats: a single step count for all patterns
    for (int pi = 0; pi < 4; pi++) C.psteps[pi] = s0;
    C.vol = pr[1]; C.pitch = constrain((int)(int8_t)pr[2], -12, 12); C.lp = pr[3]; C.hp = pr[4];
    C.crush = constrain((int)pr[5], 1, 8); C.wave = pr[6] & 3;
    C.atk = pr[7]; C.dec = pr[8]; C.sus = pr[9]; C.rel = pr[10]; C.mute = pr[11];
  }
  if (hasDrumVol) f.read(drumVol, sizeof(drumVol));
  else for (int i = 0; i < 8; i++) drumVol[i] = 255;
  if (hd[2] == '8' || hd[2] == '9' || v10) for (int c = 0; c < 4; c++) {
    uint8_t ps[4]; f.read(ps, 4);
    for (int pi = 0; pi < 4; pi++) if (validSteps(ps[pi])) ch[c].psteps[pi] = ps[pi];
  }
  for (int c = 0; c < 4; c++) {
    Chan& C = ch[c];
    if (hd[2] != '9' && !v10) {                               // old chain -> cells (same duration)
      memset(C.song, 0, sizeof(C.song));
      int cell = 0;
      for (int i = 0; i < 32; i++) {
        uint8_t pid = oldSong[c][i];
        if (pid && cell < 128) C.song[cell] = pid;
        cell += pid ? cellsOf(C.psteps[pid - 1]) : 1;
      }
    }
    normalizeChain(c);
  }
  if (v10) {
    if (f.read((uint8_t*)microTiming, sizeof(microTiming)) != sizeof(microTiming)) { f.close(); return false; }
    for (int c = 0; c < 4; c++) for (int pi = 0; pi < 4; pi++) for (int st = 0; st < 128; st++) microTiming[c][pi][st] = constrain((int)microTiming[c][pi][st], -16, 16);
  } else memset(microTiming, 0, sizeof(microTiming));
  if (v11) {
    if (f.read((uint8_t*)stepVel, sizeof(stepVel)) != sizeof(stepVel)) { f.close(); return false; }
    for (int c = 0; c < 4; c++) for (int pi = 0; pi < 4; pi++) for (int st = 0; st < 128; st++) stepVel[c][pi][st] = constrain((int)stepVel[c][pi][st], 1, 127);
    uint8_t swc[4]; if (f.read(swc, 4) != 4) { f.close(); return false; }
    for (int c = 0; c < 4; c++) swingCh[c] = constrain((int)swc[c], 50, 75);
  } else {                                                    // older songs: full velocity, same swing on all channels
    memset(stepVel, 127, sizeof(stepVel));
    for (int c = 0; c < 4; c++) swingCh[c] = swing;
  }
  f.close();
  for (int k = 0; k < 3; k++) for (int v = 0; v < NVMAX; v++) { ov[k][v].st = 0; ov[k][v].env = 0; ov[k][v].gate = 0; }
  resetPlayheads();
  savedHash = songHash();
  if (kp[0] && strcmp(kp, kitPath)) loadKit(kp, "loading kit...", true);   // load the kit saved with the song
  bankSync();                                                // v25: load/free samples according to the new song
  return true;
}

// Deleting a saved song (.bin + any leftover .bak/.tmp files). The "are you sure" prompt is in inSong().
bool deleteSong(int s) {
  if (!sdOk) return false;
  char b[24], p[24], bak[24], tmp[24]; songBase(b, s);
  snprintf(p, sizeof(p), "%s.bin", b);
  snprintf(bak, sizeof(bak), "%s.bak", b);
  snprintf(tmp, sizeof(tmp), "%s.tmp", b);
  SD.remove(p); SD.remove(bak); SD.remove(tmp);
  bool gone = !SD.exists(p) && !SD.exists(bak);
  refreshSlots();
  return gone;
}

// ---------------- EXPORT WAV ----------------
// Renders the song offline (faster than real time) to /exports/song_NNN.wav (22050 Hz, 16-bit, mono).
// Length = one pass of the longest (unmuted) chain + a tail for release. Shorter chains loop as they do during playback.
// Performance FX (beat repeat/delay/high-pass/low-pass) are NOT recorded - clean mix. Holding the CENTER button aborts the export.
#define WAV_TAIL_S 2                 // seconds of tail after the last step
#define WAV_MAX_S  600               // safety limit: 10 min
void wavHeader(uint8_t* h, uint32_t dataBytes) {
  auto w32 = [&](int o, uint32_t v) { h[o] = v; h[o + 1] = v >> 8; h[o + 2] = v >> 16; h[o + 3] = v >> 24; };
  auto w16 = [&](int o, uint16_t v) { h[o] = v; h[o + 1] = v >> 8; };
  memcpy(h, "RIFF", 4); w32(4, 36 + dataBytes); memcpy(h + 8, "WAVEfmt ", 8);
  w32(16, 16); w16(20, 1); w16(22, 1); w32(24, SR); w32(28, SR * 2); w16(32, 2); w16(34, 16);
  memcpy(h + 36, "data", 4); w32(40, dataBytes);
}
uint32_t chainSteps(int c) {                       // total steps of the chain up to the last pattern
  Chan& C = ch[c];
  int last = -1;
  for (int i = 0; i < 128; i++) if (C.song[i]) last = i;
  if (last < 0) return 0;
  uint32_t steps = 0; int i = 0;
  while (i <= last && i < 128) {
    uint8_t pid = C.song[i];
    steps += pid ? C.psteps[pid - 1] : 16;
    i += pid ? cellsOf(C.psteps[pid - 1]) : 1;
  }
  return steps;
}
// Returns: >0 = file number, -1 aborted, -2 nothing to export, -3 no SD, 0 write error, -4 audio busy
int exportWav() {
  if (!sdOk) return -3;
  uint32_t total = 0;
  for (int c = 0; c < 4; c++) if (!ch[c].mute) { uint32_t st = chainSteps(c); if (st > total) total = st; }
  if (!total) return -2;
  playing = false;
  bankSync();                                    // v25: all used samples must be in RAM before rendering
  bool pausedHere = false;
  if (audioTaskStarted) {
    audioPauseReq = true;
    uint32_t pt = millis();
    while (!audioPaused && millis() - pt < 300) delay(1);
    if (!audioPaused) { audioPauseReq = false; return -4; }
    pausedHere = true;
    i2s_zero_dma_buffer(I2S_NUM_0);                // silence on the output while rendering
  }
  SD.mkdir("/exports");
  char path[32]; int num = 0;
  for (int i = 1; i < 1000; i++) { snprintf(path, sizeof(path), "/exports/song_%03d.wav", i); if (!SD.exists(path)) { num = i; break; } }
  File f; if (num) f = SD.open(path, FILE_WRITE);
  if (!num || !f) { if (pausedHere) audioPauseReq = false; return 0; }
  uint8_t hdr[44]; wavHeader(hdr, 0);
  bool ok = f.write(hdr, 44) == 44, cancelled = false;

  // clean initial state
  for (int l = 0; l < 8; l++) vc[l].on = false;
  for (int k = 0; k < 3; k++) for (int v = 0; v < NVMAX; v++) { ov[k][v].st = 0; ov[k][v].env = 0; ov[k][v].gate = 0; ov[k][v].ph = 0; }
  for (int c = 0; c < 4; c++) { ch[c].lpS = 0; ch[c].hpS = 0; }
  resetPlayheads();

  float sps = SR * 60.0f / bpm / 4.0f; gsps = sps;
  uint64_t cap = (uint64_t)SR * WAV_MAX_S, tail = (uint64_t)SR * WAV_TAIL_S;
  uint64_t songLen = (uint64_t)(total * sps);
  if (songLen + tail > cap) songLen = cap - tail;
  uint64_t endAll = songLen + tail, clk = 0;
  uint64_t nextDue[4] = {0, 0, 0, 0};
  uint32_t stepClock[4] = {1, 1, 1, 1};
  int32_t lastMicro[4] = {0, 0, 0, 0};
  static int16_t ob[512];
  uint32_t written = 0, t0 = millis(), lastUi = 0;
  while (ok && clk < endAll) {
    prepBuf();
    int n = (endAll - clk < 512) ? (int)(endAll - clk) : 512;
    for (int i = 0; i < n; i++) {
      if (clk < songLen) {
        for (int c = 0; c < 4; c++) if (clk >= nextDue[c]) {
          triggerStep(c);
          int32_t nextMicro = stepMicroSamples(c, sps);
          float sw = swingCh[c] / 100.0f;
          float interval = (stepClock[c] & 1) ? 2.0f * sps * sw : 2.0f * sps * (1.0f - sw);
          int32_t delta = (int32_t)interval + nextMicro - lastMicro[c];
          if (delta < 1) delta = 1;
          nextDue[c] += (uint32_t)delta;
          lastMicro[c] = nextMicro;
          stepClock[c]++;
        }
      }
      float mix = 0;
      for (int c = 0; c < 4; c++) mix += renderCh(c);
      mix *= 0.5f;
      if (mix > 1) mix = 1; if (mix < -1) mix = -1;
      ob[i] = (int16_t)(mix * 32000.0f);
      clk++;
    }
    if (f.write((const uint8_t*)ob, n * 2) != (size_t)(n * 2)) { ok = false; break; }
    written += n * 2;
    uint32_t now = millis();
    if (now - lastUi > 300) {
      lastUi = now;
      int pct = (int)(clk * 100 / endAll);
      oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(WHITE);
      oled.setCursor(0, 4);  oled.print("EXPORT WAV");
      oled.setCursor(0, 16); oled.printf("song_%03d.wav  %d%%", num, pct);
      oled.drawRect(14, 30, 100, 10, WHITE);
      oled.fillRect(16, 32, (96 * pct) / 100, 6, WHITE);
      oled.setCursor(0, 54); oled.print("hold CENTER = cancel");
      oled.display();
      if (now - t0 > 600 && !digitalRead(PIN_B[B_CE])) { cancelled = true; break; }
    }
  }
  if (ok && !cancelled) {
    wavHeader(hdr, written);
    ok = f.seek(0) && f.write(hdr, 44) == 44;     // write the real size into the header
    f.flush();
  }
  f.close();
  if (!ok || cancelled) SD.remove(path);
  // return to the stopped state
  for (int l = 0; l < 8; l++) vc[l].on = false;
  for (int k = 0; k < 3; k++) for (int v = 0; v < NVMAX; v++) { ov[k][v].st = 0; ov[k][v].env = 0; ov[k][v].gate = 0; }
  for (int c = 0; c < 4; c++) { ch[c].lpS = 0; ch[c].hpS = 0; }
  resetPlayheads();
  if (cancelled) while (!digitalRead(PIN_B[B_CE])) delay(5);   // wait for release so it does not trigger a new action
  if (pausedHere) audioPauseReq = false;
  if (cancelled) return -1;
  return ok ? num : 0;
}

// Autosave tick: call from loop(). Saves to slot A only if the song has changed.
void autosaveTick() {
  uint32_t now = millis();
  if (now - autoCheckT < 500) return;
  autoCheckT = now;
  if (!sdOk) return;
  if (now - lastInputT < AUTO_IDLE_MS) return;                       // input still in progress
  if (autoLastT && now - autoLastT < AUTO_MIN_GAP_MS) return;        // too soon after the last autosave
  uint32_t h = songHash();
  if (h == autoHash) return;                                         // no changes
  uint32_t keepSaved = savedHash;                                    // autosave does not change the manual-save state
  bool ok = saveSong(AUTO_SLOT);
  savedHash = keepSaved;
  autoLastT = millis();
  if (ok) { autoHash = h; autoMsgUntil = millis() + 1200; }          // on error: try again after 30 s
}

// ---------------- UI STATE ----------------
enum Scr { S_MAIN, S_EDIT, S_FX, S_SONG, S_BROWSE, S_BT, S_SAMPLE };
Scr scr = S_MAIN;
long viewScroll = 0; uint32_t navT = 0;                               // main: bar offset (in steps) and time of the last cursor move
int mRow = 0, mCol = 0, mItem = 0;                                    // main (mRow = -1 -> transport bar)
bool velMode = false, toolMenu = false;                               // editor: velocity mode, tool menu (C4 held)
int toolSel = 0, randDens = 50, eucHits = 4, eucRot = 0;
int eCh = 0, ePat = 0, eStep = 0, eLane = 0, eNote = 48, eTop = 54;   // editor
int fxCh = 0, fxSel = 0;                                              // fx
int sSlot = 0, sAct = 0; const char* sMsg = "";
bool songNaming = false;
char songNameEdit[9] = {0};
int songNameLen = 0, keyX = 0, keyY = 0;
const char songKeyRows[4][11] = {"ABCDEFGHIJ","KLMNOPQRST","UVWXYZ0123","456789-_ ."}; uint32_t sMsgT = 0;   // song menu (sAct: 0 LOAD, 1 SAVE, 2 WAV, 3 DEL)
char sMsgBuf[16];
bool sConfirm = false, sConfYes = false;       // song delete confirmation (default NO)
char bPath[64] = "/"; static char bEnt[MAXE][32];                     // kit browser
int bCnt = 0, bWav = 0, bSel = 0, bTop = 0;
int sampleLane = 0, sampleItem = 0;
Scr sampleReturnScr = S_EDIT;
const int SAMPLE_ITEMS = 17;
const char* bMsg = ""; uint32_t bMsgT = 0;

// ---------------- v25: ON-DEMAND SAMPLE BANK ----------------
// A lane is "used" if it has at least one cell in any of the 4 CH1 patterns.
// Used lanes (+ the lane open in the sample editor) are in RAM, the others are freed.
uint8_t laneMask() { uint8_t m = 0; for (int l = 0; l < 8; l++) if (laneHas[l]) m |= (1 << l); return m; }
uint8_t usedLaneMask() {
  uint8_t m = 0;
  for (int p = 0; p < 4; p++) for (int s = 0; s < 128; s++) m |= ch[0].pat[p].drum[s];
  return m;
}
uint32_t laneNeed(int l) { return (laneInfo[l].n + laneInfo[l].div - 1) / laneInfo[l].div; }
uint32_t bankRoom() {
  uint32_t u = 0; for (int l = 0; l < 8; l++) u += laneAlloc[l];
  return bankBudget > u ? bankBudget - u : 0;
}
bool bankPause() {                                          // true = audio has stopped (or does not exist yet)
  if (!audioTaskStarted) return true;
  audioPauseReq = true;
  uint32_t t = millis();
  while (!audioPaused && millis() - t < 150) delay(1);
  if (!audioPaused) { audioPauseReq = false; return false; }
  return true;
}
void bankResume() { if (audioTaskStarted) audioPauseReq = false; }
void bankFree(uint8_t mask) {
  if (!mask) return;
  if (!bankPause()) return;                                 // we will try again on the next tick
  for (int l = 0; l < 8; l++) if (mask & (1 << l)) {
    vc[l].on = false;
    uint8_t* d = smp[l].d;
    smp[l].d = nullptr; smp[l].n = 0; laneAlloc[l] = 0;
    if (d) free(d);
  }
  bankResume();
}
bool bankLoadLane(int l, uint32_t room) {
  if (!laneHas[l] || smp[l].d) return false;
  uint32_t need = laneNeed(l);
  uint32_t cap = need < room ? need : room;
#if USE_BT
  if (btA2dpStarted) {                                      // v26: while BT is running, never lower the free heap below BT_LOAD_FLOOR
    int32_t fh = (int32_t)ESP.getFreeHeap() - BT_LOAD_FLOOR;
    if (fh < 16) return false;
    if ((uint32_t)fh < cap) cap = (uint32_t)fh;
  }
#endif
  if (cap < 16) return false;
  char full[96]; joinPath(full, sizeof(full), kitPath, laneFile[l]);
  File f = SD.open(full);
  if (!f) return false;
  uint32_t tgt = SAMPLE_SR / laneInfo[l].div;
  bool loaded = false;
  while (cap >= 16 && !loaded) {                            // if the heap is fragmented, shrink only this sample
    loaded = loadWav(f, smp[l], cap, tgt);
    if (!loaded) cap = cap * 3 / 4;
  }
  f.close();
  if (!loaded) return false;
  laneAlloc[l] = cap;
  if (sampleFx[l].normalize) calcNormalize(l);
  return true;
}
void bankSync() {
  if (!sdOk || !kitPath[0]) return;
  uint8_t want = usedLaneMask();
  if (scr == S_SAMPLE) want |= (1 << sampleLane);           // the sample editor needs the data immediately
  want &= laneMask();
  uint8_t have = 0; for (int l = 0; l < 8; l++) if (smp[l].d) have |= (1 << l);
  uint8_t add = want & ~have, drop = have & ~want;
  if (playing || scr == S_EDIT || scr == S_SAMPLE) {        // do not free while playing/editing, unless space is needed
    uint32_t needAdd = 0; for (int l = 0; l < 8; l++) if (add & (1 << l)) needAdd += laneNeed(l);
    if (!(add && needAdd > bankRoom())) drop = 0;
  }
  if (drop) { bankFree(drop); bankFailWant = -1; }
  if (add && bankFailWant != want) {
    bool failed = false;
    for (int l = 0; l < 8; l++) if (add & (1 << l)) if (!bankLoadLane(l, bankRoom())) failed = true;
    if (failed) bankFailWant = want;
  }
  bool shortRam = false;                                    // warning (!) only if a used lane is not fully in RAM
  for (int l = 0; l < 8; l++) if ((want & (1 << l)) && (!smp[l].d || laneAlloc[l] < laneNeed(l))) shortRam = true;
  bankShort = shortRam;
  wavRamWarning = kitSkipped || shortRam;
}
void bankTick() {
  static uint32_t t = 0;
  if (millis() - t < 250) return;
  t = millis();
  bankSync();
}
#if USE_BT
// v26: the BT stack allocates its RAM when connecting and does not change it afterwards. When BT is stably connected and some used
// sample is truncated or not loaded, free heap above BT_FREE_FLOOR is added to the bank budget.
// Truncated lanes are reloaded (only while not playing) to get the full sample.
void bankGrowTick() {
  static uint32_t upSince = 0, last = 0;
  if (psramFound() || !btConnected()) { upSince = 0; return; }
  uint32_t now = millis();
  if (!upSince) upSince = now;
  if (now - upSince < 5000 || now - last < 2000) return;
  last = now;
  if (!bankShort) return;
  int32_t extra = (int32_t)ESP.getFreeHeap() - BT_FREE_FLOOR;
  if (extra < 2048) return;
  uint8_t want = usedLaneMask();
  if (scr == S_SAMPLE) want |= (1 << sampleLane);
  want &= laneMask();
  uint32_t miss = 0; uint8_t trunc = 0;
  for (int l = 0; l < 8; l++) if (want & (1 << l)) {
    uint32_t nd = laneNeed(l), have = smp[l].d ? laneAlloc[l] : 0;
    if (have < nd) { miss += nd - have; if (smp[l].d) trunc |= (1 << l); }
  }
  uint32_t room = bankRoom();
  if (miss <= room) return;                                 // the budget is sufficient - the cause is something else (e.g. SD), not RAM
  uint32_t grow = miss - room;
  if (grow > (uint32_t)extra) grow = (uint32_t)extra;
  bankBudget += grow;
  bankFailWant = -1;
  Serial.printf("[MEM] BT spojen: banka +%u B -> %u B (free heap %u)\n", (unsigned)grow, (unsigned)bankBudget, (unsigned)ESP.getFreeHeap());
  if (trunc && !playing) bankFree(trunc);                   // bankSync() reloads them within <=250 ms, now in full
}
#endif

void previewNote(int c, int n, int v, float vel) {
  OscV& o = ov[c - 1][v];
  o.freq = noteFreq(n, ch[c].pitch);
  o.gate = (int32_t)(gsps * 2);
  o.vel = vel;
  o.st = 1;
}
void previewLane(int l, int pitch, float vel) {
  SampleFx& fx = sampleFx[l];
  vc[l].inc = (smp[l].sr / (float)SR) * (fx.speed / 100.0f) * powf(2.0f, (ch[0].pitch + pitch + fx.pitch + fx.fine/100.0f) / 12.0f);
  uint32_t st=(uint64_t)smp[l].n*fx.start/100, en=(uint64_t)smp[l].n*fx.end/100;
  if(en<=st+1){st=0;en=smp[l].n;}
  vc[l].pos = fx.reverse && en ? (float)(en-1) : (float)st;
  vc[l].env=0; vc[l].vel = vel; vc[l].on = smp[l].d && en>st;
}

// piano roll range per channel (MIDI note)
void rollRange(int c, int& lo, int& hi) {
  if (c == 1)      { lo = 24; hi = 59; }     // bass
  else if (c == 2) { lo = 48; hi = 95; }     // melody
  else             { lo = 36; hi = 83; }     // chords
}

void initEdit() {
  eStep = 0; eLane = 0; microMode = false; velMode = false; toolMenu = false;
  eNote = (eCh == 1) ? 36 : 60;
  eTop = eNote + 6;
}

// ---------------- SAMPLE EDITOR ----------------
// A separate file preserves compatibility of existing songs and pocketdaw.cfg.
void saveSampleFx() {
  if (!sdOk) return;
  const char* tmp = "/samplefx.tmp";
  SD.remove(tmp);
  File f = SD.open(tmp, FILE_WRITE);
  if (!f) return;
  uint8_t magic[4] = {'S','F','X','1'};
  bool ok = f.write(magic, 4) == 4 && f.write((uint8_t*)sampleFx, sizeof(sampleFx)) == sizeof(sampleFx)
            && f.write(drumVol, sizeof(drumVol)) == sizeof(drumVol);
  f.flush(); f.close();
  if (!ok) { SD.remove(tmp); return; }
  SD.remove("/samplefx.bin");
  SD.rename(tmp, "/samplefx.bin");
}
void loadSampleFx() {
  if (!sdOk || !SD.exists("/samplefx.bin")) return;
  File f = SD.open("/samplefx.bin");
  if (!f) return;
  uint8_t magic[4];
  if (f.read(magic, 4) == 4 && !memcmp(magic, "SFX1", 4) && f.size() == 4 + sizeof(sampleFx) + sizeof(drumVol)) {
    f.read((uint8_t*)sampleFx, sizeof(sampleFx));
    f.read(drumVol, sizeof(drumVol));
  }
  f.close();
  for (int i=0;i<8;i++) {
    sampleFx[i].start = constrain(sampleFx[i].start, 0, 99);
    sampleFx[i].end = constrain(sampleFx[i].end, 1, 100);
    if (sampleFx[i].end <= sampleFx[i].start) { sampleFx[i].start=0; sampleFx[i].end=100; }
    sampleFx[i].speed = constrain(sampleFx[i].speed, 25, 200);
    sampleFx[i].choke = constrain(sampleFx[i].choke, 0, 4);
  }
}
void calcNormalize(int l) {
  if (!smp[l].d || !smp[l].n) { sampleFx[l].normGain = 1.0f; return; }
  int peak = 0;
  for (uint32_t i=0;i<smp[l].n;i++) { int v = abs((int)smp[l].d[i]-128); if (v>peak) peak=v; }
  sampleFx[l].normGain = peak > 0 ? fminf(2.5f, 120.0f / peak) : 1.0f;
}
void detectTransient(int l) {
  if (!smp[l].d || smp[l].n < 32) return;
  int peak = 0; for (uint32_t i=0;i<smp[l].n;i++) peak=max(peak, abs((int)smp[l].d[i]-128));
  int threshold = max(4, peak / 5); uint32_t first=0;
  for (uint32_t i=0;i<smp[l].n;i++) if (abs((int)smp[l].d[i]-128) >= threshold) { first=i; break; }
  sampleFx[l].start = (uint8_t)constrain((int)(first * 100UL / smp[l].n), 0, 99);
  sampleFx[l].end = 100;
}
void openSampleEditor() { sampleLane = constrain(eLane,0,7); sampleItem=0; sampleReturnScr=scr; scr=S_SAMPLE; bankSync(); }
void adjustSampleFx(int d) {
  SampleFx& f = sampleFx[sampleLane];
  switch(sampleItem) {
    case 0: f.start = constrain((int)f.start + d, 0, (int)f.end-1); break;
    case 1: f.end = constrain((int)f.end + d, (int)f.start+1, 100); break;
    case 2: f.reverse = !f.reverse; break;
    case 3: f.normalize = !f.normalize; if(f.normalize) calcNormalize(sampleLane); break;
    case 4: f.pitch = constrain((int)f.pitch+d,-24,24); break;
    case 5: f.fine = constrain((int)f.fine+d*5,-100,100); break;
    case 6: f.speed = constrain((int)f.speed+d,25,200); break;
    case 7: f.attack = constrain((int)f.attack+d*2,0,250); break;
    case 8: f.decay = constrain((int)f.decay+d*2,0,250); break;
    case 9: f.sustain = constrain((int)f.sustain+d,0,100); break;
    case 10: f.release = constrain((int)f.release+d*2,0,250); break;
    case 11: f.fadeIn = constrain((int)f.fadeIn+d,0,50); break;
    case 12: f.fadeOut = constrain((int)f.fadeOut+d,0,50); break;
    case 13: f.loop = !f.loop; break;
    case 14: f.choke = (f.choke + (d>0?1:4)) % 5; break;
    case 15: detectTransient(sampleLane); break;
    case 16: drumVol[sampleLane] = constrain((int)drumVol[sampleLane]+d*8,0,255); break;
  }
}
void inSample() {
  if (btn[B_CE].lng) { saveSampleFx(); scr=sampleReturnScr; return; }
  if (!(btn[B_UP].down && btn[B_DN].down)) {
    if (btn[B_UP].press && sampleItem>0) sampleItem--;
    if (btn[B_DN].press && sampleItem<SAMPLE_ITEMS-1) sampleItem++;
  }
  if (btn[B_LF].press && sampleLane>0) { sampleLane--; bankSync(); }
  if (btn[B_RT].press && sampleLane<7) { sampleLane++; bankSync(); }
  if (btn[B_PL].press) adjustSampleFx(1);
  if (btn[B_MI].press) adjustSampleFx(-1);
  if (btn[B_CE].rel) previewLane(sampleLane, sampleFx[sampleLane].pitch, 1.0f);
  if (btn[B_C1].lng) { saveSampleFx(); editMsg="SAVED"; editMsgUntil=millis()+800; }
}

// ---------------- BROWSER ----------------
void browseRefresh() { bCnt = listDir(bPath, bEnt, MAXE, true); bWav = countWavs(bPath); bSel = 0; bTop = 0; }
void browseUp() {
  if (!strcmp(bPath, "/")) return;
  char* s = strrchr(bPath, '/');
  if (s == bPath) strcpy(bPath, "/"); else *s = 0;
  browseRefresh();
}
void openBrowser() {
  bMsg = "";
  strcpy(bPath, "/");
  if (kitPath[0] && SD.exists(kitPath)) { strncpy(bPath, kitPath, 63); bPath[63] = 0; }
  browseRefresh();
  scr = S_BROWSE;
}
void browseEnter() {
  bool root = !strcmp(bPath, "/");
  if (!root && bSel == 0) { browseUp(); return; }
  int idx = bSel - (root ? 0 : 1);
  if (idx < 0 || idx >= bCnt) return;
  char np[96]; joinPath(np, sizeof(np), bPath, bEnt[idx]);
  if (strlen(np) > 62) return;
  strcpy(bPath, np);
  browseRefresh();
}

#if USE_BT
void openBt() {
  bool wasStarted = btA2dpStarted;
  btBegin();
  if (wasStarted && !btConnected() && millis() - btScanT > BT_SCAN_MS) {   // fresh list + new scan
    btDevN = 0;
    btRestartReq = true;
  }
  btSel = 0; btTop = 0;
  scr = S_BT;
}
void btConnectTo(const char* nm) {
  if (btConnected() && !strncmp(nm, btTarget, sizeof(btTarget) - 1)) return;   // already connected
  strncpy(btTarget, nm, sizeof(btTarget) - 1); btTarget[sizeof(btTarget) - 1] = 0;
  strcpy(btName, btTarget);
  saveCfg();
  btRestartReq = true;                            // end()+start(): disconnect the old one and scan again -> connect to btTarget
  btScanT = 0;                                    // no waiting between restarts
}
void btForget() {
  btTarget[0] = 0; btName[0] = 0;
  saveCfg();
  if (btConnected()) a2dp.set_connected(false);
  btRestartReq = false;
}
void inBt(bool mi) {
  int n = btDevN;
  if (btn[B_UP].press && btSel > 0) btSel--;
  if (btn[B_DN].press && btSel < n - 1) btSel++;
  if (btSel < btTop) btTop = btSel;
  if (btSel > btTop + 4) btTop = btSel - 4;
  if (btn[B_CE].lng) btForget();                                  // hold center = forget / disconnect
  else if (btn[B_CE].rel && n > 0) btConnectTo(btDev[btSel]);     // center = connect
  if (btn[B_PL].lng && btTarget[0] && !btConnected()) {           // hold + = save and restart (clean connect by name at power-on)
    if (sdOk) saveSong(AUTO_SLOT);
    delay(200);
    ESP.restart();
  }
  if (mi) scr = S_FX;                                             // - = back
}
#endif

// ---------------- INPUT ----------------
void inMain(bool pl, bool mi) {
  // Long press + = PLAY, long press - = STOP.
  if (btn[B_PL].lng && !playing) { togglePlay(); return; }
  if (btn[B_MI].lng && playing) { togglePlay(); return; }
  if (btn[B_UP].press && mRow > -1) mRow--;
  if (btn[B_DN].press && mRow < 3) mRow++;
  if (mRow < 0) {                                           // transport: [PLAY/STOP] [BPM] [SWING]
    if (btn[B_LF].press && mItem > 0) mItem--;
    if (btn[B_RT].press && mItem < 2) mItem++;
    if (mItem == 0 && btn[B_CE].rel) togglePlay();
    if (mItem == 1) { if (pl) bpm = constrain((int)bpm + 1, 40, 240); if (mi) bpm = constrain((int)bpm - 1, 40, 240); }
    if (mItem == 2 && (pl || mi)) {                          // shared swing: sets all channels (each can be changed later in the FX menu)
      swing = constrain((int)swing + (pl ? 1 : -1), 50, 75);
      for (int c = 0; c < 4; c++) swingCh[c] = swing;
    }
  } else {
    mCol = blockStart(mRow, mCol);                           // cursor always at the start of a pattern
    if (btn[B_LF].press) mCol = prevBlock(mRow, mCol);
    if (btn[B_RT].press) { int nx = mCol + spanOf(mRow, mCol); if (nx < 128) mCol = nx; }
    uint8_t& v = ch[mRow].song[mCol];
    if (pl && v < 4) v++;
    if (mi && v > 0) v--;
    if (btn[B_CE].rel && !v) v = 1;
    if (pl || mi || btn[B_CE].rel) normalizeChain(mRow);     // a new pattern replaces the cells it occupies
    if (btn[B_CE].rel) { eCh = mRow; ePat = v - 1; initEdit(); scr = S_EDIT; }
  }
  if (mRow >= 0 && (btn[B_UP].press || btn[B_DN].press || btn[B_LF].press || btn[B_RT].press || pl || mi)) navT = millis();
  if (btn[B_CE].lng) { refreshSlots(); sMsg = ""; scr = S_SONG; }            // hold center = song menu
  for (int i = 0; i < 4; i++) {
    if (btn[B_C1 + i].rel && (millis() - btn[B_C1 + i].t) < 250) ch[i].mute = !ch[i].mute; // short tap = mute; longer hold = FX
    if (btn[B_C1 + i].lng && !playing) {
      fxCh = i; fxSel = 0; scr = S_FX; // while held = FX channel
    }
  }
}

void copyPattern(int c, int ptn) {
  memcpy(&patternClipboard, &ch[c].pat[ptn], sizeof(Pattern));
  memcpy(clipboardMicro, microTiming[c][ptn], sizeof(clipboardMicro));
  memcpy(clipboardVel, stepVel[c][ptn], sizeof(clipboardVel));
  clipboardSteps = ch[c].psteps[ptn]; clipboardValid = true;
  editMsg = "COPIED"; editMsgUntil = millis() + 900;
}
void pastePattern(int c, int ptn) {
  if (!clipboardValid) { editMsg = "EMPTY CLIP"; editMsgUntil = millis() + 900; return; }
  memcpy(&ch[c].pat[ptn], &patternClipboard, sizeof(Pattern));
  memcpy(microTiming[c][ptn], clipboardMicro, sizeof(clipboardMicro));
  memcpy(stepVel[c][ptn], clipboardVel, sizeof(clipboardVel));
  ch[c].psteps[ptn] = clipboardSteps;
  normalizeChain(c);
  editMsg = "PASTED"; editMsgUntil = millis() + 900;
}

void inEdit(bool pl, bool mi) {
  Chan& C = ch[eCh];
  if (btn[B_CE].lng) { scr = S_MAIN; return; }
  // Auto-repeat of the left/right buttons also works after a long press.
  if (btn[B_LF].press && eStep > 0) eStep--;
  if (btn[B_RT].press && eStep < C.psteps[ePat] - 1) eStep++;
  if (btn[B_C1].lng) { copyPattern(eCh, ePat); return; }
  if (btn[B_C2].lng) { pastePattern(eCh, ePat); return; }
  if (btn[B_C3].lng) { microMode = !microMode; editMsg = microMode ? "MICRO ON" : "MICRO OFF"; editMsgUntil = millis() + 900; return; }
  // Hold CH4 in the editor for the velocity of the selected cell/step.
  // While the button is held, +/- changes velocity; releasing returns to the normal editor.
  velMode = btn[B_C4].down;
  for (int i = 0; i < 4; i++) if (btn[B_C1 + i].rel && !btn[B_C1 + i].lngDone) ePat = i; // short C1-C4 = pattern 1-4
  if (eStep >= C.psteps[ePat]) eStep = C.psteps[ePat] - 1;          // another pattern may have fewer steps
  Pattern& P = C.pat[ePat];
  if (velMode) {
    if (pl) stepVel[eCh][ePat][eStep] = (uint8_t)constrain((int)stepVel[eCh][ePat][eStep] + 4, 1, 127);
    if (mi) stepVel[eCh][ePat][eStep] = (uint8_t)constrain((int)stepVel[eCh][ePat][eStep] - 4, 1, 127);
    return;
  }
  if (microMode) {
    bool plusTap = btn[B_PL].press;
    bool minusTap = btn[B_MI].press;
    if (plusTap) microTiming[eCh][ePat][eStep] = constrain((int)microTiming[eCh][ePat][eStep] + 1, -16, 16);
    if (minusTap) microTiming[eCh][ePat][eStep] = constrain((int)microTiming[eCh][ePat][eStep] - 1, -16, 16);
    return;
  }
  if (isDrum(eCh)) {
    if (btn[B_UP].press && eLane > 0) eLane--;
    if (btn[B_DN].press && eLane < 7) eLane++;
    if (btn[B_CE].rel) {
      P.drum[eStep] ^= (1 << eLane);
      P.dpitch[eStep][eLane] = 0;                                  // new/cleared step -> pitch 0
      if (P.drum[eStep] & (1 << eLane)) { bankSync(); previewLane(eLane, 0, 1.0f); }   // v25: the lane is loaded as soon as it gets a step
    }
    if (drumVolMode) {
      if (pl) drumVol[eLane] = (uint8_t)constrain((int)drumVol[eLane] + 8, 0, 255);
      if (mi) drumVol[eLane] = (uint8_t)constrain((int)drumVol[eLane] - 8, 0, 255);
      if (btn[B_CE].rel && (P.drum[eStep] & (1 << eLane))) previewLane(eLane, P.dpitch[eStep][eLane], 1.0f);
    } else if (P.drum[eStep] & (1 << eLane)) {                    // +/- = pitch of THIS step only
      int8_t& tp = P.dpitch[eStep][eLane];
      if (pl && tp < 12)  { tp++; previewLane(eLane, tp, 1.0f); }
      if (mi && tp > -12) { tp--; previewLane(eLane, tp, 1.0f); }
    }
  } else {
    int lo, hi, nv = nvOf(eCh); rollRange(eCh, lo, hi);
    if (btn[B_UP].press && eNote < hi) eNote++;
    if (btn[B_DN].press && eNote > lo) eNote--;
    if (eNote > eTop) eTop = eNote;
    if (eNote < eTop - 13) eTop = eNote + 13;
    if (btn[B_CE].rel) {
      int idx = -1;
      for (int v = 0; v < nv; v++) if (P.note[eStep][v] == eNote) idx = v;
      if (idx >= 0) P.note[eStep][idx] = 0;                          // remove note
      else {
        int fr = -1;
        for (int v = 0; v < nv; v++) if (!P.note[eStep][v]) { fr = v; break; }
        if (fr >= 0) {                                               // add note (max nv per step)
          P.note[eStep][fr] = eNote;
          if (!P.len[eStep]) P.len[eStep] = 1;
          previewNote(eCh, eNote, fr, 1.0f);
        }
      }
    }
    bool has = false;
    for (int v = 0; v < nv; v++) if (P.note[eStep][v]) has = true;
    if (has) {
      if (pl && P.len[eStep] < 16) P.len[eStep]++;
      if (mi && P.len[eStep] > 1) P.len[eStep]--;
    }
  }
}

inline int fxCount(int c) { return (isDrum(c) ? 13 : 16) + (USE_BT ? 1 : 0); }   // CH1 gets SAMPLE EDITOR before BLUETOOTH

void inFx(bool pl, bool mi) {
  if (btn[B_CE].lng || btn[B_CE].rel) { scr = S_MAIN; return; }
  if (btn[B_LF].press && fxCh > 0) fxCh--;
  if (btn[B_RT].press && fxCh < 3) fxCh++;
  int maxSel = fxCount(fxCh) - 1;
  if (btn[B_UP].press && fxSel > 0) fxSel--;
  if (btn[B_DN].press && fxSel < maxSel) fxSel++;
  if (fxSel > maxSel) fxSel = maxSel;
  Chan& C = ch[fxCh];
  int d = pl ? 1 : (mi ? -1 : 0);
  if (!d) return;
#if USE_BT
  if (isDrum(fxCh) && fxSel == 12) { if (d > 0) openSampleEditor(); return; } // CH1: SAMPLE EDITOR
  if (fxSel == maxSel) { if (d > 0) openBt(); return; }   // last row = BLUETOOTH: + opens the list
#endif
  switch (fxSel) {
    case 0:  bpm = constrain((int)bpm + d, 40, 240); break;
    case 1:  swing = constrain((int)swing + d, 50, 75); break;
    case 2:  C.vol = constrain((int)C.vol + d * 8, 0, 255); break;
    case 3:  C.pitch = constrain((int)C.pitch + d, -12, 12); break;
    case 4:  C.lp = constrain((int)C.lp + d * 8, 8, 255); break;
    case 5:  C.hp = constrain((int)C.hp + d * 8, 0, 255); break;
    case 6:  C.crush = constrain((int)C.crush + d, 1, 8); break;
    case 7: case 8: case 9: case 10: {       // P1..P4 STEPS
      int p = fxSel - 7, idx = 0;
      for (int i = 0; i < 5; i++) if (STEPS_OPT[i] == C.psteps[p]) idx = i;
      idx = constrain(idx + d, 0, 4);
      C.psteps[p] = STEPS_OPT[idx];
      normalizeChain(fxCh);
      break;
    }
    case 11:
      if (isDrum(fxCh)) { if (d > 0) openBrowser(); }   // KIT: + opens the browser
      else C.wave = constrain((int)C.wave + d, 0, 3);
      break;
    case 12: C.atk = constrain((int)C.atk + d * 4, 0, 255); break;
    case 13: C.dec = constrain((int)C.dec + d * 4, 0, 255); break;
    case 14: C.sus = constrain((int)C.sus + d * 4, 0, 255); break;
    case 15: C.rel = constrain((int)C.rel + d * 4, 0, 255); break;
  }
}

void inSong() {
  if (songNaming) {
    if (btn[B_UP].press) keyY=(keyY+3)%4;
    if (btn[B_DN].press) keyY=(keyY+1)%4;
    if (btn[B_LF].press) keyX=(keyX+9)%10;
    if (btn[B_RT].press) keyX=(keyX+1)%10;
    if (btn[B_CE].rel && songNameLen<8) { songNameEdit[songNameLen++]=songKeyRows[keyY][keyX]; songNameEdit[songNameLen]=0; }
    if (btn[B_PL].press && songNameLen>0) songNameEdit[--songNameLen]=0;
    if (btn[B_MI].press || btn[B_CE].lng) {
      songNameEdit[songNameLen]=0;
      if (songNameLen>0) { strncpy(songNames[sSlot],songNameEdit,8); songNames[sSlot][8]=0; saveSongNames(); }
      songNaming=false;
      sMsg = saveSong(sSlot) ? "SAVED" : "SD ERROR"; sMsgT=millis();
    }
    return;
  }
  if (sConfirm) {                                   // "Are you sure?" - default is NO
    if (btn[B_LF].press) sConfYes = false;
    if (btn[B_RT].press) sConfYes = true;
    if (btn[B_UP].press || btn[B_DN].press || btn[B_CE].lng) { sConfirm = false; sMsg = "CANCELLED"; sMsgT = millis(); }
    else if (btn[B_CE].rel) {
      sConfirm = false;
      sMsg = sConfYes ? (deleteSong(sSlot) ? "DELETED" : "SD ERROR") : "CANCELLED";
      sMsgT = millis();
    }
    return;
  }
  if (btn[B_CE].lng) { scr = S_MAIN; return; }
  if (btn[B_UP].press && sSlot > 0) sSlot--;
  if (btn[B_DN].press && sSlot < SONG_SLOTS - 1) sSlot++;
  if (btn[B_LF].press && sAct > 0) sAct--;      // LOAD <- SAVE <- WAV
  if (btn[B_RT].press && sAct < 3) sAct++;
  if (btn[B_CE].rel) {
    if (sAct == 0)      sMsg = loadSong(sSlot) ? "LOADED" : "EMPTY / ERROR";
    else if (sAct == 1) {
      if (sSlot == AUTO_SLOT) sMsg = "AUTO SLOT";
      else { memset(songNameEdit,0,sizeof(songNameEdit)); strncpy(songNameEdit,songNames[sSlot],8); songNameLen=strlen(songNameEdit); keyX=keyY=0; songNaming=true; return; }
    }
    else if (sAct == 3) {                           // DEL: ask first
      if (!slotUsed[sSlot]) sMsg = "EMPTY";
      else { sConfirm = true; sConfYes = false; sMsg = ""; }
    }
    else {                                          // WAV: exports the song currently in memory
      int r = exportWav();
      if (r > 0) { snprintf(sMsgBuf, sizeof(sMsgBuf), "WAV %03d", r); sMsg = sMsgBuf; }
      else sMsg = r == -1 ? "CANCELLED" : r == -2 ? "NOTHING" : r == -3 ? "NO SD" : r == -4 ? "AUDIO BUSY" : "WAV ERROR";
    }
    sMsgT = millis();
  }
}

void inBrowse(bool mi) {
  bool root = !strcmp(bPath, "/");
  int total = bCnt + (root ? 0 : 1);
  if (btn[B_UP].press && bSel > 0) bSel--;
  if (btn[B_DN].press && bSel < total - 1) bSel++;
  if (bSel < bTop) bTop = bSel;
  if (bSel > bTop + 4) bTop = bSel - 4;
  if (btn[B_LF].press) browseUp();
  if (btn[B_RT].press || btn[B_CE].rel) browseEnter();
  if (btn[B_CE].lng && bWav > 0) {                          // hold center = load this folder as a kit
    if (loadKit(bPath, "loading kit...", true)) scr = S_FX;
    else { bMsg = "LOAD ERROR"; bMsgT = millis(); }
  }
  if (mi) scr = S_FX;                                       // - = cancel
}

void handleInput() {
  // RAM monitor: hold CH1 + CH2 + CH3 + CH4 together for 1 second;
  // repeat the combination to close.
  static uint32_t channelComboT = 0;
  static bool channelComboActive = false;
  bool channelsDown = btn[B_C1].down && btn[B_C2].down &&
                      btn[B_C3].down && btn[B_C4].down;

  if (channelsDown) {
    if (!channelComboActive) {
      channelComboActive = true;
      channelComboT = millis();
    }
    return;  // block normal CH commands while the combination is held
  }

  if (channelComboActive) {
    uint32_t heldMs = millis() - channelComboT;
    channelComboActive = false;
    channelComboT = 0;
    if (heldMs >= 1000) {
      ramOverlay = !ramOverlay;
      return;
    }
  }

  // While the RAM monitor is open, buttons do not change the project or playback.
  if (ramOverlay) return;

  // PLUS/MINUS events for the existing menu and editor controls.
  bool pl = btn[B_PL].press;
  bool mi = btn[B_MI].press;

  // Performance FX are momentary: active only while the corresponding button is held
  // on the main screen during playback; releasing turns them off immediately.
  if (scr == S_MAIN && playing) {
    if      (btn[B_C1].down) performanceFx = 1; // Beat Repeat
    else if (btn[B_C2].down) performanceFx = 2; // Delay
    else if (btn[B_C3].down) performanceFx = 3; // High-pass sweep
    else if (btn[B_C4].down) performanceFx = 4; // Low-pass sweep
    else performanceFx = 0;
  } else {
    performanceFx = 0;
  }

// The Sample Editor opens from the CH1 FX settings, right before BLUETOOTH.
  if (scr == S_MAIN) inMain(pl, mi);
  else if (scr == S_EDIT) inEdit(pl, mi);
  else if (scr == S_SAMPLE) inSample();
  else if (scr == S_FX) inFx(pl, mi);
  else if (scr == S_SONG) inSong();
#if USE_BT
  else if (scr == S_BT) inBt(mi);
#endif
  else inBrowse(mi);
}

// ---------------- DRAWING ----------------
uint16_t drawBox(int x, int w, bool sel) {
  if (sel) oled.fillRect(x, 0, w, 10, WHITE); else oled.drawRect(x, 0, w, 10, WHITE);
  return sel ? BLACK : WHITE;
}

// x position on the bar: t = steps from the left edge (16 steps = 12 px = 1 cell)
inline int tlX(long t) { return 28 + (int)((t * 3) >> 2); }

// channel playhead position in steps (cell * 16)
long playPos(int r) {
  Chan& C = ch[r];
  int sp = C.songPos; if (sp > 127) sp = 127;
  uint8_t pid = C.song[sp];
  int len = pid ? C.psteps[pid - 1] : 16;
  int st = C.step; if (st > len) st = len;
  int dl = cellsOf(len) * 16;                                   // pattern width in display steps (cells * 16)
  return (long)sp * 16 + (long)st * dl / len;
}

void drawMain() {
  bool t = (mRow < 0);
  uint16_t col = drawBox(0, 44, t && mItem == 0);                        // PLAY/STOP button
  if (playing) oled.fillRect(4, 2, 6, 6, col); else oled.fillTriangle(4, 1, 4, 8, 9, 4, col);
  oled.setTextColor(col); oled.setCursor(14, 1); oled.print(playing ? "STOP" : "PLAY");
  col = drawBox(47, 47, t && mItem == 1);
  oled.setTextColor(col); oled.setCursor(50, 1); oled.printf("BPM %d", (int)bpm);
  col = drawBox(97, 25, t && mItem == 2);
  oled.setTextColor(col); oled.setCursor(98, 1);
  if (performanceFx) oled.printf("FX%d", (int)performanceFx); else oled.printf("SW%d", (int)swing);

  // One shared offset for all channels (the grid is fixed, channels are aligned)
  if (mRow >= 0) {
    mCol = blockStart(mRow, mCol);
    long a = (long)mCol * 16, b = a + (long)spanOf(mRow, mCol) * 16;
    if (b > viewScroll + 128) viewScroll = b - 128;
    if (a < viewScroll) viewScroll = a;
    if (viewScroll < 0) viewScroll = 0;
  }
  long sc = viewScroll;
  if (playing && (millis() - navT > 2500)) {                    // during playback the bar follows the playhead (except while you move the cursor)
    sc = playPos(mRow >= 0 ? mRow : 0) - 32;
    if (sc < 0) sc = 0;
  }

  for (int r = 0; r < 4; r++) {
    int y = 12 + r * 13;
    int i = 0;
    while (i < 128) {
      int n = spanOf(r, i);
      long a = (long)i * 16 - sc;
      if (a >= 134) break;
      if (a + n * 16 > 0) {
        uint8_t v = ch[r].song[i];
        bool cur = (r == mRow && i == mCol);
        int x = tlX(a), w = n * 12 - 1;                          // n cells = one wide block, no dividers
        if (cur) oled.fillRect(x, y, w, 11, WHITE); else oled.drawRect(x, y, w, 11, WHITE);
        int vl = x < 28 ? 28 : x, vr = (x + w) > 128 ? 128 : (x + w);
        if (vr - vl >= 7) {                                      // number in the middle (of the visible part)
          oled.setTextColor(cur ? BLACK : WHITE);
          oled.setCursor((vl + vr) / 2 - 2, y + 2);
          if (v) oled.print(v); else oled.print('-');
        }
      }
      i += n;
    }
    oled.fillRect(0, y - 1, 28, 13, BLACK);                      // cover blocks that slid out to the left
    oled.setTextColor(WHITE);
    oled.setCursor(0, y + 2);
    oled.print(ch[r].mute ? 'x' : ' '); oled.print(CN[r]);
  }
  if (playing) {                                                 // one shared playhead across all 4 channels
    int xp = tlX(playPos(mRow >= 0 ? mRow : 0) - sc);            // follows the same channel the bar scrolls by
    if (xp >= 28 && xp <= 127) oled.drawFastVLine(xp, 11, 53, SSD1306_INVERSE);
  }
  oled.setTextColor(WHITE);
}

bool playheadInPat(int& ph) {
  Chan& C = ch[eCh];
  if (!playing || C.song[C.songPos] - 1 != ePat) return false;
  int L = C.psteps[ePat]; ph = (C.step + L - 1) % L;
  return true;
}

void drawDrum() {
  Chan& C = ch[eCh]; Pattern& P = C.pat[ePat];
  bool curOn = P.drum[eStep] & (1 << eLane);
  oled.setCursor(0, 0);
  if (millis() < editMsgUntil) oled.print(editMsg);
  else if (velMode) oled.printf("VEL %d/127  STEP %d", stepVel[eCh][ePat][eStep], eStep + 1);
  else if (microMode) oled.printf("MICRO %+d", (int)microTiming[eCh][ePat][eStep]);
  else oled.printf("%s P%d L%d S%d ", CN[eCh], ePat + 1, C.psteps[ePat], eStep + 1);
  if (!microMode) {
    if (drumVolMode) oled.printf("V%u", (unsigned)drumVol[eLane]);
    else if (curOn) oled.printf("T%+d", (int)P.dpitch[eStep][eLane]); else oled.print("T-");
  }
  int v0 = (eStep / 16) * 16;
  for (int l = 0; l < 8; l++) {
    int y = 9 + l * 7;
    oled.setCursor(0, y); oled.print(l + 1);
    for (int k = 0; k < 16; k++) {
      int s = v0 + k; if (s >= C.psteps[ePat]) break;
      int x = 14 + k * 7;
      if (P.drum[s] & (1 << l)) {
        oled.fillRect(x, y, 6, 6, WHITE);
        int tp = P.dpitch[s][l];                                   // pitch label inside the cell
        if (tp > 0)      { oled.drawFastHLine(x + 1, y + 1, 4, BLACK); if (tp > 6)  oled.drawFastHLine(x + 1, y + 2, 4, BLACK); }
        else if (tp < 0) { oled.drawFastHLine(x + 1, y + 4, 4, BLACK); if (tp < -6) oled.drawFastHLine(x + 1, y + 3, 4, BLACK); }
      }
      else if (s % 4 == 0) oled.drawRect(x, y, 6, 6, WHITE);
      else oled.drawPixel(x + 2, y + 2, WHITE);
    }
  }
  oled.drawRect(14 + (eStep - v0) * 7 - 1, 9 + eLane * 7 - 1, 8, 8, SSD1306_INVERSE);
  int ph; if (playheadInPat(ph) && ph >= v0 && ph < v0 + 16) oled.drawFastHLine(14 + (ph - v0) * 7, 8, 6, WHITE);
  // Hold C4 to enter the Sample Editor.
}

void drawRoll() {
  Chan& C = ch[eCh]; Pattern& P = C.pat[ePat];
  int nv = nvOf(eCh);
  int cnt = 0; for (int v = 0; v < nv; v++) if (P.note[eStep][v]) cnt++;
  oled.setCursor(0, 0);
  if (millis() < editMsgUntil) oled.print(editMsg);
  else if (velMode) oled.printf("VEL %d/127  STEP %d", stepVel[eCh][ePat][eStep], eStep + 1);
  else if (microMode) oled.printf("MICRO %+d", (int)microTiming[eCh][ePat][eStep]);
  else oled.printf("%s P%d %s%d L%d V%d/%d", CN[eCh], ePat + 1, NN[eNote % 12], eNote / 12 - 1, cnt ? P.len[eStep] : 0, cnt, nv);
  int v0 = (eStep / 16) * 16;
  for (int r = 0; r < 14; r++) {   // keyboard
    int n = eTop - r, pc = n % 12, y = 9 + r * 4;
    bool blk = (pc == 1 || pc == 3 || pc == 6 || pc == 8 || pc == 10);
    if (blk) oled.fillRect(0, y, 9, 3, WHITE); else oled.drawFastHLine(0, y + 3, 12, WHITE);
  }
  for (int k = 0; k < 16; k++) {
    int s = v0 + k; if (s >= C.psteps[ePat]) break;
    int x = 14 + k * 7;
    if (s % 4 == 0) for (int yy = 9; yy < 64; yy += 3) oled.drawPixel(x - 1, yy, WHITE);
    for (int v = 0; v < nv; v++) {
      uint8_t n = P.note[s][v];
      if (n && n <= eTop && n > eTop - 14) {
        int r = eTop - n, l = min((int)P.len[s], 16 - k);
        oled.fillRect(x, 9 + r * 4, l * 7 - 1, 3, WHITE);
      }
    }
  }
  oled.drawRect(14 + (eStep - v0) * 7 - 1, 9 + (eTop - eNote) * 4 - 1, 8, 5, SSD1306_INVERSE);
  int ph; if (playheadInPat(ph) && ph >= v0 && ph < v0 + 16) oled.drawFastHLine(14 + (ph - v0) * 7, 8, 6, WHITE);
}

void drawFx() {
  Chan& C = ch[fxCh];
  oled.setCursor(0, 0); oled.printf("FX  CH%d %s", fxCh + 1, CN[fxCh]);
  const char* names[16] = { "BPM", "SWING", "VOL", "PITCH", "LO-PASS", "HI-PASS", "CRUSH", "P1 STEPS", "P2 STEPS", "P3 STEPS", "P4 STEPS", "WAVE", "ATTACK", "DECAY", "SUSTAIN", "RELEASE" };
  const char* waves[4] = { "SAW", "SQR", "TRI", "SIN" };
  int n = fxCount(fxCh);
  int top = max(0, fxSel - 5);
  for (int i = 0; i < 6; i++) {
    int idx = top + i; if (idx >= n) break;
    int y = 10 + i * 9;
    bool sel = (idx == fxSel);
    if (sel) oled.fillRect(0, y - 1, 128, 9, WHITE);
    oled.setTextColor(sel ? BLACK : WHITE);
    oled.setCursor(2, y);
    if (USE_BT && idx == n - 1) oled.print("BLUETOOTH");
    else if (isDrum(fxCh) && idx == 12) oled.print("SAMPLE EDITOR");
    else oled.print((isDrum(fxCh) && idx == 11) ? "KIT" : names[idx]);
    oled.setCursor(80, y);
#if USE_BT
    if (idx == n - 1) { oled.print(btConnected() ? "ON" : (btA2dpStarted ? "scan" : "off")); continue; }
#endif
    switch (idx) {
      case 0:  oled.print((int)bpm); break;
      case 1:  oled.print((int)swing); oled.print('%'); break;
      case 2:  oled.print(C.vol); break;
      case 3:  oled.print(C.pitch); break;
      case 4:  oled.print(C.lp); break;
      case 5:  oled.print(C.hp); break;
      case 6:  oled.print(C.crush); break;
      case 7: case 8: case 9: case 10: oled.print((int)C.psteps[idx - 7]); break;
      case 11:
        if (isDrum(fxCh)) {
          const char* s = strrchr(kitPath, '/'); s = s ? s + 1 : kitPath;
          oled.setCursor(36, y); oled.printf("%.14s", *s ? s : "-");
        } else oled.print(waves[C.wave]);
        break;
      case 12: oled.print(C.atk); break;
      case 13: oled.print(C.dec); break;
      case 14: oled.print(C.sus); break;
      case 15: oled.print(C.rel); break;
    }
  }
  oled.setTextColor(WHITE);
}

void drawSample() {
  SampleFx& f = sampleFx[sampleLane];
  const char* names[SAMPLE_ITEMS] = {"START", "END", "REVERSE", "NORMALIZE", "PITCH", "FINE", "SPEED", "ATTACK", "DECAY", "SUSTAIN", "RELEASE", "FADE IN", "FADE OUT", "LOOP", "CHOKE", "TRANSIENT", "VOLUME"};
  char val[20];
  switch(sampleItem) {
    case 0: snprintf(val,sizeof(val),"%u%%",f.start); break;
    case 1: snprintf(val,sizeof(val),"%u%%",f.end); break;
    case 2: strcpy(val,f.reverse?"ON":"OFF"); break;
    case 3: strcpy(val,f.normalize?"ON":"OFF"); break;
    case 4: snprintf(val,sizeof(val),"%+d st",f.pitch); break;
    case 5: snprintf(val,sizeof(val),"%+d ct",f.fine); break;
    case 6: snprintf(val,sizeof(val),"%u%%",f.speed); break;
    case 7: snprintf(val,sizeof(val),"%u ms",f.attack); break;
    case 8: snprintf(val,sizeof(val),"%u ms",f.decay); break;
    case 9: snprintf(val,sizeof(val),"%u%%",f.sustain); break;
    case 10: snprintf(val,sizeof(val),"%u ms",f.release); break;
    case 11: snprintf(val,sizeof(val),"%u%%",f.fadeIn); break;
    case 12: snprintf(val,sizeof(val),"%u%%",f.fadeOut); break;
    case 13: strcpy(val,f.loop?"ON":"OFF"); break;
    case 14: snprintf(val,sizeof(val),"G%u",f.choke); break;
    case 15: strcpy(val,"DETECT"); break;
    default: snprintf(val,sizeof(val),"%u",drumVol[sampleLane]); break;
  }

  oled.setCursor(0,0); oled.printf("SAMPLE LANE %d",sampleLane+1);

  // Larger waveform window: per column it shows the sample range (min/max),
  // and connects the mean values with a line so that short kick transients do not become dots.
  const int waveTop = 10, waveHeight = 24;
  const int waveCenter = waveTop + waveHeight/2;
  const int waveAmp = 10;
  oled.drawRect(0,waveTop,128,waveHeight,WHITE);
  if(smp[sampleLane].n && smp[sampleLane].d) {
    uint32_t total = smp[sampleLane].n;
    uint32_t st = (uint64_t)total*f.start/100;
    uint32_t en = (uint64_t)total*f.end/100;
    int prevX = -1, prevY = waveCenter;
    for(int x=1;x<127;x++) {
      uint32_t a = (uint64_t)(x-1)*total/124;
      uint32_t b = (uint64_t)x*total/124;
      if(b<=a) b=a+1;
      if(a<st) a=st;
      if(b>en) b=en;
      if(a>=b || a>=total) { prevX=-1; continue; }

      int lo=127, hi=-128, sum=0, count=0;
      // Limited number of reads per column so that drawing does not load the audio engine.
      uint32_t span=b-a;
      uint32_t stride=max((uint32_t)1, (span+23)/24);
      for(uint32_t i=a;i<b && i<total;i+=stride) {
        int v=(int)smp[sampleLane].d[i]-128;
        if(v<lo) lo=v;
        if(v>hi) hi=v;
        sum+=v; count++;
      }
      if(!count) continue;
      int yHi=waveCenter-(hi*waveAmp/128);
      int yLo=waveCenter-(lo*waveAmp/128);
      int yMid=waveCenter-((sum/count)*waveAmp/128);
      yHi=constrain(yHi,waveTop+2,waveTop+waveHeight-2);
      yLo=constrain(yLo,waveTop+2,waveTop+waveHeight-2);
      yMid=constrain(yMid,waveTop+2,waveTop+waveHeight-2);
      if(prevX>=0) oled.drawLine(prevX,prevY,x,yMid,WHITE);
      oled.drawFastVLine(x,min(yHi,yLo),max(1,abs(yLo-yHi)+1),WHITE);
      prevX=x; prevY=yMid;
    }
  } else {
    oled.setCursor(35,18); oled.print("NO SAMPLE");
  }

  // Three options stay visible; the list scrolls so that the selected item is in the middle.
  int top = constrain(sampleItem-1,0,SAMPLE_ITEMS-3);
  for(int r=0;r<3;r++) {
    int idx=top+r, y=37+r*9;
    bool sel=idx==sampleItem;
    if(sel) oled.fillRect(0,y-1,128,9,WHITE);
    oled.setTextColor(sel?BLACK:WHITE);
    oled.setCursor(2,y); oled.print(names[idx]);
    oled.setCursor(88,y); oled.print(idx==sampleItem?val:"");
  }
  oled.setTextColor(WHITE);
}

void drawSong() {
  if (songNaming) {
    oled.setCursor(0,0); oled.print("NAME (8 CHARS MAX)");
    oled.drawRect(0,11,128,11,WHITE);
    oled.setCursor(3,13); oled.print(songNameEdit);
    oled.setCursor(0,26); oled.print("UP/DN:ROW LF/RT:KEY");
    for(int y=0;y<4;y++) {
      oled.setCursor(2,36+y*7);
      for(int x=0;x<10;x++) {
        bool sel=(x==keyX && y==keyY);
        if(sel) oled.fillRect(1+x*12,35+y*7,12,8,WHITE);
        oled.setTextColor(sel?BLACK:WHITE);
        oled.setCursor(3+x*12,36+y*7); oled.write(songKeyRows[y][x]);
      }
    }
    oled.setTextColor(WHITE);
    return;
  }
  if (sConfirm) {                                   // delete confirmation screen
    oled.setCursor(0, 0); oled.print("DELETE SONG");
    oled.setCursor(0, 14);
    if (sSlot == AUTO_SLOT) oled.print("Delete slot A (auto)?");
    else oled.printf("Delete slot %d?", sSlot + 1);
    oled.setCursor(0, 26); oled.print("Cannot be undone!");
    for (int k = 0; k < 2; k++) {                   // k=0 NO, k=1 YES
      bool sel = (sConfYes == (k == 1));
      int x = 14 + k * 58;
      if (sel) oled.fillRect(x, 42, 42, 14, WHITE); else oled.drawRect(x, 42, 42, 14, WHITE);
      oled.setTextColor(sel ? BLACK : WHITE);
      oled.setCursor(x + (k ? 12 : 15), 46); oled.print(k ? "YES" : "NO");
      oled.setTextColor(WHITE);
    }
    return;
  }
  oled.setCursor(0, 0); oled.print("SONG / NAME");
  int top = (sSlot >= 8) ? sSlot - 7 : 0;       // 8 visible out of 16 slots + AUTO
  for (int r = 0; r < 8; r++) {
    int i = top + r;
    int y = 10 + r * 7;
    bool sel = (i == sSlot);
    if (sel) oled.fillRect(0, y - 1, 78, 8, WHITE);
    oled.setTextColor(sel ? BLACK : WHITE);
    oled.setCursor(2, y);
    if (i == AUTO_SLOT) {
      oled.print("A AUTO");
    } else {
      oled.printf("%d ", i + 1);
      if (songNames[i][0]) oled.print(songNames[i]);
      else oled.print(slotUsed[i] ? "UNTITLED" : "-");
    }
  }
  oled.setTextColor(WHITE);
  for (int a = 0; a < 4; a++) {
    int y = 10 + a * 10;
    if (sAct == a) oled.fillRect(82, y - 1, 44, 10, WHITE);
    oled.setTextColor(sAct == a ? BLACK : WHITE);
    oled.setCursor(86, y); oled.print(a == 0 ? "LOAD" : a == 1 ? "SAVE" : a == 2 ? "WAV" : "DEL");
  }
  oled.setTextColor(WHITE);
  if (millis() - sMsgT < 2000 && sMsg[0]) { oled.setCursor(66, 54); oled.print(sMsg); }
}

void drawBrowse() {
  oled.setCursor(0, 0);
  if (millis() - bMsgT < 2000 && bMsg[0]) oled.print(bMsg);
  else if (bWav > 0) oled.printf("CH1 %dwav HOLD=LOAD", bWav);
  else oled.print("CH1 no wav here");
  int pl = strlen(bPath);
  oled.setCursor(0, 10); oled.print(pl > 21 ? bPath + pl - 21 : bPath);
  bool root = !strcmp(bPath, "/");
  int total = bCnt + (root ? 0 : 1);
  for (int i = 0; i < 5; i++) {
    int r = bTop + i; if (r >= total) break;
    int y = 20 + i * 9;
    bool sel = (r == bSel);
    if (sel) oled.fillRect(0, y - 1, 128, 9, WHITE);
    oled.setTextColor(sel ? BLACK : WHITE);
    oled.setCursor(2, y);
    if (!root && r == 0) oled.print("..");
    else { oled.print('/'); oled.print(bEnt[r - (root ? 0 : 1)]); }
  }
  oled.setTextColor(WHITE);
}

#if USE_BT
void drawBt() {
  oled.setCursor(0, 0);
  if (btConnected())    oled.printf("BT OK %.15s", btTarget);
  else if (btTarget[0]) oled.printf("BT .. %.15s", btTarget);
  else                  oled.print("BT scanning...");
  oled.setCursor(0, 10);
  oled.printf("st%d cb%u m%u %uk", (int)a2dp.get_connection_state(), (unsigned)btCbCnt, (unsigned)btMatchCnt, (unsigned)(ESP.getFreeHeap() / 1000));
  int n = btDevN;
  if (n == 0) { oled.setCursor(2, 28); oled.print("pairing mode on?"); oled.setCursor(2, 38); oled.print("hold + = reboot+conn"); }
  for (int i = 0; i < 5; i++) {
    int r = btTop + i; if (r >= n) break;
    int y = 20 + i * 9;
    bool sel = (r == btSel);
    if (sel) oled.fillRect(0, y - 1, 128, 9, WHITE);
    oled.setTextColor(sel ? BLACK : WHITE);
    oled.setCursor(2, y);
    bool tgt = btTarget[0] && !strncmp(btDev[r], btTarget, sizeof(btTarget) - 1);
    oled.print(tgt ? '*' : ' ');
    oled.printf("%.19s", btDev[r]);
  }
  oled.setTextColor(WHITE);
}
#endif

void drawRam() {
  uint32_t heapFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  uint32_t heapBlock = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  bool hasPsram = psramFound();
  uint32_t psramFree = hasPsram ? heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : 0;
  uint32_t psramBlock = hasPsram ? heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : 0;

  oled.setCursor(0,0);  oled.print("RAM MONITOR");
  oled.setCursor(0,11); oled.printf("HEAP FREE:  %u KB", (unsigned)(heapFree / 1024));
  oled.setCursor(0,21); oled.printf("HEAP BLOCK: %u KB", (unsigned)(heapBlock / 1024));
  if (hasPsram) {
    oled.setCursor(0,31); oled.printf("PSRAM FREE: %u KB", (unsigned)(psramFree / 1024));
    oled.setCursor(0,41); oled.printf("PSRAM BLOCK:%u KB", (unsigned)(psramBlock / 1024));
  } else {
    oled.setCursor(0,34); oled.print("PSRAM: NOT FOUND");
  }
  oled.setCursor(0,53); oled.print(playing ? "PLAYING" : "STOPPED");
  oled.setCursor(64,53); oled.print("CH1-4: CLOSE");
}

void draw() {
  oled.clearDisplay();
  oled.setTextSize(1); oled.setTextColor(WHITE);
  if (ramOverlay) {
    drawRam();
    oled.display();
    return;
  }
  if (scr == S_MAIN) drawMain();
  else if (scr == S_EDIT) { if (isDrum(eCh)) drawDrum(); else drawRoll(); }
  else if (scr == S_FX) drawFx();
  else if (scr == S_SONG) drawSong();
  else if (scr == S_SAMPLE) drawSample();
#if USE_BT
  else if (scr == S_BT) drawBt();
#endif
  else drawBrowse();
  // Small warning sign in the top-right corner if some WAV was not loaded.
  if (wavRamWarning) {
    oled.setTextSize(1); oled.setTextColor(WHITE);
    oled.drawFastVLine(126, 1, 4, WHITE);
    oled.drawPixel(126, 7, WHITE);
  }
  if ((int32_t)(autoMsgUntil - millis()) > 0) oled.fillRect(120, 0, 4, 4, WHITE);   // small square = autosave just saved
  oled.display();
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
#if USE_BT
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);           // A2DP is classic BT: free the RAM reserved for BLE (~30 kB) BEFORE computing the budget
#endif
  for (int i = 0; i < NB; i++) pinMode(PIN_B[i], INPUT_PULLUP);
  Wire.begin(21, 22); Wire.setClock(400000);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setTextWrap(false);
  splashBar("loading samples...", 0);
  uint32_t t0 = millis();
  for (int i = 0; i < 256; i++) sinT[i] = sinf(i * 2 * PI / 256);
  memset(kitPath, 0, sizeof(kitPath));
  sdOk = SD.begin(SD_CS);
  splashBar("loading samples...", 10);
  if (sdOk) {
    SD.mkdir("/songs");
    loadSongNames();  // Load saved names from the SD card after every start/reset of the ESP32
    int32_t fm = psramFound() ? (int32_t)ESP.getFreePsram() : (int32_t)ESP.getFreeHeap();
    // On an ESP32 without PSRAM a 100 kB reservation for BT could have lowered
    // the sample budget to ZERO, so the kit would load without a single drum.
    // 45 kB is left for BT/audio/SD, and the actual malloc in loadWav still
    // checks how much memory really exists.
    int32_t heapReserve = BT_HEAP_RESERVE;          // v26: 70 kB (was 80 kB); with 80 kB BT stayed >20 kB free. The rest is added in bankGrowTick() after connecting
    int32_t bb = psramFound() ? (fm - 200000) : (fm - 50000 - heapReserve);
    if (bb < 0) bb = 0;                                      // don't promise RAM that doesn't really exist
    if (!psramFound() && bb < 32000) bb = 32000;             // v25: the lazy bank needs little - never leave the sound completely silent (loadWav still checks the actual malloc)
    if (psramFound() && bb > 3000000) bb = 3000000;        // safety limit for the PSRAM bank
    bankBudget = (uint32_t)bb;
    Serial.printf("[MEM] free=%d bankBudget=%u\n", (int)fm, (unsigned)bankBudget);                              // the whole budget goes to the CH1 kit (there is no CH4 bank anymore)
    if (!loadCfg()) {                                       // first time: try the old /ch1 folder
      if (SD.exists("/ch1")) strcpy(kitPath, "/ch1");
    }
    char want[64]; memcpy(want, kitPath, sizeof(want)); memset(kitPath, 0, sizeof(kitPath));
    kitBarLo = 20; kitBarHi = 100;                           // loading bar: 20..100 % = kit loading
    if (want[0]) loadKit(want, "loading samples...", false);
    else { for (int p = 20; p <= 100; p += 4) { splashBar("loading samples...", p); delay(15); } }
    loadSampleFx();
    // v25: normalization is computed in bankLoadLane() when a lane is actually loaded
    kitBarLo = 0; kitBarHi = 100;
    splashBar("loading samples...", 100);
  } else {
    splash("no SD card");
    delay(1500);
  }
  while (millis() - t0 < 1500) delay(10);                   // splash stays visible for at least 1.5 s
  for (int c = 0; c < 4; c++) ch[c].song[0] = 1;
  ch[1].wave = 0; ch[2].wave = 1; ch[3].wave = 2;           // BASS saw, MEL square, CHD triangle
  refreshSlots();
  memset(stepVel, 127, sizeof(stepVel)); 
  savedHash = autoHash = songHash();                        // the empty/initial song is not autosaved until something changes
  lastInputT = millis();
#if USE_BT
  if (btName[0]) { strcpy(btTarget, btName); btBegin(); }   // saved headphones -> connect automatically
#endif
  initAudio();
}

void loop() {
  scanButtons();
  for (int i = 0; i < NB; i++) if (btn[i].down || btn[i].press || btn[i].rel) { lastInputT = millis(); break; }
  handleInput();
  autosaveTick();
  bankTick();
#if USE_BT
  btTick();
  bankGrowTick();
#endif
  static uint32_t lastD = 0;
  if (millis() - lastD > 40) { lastD = millis(); draw(); }
  delay(2);
}
