# PocketDAW
# PocketDAW

**A pocket-sized, battery-friendly sampler / sequencer / chord machine built on a single ESP32.**
128×64 OLED, 11 buttons, a microSD card for your samples and songs, and sound out through the built-in DAC or straight to Bluetooth headphones.

PocketDAW is open source. Build one, hack it, send pull requests.

> **Status:** working firmware (`PocketDAW_v34_performance_fx_2khz.ino`). The project is under active development; see [Known limitations](#known-limitations).

---

## Table of contents

1. [Features](#features)
2. [What you need (bill of materials)](#what-you-need-bill-of-materials)
3. [Pinout and wiring](#pinout-and-wiring)
4. [Building the firmware](#building-the-firmware)
5. [Preparing the SD card](#preparing-the-sd-card)
6. [Quick start](#quick-start)
7. [User manual](#user-manual)
   - [Button reference](#button-reference)
   - [Main screen (song / pattern chain)](#main-screen-song--pattern-chain)
   - [Pattern editor](#pattern-editor)
   - [FX / channel settings menu](#fx--channel-settings-menu)
   - [Sample editor](#sample-editor)
   - [Kit browser](#kit-browser)
   - [Song menu (load / save / export / delete)](#song-menu-load--save--export--delete)
   - [Performance FX](#performance-fx)
   - [Bluetooth headphones](#bluetooth-headphones)
   - [RAM monitor and status indicators](#ram-monitor-and-status-indicators)
8. [How it works (technical notes)](#how-it-works-technical-notes)
9. [Known limitations](#known-limitations)
10. [Troubleshooting](#troubleshooting)
11. [Contributing](#contributing)
12. [License](#license)

---

## Features

**Channels (4)**

| Channel | Name | Type | Polyphony |
|---------|------|------|-----------|
| CH1 | `DRM` | Drum sampler, 8 lanes (one WAV per lane) | 8 lanes |
| CH2 | `BAS` | Oscillator, default saw (bass) | 4 voices |
| CH3 | `MEL` | Oscillator, default square (melody) | 4 voices |
| CH4 | `CHD` | Oscillator, default triangle (chords), piano roll | 6 voices |

**Sequencing**

- 4 patterns (P1–P4) per channel, each with its own length: **8 / 16 / 32 / 64 / 128 steps**
- Song chain per channel: a fixed grid of **128 cells**; place patterns in any order (e.g. `1 3 2 4 3 2 1`). A 16-step pattern occupies 1 cell, 32 = 2, 64 = 4, 128 = 8
- Piano roll style note editor for CH2–CH4, with per-step note length
- Drum editor with **per-step pitch** (±12 semitones, each hit can have its own pitch)
- **Per-step velocity** and **micro timing** (shift a step by up to ±¼ of a 1/16 note)
- **Swing** per channel (50–75 %)
- BPM 40–240
- Pattern copy / paste
- Mute per channel

**Sound**

- Per-channel volume, pitch, low-pass, high-pass, bitcrusher and ADSR (oscillator channels)
- Oscillator waveforms: saw, square, triangle, sine
- **Sample editor** per drum lane: start/end trim, reverse, normalize, pitch, fine tune, speed, ADSR, fade in/out, loop, choke groups, transient detection, volume
- **Momentary performance FX** while playing: beat repeat, delay, high-pass sweep, low-pass sweep

**Storage**

- Sample **kits** are just folders of `.wav` files on the SD card (browse them on the device)
- 16 song slots + an **autosave** slot, each song can be named (8 characters)
- **Export the song to a WAV file** (22050 Hz, 16-bit, mono) straight to the SD card
- Safe saving (temp + backup files, size verification)

**Audio output**

- ESP32 built-in DAC (GPIO25) at 22050 Hz
- Optional **Bluetooth A2DP** output to wireless headphones/speakers (classic ESP32 only), with automatic reconnect to the last used device

**Memory management**

- Samples are loaded **on demand**: only drum lanes that are actually used in a pattern are kept in RAM
- Silent sample tails are trimmed, and bass-heavy samples are automatically stored at 8 kHz or 4 kHz (only if it is inaudible) to save RAM
- Works with or without PSRAM

---

## What you need (bill of materials)

| Part | Notes |
|------|-------|
| ESP32 dev board (classic ESP32, e.g. ESP32-DevKitC / WROOM-32) | **Classic ESP32 only.** The built-in DAC and Bluetooth classic (A2DP) do not exist on ESP32-S2/S3/C3. PSRAM (WROVER) is optional but gives you a much larger sample bank. |
| SSD1306 OLED, 128×64, I²C | Address `0x3C` |
| microSD card module (SPI) | Use a module that works at 3.3 V logic (or has a level shifter). Card formatted as FAT32 |
| microSD card | Any size supported by your module, FAT32 |
| 11 × tactile push buttons | Wired between the GPIO and GND (internal pull-ups are used, no external resistors needed) |
| Audio output | 3.5 mm jack **or** small amplifier/speaker, see [Audio output](#audio-output) |
| Hook-up wire / perfboard / PCB / enclosure | Your choice |
| Bluetooth headphones (optional) | Any A2DP-capable headphones or speaker |

---

## Pinout and wiring

### Buttons

All buttons connect **one side to the GPIO pin, the other side to GND**. The firmware enables the internal pull-up resistors.

| Function | Name in code | ESP32 GPIO |
|----------|--------------|-----------|
| Up | `B_UP` | **27** |
| Down | `B_DN` | **12** |
| Left | `B_LF` | **32** |
| Right | `B_RT` | **33** |
| Center (OK / select) | `B_CE` | **4** |
| Plus (`+`) | `B_PL` | **16** |
| Minus (`-`) | `B_MI` | **17** |
| Channel 1 (`C1`) | `B_C1` | **13** |
| Channel 2 (`C2`) | `B_C2` | **14** |
| Channel 3 (`C3`) | `B_C3` | **15** |
| Channel 4 (`C4`) | `B_C4` | **3** |

### OLED display (I²C)

| OLED pin | ESP32 |
|----------|-------|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO **21** |
| SCL | GPIO **22** |

I²C runs at 400 kHz, display address `0x3C`.

### microSD card (SPI, VSPI)

| SD module pin | ESP32 |
|---------------|-------|
| VCC | 3V3 (or 5 V if your module has a regulator and level shifter) |
| GND | GND |
| CS | GPIO **5** |
| SCK | GPIO **18** |
| MISO | GPIO **19** |
| MOSI | GPIO **23** |

### Audio output

| Signal | ESP32 |
|--------|-------|
| DAC audio out | GPIO **25** (DAC1) |

The DAC output is a unipolar signal centered around roughly half of 3.3 V. For headphones or a line input, a simple and common approach is:

```
GPIO25 ──[ 10 µF ]──[ 1 kΩ ]──┬── 3.5 mm jack TIP
        (electrolytic,         │
         + towards GPIO25)     └── (optional: 10 kΩ pot / resistor to GND for volume)
GND ───────────────────────────── 3.5 mm jack SLEEVE
```

The capacitor removes the DC offset. If you drive a speaker, put the signal through an amplifier module (for example a small class-D or LM386-type board). Component values are suggestions; adjust them to your amplifier or headphones. **Keep the volume low when testing with headphones.**

If you use Bluetooth only, you do not need any audio hardware (the DAC output keeps running in parallel anyway).

### Full wiring summary

```
                 ┌────────────────────────────┐
                 │           ESP32            │
  OLED SDA ──────┤ GPIO21                     │
  OLED SCL ──────┤ GPIO22                     │
                 │                            │
  SD CS   ───────┤ GPIO5                      │
  SD SCK  ───────┤ GPIO18                     │
  SD MISO ───────┤ GPIO19                     │
  SD MOSI ───────┤ GPIO23                     │
                 │                            │
  Audio out ─────┤ GPIO25 (DAC1)              │
                 │                            │
  UP ────────────┤ GPIO27     C1 ─────── GPIO13
  DOWN ──────────┤ GPIO12     C2 ─────── GPIO14
  LEFT ──────────┤ GPIO32     C3 ─────── GPIO15
  RIGHT ─────────┤ GPIO33     C4 ─────── GPIO3
  CENTER ────────┤ GPIO4      +  ─────── GPIO16
                 │            -  ─────── GPIO17
                 └────────────────────────────┘
        (every button: GPIO ── button ── GND)
```

### Wiring notes (please read)

Some of the chosen pins have special roles on the ESP32. Everything works as described, but be aware of the following:

- **GPIO3 (C4) is the UART0 RX pin.** Do not hold the C4 button while uploading firmware, and note that you cannot type into the Serial Monitor (serial *output* for debugging still works).
- **GPIO12 (Down) is a boot strapping pin** (flash voltage). Do **not** add an external pull-up to it, and do not hold the Down button while the board powers on or resets.
- **GPIO15 (C3) and GPIO5 (SD CS) are also strapping pins.** Avoid holding C3 during power-up.
- Do not hold any buttons while powering on or resetting the board.
- Power the SD module from the supply it is designed for; many cheap modules need 5 V on VCC and include a level shifter, others are 3.3 V only.

---

## Building the firmware

### Requirements

- **Arduino IDE** (or `arduino-cli` / PlatformIO)
- **ESP32 board support (Arduino-ESP32 core 2.0.x)**, i.e. the `esp32` package by Espressif, version 2.0.x. The sketch uses the legacy I²S driver API, so core 3.x is not supported.
- Libraries (install through the Library Manager unless noted):
  - **Adafruit SSD1306**
  - **Adafruit GFX Library**
  - **ESP32-A2DP** by pschatzmann (https://github.com/pschatzmann/ESP32-A2DP), only needed if Bluetooth is enabled
- Built-in libraries used: `Wire`, `SPI`, `SD`, `driver/i2s`

### Arduino IDE settings

| Setting | Value |
|---------|-------|
| Board | `ESP32 Dev Module` (or the matching board for your hardware) |
| Partition Scheme | **Huge APP (3MB No OTA / 1MB SPIFFS)**, required because the Bluetooth stack makes the firmware large |
| Upload speed | 921600 (or lower if uploads are unreliable) |

### Bluetooth on/off

At the top of the sketch:

```cpp
#define USE_BT 1   // 1 = Bluetooth A2DP headphones, 0 = wired output only (GPIO25)
```

Set `USE_BT` to `0` if you do not need Bluetooth. This frees RAM (more samples fit) and removes the dependency on the ESP32-A2DP library.

### Upload

1. Open `PocketDAW_v34_performance_fx_2khz.ino`.
2. Select board, port and partition scheme as above.
3. Click **Upload**. (Release all buttons, especially **C4/GPIO3**.)

---

## Preparing the SD card

1. Format the card as **FAT32**.
2. Create one folder per sample kit and put `.wav` files in it, for example:

```
/
├── 808kit/
│   ├── 01_kick.wav
│   ├── 02_snare.wav
│   ├── 03_hat_closed.wav
│   ├── 04_hat_open.wav
│   ├── 05_clap.wav
│   ├── 06_tom.wav
│   ├── 07_rim.wav
│   └── 08_perc.wav
├── hiphop/
│   └── ...
├── songs/            ← created automatically
└── exports/          ← created automatically
```

**Kit rules**

- A kit is a **folder**. The **first 8 `.wav` files in alphabetical order** become the 8 drum lanes of CH1 (lane 1 = first file). Name your files `01_...`, `02_...` to control the order.
- Files or folders whose names start with `.` or `_` are ignored. A folder named `songs` is not shown as a kit.
- File names must be shorter than 32 characters.
- Supported WAV formats: **PCM, 8-bit or 16-bit, mono or stereo, any sample rate**. They are converted on load to mono 8-bit at up to 16 kHz.
- **Maximum length is 0.5 seconds per sample** (longer files are cut). Silent tails are trimmed automatically.
- Unsupported or unreadable files are skipped and the display shows a small warning mark in the top right corner.
- You can keep as many kits as you like on the card and switch between them with the [kit browser](#kit-browser). The selected kit is remembered (`pocketdaw.cfg`) and is stored with each song.

**Files written by PocketDAW**

| Path | Content |
|------|---------|
| `/pocketdaw.cfg` | Last used kit, Bluetooth device name |
| `/samplefx.bin` | Sample editor settings and per-lane volumes |
| `/songs/s1.bin … s16.bin` | Song slots 1–16 |
| `/songs/auto.bin` | Autosave (slot `A`) |
| `/songs/names.bin` | Song names |
| `/exports/song_001.wav …` | Exported WAV files (22050 Hz, 16-bit, mono) |

---

## Quick start

1. Wire everything up, flash the firmware, insert the SD card with at least one kit folder.
2. Power on. The splash screen shows **PocketDaw** and a loading bar while the kit is analyzed.
3. On the main screen, press **Down** to move to the `DRM` row. Press **Center** on the first cell: it creates **pattern 1** and opens the pattern editor.
4. In the editor, use **Up/Down** to choose a drum lane, **Left/Right** to choose a step and **Center** to switch a hit on or off (you hear a preview).
5. Hold **Center** to go back to the main screen.
6. **Hold `+`** (about 0.6 s) to start playback, **hold `-`** to stop.
7. Repeat on the `BAS`, `MEL` and `CHD` rows to add notes.

---

## User manual

### Button reference

The device has 11 buttons. Many actions differ between a **tap** (short press) and a **hold** (about 0.6 s).

| Button | Role |
|--------|------|
| **Up / Down / Left / Right** | Move the cursor / change selection |
| **Center** | Select, toggle a step, enter the editor. Hold = back / open song menu (depends on screen) |
| **+ / −** | Increase / decrease the selected value. Holding auto-repeats. On the main screen, hold `+` = PLAY, hold `-` = STOP |
| **C1–C4** | Channel buttons. Tap on the main screen = mute/unmute that channel. Hold while stopped = open that channel's FX menu. Hold while playing = momentary performance FX. In the editor they have special functions (see below) |

### Main screen (song / pattern chain)

The main screen is a **time bar**: four rows (one per channel) with a fixed grid of 128 cells each. Each cell holds either `-` (empty) or a pattern number `1`–`4`. A pattern of 16/32/64/128 steps is drawn as one block of 1/2/4/8 cells with its number centered. The bar scrolls while you move the cursor and follows the playhead during playback.

The top row (the **transport bar**) has three items: **PLAY/STOP**, **BPM**, **SWING**. A channel that is muted is marked with an `x` before its name.

| Action | Result |
|--------|--------|
| **Up / Down** | Move between the transport bar and the four channel rows |
| **Left / Right** (transport) | Select PLAY/STOP, BPM or SWING |
| **+ / −** (transport on BPM) | Change tempo (40–240) |
| **+ / −** (transport on SWING) | Change swing for **all** channels (50–75). Each channel can still be tweaked afterwards in its FX menu |
| **Center** (transport on PLAY/STOP) | Start / stop playback |
| **Left / Right** (channel row) | Move to the previous/next pattern block |
| **+ / −** (channel row) | Change the pattern number in the cell (0 = empty, 1–4). A new pattern **replaces** the cells it occupies |
| **Center** (empty cell) | Creates pattern 1 and opens the editor |
| **Center** (pattern) | Opens the pattern editor |
| **Hold +** (stopped) | Play |
| **Hold −** (playing) | Stop |
| **Hold Center** | Open the song menu |
| **Tap C1–C4** | Mute / unmute the channel |
| **Hold C1–C4** (stopped) | Open the FX menu of that channel |
| **Hold C1–C4** (playing) | Performance FX (see below) |

Patterns in a chain are played in order. Empty cells between patterns stay silent for their duration. When no patterns remain after the current position, playback wraps around to the start of the chain. Because every channel has its own chain, channels can loop at different lengths.

### Pattern editor

Open it from the main screen with **Center** on a pattern. The header shows channel, pattern, pattern length and the current step (for example `DRM P1 L16 S1`).

**Common controls**

| Action | Result |
|--------|--------|
| **Left / Right** | Previous / next step |
| **Tap C1–C4** | Switch to pattern 1–4 of this channel |
| **Hold C1** | **Copy** the current pattern |
| **Hold C2** | **Paste** the copied pattern into the current slot (note data, pitches, lengths, micro timing, velocity and step count) |
| **Hold C3** | Toggle **micro timing mode** (`MICRO ON/OFF`). In this mode `+`/`−` shift the selected step in time by −16…+16 (up to ±¼ of a 1/16 note) |
| **Hold C4** + `+`/`−` | Change the **velocity** of the selected step (1–127, steps of 4). Release C4 to return to normal editing |
| **Hold Center** | Back to the main screen |

**CH1 drum editor**

The screen shows 8 lanes (one per sample) and the steps of the pattern.

| Action | Result |
|--------|--------|
| **Up / Down** | Select drum lane (1–8) |
| **Center** | Switch the hit on/off at the current step and lane (plays a preview) |
| **+ / −** | Pitch of **this hit only**, −12…+12 semitones (shown as `T+2`, `T-1`, …) |

Because pitch is stored per hit, one kick can be +2 and the next −1 in the same pattern.

A lane's sample is loaded into RAM as soon as you place its first hit.

**CH2 / CH3 / CH4 note editor (piano roll)**

| Action | Result |
|--------|--------|
| **Up / Down** | Move the note cursor; the view scrolls. Range per channel (MIDI notes): CH2 bass 24–59, CH3 melody 48–95, CH4 chords 36–83 |
| **Center** | Add the note at the current step, or remove it if it is already there (polyphony limit per step: 4 on CH2/CH3, 6 on CH4) |
| **+ / −** | Note length of the step in 1/16 notes (1–16). All notes in a step share one length |

### FX / channel settings menu

Open it by **holding C1–C4** on the main screen while playback is stopped. **Left/Right** switch between channels, **Up/Down** select a row, **+/−** change the value. **Center** (tap or hold) closes the menu.

| Row | Range / values | Notes |
|-----|----------------|-------|
| BPM | 40–240 | Global |
| SWING | 50–75 | Global value in this menu |
| VOL | 0–255 | Channel volume |
| PITCH | −12…+12 | Channel pitch in semitones |
| LO-PASS | 8–255 | Low-pass filter (higher = more open) |
| HI-PASS | 0–255 | High-pass filter |
| CRUSH | 1–8 | Bitcrusher, bit depth |
| P1 STEPS | 8 / 16 / 32 / 64 / 128 | Length of pattern 1 |
| P2 STEPS | 8 / 16 / 32 / 64 / 128 | Length of pattern 2 |
| P3 STEPS | 8 / 16 / 32 / 64 / 128 | Length of pattern 3 |
| P4 STEPS | 8 / 16 / 32 / 64 / 128 | Length of pattern 4 |
| KIT *(CH1 only)* | `+` opens the kit browser | |
| WAVE *(CH2–CH4)* | saw / square / triangle / sine | |
| ATTACK *(CH2–CH4)* | 0–255 | ADSR |
| DECAY *(CH2–CH4)* | 0–255 | ADSR |
| SUSTAIN *(CH2–CH4)* | 0–255 | ADSR |
| RELEASE *(CH2–CH4)* | 0–255 | ADSR |
| SAMPLE EDITOR *(CH1 only)* | `+` opens the sample editor | Right before BLUETOOTH |
| BLUETOOTH *(if enabled)* | `+` opens the device list | Always the last row |

> Changing a pattern's step count rearranges the chain: the pattern now covers 1/2/4/8 cells and replaces what was in them.

### Sample editor

Open it from the CH1 FX menu (**SAMPLE EDITOR**). Settings are stored per drum lane.

| Control | Action |
|---------|--------|
| **Up / Down** | Select parameter |
| **Left / Right** | Previous / next drum lane |
| **+ / −** | Change the value (or toggle for on/off items) |
| **Center** | Preview the sample |
| **Hold C1** | Save the sample settings |
| **Hold Center** | Save and leave the sample editor |

| Parameter | Range | Description |
|-----------|-------|-------------|
| START | 0–99 % | Playback start (trim) |
| END | 1–100 % | Playback end (trim) |
| REVERSE | on/off | Play backwards |
| NORMALIZE | on/off | Boost the sample to a consistent peak level |
| PITCH | −24…+24 | Semitones |
| FINE | −100…+100 | Fine tune in cents |
| SPEED | 25–200 % | Playback speed |
| ATTACK | 0–250 | Envelope attack |
| DECAY | 0–250 | Envelope decay |
| SUSTAIN | 0–100 | Envelope sustain level |
| RELEASE | 0–250 | Envelope release |
| FADE IN | 0–50 | Fade in (percent of the duration) |
| FADE OUT | 0–50 | Fade out (percent of the duration) |
| LOOP | on/off | Loop the sample |
| CHOKE | 0–4 | Choke group (0 = off). Lanes in the same group cut each other off, e.g. open/closed hi-hat |
| TRANSIENT | press `+`/`−` | Auto-detects the first transient and sets START to it |
| VOLUME | 0–255 | Lane volume in the drum rack |

### Kit browser

Open it from the CH1 FX menu (**KIT**, press `+`). It lists folders on the SD card.

| Action | Result |
|--------|--------|
| **Up / Down** | Select entry |
| **Right** or **Center** | Open folder |
| **Left** | Go up one level |
| **Hold Center** | Load the current folder as the kit (only when it contains `.wav` files) |
| **−** | Cancel and go back to the FX menu |

### Song menu (load / save / export / delete)

Open it by **holding Center** on the main screen. There are 16 slots plus one autosave slot (`A`).

| Action | Result |
|--------|--------|
| **Up / Down** | Select slot |
| **Left / Right** | Select the action: **LOAD**, **SAVE**, **WAV**, **DEL** |
| **Center** | Run the selected action |
| **Hold Center** | Back to the main screen |

- **LOAD**: loads the song and the kit that was saved with it.
- **SAVE**: opens an on-screen keyboard to name the song (up to 8 characters). Move with the arrows, **Center** types a character, **`+`** deletes the last character, **`−`** (or hold Center) confirms and saves.
- **WAV**: renders the song **offline** (faster than real time) to `/exports/song_NNN.wav` (22050 Hz, 16-bit, mono). The length is one pass of the longest un-muted chain plus a short tail for releases. Performance FX are not recorded. **Hold Center to abort** a running export.
- **DEL**: asks for confirmation first (default answer is NO). Use Left/Right to choose and Center to confirm.
- **Autosave:** the current song is saved to slot `A` automatically after 3 s without any button press, at most every 30 s, and only when something changed. A small square in the top right corner flashes when an autosave happened. Slot `A` can be loaded like any other slot; it cannot be overwritten manually.

Song file format: `SM11`. Older songs (`SM5`–`SM10`) still load.

### Performance FX

While the song is **playing** and you are on the main screen, **press and hold** a channel button for a momentary effect. Releasing it switches the effect off immediately. The transport bar shows `FX1`–`FX4` while an effect is active.

| Hold | Effect | Description |
|------|--------|-------------|
| **C1** | Beat repeat | Repeats a short rhythmic slice of the audio |
| **C2** | Delay | Echo effect |
| **C3** | High-pass sweep | Cutoff rises slowly from about 100 Hz to 2 kHz |
| **C4** | Low-pass sweep | Cutoff falls slowly from about 10 kHz to 250 Hz |

The filter sweeps develop over roughly 1.5 seconds while the button is held. A quick tap (shorter than about 0.25 s) toggles mute instead.

### Bluetooth headphones

*(Only available when `USE_BT` is `1`. Classic ESP32 only.)*

1. Put your headphones in pairing mode.
2. Open the FX menu of any channel and go to the last row, **BLUETOOTH**. Press **`+`**.
3. PocketDAW scans and lists the devices it finds. Pick one with **Up/Down** and press **Center** to connect.
4. Once connected, the audio is sent to the headphones. The wired DAC output (GPIO25) keeps running in parallel.

| Action (on the Bluetooth screen) | Result |
|----------------------------------|--------|
| **Up / Down** | Select device |
| **Center** | Connect to the selected device |
| **Hold Center** | Forget / disconnect the saved device |
| **Hold +** | Save and restart the device (a clean connect by name on the next power-up). Only works when a device name is set and it is not connected yet |
| **−** | Back |

The name of the headphones is saved on the SD card, and they are reconnected automatically after power-up. If the saved device is not found, PocketDAW searches again every 30 seconds.

Bluetooth audio always has some latency (a prebuffer is used to prevent crackling). For live playing and tight timing, use the wired output.

### RAM monitor and status indicators

- **RAM monitor:** press and hold **C1 + C2 + C3 + C4 together for 1 second** to toggle a memory overlay on the main screen. While it is open, the buttons do not change the project; repeat the combination to close it.
- **Warning mark (top right):** a small `!` means some WAV file in the kit could not be loaded (unsupported format, unreadable, or not enough RAM).
- **Small square (top right):** an autosave has just completed.
- **`x` before a channel name** on the main screen: the channel is muted.

---

## How it works (technical notes)

- **Audio engine:** 22050 Hz, rendered in blocks of 128 samples by a dedicated FreeRTOS task and output through the ESP32 I²S peripheral in built-in DAC mode (8-bit effective resolution). The audio task runs on core 1 when Bluetooth is enabled (the Bluetooth stack uses core 0) and on core 0 otherwise.
- **Samples:** converted on load to unsigned 8-bit mono. Silent tails below the 8-bit noise floor are trimmed. If a sample has no significant high-frequency content (kick, sub, bass) it is stored at 8 kHz or 4 kHz, which saves ½ or ¼ of the RAM; the decision is made by measuring the interpolation error, so the sample is never downsampled if it would be audible.
- **On-demand bank:** only lanes used in at least one drum pattern (plus the lane open in the sample editor) live in RAM; others stay on the SD card. If RAM is tight, the budget is shared fairly: short samples stay whole and only the longest tails are shortened. Without PSRAM a minimum budget of 32 kB is guaranteed. With Bluetooth connected, the bank automatically grows into free heap that the Bluetooth stack no longer needs.
- **Bluetooth path:** audio is copied into a ring buffer with a prebuffer (~58 ms), then resampled 22050 → 44100 Hz stereo with a cubic (Catmull-Rom) resampler whose step is adaptively regulated by ±0.5 % to follow the difference between the I²S and Bluetooth clocks. This avoids buffer underruns and crackling.
- **Timing:** every channel schedules its own events, so micro timing and per-channel swing can move steps both earlier and later independently.
- **Safe saving:** songs are written to a temporary file, verified by size, and the previous file is kept as a backup during replacement, so an interrupted save does not destroy a valid song.

---

## Known limitations

- Samples are limited to **0.5 s**, mono, 8-bit. PocketDAW is a lo-fi, character-driven sampler, not a hi-fi one.
- The built-in ESP32 DAC has 8-bit resolution and no analog filtering; a simple low-pass on the output can make it sound smoother.
- **Classic ESP32 only** (built-in DAC and Bluetooth classic A2DP).
- Requires Arduino-ESP32 **core 2.0.x** (legacy I²S API).
- RAM is limited, especially without PSRAM and with Bluetooth enabled. Large kits with many active lanes may not fit completely (the `!` warning appears).
- Bluetooth adds latency.
- Some buttons use boot-strapping or UART pins (see [Wiring notes](#wiring-notes-please-read)).

---

## Troubleshooting

| Problem | What to check |
|---------|---------------|
| Display stays black | Check SDA = GPIO21, SCL = GPIO22, 3.3 V power, and that your display uses address `0x3C` |
| `no SD card` on the splash screen | Card must be FAT32, check CS = GPIO5 and the SPI wires, check the module's supply voltage |
| No sound | Check the audio wiring on GPIO25, the output circuit, VOL in the FX menu, and that the channel is not muted (`x`). Make sure at least one pattern with notes/hits exists |
| Drum lane is silent | The lane needs a WAV in the kit (first 8 files alphabetically) and at least one hit placed. Look for the `!` mark |
| Upload fails | Release all buttons, especially **C4 (GPIO3)**. Lower the upload speed. Try holding the BOOT button on boards that need it |
| Board does not boot / resets | Release all buttons at power-up, especially **Down (GPIO12)** and **C3 (GPIO15)**; remove any external pull-up on GPIO12 |
| Bluetooth headphones are not found | Put them in pairing mode **before** opening the Bluetooth screen, reopen the screen to rescan, or hold `+` on that screen to save and restart |
| Bluetooth will not connect when many samples are loaded | The Bluetooth stack needs RAM to connect. Use a smaller kit or `USE_BT 0` if you do not need it |
| Crackling on Bluetooth | Move closer to the headphones, avoid heavy 2.4 GHz interference |
| Compile error about I²S | Install Arduino-ESP32 **2.0.x**, not 3.x |
| Compile error: `BluetoothA2DPSource.h` not found | Install the **ESP32-A2DP** library or set `USE_BT` to `0` |

---

## Contributing

Issues and pull requests are welcome. Ideas for contributions: more effects, MIDI sync, a PCB design, enclosure files, documentation, translations of this README, more sample kits.

When you open an issue, please include:

- your board and whether it has PSRAM
- the Arduino-ESP32 core version
- the `USE_BT` setting
- a description of what you did and what you expected

---

## License

*Choose a license for the project (for example MIT, GPL-3.0 or CC BY-SA for hardware files) and put the full text in a `LICENSE` file, then reference it here.*

---

## Credits

- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306) and [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP) by Phil Schatzmann
- Espressif [Arduino-ESP32](https://github.com/espressif/arduino-esp32)
