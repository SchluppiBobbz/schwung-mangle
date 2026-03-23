# Mangle for Schwung

A 4-track sampler for your Ableton Move. Load samples, edit markers, slice, stretch, and perform with scenes — all on the device.

## Features

- 4 independent tracks, mixed in a single DSP instance
- Waveform display with zoom and minimap overview
- Start/end marker selection with knobs
- Real-time gain adjustment and normalization
- Cut, copy, paste, and mute operations
- Loop view: seam editor for seamless loops with zero-crossing snap
- BPM-grid slicing and beat-aligned marker snapping
- Record audio directly into the sampler
- Undo support for all destructive edits
- Export selection to new file
- **Three edit/playback modes:** FREE (by time), STRETCH (time-stretch, pitch preserved), VARISPEED (tape-style: pitch+speed together)
- **Three pad modes per scene:** Trigger (toggle), Gate (hold), One Shot (always restarts)
- **Quantized scene launch:** FREE, BEAT, 1BAR, 2BAR, 4BAR, SAMPLE END

## Prerequisites

- [Schwung](https://github.com/charlesvestal/schwung) installed on your Ableton Move
- SSH access enabled: http://move.local/development/ssh

## Install

```bash
git clone https://github.com/SchluppiBobbz/schwung-mangle
cd schwung-mangle
./scripts/build.sh
./scripts/install.sh
```

## Controls

### Track Selection (MoveRow Buttons)

| Button | Action |
|--------|--------|
| MoveRow 1–4 | Select active track (press again to toggle play/stop) |
| Shift+MoveRow | Open file browser for that track |
| Play | Start/stop ALL loaded tracks |

### View Selection (Step Buttons)

| Step | Action |
|------|--------|
| Step 1 | Enter last-used edit mode (FREE or SYNC). LED: PaleGreen=FREE, BrightOrange=STRETCH, VividYellow=VARISPEED |
| Shift+Step 1 | Cycle edit mode: FREE → VARISPEED → STRETCH → FREE |
| Step 2 | Enter Slice mode |
| Step 4 | Cycle pad mode: Trigger → Gate → One Shot (per active track) |
| Step 5 | Toggle Clock Sync on/off |
| Shift+Step 5 | Cycle quantize mode: FREE → BEAT → 1BAR → 2BAR → 4BAR → SAMPLE END |

### FREE View

| Control | Function |
|---------|----------|
| Jog wheel | Scroll view when zoomed in |
| Jog click | Edit menu: Copy, Cut, Truncate, Normalize Sel, BPM Step, Mute |
| Shift+Jog click | Paste/Export menu |
| Knob 1 | Move start marker (Shift: fine) |
| Knob 2 | Move end marker (Shift: fine) |
| Knob 3 | Zoom |
| Knob 4 | Vertical scale |
| Knob 5 | Gain (Shift: normalize) |
| Knob 7 (touch+twist) | Toggle loop mode |
| Knob 8 (touch+twist) | Snap markers to zero crossing |
| Left/Right | Nudge selection |
| Shift+L/R | Jump selection by one length |
| Step 9 / 10 | Fade in / Fade out selection |
| Step 15 / 16 | Set start / end marker at playback position |
| Copy | Copy selection |
| Remove | Cut selection |
| Undo | Undo last edit |

### STRETCH / VARISPEED View (Shift+Step 1)

**STRETCH:** tempo changes via time-stretching — pitch stays constant.
**VARISPEED:** tempo changes affect pitch too (tape-style). No stretcher, lower CPU.

| Control | Function |
|---------|----------|
| Knob 1 | Move start marker (bars:beats, Shift: cycle beat division) |
| Knob 2 | Adjust musical length (bars:beats) |
| Knob 3 | Zoom |
| Knob 8 | Adjust scene BPM (Shift: fine ±0.1) — length stays constant |
| Down | Read BPM from filename |
| Up | Estimate BPM from selection (assumes 1 bar, 4/4) |
| Left/Right | Move entire selection by one division |

### Pad Modes (Step 4 cycles)

| Mode | LED | Behaviour |
|------|-----|-----------|
| Trigger | DarkGrey | Press to start; press playing pad to stop |
| Gate | BrightRed | Plays only while pad is held |
| One Shot | VividYellow | Always restarts from start on press; loop setting respected |

Pad mode is saved **per scene**.

### Quantize Modes (Shift+Step 5 cycles)

| Mode | Description |
|------|-------------|
| FREE | Immediate launch |
| BEAT | Wait for next beat |
| 1BAR | Wait for next bar (default) |
| 2BAR / 4BAR | Wait for next 2- or 4-bar boundary |
| SAMPLE END | Wait for current clip to finish |

### Slice Mode (Step 2)

| Control | Function |
|---------|----------|
| Jog wheel | Cycle slice mode: Even / Auto / Lazy / BPM |
| Knob 1 | Move slice start boundary |
| Knob 2 | Move slice end boundary |
| Knob 3 | Zoom |
| Knob 5 | Count (Even) / Threshold (Auto) / BPM value (BPM) |
| Knob 6 | Beat division (BPM mode) |
| Knob 8 | Select slice |
| Left/Right | Previous / next slice |
| Shift+L/R | Move selected slice left/right |
| Up | Merge slice with previous |
| Down | Split slice at midpoint |
| Copy | Copy selected slice |
| Remove | Cut selected slice |
| Step 9 / 10 | Fade in / Fade out selected slice |
| Step 16 | Shuffle all slices randomly |
| Pads | Audition slice |
| Sample | Save slices / Drum Preset / REX Loop |

## Header Indicators

- `T1`–`T4` — active track
- `*` — unsaved changes
- `L` — loop mode enabled
- `G` — Gate pad mode
- `O` — One Shot pad mode

## Credits

- **Schwung framework**: [Charles Vestal](https://github.com/charlesvestal/schwung)
- Based on [Wave Edit](https://github.com/charlesvestal/move-anything-waveform-editor) by Charles Vestal

## License

MIT License — See [LICENSE](LICENSE)

## AI Assistance

Developed with AI assistance (Claude). All architecture and release decisions reviewed by human maintainers.
