# CLAUDE.md — Wave Edit Module

External tool module for Move Everything. 4-track audio editor with jamming, gate/trigger, and clock sync.

## Module Structure

```
src/
  module.json       # Module metadata (component_type: "tool", interactive)
  ui.js             # QuickJS UI (waveform display, markers, editing)
  dsp/plugin.c      # Audio DSP plugin source (cross-compiled to ARM64)
  help.json         # On-device help content
```

## Build & Deploy

```bash
./scripts/build.sh        # Cross-compile DSP + package to dist/waveform-editor/
./scripts/install.sh      # Deploy to Move device
```

Requires `aarch64-linux-gnu-gcc` cross-compiler (set `CROSS_PREFIX` to override).

## Release

1. Update version in `src/module.json`
2. `git commit -am "bump to vX.Y.Z"`
3. `git tag vX.Y.Z && git push --tags`
4. GitHub Actions builds and creates release

## How It Works

This is an interactive tool — the shadow UI loads the DSP and UI together.
- DSP handles WAV loading, playback, gain, normalize, copy/paste, undo
- UI renders waveform display, handles marker editing and input
- Knobs provide direct control of markers, zoom, and gain
- Pads trigger hold-to-audition playback

## Multi-Track Architecture (v0.5.0)

### Constraint
Shadow UI creates exactly **one DSP instance** per tool module. All 4 tracks live inside that single instance.

### DSP (plugin.c)

**Structs:**
- `track_t` — per-track state: audio data, markers, zoom, playback, gain, undo, clipboard, slices, gate mode
- `instance_t` — container with `tracks[NUM_TRACKS]`, `active_track`, `sync_to_clock`, `module_dir`

**Parameter routing** via `parse_track_prefix()`:
- Prefixed params (`t0:file_path`, `t2:gain`) route to specific track
- Unprefixed params route to `active_track` (backwards compatible)
- Global params: `active_track`, `play_all`, `sync_clock`
- Per-track params: `gate_mode`, `gate_held` (plus all existing params)

**Rendering:** `render_block` iterates all 4 tracks, renders each into temp buffer, mixes with int32 saturating addition (clamp +-32767).

**All static functions** take `track_t*` (not `instance_t*`): `load_wav`, `compute_waveform`, `save_undo`, `do_trim`, `do_copy`, `do_paste`, `render_track`, etc.

### UI (ui.js)

**State management — proxy pattern:**
- Global variables (`startSample`, `endSample`, `fileName`, `zoomLevel`, etc.) act as proxy for the active track
- `trackStates[]` array stores per-track state
- `saveTrackUIState(idx)` copies globals into `trackStates[idx]`
- `restoreTrackUIState(idx)` copies from `trackStates[idx]` into globals
- On track switch: save old, restore new — existing draw/edit functions work unchanged

**Button mapping:**
- MoveRow1-4 (CC 43,42,41,40): select track (second press = toggle play)
- Shift+MoveRow: open file browser for that track
- MovePlay (CC 85): start/stop ALL loaded tracks
- Step 4: toggle Gate/Trigger for active track
- Step 5: toggle Clock Sync

**LEDs:**
- Track LEDs: Green=playing, WhiteBright=active, WhiteDim=loaded, Black=empty
- Play LED: Green if any track playing

**Header:** Shows `T1`..`T4` prefix + gate indicator `G`

### Known Issues / Not Yet Tested
- Feature is implemented but not yet tested on hardware
- Clock sync parameter is wired but MIDI clock tracking in `on_midi` is minimal
- Gate mode pad hold/release needs hardware verification
