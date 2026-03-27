# CLAUDE.md — Mangle Module

External tool module for Schwung. 4-track sampler with varispeed, slicing, gate/trigger/oneshot, and quantized scene launch.

## Module Structure

```
src/
  module.json       # Module metadata (component_type: "tool", interactive)
  ui.js             # QuickJS UI (waveform display, markers, editing)
  dsp/plugin.cpp      # Audio DSP plugin source (cross-compiled to ARM64)
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

**View modes (Step LEDs):**
- Step 1: enter `editMode` (FREE or SYNC — whichever was last active). LED: BrightOrange = SYNC, PaleGreen = FREE
- Shift+Step 1: toggle `editMode` between FREE and SYNC
- Step 2: SLICE mode
- Step 3: freed (no function)
- Step 4: toggle Gate/Trigger for active track
- Step 5: toggle Clock Sync

**SYNC mode behaviour:**
- Clip has `sceneBpm` (scene BPM) and a musical length in bars:beats
- `musicalLength` is preserved when `sceneBpm` changes: `endSample = startSample + oldLen * oldBpm / newBpm`
- `tempoPercent = sceneBpm / globalBpm * 100` → Bungee stretch ratio
- DSP `sync_tempo` param updates `start_sample`, `end_sample`, `sync_play_end`, `bng_play_end`, and `bng_req.speed` atomically — no stretcher reset during playback so loop boundary takes effect on the next audio block
- E1: start marker (bars:beats, Shift: cycle beat division), E2: length (bars:beats), E8: scene BPM

**Global BPM (Shift+E6):**
- `globalBpm` is owned by Mangle — the overlay's `samplerBpm` does NOT overwrite it automatically
- On module load: initialized from overlay as starting reference (`initGlobalBpmFromOverlay`)
- Shift+E6 changes `globalBpm` locally and sends `host_module_set_param("project_bpm", ...)` to the shim
- The shim writes `/tmp/link-tempo`; the `link-subscriber` process polls it every 500ms and propagates the new tempo to the Ableton Link session → Move hardware metronome follows
- `lockBpmToProject` (default `false`): if `true`, the tick block re-enables overlay → `globalBpm` sync (old behavior)
- `globalBpm` and `lockBpmToProject` are persisted in `project.json`
- Header shows `<bpm>B` (Mangle-owned) or `<bpm>~` (locked to project BPM)
- **Requires:** `link_audio_enabled: true` in `features.json` and `link-subscriber` running (started automatically by shim)

**Button mapping:**
- MoveRow1-4 (CC 43,42,41,40): select track (second press = toggle play)
- Shift+MoveRow: open file browser for that track
- MovePlay (CC 85): start/stop ALL loaded tracks
- MoveBack: no action in main view (exit removed; use Menu → Minimize)
- Shift+MoveBack: stop all playing scenes (`play_all=0` + clear UI state)
- MoveMenu: open project menu (Save Project / Switch Project / Minimize / Close)

**Scene pad model (4 states):**
- Inactive → **Selected** (Shift+Pad, no DSP file load, no launch)
- Selected → **Cue** (Pad press while sync+transport: queued for bar boundary)
- Selected → **Active** (Pad press, immediate)
- Active → **Inactive** (Pad press on playing scene in Trigger mode)
- Pad mode (Gate/OneShot): always immediate launch, no selection step

**LEDs:**
- Track LEDs: Green=playing, WhiteBright=active, WhiteDim=loaded, Black=empty
- Play LED: Green if any track playing
- Step 1: PaleGreen = FREE active, BrightOrange = SYNC active
- Cue LED: VividYellow flashing on pending pad (`dspQueuedPlayScene` or `pendingSceneSwitch`)

**Header:** Shows `T1`..`T4` prefix + gate indicator `G`

### Known Issues / Not Yet Tested
- Clock sync parameter is wired but MIDI clock tracking in `on_midi` is minimal
- Gate mode pad hold/release needs hardware verification
- `lockBpmToProject` toggle has no UI gesture yet (wired, persisted, but always `false` by default)
