/*
 * Waveform Editor DSP Plugin — Multi-Track Edition
 *
 * Provides WAV file loading, playback, trimming, gain adjustment,
 * normalization, and waveform visualization for the Move Everything
 * waveform editor tool.
 *
 * Supports 4 independent tracks with mixing, gate/trigger modes,
 * and optional clock sync.
 *
 * V2 API only - instance-based for multi-instance support.
 * Audio stored as interleaved stereo int16 (L,R,L,R,...).
 *
 * Uses Bungee library for high-quality time-stretching and pitch-shifting.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <new>
#include <unistd.h>
#include <pwd.h>

#include <bungee/Bungee.h>
#include "paulxstretch/ProcessedStretch.h"
#include "paulxstretch/BinauralBeats.h"

extern "C" {

/* ============================================================================
 * Plugin API definitions (inlined to avoid path issues during cross-compile)
 * ============================================================================ */

#define MOVE_PLUGIN_API_VERSION_2 2
#define MOVE_SAMPLE_RATE 44100
#define MOVE_FRAMES_PER_BLOCK 128

/* Clock status identifiers */
#define MOVE_CLOCK_STATUS_UNAVAILABLE 0
#define MOVE_CLOCK_STATUS_STOPPED 1
#define MOVE_CLOCK_STATUS_RUNNING 2

typedef struct host_api_v1 {
    uint32_t api_version;
    int sample_rate;
    int frames_per_block;
    uint8_t *mapped_memory;
    int audio_out_offset;
    int audio_in_offset;
    void (*log)(const char *msg);
    int (*midi_send_internal)(const uint8_t *msg, int len);
    int (*midi_send_external)(const uint8_t *msg, int len);
    int (*get_clock_status)(void);
} host_api_v1_t;

typedef struct plugin_api_v2 {
    uint32_t api_version;
    void* (*create_instance)(const char *module_dir, const char *json_defaults);
    void (*destroy_instance)(void *instance);
    void (*on_midi)(void *instance, const uint8_t *msg, int len, int source);
    void (*set_param)(void *instance, const char *key, const char *val);
    int (*get_param)(void *instance, const char *key, char *buf, int buf_len);
    int (*get_error)(void *instance, char *buf, int buf_len);
    void (*render_block)(void *instance, int16_t *out_interleaved_lr, int frames);
} plugin_api_v2_t;

} /* extern "C" */

/* ============================================================================
 * Constants
 * ============================================================================ */

#define MAX_WAVEFORM_COLS 256
#define WAVEFORM_GET_COLS 128   /* must match UI SCREEN_W */
#define NORMALIZE_TARGET_DB (-0.3f)

/* Maximum audio duration per track: 5 minutes at 44100 Hz */
#define MAX_AUDIO_FRAMES (MOVE_SAMPLE_RATE * 300)

/* Samples per frame (stereo interleaved) */
#define SAMPLES_PER_FRAME 2

/* Number of tracks */
#define NUM_TRACKS 4

/* Skipback ring buffer size in frames.
 * 8 bars at 60 BPM = 44100 * 4 * 8 = 1,411,200 frames.
 * At 120 BPM this covers 16 bars — always sufficient for max 8-bar skipback. */
#define RB_FRAMES (44100 * 4 * 8)

/* Bungee output accumulator capacity per track (frames).
 * Must hold several Bungee output grains. 8192 frames ≈ 0.19s at 44.1kHz. */
#define BNG_OUT_BUF_CAPACITY 8192
#define BNG_LOOP_FADE_SAMPLES 128  /* crossfade length at loop boundaries (~3 ms @ 44100 Hz) */

/* ============================================================================
 * Track structure — per-track audio state
 * ============================================================================ */

typedef struct {
    char file_path[512];
    char file_name[128];
    int sample_rate;
    int channels;
    int total_frames;
    uint16_t orig_format;       /* 1=PCM, 3=IEEE float */
    uint16_t orig_bits;         /* 16, 24, or 32 */
    uint16_t orig_channels;     /* 1=mono, 2=stereo */
    float duration_secs;

    int16_t *audio_data;        /* Stereo interleaved PCM buffer (L,R,L,R,...) */
    int audio_frames;           /* Number of frames (each frame = 2 samples) */
    int16_t *undo_buffer;
    int undo_frames;
    int has_undo;
    int dirty;

    int16_t *clipboard_data;    /* Clipboard buffer (stereo interleaved) */
    int clipboard_frames;

    int16_t waveform_min[MAX_WAVEFORM_COLS]; /* Full-file overview, updated on audio data change */
    int16_t waveform_max[MAX_WAVEFORM_COLS];
    int waveform_valid;         /* 1 once full-file waveform has been computed */

    int start_sample;
    int end_sample;
    int zoom_level;             /* 0=fit all, 1-12 zoom levels */
    int zoom_center;

    int playing;
    int play_pos;
    int play_loop;
    int play_whole;

    float gain_db;
    float peak_db;
    float pan;              /* stereo pan: -1.0=left, 0.0=center, +1.0=right */

    int mode;                   /* 0=trim, 1=gain */

    int gate_mode;              /* 0=trigger (toggle), 1=gate (hold-to-play), 2=oneshot */
    int gate_held;              /* 1 while gate is held (pad/key down) */
    int queued_play;            /* 1 = waiting for next bar boundary to start */
    int queued_whole;           /* play mode for queued_play: 0=selection, 1=whole file */
    int muted;                  /* 1=track muted (non-destructive, skips render) */
    int play_mode;              /* 0=stretch (Bungee), 1=varispeed (repitch) */

    float pitch_semitones;      /* -12.0 to +12.0 (fractional = cents) */
    int   tempo_percent;        /* 50 to 200 */
    double pitch_ratio;         /* cached: pow(2, pitch_semitones/12) */
    double tempo_ratio;         /* cached: tempo_percent / 100.0 */
    double tempo_speed_precise; /* full-precision speed from sync_tempo (tp/100), survives loop wraps */
    double play_pos_frac;       /* fractional playback position */

    /* Bungee time-stretch state */
    Bungee::Stretcher<Bungee::Basic> *stretcher;
    Bungee::Request bng_req;
    float *bng_grain_input;     /* non-interleaved buffer [max_grain * 2] */
    int    bng_max_grain;       /* maxInputFrameCount from stretcher */
    float *bng_out_buf;         /* output accumulator (interleaved stereo float) */
    int    bng_out_count;       /* frames currently buffered */
    int    bng_play_start;      /* playback region start for Bungee */
    int    bng_play_end;        /* playback region end for Bungee */
    int    sync_play_end;       /* SYNC mode: tempo-scaled loop boundary (0 = not set, use end_sample) */
    double loop_len_exact;      /* exact musical loop length in input samples (0 = use integer region_len) */

    char copy_result[256];
    char load_error[256];
    char slice_state[2048];     /* JSON blob for UI slice state persistence */
    char scene_state[1024];     /* JSON blob for UI scene state persistence */

    /* PaulXStretch spectral processing state */
    struct {
        int enabled;              /* master on/off for PaulXStretch chain */
        float stretch_amount;     /* 0.1–1024.0 */
        float fft_size_norm;      /* 0.0–1.0 → maps to 256–65536 */
        int fft_size;             /* actual power-of-2 FFT size */

        /* Per-effect enables */
        int stretch_en;
        int freqshift_en;
        int spread_en;
        int ratiomix_en;
        int binaural_en;
        int filter_en;
        int compressor_en;

        /* Parameters */
        float freq_shift_hz;      /* -1000 to +1000 */
        float spread_bw;          /* 0.0–1.0 */
        float ratio_levels[8];    /* 0.0–1.0 each */
        float ratio_values[8];    /* 0.125–8.0 (frequency ratio multiplier) */
        float binaural_power;     /* 0.0–1.0 */
        int   binaural_mode;      /* 0=LR, 1=RL, 2=Symmetric */
        float binaural_freq;      /* 0.05–50.0 Hz */
        float filter_low;         /* 20–20000 Hz */
        float filter_high;        /* 20–20000 Hz */
        float volume_db;          /* -24 to +6 dB */

        /* Runtime (allocated on enable, freed on disable) */
        ProcessedStretch *stretch_l;
        ProcessedStretch *stretch_r;
        BinauralBeats *binaural;
        ProcessParameters proc_pars;

        /* Input ring buffers: accumulate 128-frame blocks until FFT-size ready */
        float *inbuf_l;
        float *inbuf_r;
        int    inbuf_count;
        int    inbuf_capacity;

        /* Output ring buffer: spectral output drains into render */
        float *outbuf;            /* stereo interleaved */
        int    outbuf_count;
        int    outbuf_capacity;
        int    outbuf_read;
        int    outbuf_write;

        int    needs_reinit;      /* flag: rebuild stretchers on next render */
        int    fill_needed;       /* 1 = initial fill required after reinit */

        /* Direct audio_data reading position (bypasses render_track) */
        double read_pos;          /* fractional sample position in audio_data */

        /* Performance monitor — smoothed CPU usage of PSX processing */
        float cpu_percent;        /* 0–100+, EMA-smoothed */
    } psx;
} track_t;

/* ============================================================================
 * Instance structure — container for all tracks
 * ============================================================================ */

typedef struct {
    track_t tracks[NUM_TRACKS];
    int active_track;           /* 0-3, which track is being edited */
    int sync_to_clock;          /* 1 = MIDI clock sync enabled */
    float project_bpm;          /* global project BPM (used for internal bar timing) */
    float master_volume_db;     /* global output gain in dB (-60..+6) */

    /* Clock sync state — SYNC_CLOCK path (mirrors SuperArp pattern) */
    int clock_running;          /* 1 if MIDI transport is running */
    int clock_counter;          /* ticks since last bar start (0..clocks_per_bar-1) */
    uint64_t clock_tick_total;  /* cumulative ticks since last 0xFA Start */
    int clocks_per_bar;         /* MIDI clocks per bar = beats_per_bar * 24 */
    int beats_per_bar;          /* time signature numerator (default 4) */
    int pending_bar_triggers;   /* bar-start triggers queued for render_block */
    int delayed_bar_triggers;   /* bar triggers delayed 1 F8 tick (block-boundary safety) */

    /* Internal sync — BPM-based sample counting when clock not running */
    double samples_per_bar_f;   /* bar duration in samples */
    double samples_until_bar_f; /* countdown to next bar boundary */
    int timing_dirty;           /* 1 = recalc_timing() needed */
    int cached_sample_rate;     /* sample rate used in last recalc_timing() */

    char module_dir[512];

    /* Skipback ring buffers — filled continuously during render_block */
    int16_t *master_rb;              /* stereo circular buffer of master mix */
    int      master_rb_write;        /* write position in frames */
    int16_t *track_rb[NUM_TRACKS];   /* per-track stereo circular buffers */
    int      track_rb_write[NUM_TRACKS];
    char     skipback_result[512];   /* path of last successfully written skipback WAV */
    char     project_dir[512];       /* current project dir, persisted for UI reconnect */
} instance_t;

/* ============================================================================
 * Globals
 * ============================================================================ */

static const host_api_v1_t *g_host = NULL;

static void plugin_log(const char *msg) {
    if (g_host && g_host->log) {
        char buf[512];
        snprintf(buf, sizeof(buf), "[mangle] %s", msg);
        g_host->log(buf);
    }
}

/* Escape a string for JSON output (handles " and \) */
static int json_escape(char *dst, int dst_len, const char *src) {
    int i = 0;
    while (*src && i < dst_len - 1) {
        if (*src == '"' || *src == '\\') {
            if (i + 2 >= dst_len) break;
            dst[i++] = '\\';
        }
        dst[i++] = *src++;
    }
    dst[i] = '\0';
    return i;
}

/* ============================================================================
 * WAV file utilities
 * ============================================================================ */

/* Read a 16-bit little-endian unsigned value */
static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

/* Read a 32-bit little-endian unsigned value */
static uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

/* Write a 16-bit little-endian value */
static void write_u16_le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/* Write a 32-bit little-endian value */
static void write_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/*
 * Load a WAV file into a track.
 * Supports 16-bit PCM, 24-bit PCM, and 32-bit float, mono or stereo.
 * Mono files are expanded to stereo. All formats converted to int16 stereo.
 * Returns 0 on success, -1 on error.
 */
static int load_wav(track_t *t, const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Cannot open file: %s", path);
        plugin_log(t->load_error);
        return -1;
    }

    /* Read entire file into memory for parsing */
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (file_size < 44) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "File too small to be WAV: %ld bytes", file_size);
        plugin_log(t->load_error);
        fclose(f);
        return -1;
    }

    uint8_t *file_data = (uint8_t *)malloc((size_t)file_size);
    if (!file_data) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Out of memory reading file (%ld bytes)", file_size);
        plugin_log(t->load_error);
        fclose(f);
        return -1;
    }

    size_t bytes_read = fread(file_data, 1, (size_t)file_size, f);
    fclose(f);

    if ((long)bytes_read != file_size) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Short read: expected %ld, got %zu", file_size, bytes_read);
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }

    /* Verify RIFF header */
    if (memcmp(file_data, "RIFF", 4) != 0 ||
        memcmp(file_data + 8, "WAVE", 4) != 0) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Not a valid WAV file (missing RIFF/WAVE header)");
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }

    /* Search for fmt and data chunks */
    uint16_t audio_format = 0;
    uint16_t num_channels = 0;
    uint32_t wav_sample_rate = 0;
    uint16_t bits_per_sample = 0;
    const uint8_t *data_ptr = NULL;
    uint32_t data_size = 0;
    int found_fmt = 0;
    int found_data = 0;

    size_t pos = 12; /* Skip RIFF header + "WAVE" */
    while (pos + 8 <= (size_t)file_size) {
        const uint8_t *chunk = file_data + pos;
        uint32_t chunk_size = read_u32_le(chunk + 4);

        if (memcmp(chunk, "fmt ", 4) == 0 && chunk_size >= 16) {
            audio_format    = read_u16_le(chunk + 8);
            num_channels    = read_u16_le(chunk + 10);
            wav_sample_rate = read_u32_le(chunk + 12);
            /* byte_rate at +16, block_align at +20 */
            bits_per_sample = read_u16_le(chunk + 22);
            found_fmt = 1;
        } else if (memcmp(chunk, "data", 4) == 0) {
            data_ptr = chunk + 8;
            data_size = chunk_size;
            found_data = 1;
        }

        /* Advance to next chunk (chunk sizes are word-aligned) */
        pos += 8 + chunk_size;
        if (chunk_size & 1) pos++; /* Pad byte for odd-sized chunks */
    }

    if (!found_fmt || !found_data) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Missing %s%s%s chunk",
                 found_fmt ? "" : "fmt ",
                 (!found_fmt && !found_data) ? "and " : "",
                 found_data ? "" : "data");
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }

    /* Supported formats: PCM 16-bit, PCM 24-bit, IEEE float 32-bit */
    if (audio_format == 1 && bits_per_sample != 16 && bits_per_sample != 24) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Unsupported PCM bit depth %u (16 or 24 supported)",
                 bits_per_sample);
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }
    if (audio_format == 3 && bits_per_sample != 32) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Unsupported float bit depth %u (only 32-bit supported)",
                 bits_per_sample);
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }
    if (audio_format != 1 && audio_format != 3) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Unsupported audio format %u (PCM=1 or float=3 supported)",
                 audio_format);
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }

    if (num_channels < 1 || num_channels > 2) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Unsupported channel count %u (only mono/stereo supported)",
                 num_channels);
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }

    /* Calculate number of frames */
    int bytes_per_frame = num_channels * (bits_per_sample / 8);
    int total_frames = (int)(data_size / (uint32_t)bytes_per_frame);

    if (total_frames <= 0) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "WAV file contains no audio data");
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }

    if (total_frames > MAX_AUDIO_FRAMES) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "File too long: %d frames (max %d)", total_frames, MAX_AUDIO_FRAMES);
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }

    /* Allocate stereo interleaved audio buffer */
    int16_t *audio = (int16_t *)malloc((size_t)total_frames * SAMPLES_PER_FRAME * sizeof(int16_t));
    if (!audio) {
        snprintf(t->load_error, sizeof(t->load_error),
                 "Out of memory allocating %d stereo frames", total_frames);
        plugin_log(t->load_error);
        free(file_data);
        return -1;
    }

    /* Convert to stereo int16 — handles PCM16, PCM24, and float32 */
    if (audio_format == 3 && bits_per_sample == 32) {
        /* IEEE float 32-bit */
        const float *fsrc = (const float *)data_ptr;
        if (num_channels == 1) {
            for (int i = 0; i < total_frames; i++) {
                float s = fsrc[i];
                if (s > 1.0f) s = 1.0f;
                if (s < -1.0f) s = -1.0f;
                int16_t v = (int16_t)(s * 32767.0f);
                audio[i * 2]     = v;
                audio[i * 2 + 1] = v;
            }
        } else {
            for (int i = 0; i < total_frames; i++) {
                float left  = fsrc[i * 2];
                float right = fsrc[i * 2 + 1];
                if (left > 1.0f) left = 1.0f;
                if (left < -1.0f) left = -1.0f;
                if (right > 1.0f) right = 1.0f;
                if (right < -1.0f) right = -1.0f;
                audio[i * 2]     = (int16_t)(left * 32767.0f);
                audio[i * 2 + 1] = (int16_t)(right * 32767.0f);
            }
        }
    } else if (audio_format == 1 && bits_per_sample == 24) {
        /* PCM 24-bit */
        const uint8_t *bsrc = (const uint8_t *)data_ptr;
        int src_bytes_per_frame = 3 * num_channels;
        if (num_channels == 1) {
            for (int i = 0; i < total_frames; i++) {
                const uint8_t *p = bsrc + i * 3;
                int32_t s = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16));
                if (s & 0x800000) s |= (int32_t)0xFF000000;
                int16_t v = (int16_t)(s >> 8);
                audio[i * 2]     = v;
                audio[i * 2 + 1] = v;
            }
        } else {
            for (int i = 0; i < total_frames; i++) {
                const uint8_t *p = bsrc + i * src_bytes_per_frame;
                int32_t left  = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16));
                int32_t right = (int32_t)((uint32_t)p[3] | ((uint32_t)p[4] << 8) | ((uint32_t)p[5] << 16));
                if (left & 0x800000)  left  |= (int32_t)0xFF000000;
                if (right & 0x800000) right |= (int32_t)0xFF000000;
                audio[i * 2]     = (int16_t)(left >> 8);
                audio[i * 2 + 1] = (int16_t)(right >> 8);
            }
        }
    } else {
        /* PCM 16-bit */
        const int16_t *src = (const int16_t *)data_ptr;
        if (num_channels == 1) {
            for (int i = 0; i < total_frames; i++) {
                audio[i * 2]     = src[i];
                audio[i * 2 + 1] = src[i];
            }
        } else {
            memcpy(audio, src, (size_t)total_frames * SAMPLES_PER_FRAME * sizeof(int16_t));
        }
    }

    /* Free old audio data */
    if (t->audio_data) {
        free(t->audio_data);
        t->audio_data = NULL;
    }
    if (t->undo_buffer) {
        free(t->undo_buffer);
        t->undo_buffer = NULL;
    }

    /* Update track state */
    t->audio_data = audio;
    t->audio_frames = total_frames;
    t->undo_buffer = NULL;
    t->undo_frames = 0;
    t->has_undo = 0;
    t->dirty = 0;
    t->waveform_valid = 0;

    t->sample_rate = (int)wav_sample_rate;
    t->channels = 2; /* Always stereo internally */
    t->total_frames = total_frames;
    t->orig_format = audio_format;
    t->orig_bits = bits_per_sample;
    t->orig_channels = num_channels;
    t->duration_secs = (float)total_frames / (float)wav_sample_rate;

    t->start_sample = 0;
    t->end_sample = total_frames;
    t->zoom_level = 0;
    t->zoom_center = total_frames / 2;

    t->playing = 0;
    t->play_pos = 0;
    t->play_pos_frac = 0.0;
    t->gain_db = 0.0f;
    t->mode = 0;

    t->load_error[0] = '\0';
    t->copy_result[0] = '\0';

    free(file_data);

    {
        char log_buf[256];
        snprintf(log_buf, sizeof(log_buf),
                 "Loaded WAV: %d frames, %u Hz, %u ch -> stereo int16",
                 total_frames, wav_sample_rate, num_channels);
        plugin_log(log_buf);
    }

    return 0;
}

/*
 * Write WAV file from interleaved stereo int16 data.
 * out_fmt: 1=PCM, 3=IEEE float.  out_bits: 16, 24, or 32.
 * out_channels: 1=mono (uses L channel), 2=stereo.
 * Returns 0 on success, -1 on error.
 */
/* Chown file to ableton user so Move's UI can see it.
 * The shim runs as root but Move's UI runs as ableton. */
static void chown_to_ableton(const char *path) {
    struct passwd *pw = getpwnam("ableton");
    if (pw) chown(path, pw->pw_uid, pw->pw_gid);
}

static int write_wav(const char *path, const int16_t *data, int num_frames,
                     int sample_rate, uint16_t out_fmt, uint16_t out_bits,
                     uint16_t out_channels) {
    if (!path || !data || num_frames <= 0) return -1;

    FILE *f = fopen(path, "wb");
    if (!f) {
        char log_buf[256];
        snprintf(log_buf, sizeof(log_buf), "Cannot open for writing: %s", path);
        plugin_log(log_buf);
        return -1;
    }

    int bytes_per_sample = out_bits / 8;
    int block_align = out_channels * bytes_per_sample;
    uint32_t data_size = (uint32_t)num_frames * (uint32_t)block_align;
    uint32_t file_size = 36 + data_size;

    uint8_t header[44];

    /* RIFF header */
    memcpy(header, "RIFF", 4);
    write_u32_le(header + 4, file_size);
    memcpy(header + 8, "WAVE", 4);

    /* fmt chunk */
    memcpy(header + 12, "fmt ", 4);
    write_u32_le(header + 16, 16);
    write_u16_le(header + 20, out_fmt);
    write_u16_le(header + 22, out_channels);
    write_u32_le(header + 24, (uint32_t)sample_rate);
    write_u32_le(header + 28, (uint32_t)(sample_rate * block_align));
    write_u16_le(header + 32, (uint16_t)block_align);
    write_u16_le(header + 34, out_bits);

    /* data chunk */
    memcpy(header + 36, "data", 4);
    write_u32_le(header + 40, data_size);

    size_t written = fwrite(header, 1, 44, f);
    if (written != 44) { fclose(f); return -1; }

    /* Write sample data, converting from internal stereo int16 */
    int ok = 1;
    for (int i = 0; i < num_frames && ok; i++) {
        for (int ch = 0; ch < out_channels; ch++) {
            int16_t s = data[i * 2 + ch]; /* ch=0 is L, ch=1 is R */

            if (out_fmt == 3 && out_bits == 32) {
                /* IEEE float 32-bit */
                float fs = (float)s / 32767.0f;
                if (fwrite(&fs, sizeof(float), 1, f) != 1) ok = 0;
            } else if (out_fmt == 1 && out_bits == 24) {
                /* PCM 24-bit */
                int32_t s24 = (int32_t)s << 8;
                uint8_t b[3] = { (uint8_t)(s24 & 0xFF),
                                 (uint8_t)((s24 >> 8) & 0xFF),
                                 (uint8_t)((s24 >> 16) & 0xFF) };
                if (fwrite(b, 3, 1, f) != 1) ok = 0;
            } else {
                /* PCM 16-bit */
                if (fwrite(&s, sizeof(int16_t), 1, f) != 1) ok = 0;
            }
        }
    }
    fclose(f);
    if (!ok) return -1;

    {
        char log_buf[256];
        const char *fmt_name = (out_fmt == 3) ? "float" : "PCM";
        snprintf(log_buf, sizeof(log_buf),
                 "Wrote WAV: %s (%d frames, %s %d-bit %s)",
                 path, num_frames, fmt_name, out_bits,
                 out_channels == 1 ? "mono" : "stereo");
        plugin_log(log_buf);
    }

    chown_to_ableton(path);
    return 0;
}

/* ============================================================================
 * Waveform computation
 * ============================================================================ */

/*
 * Compute visible range based on zoom level and center position.
 * zoom_level 0 = show entire file.
 * zoom_level 1-12 = progressively narrower window.
 */
static void get_visible_range(const track_t *t,
                              int *out_start, int *out_end) {
    if (!t->audio_data || t->audio_frames <= 0) {
        *out_start = 0;
        *out_end = 0;
        return;
    }

    if (t->zoom_level <= 0) {
        /* Show entire file */
        *out_start = 0;
        *out_end = t->audio_frames;
        return;
    }

    /* Each zoom level halves the visible window.
     * Level 1: 1/2 of file, Level 2: 1/4, ..., Level 12: 1/4096 */
    float fraction = 1.0f / (float)(1 << t->zoom_level);
    int visible_frames = (int)((float)t->audio_frames * fraction);
    if (visible_frames < 128) visible_frames = 128; /* Minimum visible */

    int half = visible_frames / 2;
    int center = t->zoom_center;

    /* Clamp center so visible window stays within bounds */
    if (center < half) center = half;
    if (center > t->audio_frames - half) center = t->audio_frames - half;
    if (center < half) center = half; /* Handle very short files */

    *out_start = center - half;
    *out_end = center + half;

    /* Final clamp */
    if (*out_start < 0) *out_start = 0;
    if (*out_end > t->audio_frames) *out_end = t->audio_frames;
}

/* Maximum samples to scan directly from raw audio in the GET handler.
 * Visible ranges up to this size are served by raw scan (exact per-sample min/max).
 * Larger ranges use the precomputed full-file overview to avoid audio-thread stalls.
 * At 44100 Hz, 524288 samples ≈ 12 seconds — scanning takes <0.5 ms on ARM64. */
#define RAW_SCAN_THRESHOLD 524288

/*
 * Raw-scan waveform: compute min/max per column by scanning actual samples.
 * Only safe to call for small vis_range (see RAW_SCAN_THRESHOLD).
 * Gives pixel-accurate display at any zoom level.
 */
static void compute_waveform_raw_range(track_t *t,
                                       int16_t *out_min, int16_t *out_max,
                                       int num_cols, int vis_start, int vis_end) {
    if (!t->audio_data || t->audio_frames <= 0 || num_cols <= 0) {
        memset(out_min, 0, (size_t)num_cols * sizeof(int16_t));
        memset(out_max, 0, (size_t)num_cols * sizeof(int16_t));
        return;
    }
    if (vis_start < 0) vis_start = 0;
    if (vis_end > t->audio_frames) vis_end = t->audio_frames;
    int vis_frames = vis_end - vis_start;
    if (vis_frames <= 0) {
        memset(out_min, 0, (size_t)num_cols * sizeof(int16_t));
        memset(out_max, 0, (size_t)num_cols * sizeof(int16_t));
        return;
    }
    for (int col = 0; col < num_cols; col++) {
        int col_start = vis_start + (int)((long)vis_frames * col       / num_cols);
        int col_end   = vis_start + (int)((long)vis_frames * (col + 1) / num_cols);
        if (col_end <= col_start) col_end = col_start + 1;
        if (col_end > vis_end) col_end = vis_end;
        int16_t mn = 32767, mx = -32768;
        for (int i = col_start; i < col_end; i++) {
            int16_t s = t->audio_data[i * 2]; /* L channel */
            if (s < mn) mn = s;
            if (s > mx) mx = s;
        }
        out_min[col] = mn;
        out_max[col] = mx;
    }
}

/*
 * Compute full-file waveform overview (min/max per column).
 * Always covers [0, audio_frames] at MAX_WAVEFORM_COLS resolution.
 * Called only when audio data changes (file load, normalize, undo, paste, trim).
 * NOT called from the waveform GET handler — use resample_waveform_range() there.
 */
static void compute_waveform(track_t *t, int num_cols) {
    (void)num_cols; /* Always uses MAX_WAVEFORM_COLS for the full-file overview */
    int n = MAX_WAVEFORM_COLS;
    t->waveform_valid = 0;

    if (!t->audio_data || t->audio_frames <= 0) {
        memset(t->waveform_min, 0, (size_t)n * sizeof(int16_t));
        memset(t->waveform_max, 0, (size_t)n * sizeof(int16_t));
        return;
    }

    int total = t->audio_frames;
    for (int col = 0; col < n; col++) {
        int col_start = (col * total) / n;
        int col_end   = ((col + 1) * total) / n;
        if (col_end <= col_start) col_end = col_start + 1;
        if (col_end > total) col_end = total;

        int16_t mn = 32767;
        int16_t mx = -32768;
        for (int i = col_start; i < col_end; i++) {
            int16_t s = t->audio_data[i * 2]; /* L channel */
            if (s < mn) mn = s;
            if (s > mx) mx = s;
        }
        t->waveform_min[col] = mn;
        t->waveform_max[col] = mx;
    }
    t->waveform_valid = 1;
}

/*
 * Resample the precomputed full-file waveform for a sub-range [vis_start, vis_end].
 * O(MAX_WAVEFORM_COLS) — no raw sample scanning, safe to call from the audio thread.
 * out_min/out_max must each have room for num_cols entries.
 */
static void resample_waveform_range(track_t *t,
                                    int16_t *out_min, int16_t *out_max,
                                    int num_cols, int vis_start, int vis_end) {
    int total = t->audio_frames;
    if (!t->waveform_valid || total <= 0 || num_cols <= 0) {
        memset(out_min, 0, (size_t)num_cols * sizeof(int16_t));
        memset(out_max, 0, (size_t)num_cols * sizeof(int16_t));
        return;
    }
    if (vis_start < 0) vis_start = 0;
    if (vis_end > total) vis_end = total;
    if (vis_end <= vis_start) {
        memset(out_min, 0, (size_t)num_cols * sizeof(int16_t));
        memset(out_max, 0, (size_t)num_cols * sizeof(int16_t));
        return;
    }

    int range = vis_end - vis_start;
    for (int col = 0; col < num_cols; col++) {
        /* Sample range covered by this output column */
        int samp_start = vis_start + (int)((long)range * col       / num_cols);
        int samp_end   = vis_start + (int)((long)range * (col + 1) / num_cols);
        if (samp_end <= samp_start) samp_end = samp_start + 1;

        /* Map sample range to full-file precomputed column indices */
        int fc_start = (int)((long)samp_start * MAX_WAVEFORM_COLS / total);
        int fc_end   = (int)((long)(samp_end - 1) * MAX_WAVEFORM_COLS / total);
        if (fc_start < 0) fc_start = 0;
        if (fc_end >= MAX_WAVEFORM_COLS) fc_end = MAX_WAVEFORM_COLS - 1;
        if (fc_start > fc_end) fc_start = fc_end;

        int16_t mn = 32767, mx = -32768;
        for (int fc = fc_start; fc <= fc_end; fc++) {
            if (t->waveform_min[fc] < mn) mn = t->waveform_min[fc];
            if (t->waveform_max[fc] > mx) mx = t->waveform_max[fc];
        }
        out_min[col] = mn;
        out_max[col] = mx;
    }
}

/* ============================================================================
 * Peak level computation
 * ============================================================================ */

/*
 * Scan stereo audio buffer and compute peak level in dB.
 * Scans both L and R channels.
 * Returns -inf for silence.
 */
static float compute_peak_db(const int16_t *data, int num_frames) {
    if (!data || num_frames <= 0) return -96.0f;

    int32_t peak = 0;
    int total_samples = num_frames * SAMPLES_PER_FRAME;
    for (int i = 0; i < total_samples; i++) {
        int32_t abs_val = (int32_t)data[i];
        if (abs_val < 0) abs_val = -abs_val;
        if (abs_val > peak) peak = abs_val;
    }

    if (peak == 0) return -96.0f;

    return 20.0f * log10f((float)peak / 32768.0f);
}

/* ============================================================================
 * Undo management
 * ============================================================================ */

/*
 * Save current audio state to undo buffer.
 */
static void save_undo(track_t *t) {
    if (!t->audio_data || t->audio_frames <= 0) return;

    /* Free old undo buffer */
    if (t->undo_buffer) {
        free(t->undo_buffer);
        t->undo_buffer = NULL;
    }

    size_t buf_size = (size_t)t->audio_frames * SAMPLES_PER_FRAME * sizeof(int16_t);
    t->undo_buffer = (int16_t *)malloc(buf_size);
    if (!t->undo_buffer) {
        t->undo_frames = 0;
        t->has_undo = 0;
        plugin_log("Failed to allocate undo buffer");
        return;
    }

    memcpy(t->undo_buffer, t->audio_data, buf_size);
    t->undo_frames = t->audio_frames;
    t->has_undo = 1;
}

/* ============================================================================
 * Filename utilities
 * ============================================================================ */

/*
 * Extract filename from a full path.
 */
static const char* basename_ptr(const char *path) {
    const char *last_slash = strrchr(path, '/');
    return last_slash ? last_slash + 1 : path;
}

/*
 * Generate an edited copy filename from the original with timestamp.
 * "sample.wav" -> "sample_edit_0306_1423.wav"
 * Writes into out_buf with max out_len.
 */
static void make_edit_filename(const char *original_path, char *out_buf,
                               int out_len) {
    /* Get timestamp */
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    char ts[16];
    snprintf(ts, sizeof(ts), "%02d%02d_%02d%02d",
             tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min);

    /* Find the directory */
    const char *filename = basename_ptr(original_path);
    int dir_len = (int)(filename - original_path);

    /* Find the extension */
    const char *dot = strrchr(filename, '.');
    if (!dot) {
        snprintf(out_buf, (size_t)out_len, "%s_edit_%s.wav", original_path, ts);
        return;
    }

    int name_part_len = (int)(dot - filename);

    /* Build: dir + name_part + "_edit_MMDD_HHMM" + extension */
    snprintf(out_buf, (size_t)out_len, "%.*s%.*s_edit_%s%s",
             dir_len, original_path,
             name_part_len, filename,
             ts, dot);
}

/* ============================================================================
 * Track helper — free all track memory
 * ============================================================================ */

/* ============================================================================
 * PaulXStretch helpers — init, free, reinit, process, param handling
 * ============================================================================ */

#define PSX_DEFAULT_FFT_SIZE  8192
#define PSX_DEFAULT_STRETCH   8.0f
#define PSX_MAX_FFT_SIZE      32768   /* cap at 32k for ARM64 CPU budget */
#define PSX_OUTBUF_MULT       8       /* output ring = fft_size * this */

static int psx_fft_size_from_norm(float v) {
    /* 0.0 -> 128 (2^7), 1.0 -> 32768 (2^15) — 9 steps */
    int exp_val = 7 + (int)(v * 8.0f + 0.5f);
    if (exp_val < 7) exp_val = 7;
    if (exp_val > 15) exp_val = 15;
    return 1 << exp_val;
}

static void ps_free_buffers(track_t *t) {
    if (t->psx.stretch_l) { delete t->psx.stretch_l; t->psx.stretch_l = NULL; }
    if (t->psx.stretch_r) { delete t->psx.stretch_r; t->psx.stretch_r = NULL; }
    if (t->psx.binaural)  { delete t->psx.binaural;  t->psx.binaural = NULL; }
    if (t->psx.inbuf_l)   { free(t->psx.inbuf_l);    t->psx.inbuf_l = NULL; }
    if (t->psx.inbuf_r)   { free(t->psx.inbuf_r);    t->psx.inbuf_r = NULL; }
    if (t->psx.outbuf)    { free(t->psx.outbuf);      t->psx.outbuf = NULL; }
    t->psx.inbuf_count = 0;
    t->psx.outbuf_count = 0;
    t->psx.outbuf_read = 0;
    t->psx.outbuf_write = 0;
}

static void ps_alloc_stretchers(track_t *t) {
    int fft_sz = t->psx.fft_size;
    if (fft_sz < 128) fft_sz = 128;

    /* Create L and R stretchers (mono each, stereo_mode 1=left, 2=right) */
    t->psx.stretch_l = new ProcessedStretch(
        t->psx.stretch_amount, fft_sz, W_HAMMING, false, (float)MOVE_SAMPLE_RATE, 1);
    t->psx.stretch_r = new ProcessedStretch(
        t->psx.stretch_amount, fft_sz, W_HAMMING, false, (float)MOVE_SAMPLE_RATE, 2);

    t->psx.stretch_l->setBufferSize(fft_sz);
    t->psx.stretch_r->setBufferSize(fft_sz);

    t->psx.stretch_l->set_rap(t->psx.stretch_amount);
    t->psx.stretch_r->set_rap(t->psx.stretch_amount);

    /* Set up spectrum processing chain */
    std::vector<SpectrumProcess> chain = {
        {SPT_FreqShift,    t->psx.freqshift_en != 0},
        {SPT_Spread,       t->psx.spread_en != 0},
        {SPT_RatioMix,     t->psx.ratiomix_en != 0},
        {SPT_Filter,       false},
        {SPT_Compressor,   false},
    };
    t->psx.stretch_l->m_spectrum_processes = chain;
    t->psx.stretch_r->m_spectrum_processes = chain;

    t->psx.stretch_l->set_parameters(&t->psx.proc_pars);
    t->psx.stretch_r->set_parameters(&t->psx.proc_pars);

    /* Binaural beats */
    t->psx.binaural = new BinauralBeats(MOVE_SAMPLE_RATE);

    /* Input ring buffers (mono each) */
    int max_buf = t->psx.stretch_l->get_max_bufsize();
    t->psx.inbuf_capacity = max_buf;
    t->psx.inbuf_l = (float *)calloc(max_buf, sizeof(float));
    t->psx.inbuf_r = (float *)calloc(max_buf, sizeof(float));
    t->psx.inbuf_count = 0;

    /* Output ring buffer (stereo interleaved) */
    int out_cap = fft_sz * PSX_OUTBUF_MULT;
    if (out_cap < 4096) out_cap = 4096;
    t->psx.outbuf_capacity = out_cap;
    t->psx.outbuf = (float *)calloc(out_cap * 2, sizeof(float));
    t->psx.outbuf_count = 0;
    t->psx.outbuf_read = 0;
    t->psx.outbuf_write = 0;

    t->psx.needs_reinit = 0;
    t->psx.fill_needed = 1;
}

static void ps_init_track(track_t *t) {
    memset(&t->psx, 0, sizeof(t->psx));
    /* Reconstruct proc_pars to restore constructor defaults (ratios, filter bounds, etc.)
     * that were zeroed by memset above */
    new (&t->psx.proc_pars) ProcessParameters();
    t->psx.stretch_amount = PSX_DEFAULT_STRETCH;
    t->psx.fft_size_norm = 0.75f;  /* maps to 8192 with new scale: 2^(7 + round(0.75*8)) = 2^13 */
    t->psx.fft_size = PSX_DEFAULT_FFT_SIZE;
    t->psx.binaural_power = 0.5f;
    t->psx.binaural_freq = 7.0f;
    t->psx.filter_low = 20.0f;
    t->psx.filter_high = 20000.0f;
    t->psx.volume_db = 0.0f;
    /* Default ratio levels: only ratio 3 (1.0x = unison) active */
    t->psx.ratio_levels[2] = 1.0f;
    t->psx.proc_pars.ratiomix.ratiolevels[2] = 1.0;
    /* Default ratio values mirror ProcessParameters constructor defaults */
    static const float DEFAULT_RATIO_VALUES[8] = {
        0.25f, 0.5f, 1.0f, 2.0f, 3.0f, 4.0f, 1.5f, 1.0f/1.5f
    };
    for (int i = 0; i < 8; i++) {
        t->psx.ratio_values[i] = DEFAULT_RATIO_VALUES[i];
        /* proc_pars.ratiomix.ratios already set by ProcessParameters() constructor */
    }
}

static void ps_reinit_track(track_t *t) {
    ps_free_buffers(t);
    if (t->psx.enabled) {
        ps_alloc_stretchers(t);
    }
}

static void ps_update_chain_enables(track_t *t) {
    if (!t->psx.stretch_l || !t->psx.stretch_r) return;
    auto update = [&](std::vector<SpectrumProcess>& chain) {
        for (auto& e : chain) {
            switch (e.m_index) {
                case SPT_FreqShift:  e.m_enabled = t->psx.freqshift_en != 0; break;
                case SPT_Spread:     e.m_enabled = t->psx.spread_en != 0; break;
                case SPT_RatioMix:   e.m_enabled = t->psx.ratiomix_en != 0; break;
                case SPT_Filter:     e.m_enabled = t->psx.filter_en != 0; break;
                case SPT_Compressor: e.m_enabled = t->psx.compressor_en != 0; break;
                default: break;
            }
        }
    };
    update(t->psx.stretch_l->m_spectrum_processes);
    update(t->psx.stretch_r->m_spectrum_processes);
}

static void ps_update_params(track_t *t) {
    if (!t->psx.stretch_l || !t->psx.stretch_r) return;
    t->psx.stretch_l->set_parameters(&t->psx.proc_pars);
    t->psx.stretch_r->set_parameters(&t->psx.proc_pars);
    t->psx.stretch_l->set_rap(t->psx.stretch_amount);
    t->psx.stretch_r->set_rap(t->psx.stretch_amount);
}

static void ps_update_binaural(track_t *t) {
    if (!t->psx.binaural) return;
    t->psx.binaural->pars.mono = t->psx.binaural_power;
    t->psx.binaural->pars.stereo_mode = (BB_STEREO_MODE)t->psx.binaural_mode;
    /* Configure binaural frequency via FreeEdit: set a flat curve at the target freq */
    t->psx.binaural->pars.free_edit.set_enabled(t->psx.binaural_en != 0);
    if (t->psx.binaural_en) {
        t->psx.binaural->pars.free_edit.extreme_y.init(0.05f, 50.0f, FE_LOG, true, true);
        t->psx.binaural->pars.free_edit.set_all_values(t->psx.binaural_freq);
        t->psx.binaural->pars.free_edit.update_curve(256);
    }
}

static void ps_handle_param(track_t *t, const char *param, const char *val) {
    if (!val) return;

    if (strcmp(param, "psx_enabled") == 0) {
        int new_en = atoi(val) ? 1 : 0;
        if (new_en != t->psx.enabled) {
            t->psx.enabled = new_en;
            t->psx.needs_reinit = 1;
            /* Sync PSX read position from current playback position */
            if (new_en) {
                t->psx.read_pos = t->play_pos_frac;
            }
        }
        return;
    }
    if (strcmp(param, "psx_stretch") == 0) {
        float v = (float)atof(val);
        if (v < 0.1f) v = 0.1f;
        if (v > 1024.0f) v = 1024.0f;
        t->psx.stretch_amount = v;
        if (t->psx.stretch_l) {
            t->psx.stretch_l->set_rap(v);
            t->psx.stretch_r->set_rap(v);
        }
        return;
    }
    if (strcmp(param, "psx_fft_size") == 0) {
        float v = (float)atof(val);
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        t->psx.fft_size_norm = v;
        int new_sz = psx_fft_size_from_norm(v);
        if (new_sz != t->psx.fft_size) {
            t->psx.fft_size = new_sz;
            if (t->psx.enabled) t->psx.needs_reinit = 1;
        }
        return;
    }
    if (strcmp(param, "psx_freqshift_en") == 0) {
        t->psx.freqshift_en = atoi(val) ? 1 : 0;
        ps_update_chain_enables(t);
        return;
    }
    if (strcmp(param, "psx_freqshift_hz") == 0) {
        float v = (float)atof(val);
        if (v < -1000.0f) v = -1000.0f;
        if (v > 1000.0f) v = 1000.0f;
        t->psx.freq_shift_hz = v;
        t->psx.proc_pars.freq_shift.Hz = (int)v;
        ps_update_params(t);
        return;
    }
    if (strcmp(param, "psx_spread_en") == 0) {
        t->psx.spread_en = atoi(val) ? 1 : 0;
        ps_update_chain_enables(t);
        return;
    }
    if (strcmp(param, "psx_spread_bw") == 0) {
        float v = (float)atof(val);
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        t->psx.spread_bw = v;
        t->psx.proc_pars.spread.bandwidth = v;
        ps_update_params(t);
        return;
    }
    if (strcmp(param, "psx_ratiomix_en") == 0) {
        t->psx.ratiomix_en = atoi(val) ? 1 : 0;
        ps_update_chain_enables(t);
        return;
    }
    /* psx_ratio_val_1 .. psx_ratio_val_8 — frequency ratio multipliers (must come before psx_ratio_) */
    if (strncmp(param, "psx_ratio_val_", 14) == 0) {
        int idx = atoi(param + 14) - 1;
        if (idx < 0 || idx >= 8) return;
        float v = (float)atof(val);
        if (v < 0.125f) v = 0.125f;
        if (v > 8.0f) v = 8.0f;
        t->psx.ratio_values[idx] = v;
        t->psx.proc_pars.ratiomix.ratios[idx] = (double)v;
        ps_update_params(t);
        return;
    }
    /* psx_ratio_1 .. psx_ratio_8 — mix levels */
    if (strncmp(param, "psx_ratio_", 10) == 0) {
        int idx = atoi(param + 10) - 1;
        if (idx < 0 || idx >= 8) return;
        float v = (float)atof(val);
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        t->psx.ratio_levels[idx] = v;
        t->psx.proc_pars.ratiomix.ratiolevels[idx] = (double)v;
        ps_update_params(t);
        return;
    }
    if (strcmp(param, "psx_binaural_en") == 0) {
        t->psx.binaural_en = atoi(val) ? 1 : 0;
        ps_update_binaural(t);
        return;
    }
    if (strcmp(param, "psx_binaural_pow") == 0) {
        float v = (float)atof(val);
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        t->psx.binaural_power = v;
        ps_update_binaural(t);
        return;
    }
    if (strcmp(param, "psx_binaural_mode") == 0) {
        int v = atoi(val);
        if (v < 0) v = 0;
        if (v > 2) v = 2;
        t->psx.binaural_mode = v;
        ps_update_binaural(t);
        return;
    }
    if (strcmp(param, "psx_binaural_freq") == 0) {
        float v = (float)atof(val);
        if (v < 0.05f) v = 0.05f;
        if (v > 50.0f) v = 50.0f;
        t->psx.binaural_freq = v;
        ps_update_binaural(t);
        return;
    }
    if (strcmp(param, "psx_stretch_en") == 0) {
        t->psx.stretch_en = atoi(val) ? 1 : 0;
        return;
    }
    if (strcmp(param, "psx_filter_en") == 0) {
        t->psx.filter_en = atoi(val) ? 1 : 0;
        ps_update_chain_enables(t);
        return;
    }
    if (strcmp(param, "psx_filter_low") == 0) {
        float v = (float)atof(val);
        if (v < 20.0f) v = 20.0f;
        if (v > 20000.0f) v = 20000.0f;
        t->psx.filter_low = v;
        t->psx.proc_pars.filter.low = v;
        ps_update_params(t);
        return;
    }
    if (strcmp(param, "psx_filter_high") == 0) {
        float v = (float)atof(val);
        if (v < 20.0f) v = 20.0f;
        if (v > 20000.0f) v = 20000.0f;
        t->psx.filter_high = v;
        t->psx.proc_pars.filter.high = v;
        ps_update_params(t);
        return;
    }
    if (strcmp(param, "psx_compressor_en") == 0) {
        t->psx.compressor_en = atoi(val) ? 1 : 0;
        ps_update_chain_enables(t);
        return;
    }
    if (strcmp(param, "psx_compressor_pow") == 0) {
        float v = (float)atof(val);
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        t->psx.proc_pars.compressor.power = (double)v;
        ps_update_params(t);
        return;
    }
    if (strcmp(param, "psx_volume") == 0) {
        float v = (float)atof(val);
        if (v < -24.0f) v = -24.0f;
        if (v > 6.0f) v = 6.0f;
        t->psx.volume_db = v;
        return;
    }
}

/* Read N samples from audio_data at psx.read_pos into float L/R buffers.
 * Advances read_pos, handles loop wrapping and sample-rate conversion.
 * This is the core change: PSX reads directly from the source audio at its
 * own pace, controlled by the stretcher's remained_samples mechanism. */
static void ps_read_audio(track_t *t, float *buf_l, float *buf_r, int count,
                           int play_start, int play_end) {
    int region_len = play_end - play_start;
    double rate_inc = (double)t->sample_rate / (double)MOVE_SAMPLE_RATE;

    float linear_gain = 1.0f;
    if (t->gain_db != 0.0f) {
        linear_gain = powf(10.0f, t->gain_db / 20.0f);
    }

    for (int i = 0; i < count; i++) {
        double rp = t->psx.read_pos;
        int pos0 = (int)rp;
        float frac = (float)(rp - pos0);

        /* Wrap pos0 within region */
        if (region_len > 0) {
            while (pos0 >= play_end) pos0 -= region_len;
            while (pos0 < play_start) pos0 += region_len;
        }
        int pos1 = pos0 + 1;
        if (region_len > 0 && pos1 >= play_end) pos1 -= region_len;

        if (pos0 >= 0 && pos0 < t->audio_frames) {
            float l0 = (float)t->audio_data[pos0 * 2 + 0] / 32768.0f;
            float r0 = (float)t->audio_data[pos0 * 2 + 1] / 32768.0f;
            float l1 = (pos1 >= 0 && pos1 < t->audio_frames) ? (float)t->audio_data[pos1 * 2 + 0] / 32768.0f : l0;
            float r1 = (pos1 >= 0 && pos1 < t->audio_frames) ? (float)t->audio_data[pos1 * 2 + 1] / 32768.0f : r0;
            buf_l[i] = (l0 + frac * (l1 - l0)) * linear_gain;
            buf_r[i] = (r0 + frac * (r1 - r0)) * linear_gain;
        } else {
            buf_l[i] = 0.0f;
            buf_r[i] = 0.0f;
        }

        t->psx.read_pos += rate_inc;

        /* Loop wrap */
        if (t->psx.read_pos >= (double)play_end && region_len > 0) {
            if (t->play_loop) {
                t->psx.read_pos -= (double)region_len;
            }
            /* Non-looping: read_pos stays past end, will produce zeros */
        }
    }
}

/* PaulXStretch spectral processing — reads directly from audio_data.
 * Bypasses render_track: PSX controls its own read position and playback rate.
 * The stretcher's remained_samples mechanism controls how often new input
 * is consumed, achieving spectral time-stretch without input overflow. */
static void ps_process_track(track_t *t, int16_t *out, int frames) {
    /* Note: needs_reinit is handled by render_track before calling us */
    if (!t->psx.stretch_l || !t->psx.stretch_r) return;

    /* Determine loop boundaries */
    int play_start, play_end;
    if (t->play_whole) {
        play_start = 0;
        play_end = t->audio_frames;
    } else {
        play_start = t->start_sample;
        play_end = t->end_sample;
        if (play_start < 0) play_start = 0;
        if (play_end > t->audio_frames) play_end = t->audio_frames;
    }
    int region_len = play_end - play_start;
    if (region_len <= 0) {
        memset(out, 0, (size_t)frames * 2 * sizeof(int16_t));
        return;
    }

    /* Initial fill: feed fill_sz samples from audio_data on first use */
    if (t->psx.fill_needed) {
        int fill_sz = t->psx.stretch_l->get_nsamples_for_fill();
        if (fill_sz > t->psx.inbuf_capacity) fill_sz = t->psx.inbuf_capacity;
        ps_read_audio(t, t->psx.inbuf_l, t->psx.inbuf_r, fill_sz, play_start, play_end);
        t->psx.stretch_l->process(t->psx.inbuf_l, fill_sz);
        t->psx.stretch_r->process(t->psx.inbuf_r, fill_sz);
        t->psx.fill_needed = 0;
    }

    /* Feed stretcher and produce output.
     * When rap > 1.0, get_nsamples() returns 0 between calls — process(NULL,0)
     * advances remained_samples and resynthesizes from existing FFT data.
     * When rap < 1.0, get_nsamples() returns bufsize every call, plus
     * get_skip_nsamples() > 0 to advance read_pos faster.
     *
     * Rate-limit: produce at most ~2 FFT frames worth of output per render
     * block to spread CPU load evenly instead of burst-filling the ring
     * buffer (which causes audible pumping at large FFT sizes). */
    int bufsize = t->psx.stretch_l->get_bufsize();
    int target = frames + bufsize;  /* enough for this block + 1 frame headroom */
    int max_iters = 64;
    while (max_iters-- > 0) {
        int out_sz = bufsize;
        if (t->psx.outbuf_count + out_sz > t->psx.outbuf_capacity) break;
        if (t->psx.outbuf_count >= target) break;  /* enough buffered */

        int need = t->psx.stretch_l->get_nsamples(0.5f);

        if (need > 0) {
            /* Read from audio_data at read_pos (both L+R channels) */
            if (need > t->psx.inbuf_capacity) need = t->psx.inbuf_capacity;
            ps_read_audio(t, t->psx.inbuf_l, t->psx.inbuf_r, need, play_start, play_end);

            REALTYPE onset_l = t->psx.stretch_l->process(t->psx.inbuf_l, need);
            t->psx.stretch_l->here_is_onset(onset_l);
            REALTYPE onset_r = t->psx.stretch_r->process(t->psx.inbuf_r, need);
            t->psx.stretch_r->here_is_onset(onset_r);

            /* Handle skip samples (for stretch ratios < 1.0) */
            int skip = t->psx.stretch_l->get_skip_nsamples();
            if (skip > 0) {
                double rate_inc = (double)t->sample_rate / (double)MOVE_SAMPLE_RATE;
                t->psx.read_pos += (double)skip * rate_inc;
                /* Wrap after skip */
                if (t->play_loop && region_len > 0) {
                    while (t->psx.read_pos >= (double)play_end)
                        t->psx.read_pos -= (double)region_len;
                }
            }
            /* Drain R stretcher's skip counter to keep in sync */
            t->psx.stretch_r->get_skip_nsamples();
        } else {
            /* No new input needed this iteration — re-synthesize from existing FFT
             * spectral data. Must pass the inbuf pointer (not NULL) so process()
             * actually updates out_buf; with NULL it leaves out_buf unchanged,
             * causing the same frame to loop until new input arrives. */
            t->psx.stretch_l->process(t->psx.inbuf_l, 0);
            t->psx.stretch_r->process(t->psx.inbuf_r, 0);
        }

        /* Copy L+R output to ring buffer */
        for (int i = 0; i < out_sz && t->psx.outbuf_count < t->psx.outbuf_capacity; i++) {
            t->psx.outbuf[t->psx.outbuf_write * 2 + 0] = t->psx.stretch_l->out_buf[i];
            t->psx.outbuf[t->psx.outbuf_write * 2 + 1] = t->psx.stretch_r->out_buf[i];
            t->psx.outbuf_write = (t->psx.outbuf_write + 1) % t->psx.outbuf_capacity;
            t->psx.outbuf_count++;
        }
    }

    /* Update play_pos from PSX read position for UI cursor */
    t->play_pos = (int)t->psx.read_pos;
    if (t->play_pos < 0) t->play_pos = 0;
    if (t->play_pos >= t->audio_frames) t->play_pos = t->audio_frames - 1;
    t->play_pos_frac = t->psx.read_pos;

    /* Drain output ring buffer into int16 output */
    if (t->psx.outbuf_count >= frames) {
        float tmp_l[MOVE_FRAMES_PER_BLOCK];
        float tmp_r[MOVE_FRAMES_PER_BLOCK];

        for (int i = 0; i < frames; i++) {
            tmp_l[i] = t->psx.outbuf[t->psx.outbuf_read * 2 + 0];
            tmp_r[i] = t->psx.outbuf[t->psx.outbuf_read * 2 + 1];
            t->psx.outbuf_read = (t->psx.outbuf_read + 1) % t->psx.outbuf_capacity;
        }
        t->psx.outbuf_count -= frames;

        /* Apply binaural beats on the drained block */
        if (t->psx.binaural_en && t->psx.binaural) {
            t->psx.binaural->process(tmp_l, tmp_r, frames, 0.5f);
        }

        /* Apply PSX volume gain */
        if (t->psx.volume_db != 0.0f) {
            float vol_gain = powf(10.0f, t->psx.volume_db / 20.0f);
            for (int i = 0; i < frames; i++) {
                tmp_l[i] *= vol_gain;
                tmp_r[i] *= vol_gain;
            }
        }

        /* Convert float back to int16 */
        for (int i = 0; i < frames; i++) {
            float l = tmp_l[i] * 32767.0f;
            float r = tmp_r[i] * 32767.0f;
            if (l > 32767.0f) l = 32767.0f;
            if (l < -32767.0f) l = -32767.0f;
            if (r > 32767.0f) r = 32767.0f;
            if (r < -32767.0f) r = -32767.0f;
            out[i * 2 + 0] = (int16_t)l;
            out[i * 2 + 1] = (int16_t)r;
        }
    } else {
        /* Not enough output buffered yet — output silence while filling */
        memset(out, 0, (size_t)frames * 2 * sizeof(int16_t));
    }
}

/* ============================================================================
 * Track helper — free all memory for a track
 * ============================================================================ */

static void free_track(track_t *t) {
    if (t->audio_data) {
        free(t->audio_data);
        t->audio_data = NULL;
    }
    if (t->undo_buffer) {
        free(t->undo_buffer);
        t->undo_buffer = NULL;
    }
    if (t->clipboard_data) {
        free(t->clipboard_data);
        t->clipboard_data = NULL;
    }
    if (t->stretcher) {
        delete t->stretcher;
        t->stretcher = NULL;
    }
    if (t->bng_grain_input) {
        free(t->bng_grain_input);
        t->bng_grain_input = NULL;
    }
    if (t->bng_out_buf) {
        free(t->bng_out_buf);
        t->bng_out_buf = NULL;
    }
    ps_free_buffers(t);
}

/* ============================================================================
 * Track helper — init defaults for a track
 * ============================================================================ */

static void init_track(track_t *t) {
    memset(t, 0, sizeof(track_t));
    t->gain_db = 0.0f;
    t->peak_db = -96.0f;
    t->mode = 0;
    t->pitch_semitones = 0.0f;
    t->tempo_percent = 100;
    t->pitch_ratio = 1.0f;
    t->tempo_ratio = 1.0f;
    t->play_pos_frac = 0.0;
    t->zoom_level = 0;
    t->orig_format = 1;    /* PCM */
    t->orig_bits = 16;
    t->orig_channels = 2;  /* stereo */
    t->sample_rate = MOVE_SAMPLE_RATE;

    /* Create Bungee stretcher for this track */
    t->stretcher = new Bungee::Stretcher<Bungee::Basic>(
        Bungee::SampleRates{MOVE_SAMPLE_RATE, MOVE_SAMPLE_RATE}, 2, 0);
    t->bng_max_grain = t->stretcher->maxInputFrameCount();
    t->bng_grain_input = (float *)calloc(t->bng_max_grain * 2, sizeof(float));
    t->bng_out_buf = (float *)calloc(BNG_OUT_BUF_CAPACITY * 2, sizeof(float));
    t->bng_out_count = 0;

    /* Initialize Bungee request */
    t->bng_req.position = 0.0;
    t->bng_req.speed = 1.0;
    t->bng_req.pitch = 1.0;
    t->bng_req.reset = true;
    t->bng_req.resampleMode = resampleMode_autoOut;

    /* Initialize PaulXStretch defaults */
    ps_init_track(t);
}

static void recompute_ratios(track_t *t);
static void bng_reset_stretcher(track_t *t, double position);
static void do_apply_pitch_tempo(track_t *t);

/* ============================================================================
 * Clock sync helpers — modelled on SuperArp SYNC_CLOCK pattern
 * ============================================================================ */

/* Compute clocks_per_bar from beats_per_bar and 24 PPQN MIDI clock. */
static void recalc_clock_timing(instance_t *inst) {
    if (!inst) return;
    if (inst->beats_per_bar < 1) inst->beats_per_bar = 4;
    inst->clocks_per_bar = inst->beats_per_bar * 24; /* 24 PPQN per beat */
    if (inst->clocks_per_bar < 1) inst->clocks_per_bar = 96;
}

/* Snap clock_counter to current tick position after a beats_per_bar change.
 * Drops any queued triggers from the previous grid — same pattern as SuperArp
 * realign_clock_phase(). */
static void realign_clock_phase(instance_t *inst) {
    if (!inst) return;
    if (inst->clocks_per_bar < 1) inst->clocks_per_bar = 96;
    inst->clock_counter = (int)(inst->clock_tick_total % (uint64_t)inst->clocks_per_bar);
    inst->pending_bar_triggers = 0;
    inst->delayed_bar_triggers = 0;
}

/* Compute bar duration in samples from project_bpm for internal (non-clock) sync.
 * Called whenever project_bpm, beats_per_bar, or sample_rate changes. */
static void recalc_timing(instance_t *inst, int sample_rate) {
    double bpm;
    int bpb;
    if (!inst || sample_rate <= 0) return;
    bpm = (double)inst->project_bpm;
    if (bpm < 20.0) bpm = 20.0;
    if (bpm > 300.0) bpm = 300.0;
    bpb = inst->beats_per_bar > 0 ? inst->beats_per_bar : 4;
    inst->samples_per_bar_f = ((double)sample_rate * 60.0 / bpm) * (double)bpb;
    if (inst->samples_per_bar_f < 1.0) inst->samples_per_bar_f = 1.0;
    /* Clamp countdown so it never exceeds one bar */
    if (inst->samples_until_bar_f <= 0.0 || inst->samples_until_bar_f > inst->samples_per_bar_f)
        inst->samples_until_bar_f = inst->samples_per_bar_f;
    inst->cached_sample_rate = sample_rate;
    inst->timing_dirty = 0;
}

/* Stop all tracks and cancel queued plays — called on 0xFC (Stop) and when
 * sync_to_clock is disabled while clock was running. */
static void handle_transport_stop(instance_t *inst) {
    int i;
    if (!inst) return;
    for (i = 0; i < NUM_TRACKS; i++) {
        inst->tracks[i].playing = 0;
        inst->tracks[i].queued_play = 0;
    }
}

/* ============================================================================
 * V2 API implementation
 * ============================================================================ */

static void* v2_create(const char *module_dir, const char *json_defaults) {
    (void)json_defaults;

    instance_t *inst = (instance_t *)calloc(1, sizeof(instance_t));
    if (!inst) {
        plugin_log("Failed to allocate instance");
        return NULL;
    }

    if (module_dir) {
        strncpy(inst->module_dir, module_dir, sizeof(inst->module_dir) - 1);
    }

    /* Initialize all tracks */
    for (int i = 0; i < NUM_TRACKS; i++) {
        init_track(&inst->tracks[i]);
    }

    inst->active_track = 0;
    inst->sync_to_clock = 0;
    inst->project_bpm = 120.0f;

    /* Clock sync state */
    inst->clock_running = 0;
    inst->clock_counter = 0;
    inst->clock_tick_total = 0;
    inst->beats_per_bar = 4;
    inst->clocks_per_bar = 96; /* overwritten by recalc_clock_timing */
    inst->pending_bar_triggers = 0;
    inst->delayed_bar_triggers = 0;
    inst->samples_per_bar_f = 0.0;
    inst->samples_until_bar_f = 0.0;
    inst->timing_dirty = 1;
    inst->cached_sample_rate = 0;
    recalc_clock_timing(inst);

    /* Allocate skipback ring buffers */
    inst->master_rb = (int16_t *)calloc((size_t)RB_FRAMES * 2, sizeof(int16_t));
    inst->master_rb_write = 0;
    for (int i = 0; i < NUM_TRACKS; i++) {
        inst->track_rb[i] = (int16_t *)calloc((size_t)RB_FRAMES * 2, sizeof(int16_t));
        inst->track_rb_write[i] = 0;
    }

    plugin_log("Instance created (4-track)");
    return inst;
}

static void v2_destroy(void *instance) {
    if (!instance) return;
    instance_t *inst = (instance_t *)instance;

    for (int i = 0; i < NUM_TRACKS; i++) {
        free_track(&inst->tracks[i]);
    }

    /* Free skipback ring buffers */
    free(inst->master_rb);
    for (int i = 0; i < NUM_TRACKS; i++) {
        free(inst->track_rb[i]);
    }

    free(inst);
    plugin_log("Instance destroyed");
}

static void v2_on_midi(void *instance, const uint8_t *msg, int len,
                       int source) {
    instance_t *inst = (instance_t *)instance;
    uint8_t status;
    (void)source;
    if (!inst || !msg || len < 1) return;
    status = msg[0];

    if (inst->sync_to_clock) {
        if (status == 0xFA) { /* Start — reset phase, arm immediate bar trigger */
            inst->clock_running = 1;
            inst->clock_counter = 0;
            inst->clock_tick_total = 0;
            inst->pending_bar_triggers = 0;
            inst->delayed_bar_triggers = 0;
            /* Fire bar-zero immediately so queued tracks start on downbeat */
            inst->pending_bar_triggers = 1;
            return;
        }
        if (status == 0xFB) { /* Continue — resume at current position */
            inst->clock_running = 1;
            inst->pending_bar_triggers = 0;
            inst->delayed_bar_triggers = 0;
            return;
        }
        if (status == 0xFC) { /* Stop */
            inst->clock_running = 0;
            inst->clock_counter = 0;
            inst->pending_bar_triggers = 0;
            inst->delayed_bar_triggers = 0;
            handle_transport_stop(inst);
            return;
        }
        if (status == 0xF8) { /* Clock tick */
            if (!inst->clock_running) return;
            /* Drain delayed triggers from the previous tick first (1-tick delay
             * avoids bar triggers landing in an already-started render block,
             * same pattern as SuperArp CLOCK_ARP_OUTPUT_DELAY_TICKS). */
            if (inst->delayed_bar_triggers > 0) {
                inst->pending_bar_triggers += inst->delayed_bar_triggers;
                inst->delayed_bar_triggers = 0;
            }
            inst->clock_tick_total++;
            inst->clock_counter = (int)(inst->clock_tick_total % (uint64_t)inst->clocks_per_bar);
            if (inst->clock_counter == 0) {
                inst->delayed_bar_triggers++;
            }
            return;
        }
    } else {
        /* sync_to_clock off — still track transport to reset internal phase */
        if (status == 0xFA || status == 0xFB) {
            if (inst->timing_dirty && inst->cached_sample_rate > 0)
                recalc_timing(inst, inst->cached_sample_rate);
            inst->samples_until_bar_f = inst->samples_per_bar_f > 0.0
                                        ? inst->samples_per_bar_f : 1.0;
            return;
        }
        if (status == 0xFC) {
            handle_transport_stop(inst);
            return;
        }
    }
}

/* ============================================================================
 * Track-prefixed parameter routing
 *
 * Keys with prefix "t0:" .. "t3:" route to the specified track.
 * Keys without prefix route to the active track.
 * Global keys: "active_track", "play_all", "sync_clock", "track_count".
 * ============================================================================ */

/* Parse track prefix. Returns track index (0-3) and sets *param to the
 * key after the prefix. If no prefix, returns -1 and *param = key. */
static int parse_track_prefix(const char *key, const char **param) {
    if (key[0] == 't' && key[1] >= '0' && key[1] <= '3' && key[2] == ':') {
        *param = key + 3;
        return key[1] - '0';
    }
    *param = key;
    return -1;
}

/* ---- set_param ---- */

static void v2_set_param(void *instance, const char *key, const char *val) {
    if (!instance || !key) return;
    instance_t *inst = (instance_t *)instance;

    /* DSP-side: log every set_param call to see what actually arrives */
    if (strncmp(key, "debug_log", 9) != 0 &&
        strcmp(key, "markers") != 0 &&
        strstr(key, "scene_state") == NULL &&
        strstr(key, "slice_state") == NULL) {
        FILE *dbgf = fopen("/tmp/scene_debug.log", "a");
        if (dbgf) {
            struct timespec _ts;
            clock_gettime(CLOCK_MONOTONIC, &_ts);
            fprintf(dbgf, "[%ld.%03ld] DSP set_param key=%s val=%.80s\n",
                    (long)_ts.tv_sec, _ts.tv_nsec / 1000000,
                    key, val ? val : "(null)");
            fclose(dbgf);
        }
    }

    /* --- Global params (no track prefix) --- */

    if (strcmp(key, "active_track") == 0) {
        if (!val) return;
        int v = atoi(val);
        if (v >= 0 && v < NUM_TRACKS) inst->active_track = v;
        return;
    }

    if (strcmp(key, "play_all") == 0) {
        if (!val) return;
        int start = atoi(val);
        for (int i = 0; i < NUM_TRACKS; i++) {
            track_t *t = &inst->tracks[i];
            if (!t->audio_data || t->audio_frames <= 0) continue;
            if (start) {
                if (!t->playing) {
                    if (inst->sync_to_clock && inst->clock_running) {
                        /* Queue for next bar boundary */
                        t->queued_play = 1;
                        t->queued_whole = 0;
                    } else {
                        t->queued_play = 0;
                        t->play_whole = 0;
                        t->playing = 1;
                        t->play_pos = t->start_sample;
                        t->play_pos_frac = (double)t->start_sample;
                        t->psx.read_pos = (double)t->start_sample;
                        t->bng_out_count = 0;
                        t->bng_play_start = t->start_sample;
                        t->bng_play_end = t->end_sample;
                        bng_reset_stretcher(t, (double)t->start_sample);
                        if (t->psx.enabled) { t->psx.needs_reinit = 1; }
                    }
                }
            } else {
                t->playing = 0;
                t->queued_play = 0;
            }
        }
        return;
    }

    if (strcmp(key, "sync_clock") == 0) {
        if (!val) return;
        int enabled = atoi(val) ? 1 : 0;
        if (enabled && !inst->sync_to_clock) {
            /* Enabling sync: reset clock state, assume transport is running */
            inst->clock_running = 1;
            inst->clock_counter = 0;
            inst->clock_tick_total = 0;
            inst->pending_bar_triggers = 0;
            inst->delayed_bar_triggers = 0;
            recalc_clock_timing(inst);
            if (inst->cached_sample_rate > 0)
                recalc_timing(inst, inst->cached_sample_rate);
        } else if (!enabled && inst->sync_to_clock) {
            /* Disabling sync: cancel queued plays, reset internal phase */
            for (int _i = 0; _i < NUM_TRACKS; _i++)
                inst->tracks[_i].queued_play = 0;
            inst->clock_running = 0;
            inst->pending_bar_triggers = 0;
            inst->delayed_bar_triggers = 0;
        }
        inst->sync_to_clock = enabled;
        return;
    }

    if (strcmp(key, "project_bpm") == 0) {
        if (!val) return;
        float bpm = (float)atof(val);
        if (bpm > 0.0f) {
            inst->project_bpm = bpm;
            inst->timing_dirty = 1;
            if (inst->cached_sample_rate > 0) {
                recalc_timing(inst, inst->cached_sample_rate);
                if (inst->sync_to_clock && inst->clock_running)
                    realign_clock_phase(inst);
            }
        }
        return;
    }

    if (strcmp(key, "master_volume_db") == 0) {
        if (!val) return;
        inst->master_volume_db = (float)atof(val);
        if (inst->master_volume_db < -60.0f) inst->master_volume_db = -60.0f;
        if (inst->master_volume_db > 6.0f) inst->master_volume_db = 6.0f;
        return;
    }

    if (strcmp(key, "beats_per_bar") == 0) {
        if (!val) return;
        int bpb = atoi(val);
        if (bpb >= 1 && bpb <= 16) {
            inst->beats_per_bar = bpb;
            recalc_clock_timing(inst);
            if (inst->sync_to_clock && inst->clock_running)
                realign_clock_phase(inst);
            inst->timing_dirty = 1;
            if (inst->cached_sample_rate > 0)
                recalc_timing(inst, inst->cached_sample_rate);
        }
        return;
    }

    /* --- Debug log: append message to /tmp/scene_debug.log --- */
    if (strncmp(key, "debug_log", 9) == 0) {
        if (!val) return;
        FILE *f = fopen("/tmp/scene_debug.log", "a");
        if (f) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            fprintf(f, "[%ld.%03ld] %s\n", (long)ts.tv_sec, ts.tv_nsec / 1000000, val);
            fclose(f);
        }
        return;
    }

    /* --- Skipback: extract last N frames from a ring buffer and write WAV ---
     * Format: "source,frames,path"
     *   source: "master" or "0".."3" (track index)
     *   frames: number of frames to extract
     *   path:   output WAV file path
     */
    if (strcmp(key, "skipback") == 0) {
        if (!val) return;
        char source[16];
        int  extract_frames = 0;
        char path[512];
        /* Manual parse: "%s" in sscanf stops at whitespace, breaking paths with spaces.
         * Find first comma → source, second comma → frames, remainder → path. */
        {
            const char *c1 = strchr(val, ',');
            if (!c1 || (c1 - val) >= (int)sizeof(source)) return;
            strncpy(source, val, (size_t)(c1 - val));
            source[c1 - val] = '\0';

            const char *c2 = strchr(c1 + 1, ',');
            if (!c2) return;
            char frames_str[32];
            int flen = (int)(c2 - (c1 + 1));
            if (flen <= 0 || flen >= (int)sizeof(frames_str)) return;
            strncpy(frames_str, c1 + 1, (size_t)flen);
            frames_str[flen] = '\0';
            extract_frames = atoi(frames_str);

            strncpy(path, c2 + 1, sizeof(path) - 1);
            path[sizeof(path) - 1] = '\0';
        }
        if (extract_frames <= 0 || path[0] == '\0') return;
        if (extract_frames > RB_FRAMES) extract_frames = RB_FRAMES;

        int16_t *rb = NULL;
        int write_pos = 0;
        if (strcmp(source, "master") == 0) {
            rb = inst->master_rb;
            write_pos = inst->master_rb_write;
        } else {
            int ti = atoi(source);
            if (ti >= 0 && ti < NUM_TRACKS) {
                rb = inst->track_rb[ti];
                write_pos = inst->track_rb_write[ti];
            }
        }
        if (!rb) return;

        /* Extract last extract_frames from circular buffer (linearise) */
        int16_t *buf = (int16_t *)malloc((size_t)extract_frames * 2 * sizeof(int16_t));
        if (!buf) return;
        int start = (write_pos - extract_frames + RB_FRAMES) % RB_FRAMES;
        for (int f = 0; f < extract_frames; f++) {
            int ri = (start + f) % RB_FRAMES;
            buf[f * 2]     = rb[ri * 2];
            buf[f * 2 + 1] = rb[ri * 2 + 1];
        }

        write_wav(path, buf, extract_frames, MOVE_SAMPLE_RATE, 1, 16, 2);
        free(buf);

        strncpy(inst->skipback_result, path, sizeof(inst->skipback_result) - 1);
        inst->skipback_result[sizeof(inst->skipback_result) - 1] = '\0';
        return;
    }

    if (strcmp(key, "project_dir") == 0) {
        if (val) {
            strncpy(inst->project_dir, val, sizeof(inst->project_dir) - 1);
            inst->project_dir[sizeof(inst->project_dir) - 1] = '\0';
        } else {
            inst->project_dir[0] = '\0';
        }
        return;
    }

    /* --- Route to track --- */
    const char *param;
    int track_idx = parse_track_prefix(key, &param);
    if (track_idx < 0) track_idx = inst->active_track;
    if (track_idx < 0 || track_idx >= NUM_TRACKS) return;

    track_t *t = &inst->tracks[track_idx];

    /* --- PaulXStretch parameters --- */
    if (strncmp(param, "psx_", 4) == 0) {
        ps_handle_param(t, param, val);
        return;
    }

    /* --- File loading --- */
    if (strcmp(param, "file_path") == 0) {
        if (!val || val[0] == '\0') {
            /* Empty path = unload track */
            t->playing = 0;
            t->file_path[0] = '\0';
            t->file_name[0] = '\0';
            if (t->audio_data) { free(t->audio_data); t->audio_data = NULL; }
            t->audio_frames = 0;
            t->start_sample = 0;
            t->end_sample   = 0;
            t->play_pos     = 0.0;
            if (t->undo_buffer) { free(t->undo_buffer); t->undo_buffer = NULL; }
            t->undo_frames  = 0;
            return;
        }

        /* Strip "?t=..." cache-bust suffix appended by the UI to defeat
         * the host's file_path deduplication. */
        char clean_path[1024];
        strncpy(clean_path, val, sizeof(clean_path) - 1);
        clean_path[sizeof(clean_path) - 1] = '\0';
        {
            char *q = strchr(clean_path, '?');
            if (q) *q = '\0';
        }

        /* DSP-side debug log — writes directly, no set_param queue */
        {
            FILE *dbgf = fopen("/tmp/scene_debug.log", "a");
            if (dbgf) {
                struct timespec _ts;
                clock_gettime(CLOCK_MONOTONIC, &_ts);
                fprintf(dbgf, "[%ld.%03ld] DSP file_path track=%d val=%s\n",
                        (long)_ts.tv_sec, _ts.tv_nsec / 1000000, track_idx, clean_path);
                fclose(dbgf);
            }
        }

        strncpy(t->file_path, clean_path, sizeof(t->file_path) - 1);
        t->file_path[sizeof(t->file_path) - 1] = '\0';

        /* Extract filename */
        const char *fn = basename_ptr(val);
        strncpy(t->file_name, fn, sizeof(t->file_name) - 1);
        t->file_name[sizeof(t->file_name) - 1] = '\0';

        /* Stop any playback */
        t->playing = 0;

        /* Reset pitch/tempo on new file */
        t->pitch_semitones = 0.0f;
        t->tempo_percent = 100;
        recompute_ratios(t);

        /* Load the file */
        int load_result = load_wav(t, val);
        {
            FILE *dbgf = fopen("/tmp/scene_debug.log", "a");
            if (dbgf) {
                struct timespec _ts;
                clock_gettime(CLOCK_MONOTONIC, &_ts);
                fprintf(dbgf, "[%ld.%03ld] DSP load_wav result=%d frames=%d name=%s\n",
                        (long)_ts.tv_sec, _ts.tv_nsec / 1000000,
                        load_result, t->audio_frames, t->file_name);
                fclose(dbgf);
            }
        }
        if (load_result == 0) {
            /* Always recreate Bungee stretcher to match the file's sample rate.
             * This handles both non-44.1k files AND the case where a previous
             * 48kHz file left the stretcher at {48000,44100} and a 44.1kHz file
             * is loaded next. */
            if (t->stretcher) {
                delete t->stretcher;
                t->stretcher = new Bungee::Stretcher<Bungee::Basic>(
                    Bungee::SampleRates{t->sample_rate, MOVE_SAMPLE_RATE}, 2, 0);
                int new_max = t->stretcher->maxInputFrameCount();
                if (new_max != t->bng_max_grain) {
                    t->bng_max_grain = new_max;
                    free(t->bng_grain_input);
                    t->bng_grain_input = (float *)calloc(new_max * 2, sizeof(float));
                }
            }
            /* Compute initial waveform and peak */
            compute_waveform(t, 128);
            t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);
        }
        return;
    }

    /* --- Selection markers --- */
    /* Combined marker param: "start,end" - avoids fire-and-forget race */
    if (strcmp(param, "markers") == 0) {
        if (!val) return;
        int s = 0, e = 0;
        if (sscanf(val, "%d,%d", &s, &e) == 2) {
            if (s < 0) s = 0;
            if (t->audio_data && s > t->audio_frames) s = t->audio_frames;
            if (e < 0) e = 0;
            if (t->audio_data && e > t->audio_frames) e = t->audio_frames;
            t->start_sample = s;
            t->end_sample = e;
            /* Also update Bungee playback boundaries so the loop respects the
             * new markers immediately without waiting for a play restart.
             * Clear sync_play_end and loop_len_exact so the render path uses
             * end_sample directly as the loop boundary (not the stale SYNC value). */
            t->sync_play_end = 0;
            t->loop_len_exact = 0.0;
            t->bng_play_start = s;
            t->bng_play_end = e;
        }
        return;
    }

    if (strcmp(param, "start_sample") == 0) {
        if (!val) return;
        int v = atoi(val);
        if (v < 0) v = 0;
        if (t->audio_data && v > t->audio_frames)
            v = t->audio_frames;
        t->start_sample = v;
        return;
    }

    if (strcmp(param, "end_sample") == 0) {
        if (!val) return;
        int v = atoi(val);
        if (v < 0) v = 0;
        if (t->audio_data && v > t->audio_frames)
            v = t->audio_frames;
        t->end_sample = v;
        return;
    }

    /* --- Zoom --- */
    /* Combined zoom param: "level,center" - avoids fire-and-forget race */
    if (strcmp(param, "zoom") == 0) {
        if (!val) return;
        int level = 0, center = 0;
        if (sscanf(val, "%d,%d", &level, &center) == 2) {
            if (level < 0) level = 0;
            if (level > 12) level = 12;
            if (center < 0) center = 0;
            if (t->audio_data && center > t->audio_frames)
                center = t->audio_frames;
            t->zoom_level = level;
            t->zoom_center = center;
        }
        return;
    }

    if (strcmp(param, "zoom_level") == 0) {
        if (!val) return;
        int v = atoi(val);
        if (v < 0) v = 0;
        if (v > 12) v = 12;
        t->zoom_level = v;
        return;
    }

    if (strcmp(param, "zoom_center") == 0) {
        if (!val) return;
        int v = atoi(val);
        if (v < 0) v = 0;
        if (t->audio_data && v > t->audio_frames)
            v = t->audio_frames;
        t->zoom_center = v;
        return;
    }

    /* --- Mode --- */
    if (strcmp(param, "mode") == 0) {
        if (!val) return;
        int v = atoi(val);
        t->mode = (v == 1) ? 1 : 0;
        return;
    }

    /* --- Playback --- */
    if (strcmp(param, "play") == 0) {
        if (!t->audio_data || t->audio_frames <= 0) return;

        if (val && strcmp(val, "stop") == 0) {
            t->playing = 0;
            t->queued_play = 0;
            return;
        }

        /* Value encodes play mode: "whole" = entire file, anything else = selection */
        int whole = (val && strcmp(val, "whole") == 0) ? 1 : 0;

        if (inst->sync_to_clock && inst->clock_running) {
            /* Queue for next bar boundary; cancel any previous queue */
            t->queued_play = 1;
            t->queued_whole = whole;
            return;
        }

        t->queued_play = 0;
        t->play_whole = whole;
        t->playing = 1;
        if (whole) {
            t->play_pos = 0;
            t->play_pos_frac = 0.0;
            t->psx.read_pos = 0.0;
            t->bng_play_start = 0;
            t->bng_play_end = t->audio_frames;
            t->bng_out_count = 0;
            bng_reset_stretcher(t, 0.0);
        } else {
            t->play_pos = t->start_sample;
            t->play_pos_frac = (double)t->start_sample;
            t->psx.read_pos = (double)t->start_sample;
            t->bng_play_start = t->start_sample;
            t->bng_play_end = (t->sync_play_end > 0) ? t->sync_play_end : t->end_sample;
            t->bng_out_count = 0;
            bng_reset_stretcher(t, (double)t->start_sample);
        }
        if (t->psx.enabled) { t->psx.needs_reinit = 1; }
        {
            char dbg[256];
            snprintf(dbg, sizeof(dbg), "PLAY t%d: whole=%d start=%d end=%d pos=%d total=%d",
                     track_idx, whole, t->start_sample, t->end_sample,
                     t->play_pos, t->audio_frames);
            plugin_log(dbg);
        }
        return;
    }

    /* play_now: always starts immediately, bypasses sync queuing.
     * Used when the UI has already done its own quantize (transport start,
     * force-immediate second press, QUANT_SAMPLE_END). */
    if (strcmp(param, "play_now") == 0) {
        if (!t->audio_data || t->audio_frames <= 0) return;
        if (val && strcmp(val, "stop") == 0) {
            t->playing = 0;
            t->queued_play = 0;
            return;
        }
        int whole = (val && strcmp(val, "whole") == 0) ? 1 : 0;
        t->queued_play = 0;
        t->play_whole = whole;
        t->playing = 1;
        if (whole) {
            t->play_pos = 0;
            t->play_pos_frac = 0.0;
            t->psx.read_pos = 0.0;
            t->bng_play_start = 0;
            t->bng_play_end = t->audio_frames;
            t->bng_out_count = 0;
            bng_reset_stretcher(t, 0.0);
        } else {
            t->play_pos = t->start_sample;
            t->play_pos_frac = (double)t->start_sample;
            t->psx.read_pos = (double)t->start_sample;
            t->bng_play_start = t->start_sample;
            t->bng_play_end = (t->sync_play_end > 0) ? t->sync_play_end : t->end_sample;
            t->bng_out_count = 0;
            bng_reset_stretcher(t, (double)t->start_sample);
        }
        if (t->psx.enabled) { t->psx.needs_reinit = 1; }
        return;
    }

    if (strcmp(param, "play_from") == 0) {
        /* Start playback from an arbitrary frame position to end_sample.
         * Used for previewing near the end marker without moving markers. */
        if (!t->audio_data || t->audio_frames <= 0) return;
        if (!val) return;
        int pos = atoi(val);
        if (pos < 0) pos = 0;
        if (pos >= t->audio_frames) pos = t->audio_frames - 1;
        t->play_whole = 0;
        t->playing = 1;
        t->play_pos = pos;
        t->play_pos_frac = (double)pos;
        t->psx.read_pos = (double)pos;
        /* Reset Bungee state */
        t->bng_play_start = t->start_sample;
        t->bng_play_end = (t->sync_play_end > 0) ? t->sync_play_end : t->end_sample;
        t->bng_out_count = 0;
        bng_reset_stretcher(t, (double)pos);
        if (t->psx.enabled) { t->psx.needs_reinit = 1; }
        return;
    }

    if (strcmp(param, "stop") == 0) {
        t->playing = 0;
        return;
    }

    if (strcmp(param, "play_loop") == 0) {
        if (!val) return;
        t->play_loop = atoi(val) ? 1 : 0;
        return;
    }

    if (strcmp(param, "play_whole") == 0) {
        if (!val) return;
        t->play_whole = atoi(val) ? 1 : 0;
        return;
    }

    /* --- Gate mode --- */
    if (strcmp(param, "gate_mode") == 0) {
        if (!val) return;
        int new_mode = atoi(val);
        if (new_mode < 0 || new_mode > 2) new_mode = 0;
        /* Switching TO Gate while playing: treat as held so current playback
         * continues. Gate semantics (stop on release) apply from next launch. */
        if (new_mode == 1 && t->gate_mode != 1 && t->playing)
            t->gate_held = 1;
        /* Leaving Gate: clear held flag so it doesn't carry over */
        if (new_mode != 1)
            t->gate_held = 0;
        t->gate_mode = new_mode;
        return;
    }

    /* --- Play mode: 0=stretch (Bungee), 1=varispeed (repitch) --- */
    if (strcmp(param, "play_mode") == 0) {
        if (!val) return;
        t->play_mode = atoi(val) ? 1 : 0;
        return;
    }

    if (strcmp(param, "gate_held") == 0) {
        if (!val) return;
        t->gate_held = atoi(val) ? 1 : 0;
        return;
    }

    if (strcmp(param, "muted") == 0) {
        if (!val) return;
        t->muted = atoi(val) ? 1 : 0;
        return;
    }

    if (strcmp(param, "pitch") == 0) {
        if (!val) return;
        float v = (float)atof(val);
        if (v < -36.0f) v = -36.0f;
        if (v > 36.0f) v = 36.0f;
        /* Check if we are already using the Bungee path before the change.
         * In SYNC mode (tempo_speed_precise set) or with any existing pitch
         * offset the stretcher is already running — only update bng_req.pitch
         * directly to avoid a reset-induced click.  A reset is only needed
         * when transitioning from the direct (no-stretch) path to Bungee. */
        float old_pitch_ratio = (float)pow(2.0, (double)t->pitch_semitones / 12.0);
        int already_bungee = !(fabsf(old_pitch_ratio - 1.0f) < 0.001f &&
                                fabsf(t->tempo_ratio  - 1.0f) < 0.001f);
        t->pitch_semitones = v;
        recompute_ratios(t);
        if (t->playing) {
            if (already_bungee) {
                /* Already in Bungee: update pitch in-flight, no reset needed.
                 * This preserves loop position and avoids clicks/pops. */
                t->bng_req.pitch = pow(2.0, (double)v / 12.0);
            } else {
                /* Transitioning direct→Bungee: full init required. */
                t->bng_out_count = 0;
                bng_reset_stretcher(t, t->play_pos_frac);
            }
        } else {
            t->bng_req.pitch = pow(2.0, (double)v / 12.0);
        }
        return;
    }

    if (strcmp(param, "tempo") == 0) {
        if (!val) return;
        int v = atoi(val);
        if (v < 30) v = 30;
        if (v > 300) v = 300;
        /* Check if already using the Bungee path before the change.
         * tempo_speed_precise > 0 means SYNC mode was active (Bungee running).
         * Any non-unity pitch or tempo also means Bungee is running.
         * Only reset the stretcher when transitioning direct→Bungee to
         * avoid clicks/pops on in-flight tempo changes (same as pitch handler). */
        float cur_pitch_ratio = (float)pow(2.0, (double)t->pitch_semitones / 12.0);
        int already_bungee = (t->tempo_speed_precise > 0.0) ||
                             !(fabsf(cur_pitch_ratio - 1.0f) < 0.001f &&
                               fabsf(t->tempo_ratio  - 1.0f) < 0.001f);
        t->tempo_percent = v;
        t->sync_play_end = 0;    /* FREE mode: disable SYNC loop boundary */
        t->loop_len_exact = 0.0; /* clear exact length so render uses end_sample */
        t->tempo_speed_precise = 0.0; /* clear SYNC precise speed so recompute_ratios uses tempo_percent */
        recompute_ratios(t);
        if (t->playing) {
            if (already_bungee) {
                /* Already in Bungee: update speed in-flight, no reset needed.
                 * Preserves loop position and avoids clicks/pops. */
                t->bng_req.speed = (double)v / 100.0;
            } else {
                /* Transitioning direct→Bungee: full init required. */
                t->bng_out_count = 0;
                bng_reset_stretcher(t, t->play_pos_frac);
            }
        } else {
            t->bng_req.speed = (double)v / 100.0;
        }
        return;
    }

    /* --- SYNC mode: atomic tempo + loop-boundary update ---
     * Format: "tempoPercent,startSample,logicalEndSample"
     * Sets tempo_percent and uses the logical end sample directly as bng_play_end.
     * Output loop duration = (logicalEnd - start) / (tempoPercent/100), which
     * correctly scales with sceneBpm/projectBpm ratio — matching musicalLength.
     * The edit markers (start_sample / end_sample) are NOT changed so that
     * file_info always returns the logical positions. */
    if (strcmp(param, "sync_tempo") == 0) {
        if (!val) return;
        double tp; int s, e; double exact_len = 0.0;
        /* Accept optional 4th field: exact loop length in input samples (float).
         * When present, used for drift-free loop wrap instead of integer (e-s). */
        int n = sscanf(val, "%lf,%d,%d,%lf", &tp, &s, &e, &exact_len);
        if (n < 3) return;
        if (tp < 30.0) tp = 30.0;
        if (tp > 300.0) tp = 300.0;
        t->tempo_percent = (int)round(tp);
        t->tempo_speed_precise = tp / 100.0; /* preserve full precision before int rounding */
        recompute_ratios(t);
        if (e > t->audio_frames) e = t->audio_frames;
        if (e < s) e = s;

        t->start_sample  = s;
        t->end_sample    = e;
        t->sync_play_end = e;
        t->bng_play_start = s;
        t->bng_play_end  = e;
        t->loop_len_exact = (exact_len > 0.0) ? exact_len : 0.0;

        /* Update speed directly without resetting the stretcher so the loop
         * boundary takes effect on the very next audio block (same as E2).
         * Use the full-precision float ratio to avoid cumulative drift. */
        t->bng_req.speed = t->tempo_speed_precise;
        return;
    }

    /* --- Find zero crossing (S1000-style FIND) --- */
    /* Uses L channel for zero crossing detection */
    if (strcmp(param, "find_zero_crossing") == 0) {
        if (!t->audio_data || t->audio_frames <= 2) return;

        int end_pos = t->end_sample;
        int start_pos = t->start_sample;

        /* Reference: amplitude and slope at loop end point (L channel) */
        if (end_pos < 1) end_pos = 1;
        if (end_pos > t->audio_frames) end_pos = t->audio_frames;
        int ref_idx = end_pos - 1;
        if (ref_idx < 1) ref_idx = 1;
        int ref_amp = (int)t->audio_data[ref_idx * 2];
        int ref_slope = (int)t->audio_data[ref_idx * 2] - (int)t->audio_data[(ref_idx - 1) * 2];

        /* Search ±2048 samples around current start */
        int search_radius = 2048;
        int search_lo = start_pos - search_radius;
        int search_hi = start_pos + search_radius;
        if (search_lo < 1) search_lo = 1;
        if (search_hi >= t->audio_frames) search_hi = t->audio_frames - 1;

        int best_pos = start_pos;
        int best_score = 0x7FFFFFFF;

        for (int i = search_lo; i <= search_hi; i++) {
            int amp = (int)t->audio_data[i * 2];
            int slope = (int)t->audio_data[i * 2] - (int)t->audio_data[(i - 1) * 2];
            int amp_err = abs(amp - ref_amp);
            int slope_err = abs(slope - ref_slope);
            int score = amp_err * 2 + slope_err;
            if (score < best_score) {
                best_score = score;
                best_pos = i;
            }
        }

        t->start_sample = best_pos;

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Zero-crossing FIND: %d -> %d (score %d)",
                     start_pos, best_pos, best_score);
            plugin_log(log_buf);
        }
        return;
    }

    /* --- Gain --- */
    if (strcmp(param, "gain_db") == 0) {
        if (!val) return;
        t->gain_db = (float)atof(val);
        /* Clamp to reasonable range */
        if (t->gain_db < -60.0f) t->gain_db = -60.0f;
        if (t->gain_db > 24.0f) t->gain_db = 24.0f;
        return;
    }

    /* --- Pan --- */
    if (strcmp(param, "pan") == 0) {
        if (!val) return;
        t->pan = (float)atof(val);
        if (t->pan < -1.0f) t->pan = -1.0f;
        if (t->pan > 1.0f) t->pan = 1.0f;
        return;
    }

    /* --- Editing operations --- */

    if (strcmp(param, "trim") == 0) {
        if (!t->audio_data || t->audio_frames <= 0) return;

        int start = t->start_sample;
        int end = t->end_sample;

        /* Validate selection */
        if (start < 0) start = 0;
        if (end > t->audio_frames) end = t->audio_frames;
        if (start >= end) {
            plugin_log("Trim: invalid selection (start >= end)");
            return;
        }

        int new_frames = end - start;
        if (new_frames <= 0) return;

        /* Save play state so we can resume after buffer swap */
        int was_playing = t->playing;
        int old_play_pos = t->play_pos;
        double old_play_pos_frac = t->play_pos_frac;

        /* Stop playback during buffer swap to prevent render_track
         * from accessing freed memory. */
        t->playing = 0;

        /* Save to undo */
        save_undo(t);

        /* Extract the selected region */
        size_t buf_size = (size_t)new_frames * SAMPLES_PER_FRAME * sizeof(int16_t);
        int16_t *new_data = (int16_t *)malloc(buf_size);
        if (!new_data) {
            plugin_log("Trim: allocation failed");
            t->playing = was_playing;
            return;
        }

        memcpy(new_data, t->audio_data + start * SAMPLES_PER_FRAME, buf_size);

        /* Replace audio */
        free(t->audio_data);
        t->audio_data = new_data;
        t->audio_frames = new_frames;
        t->total_frames = new_frames;
        t->duration_secs = (float)new_frames / (float)t->sample_rate;

        /* Reset markers */
        t->start_sample = 0;
        t->end_sample = new_frames;
        t->zoom_level = 0;
        t->zoom_center = new_frames / 2;
        t->dirty = 1;

        /* Resume playback with adjusted position */
        if (was_playing) {
            double new_pos = old_play_pos_frac - (double)start;
            if (new_pos < 0) new_pos = 0;
            if (new_pos >= (double)new_frames) {
                /* Position was outside trimmed region — wrap to start */
                new_pos = 0;
            }
            t->play_pos_frac = new_pos;
            t->play_pos = (int)new_pos;
            t->psx.read_pos = new_pos;
            t->playing = 1;
            if (t->psx.enabled) { t->psx.needs_reinit = 1; }
        }

        /* Recompute */
        compute_waveform(t, 128);
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Trimmed: %d-%d -> %d frames", start, end, new_frames);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "apply_pitch_tempo") == 0) {
        do_apply_pitch_tempo(t);
        return;
    }

    if (strcmp(param, "copy") == 0) {
        /* Copy selection to clipboard (non-destructive) */
        if (!t->audio_data || t->audio_frames <= 0) return;

        int start = t->start_sample;
        int end = t->end_sample;

        if (start < 0) start = 0;
        if (end > t->audio_frames) end = t->audio_frames;
        if (start >= end) {
            plugin_log("Copy: invalid selection");
            return;
        }

        int copy_frames = end - start;

        /* Free old clipboard */
        if (t->clipboard_data) {
            free(t->clipboard_data);
            t->clipboard_data = NULL;
            t->clipboard_frames = 0;
        }

        size_t buf_size = (size_t)copy_frames * SAMPLES_PER_FRAME * sizeof(int16_t);
        t->clipboard_data = (int16_t *)malloc(buf_size);
        if (!t->clipboard_data) {
            plugin_log("Copy: clipboard allocation failed");
            return;
        }

        memcpy(t->clipboard_data, t->audio_data + start * SAMPLES_PER_FRAME, buf_size);
        t->clipboard_frames = copy_frames;

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Copied %d frames to clipboard", copy_frames);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "export") == 0) {
        /* Export selection to new file */
        if (!t->audio_data || t->audio_frames <= 0) return;

        int start = t->start_sample;
        int end = t->end_sample;

        if (start < 0) start = 0;
        if (end > t->audio_frames) end = t->audio_frames;
        if (start >= end) {
            plugin_log("Export: invalid selection");
            return;
        }

        int copy_frames = end - start;

        /* Generate output filename */
        char out_path[512];
        make_edit_filename(t->file_path, out_path, (int)sizeof(out_path));

        /* Write the selection as PCM 16-bit (internal format).
         * Always use PCM so exported files are compatible with rex-encode
         * and other tools that may not support IEEE float WAV. */
        if (write_wav(out_path, t->audio_data + start * SAMPLES_PER_FRAME, copy_frames,
                      t->sample_rate, 1, 16,
                      t->orig_channels) == 0) {
            const char *out_name = basename_ptr(out_path);
            strncpy(t->copy_result, out_name,
                    sizeof(t->copy_result) - 1);
            t->copy_result[sizeof(t->copy_result) - 1] = '\0';
        } else {
            strncpy(t->copy_result, "ERROR",
                    sizeof(t->copy_result) - 1);
        }
        return;
    }

    if (strcmp(param, "mute") == 0) {
        /* Mute (zero out) selection */
        if (!t->audio_data || t->audio_frames <= 0) return;

        int start = t->start_sample;
        int end = t->end_sample;

        if (start < 0) start = 0;
        if (end > t->audio_frames) end = t->audio_frames;
        if (start >= end) {
            plugin_log("Mute: invalid selection");
            return;
        }

        save_undo(t);

        memset(t->audio_data + start * SAMPLES_PER_FRAME, 0,
               (size_t)(end - start) * SAMPLES_PER_FRAME * sizeof(int16_t));

        t->dirty = 1;
        compute_waveform(t, 128);
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Muted %d-%d (%d frames)", start, end, end - start);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "fade_in") == 0) {
        /* Linear fade-in over selection */
        if (!t->audio_data || t->audio_frames <= 0) return;

        int start = t->start_sample;
        int end   = t->end_sample;
        if (start < 0) start = 0;
        if (end > t->audio_frames) end = t->audio_frames;
        if (start >= end) {
            plugin_log("Fade in: invalid selection");
            return;
        }

        save_undo(t);

        int len = end - start;
        for (int i = 0; i < len; i++) {
            float gain = (float)i / (float)len;
            int idx = (start + i) * SAMPLES_PER_FRAME;
            for (int ch = 0; ch < SAMPLES_PER_FRAME; ch++) {
                float s = (float)t->audio_data[idx + ch] * gain;
                if (s > 32767.0f) s = 32767.0f;
                if (s < -32768.0f) s = -32768.0f;
                t->audio_data[idx + ch] = (int16_t)s;
            }
        }

        t->dirty = 1;
        compute_waveform(t, 128);
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Fade in %d-%d (%d frames)", start, end, len);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "fade_out") == 0) {
        /* Linear fade-out over selection */
        if (!t->audio_data || t->audio_frames <= 0) return;

        int start = t->start_sample;
        int end   = t->end_sample;
        if (start < 0) start = 0;
        if (end > t->audio_frames) end = t->audio_frames;
        if (start >= end) {
            plugin_log("Fade out: invalid selection");
            return;
        }

        save_undo(t);

        int len = end - start;
        for (int i = 0; i < len; i++) {
            float gain = 1.0f - (float)i / (float)len;
            int idx = (start + i) * SAMPLES_PER_FRAME;
            for (int ch = 0; ch < SAMPLES_PER_FRAME; ch++) {
                float s = (float)t->audio_data[idx + ch] * gain;
                if (s > 32767.0f) s = 32767.0f;
                if (s < -32768.0f) s = -32768.0f;
                t->audio_data[idx + ch] = (int16_t)s;
            }
        }

        t->dirty = 1;
        compute_waveform(t, 128);
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Fade out %d-%d (%d frames)", start, end, len);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "reverse") == 0) {
        /* Reverse selection in-place (swap stereo frame pairs) */
        if (!t->audio_data || t->audio_frames <= 0) return;

        int start = t->start_sample;
        int end   = t->end_sample;
        if (start < 0) start = 0;
        if (end > t->audio_frames) end = t->audio_frames;
        if (start >= end) {
            plugin_log("Reverse: invalid selection");
            return;
        }

        save_undo(t);

        int i = start;
        int j = end - 1;
        while (i < j) {
            /* Swap stereo pairs */
            int16_t tmp_l = t->audio_data[i * 2];
            int16_t tmp_r = t->audio_data[i * 2 + 1];
            t->audio_data[i * 2]     = t->audio_data[j * 2];
            t->audio_data[i * 2 + 1] = t->audio_data[j * 2 + 1];
            t->audio_data[j * 2]     = tmp_l;
            t->audio_data[j * 2 + 1] = tmp_r;
            i++;
            j--;
        }

        t->dirty = 1;
        compute_waveform(t, 128);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Reversed %d-%d (%d frames)", start, end, end - start);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "cut") == 0) {
        /* Cut selection to clipboard (copy + remove) */
        if (!t->audio_data || t->audio_frames <= 0) return;

        int start = t->start_sample;
        int end = t->end_sample;

        if (start < 0) start = 0;
        if (end > t->audio_frames) end = t->audio_frames;
        if (start >= end) {
            plugin_log("Cut: invalid selection");
            return;
        }

        int sel_frames = end - start;

        save_undo(t);

        /* Copy selection to clipboard */
        if (t->clipboard_data) {
            free(t->clipboard_data);
            t->clipboard_data = NULL;
            t->clipboard_frames = 0;
        }

        size_t clip_size = (size_t)sel_frames * SAMPLES_PER_FRAME * sizeof(int16_t);
        t->clipboard_data = (int16_t *)malloc(clip_size);
        if (!t->clipboard_data) {
            plugin_log("Cut: clipboard allocation failed");
            return;
        }

        memcpy(t->clipboard_data, t->audio_data + start * SAMPLES_PER_FRAME, clip_size);
        t->clipboard_frames = sel_frames;

        /* Remove selection from audio_data */
        int new_frames = t->audio_frames - sel_frames;
        if (new_frames <= 0) {
            /* Cutting everything — leave a tiny silent buffer */
            new_frames = 1;
            int16_t *new_data = (int16_t *)calloc(SAMPLES_PER_FRAME, sizeof(int16_t));
            if (!new_data) {
                plugin_log("Cut: allocation failed");
                return;
            }
            free(t->audio_data);
            t->audio_data = new_data;
        } else {
            size_t new_size = (size_t)new_frames * SAMPLES_PER_FRAME * sizeof(int16_t);
            int16_t *new_data = (int16_t *)malloc(new_size);
            if (!new_data) {
                plugin_log("Cut: allocation failed");
                return;
            }

            /* Copy [0..start] + [end..audio_frames] */
            if (start > 0) {
                memcpy(new_data, t->audio_data,
                       (size_t)start * SAMPLES_PER_FRAME * sizeof(int16_t));
            }
            if (end < t->audio_frames) {
                memcpy(new_data + start * SAMPLES_PER_FRAME,
                       t->audio_data + end * SAMPLES_PER_FRAME,
                       (size_t)(t->audio_frames - end) * SAMPLES_PER_FRAME * sizeof(int16_t));
            }

            free(t->audio_data);
            t->audio_data = new_data;
        }

        t->audio_frames = new_frames;
        t->total_frames = new_frames;
        t->duration_secs = (float)new_frames / (float)t->sample_rate;

        /* Reset markers to cut point */
        t->start_sample = start;
        if (t->start_sample > new_frames) t->start_sample = new_frames;
        t->end_sample = t->start_sample;
        t->zoom_level = 0;
        t->zoom_center = new_frames / 2;
        t->dirty = 1;
        t->playing = 0;

        compute_waveform(t, 128);
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Cut %d frames to clipboard, %d remain",
                     sel_frames, new_frames);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "paste") == 0) {
        /* Paste clipboard at start_sample (insert, shift right) */
        if (!t->clipboard_data || t->clipboard_frames <= 0) {
            plugin_log("Paste: clipboard is empty");
            return;
        }
        if (!t->audio_data || t->audio_frames <= 0) return;

        int insert_at = t->start_sample;
        if (insert_at < 0) insert_at = 0;
        if (insert_at > t->audio_frames) insert_at = t->audio_frames;

        int new_frames = t->audio_frames + t->clipboard_frames;
        if (new_frames > MAX_AUDIO_FRAMES) {
            plugin_log("Paste: result would exceed max duration");
            return;
        }

        save_undo(t);

        size_t new_size = (size_t)new_frames * SAMPLES_PER_FRAME * sizeof(int16_t);
        int16_t *new_data = (int16_t *)malloc(new_size);
        if (!new_data) {
            plugin_log("Paste: allocation failed");
            return;
        }

        /* Copy [0..insert_at] + clipboard + [insert_at..audio_frames] */
        if (insert_at > 0) {
            memcpy(new_data, t->audio_data,
                   (size_t)insert_at * SAMPLES_PER_FRAME * sizeof(int16_t));
        }
        memcpy(new_data + insert_at * SAMPLES_PER_FRAME, t->clipboard_data,
               (size_t)t->clipboard_frames * SAMPLES_PER_FRAME * sizeof(int16_t));
        if (insert_at < t->audio_frames) {
            memcpy(new_data + (insert_at + t->clipboard_frames) * SAMPLES_PER_FRAME,
                   t->audio_data + insert_at * SAMPLES_PER_FRAME,
                   (size_t)(t->audio_frames - insert_at) * SAMPLES_PER_FRAME * sizeof(int16_t));
        }

        free(t->audio_data);
        t->audio_data = new_data;
        t->audio_frames = new_frames;
        t->total_frames = new_frames;
        t->duration_secs = (float)new_frames / (float)t->sample_rate;

        /* Select the pasted region */
        t->start_sample = insert_at;
        t->end_sample = insert_at + t->clipboard_frames;
        t->dirty = 1;
        t->playing = 0;

        compute_waveform(t, 128);
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Pasted %d frames at %d, total now %d",
                     t->clipboard_frames, insert_at, new_frames);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "apply_gain") == 0) {
        if (!t->audio_data || t->audio_frames <= 0) return;

        save_undo(t);

        /* Convert dB to linear gain */
        float linear_gain = powf(10.0f, t->gain_db / 20.0f);
        int total_samples = t->audio_frames * SAMPLES_PER_FRAME;

        for (int i = 0; i < total_samples; i++) {
            float s = (float)t->audio_data[i] * linear_gain;
            /* Clamp to int16 range */
            if (s > 32767.0f) s = 32767.0f;
            if (s < -32768.0f) s = -32768.0f;
            t->audio_data[i] = (int16_t)s;
        }

        t->dirty = 1;
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);
        compute_waveform(t, 128);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Applied gain: %.1f dB (linear %.4f)",
                     t->gain_db, linear_gain);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "normalize") == 0) {
        if (!t->audio_data || t->audio_frames <= 0) return;

        save_undo(t);

        /* Find current peak */
        float current_peak_db = compute_peak_db(t->audio_data,
                                                t->audio_frames);
        if (current_peak_db <= -96.0f) {
            plugin_log("Normalize: file is silent");
            return;
        }

        /* Calculate gain needed to reach target */
        float gain_needed_db = NORMALIZE_TARGET_DB - current_peak_db;
        float linear_gain = powf(10.0f, gain_needed_db / 20.0f);
        int total_samples = t->audio_frames * SAMPLES_PER_FRAME;

        for (int i = 0; i < total_samples; i++) {
            float s = (float)t->audio_data[i] * linear_gain;
            if (s > 32767.0f) s = 32767.0f;
            if (s < -32768.0f) s = -32768.0f;
            t->audio_data[i] = (int16_t)s;
        }

        t->dirty = 1;
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);
        compute_waveform(t, 128);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "Normalized: applied %.1f dB, peak now %.1f dB",
                     gain_needed_db, t->peak_db);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "normalize_selection") == 0) {
        /* Normalize only the selected region [start_sample, end_sample] to target dBFS */
        if (!t->audio_data || t->audio_frames <= 0) return;

        /* Clamp selection to valid frame range */
        int start = t->start_sample;
        int end   = t->end_sample;
        if (start < 0) start = 0;
        if (end > t->audio_frames) end = t->audio_frames;
        if (start >= end) {
            plugin_log("NormalizeSel: empty selection");
            return;
        }

        save_undo(t);

        /* Find peak within selection */
        float current_peak_db = compute_peak_db(t->audio_data + start * SAMPLES_PER_FRAME,
                                                end - start);
        if (current_peak_db <= -96.0f) {
            plugin_log("NormalizeSel: selection is silent");
            return;
        }

        /* Calculate gain needed to reach target */
        float gain_needed_db = NORMALIZE_TARGET_DB - current_peak_db;
        float linear_gain    = powf(10.0f, gain_needed_db / 20.0f);
        int   total_samples  = (end - start) * SAMPLES_PER_FRAME;
        int16_t *ptr         = t->audio_data + start * SAMPLES_PER_FRAME;

        for (int i = 0; i < total_samples; i++) {
            float s = (float)ptr[i] * linear_gain;
            if (s > 32767.0f) s = 32767.0f;
            if (s < -32768.0f) s = -32768.0f;
            ptr[i] = (int16_t)s;
        }

        t->dirty = 1;
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);
        compute_waveform(t, 128);

        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "NormalizeSel: applied %.1f dB, peak now %.1f dB",
                     gain_needed_db, t->peak_db);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "paste_overwrite") == 0) {
        /* Paste clipboard at start_sample, overwriting (file length unchanged) */
        if (!t->clipboard_data || t->clipboard_frames <= 0) {
            plugin_log("PasteOW: clipboard empty"); return; }
        if (!t->audio_data || t->audio_frames <= 0) return;

        int insert_at = t->start_sample;
        if (insert_at < 0) insert_at = 0;
        if (insert_at >= t->audio_frames) {
            plugin_log("PasteOW: cursor at/past EOF"); return; }

        int frames_to_write = t->clipboard_frames;
        if (insert_at + frames_to_write > t->audio_frames)
            frames_to_write = t->audio_frames - insert_at;

        save_undo(t);

        memcpy(t->audio_data + insert_at * SAMPLES_PER_FRAME,
               t->clipboard_data,
               (size_t)frames_to_write * SAMPLES_PER_FRAME * sizeof(int16_t));

        t->start_sample = insert_at;
        t->end_sample   = insert_at + frames_to_write;
        t->dirty   = 1;
        t->playing = 0;
        compute_waveform(t, 128);
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);
        {
            char log_buf[128];
            snprintf(log_buf, sizeof(log_buf),
                     "PasteOW: wrote %d frames at %d", frames_to_write, insert_at);
            plugin_log(log_buf);
        }
        return;
    }

    if (strcmp(param, "undo") == 0) {
        if (!t->has_undo || !t->undo_buffer) {
            plugin_log("Undo: nothing to undo");
            return;
        }

        /* Swap audio_data and undo_buffer */
        int16_t *tmp_data = t->audio_data;
        int tmp_frames = t->audio_frames;

        t->audio_data = t->undo_buffer;
        t->audio_frames = t->undo_frames;
        t->total_frames = t->undo_frames;
        t->duration_secs = (float)t->undo_frames /
                              (float)t->sample_rate;

        t->undo_buffer = tmp_data;
        t->undo_frames = tmp_frames;
        t->has_undo = 0; /* Only one level of undo */

        /* Reset markers */
        t->start_sample = 0;
        t->end_sample = t->audio_frames;
        t->zoom_level = 0;
        t->zoom_center = t->audio_frames / 2;
        t->playing = 0;

        /* Recompute */
        compute_waveform(t, 128);
        t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);

        plugin_log("Undo applied");
        return;
    }

    if (strcmp(param, "save") == 0) {
        if (!t->audio_data || t->audio_frames <= 0) {
            plugin_log("Save: no audio data");
            return;
        }
        if (t->file_path[0] == '\0') {
            plugin_log("Save: no file path");
            return;
        }

        if (write_wav(t->file_path, t->audio_data, t->audio_frames,
                      t->sample_rate, t->orig_format, t->orig_bits,
                      t->orig_channels) == 0) {
            t->dirty = 0;
            plugin_log("File saved");
        } else {
            plugin_log("Save failed");
        }
        return;
    }

    if (strcmp(param, "save_as") == 0) {
        /* Save full audio to a specified path in original format */
        if (!t->audio_data || t->audio_frames <= 0 || !val || val[0] == '\0') {
            plugin_log("save_as: no audio or no path");
            return;
        }
        if (write_wav(val, t->audio_data, t->audio_frames,
                      t->sample_rate, t->orig_format, t->orig_bits,
                      t->orig_channels) == 0) {
            char log_buf[256];
            snprintf(log_buf, sizeof(log_buf), "Saved copy to: %s", val);
            plugin_log(log_buf);
        } else {
            plugin_log("save_as failed");
        }
        return;
    }

    if (strcmp(param, "dirty") == 0) {
        t->dirty = (val && val[0] == '1') ? 1 : 0;
        return;
    }

    if (strcmp(param, "slice_state") == 0) {
        /* Store UI slice state JSON for reconnect persistence */
        if (val) {
            strncpy(t->slice_state, val, sizeof(t->slice_state) - 1);
            t->slice_state[sizeof(t->slice_state) - 1] = '\0';
        } else {
            t->slice_state[0] = '\0';
        }
        return;
    }

    if (strcmp(param, "scene_state") == 0) {
        /* Store UI scene state JSON for reconnect persistence */
        if (val) {
            strncpy(t->scene_state, val, sizeof(t->scene_state) - 1);
            t->scene_state[sizeof(t->scene_state) - 1] = '\0';
        } else {
            t->scene_state[0] = '\0';
        }
        return;
    }

    {
        char log_buf[256];
        snprintf(log_buf, sizeof(log_buf),
                 "Unknown param key: %s", key);
        plugin_log(log_buf);
    }
}

/* ---- get_param ---- */

static int v2_get_param(void *instance, const char *key, char *buf,
                        int buf_len) {
    if (!instance || !key || !buf || buf_len <= 0) return -1;
    instance_t *inst = (instance_t *)instance;

    /* --- Global params --- */

    if (strcmp(key, "active_track") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", inst->active_track);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(key, "sync_clock") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", inst->sync_to_clock);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(key, "beats_per_bar") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", inst->beats_per_bar);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(key, "project_bpm") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%.1f", inst->project_bpm);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(key, "clock_running") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", inst->clock_running);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(key, "project_dir") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%s", inst->project_dir);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(key, "track_count") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", NUM_TRACKS);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(key, "skipback_result") == 0) {
        /* Returns the path of the last written skipback WAV, then clears it */
        int n = snprintf(buf, (size_t)buf_len, "%s", inst->skipback_result);
        inst->skipback_result[0] = '\0';  /* consume once */
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    /* --- Route to track --- */
    const char *param;
    int track_idx = parse_track_prefix(key, &param);
    if (track_idx < 0) track_idx = inst->active_track;
    if (track_idx < 0 || track_idx >= NUM_TRACKS) return -1;

    track_t *t = &inst->tracks[track_idx];

    /* --- File info --- */
    if (strcmp(param, "file_info") == 0) {
        if (!t->audio_data || t->audio_frames <= 0) {
            int n = snprintf(buf, (size_t)buf_len,
                             "{\"name\":\"\",\"duration\":0,"
                             "\"frames\":0,\"start\":0,\"end\":0}");
            return (n >= 0 && n < buf_len) ? n : -1;
        }
        char escaped_name[256];
        json_escape(escaped_name, sizeof(escaped_name), t->file_name);
        int n = snprintf(buf, (size_t)buf_len,
                         "{\"name\":\"%s\",\"duration\":%.2f,"
                         "\"frames\":%d,\"start\":%d,\"end\":%d}",
                         escaped_name,
                         t->duration_secs,
                         t->audio_frames,
                         t->start_sample,
                         t->end_sample);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    /* --- Waveform data --- */
    if (strncmp(param, "waveform", 8) == 0) {
        if (!t->audio_data || !t->waveform_valid) {
            int n = snprintf(buf, (size_t)buf_len, "[]");
            return (n >= 0 && n < buf_len) ? n : -1;
        }

        int16_t col_min[WAVEFORM_GET_COLS], col_max[WAVEFORM_GET_COLS];
        int vis_start = 0, vis_end = t->audio_frames;
        if (param[8] == ':') {
            sscanf(param + 9, "%d,%d", &vis_start, &vis_end);
        }
        /* For small visible ranges, raw scan gives better per-pixel resolution.
         * For large ranges (whole-file views), use precomputed to avoid stalls. */
        int vis_range = vis_end - vis_start;
        if (vis_range < 0) vis_range = 0;
        if (!t->waveform_valid || vis_range <= RAW_SCAN_THRESHOLD) {
            compute_waveform_raw_range(t, col_min, col_max, WAVEFORM_GET_COLS,
                                      vis_start, vis_end);
        } else {
            resample_waveform_range(t, col_min, col_max, WAVEFORM_GET_COLS,
                                    vis_start, vis_end);
        }

        /* Build JSON array: [[min,max],[min,max],...] */
        int pos = 0;
        int remaining = buf_len - 1;

        if (remaining < 1) return -1;
        buf[pos++] = '[';
        remaining--;

        for (int i = 0; i < WAVEFORM_GET_COLS; i++) {
            float mn = (float)col_min[i] / 32768.0f;
            float mx = (float)col_max[i] / 32768.0f;

            /* Clamp to -1..1 */
            if (mn < -1.0f) mn = -1.0f;
            if (mn > 1.0f) mn = 1.0f;
            if (mx < -1.0f) mx = -1.0f;
            if (mx > 1.0f) mx = 1.0f;

            int n = snprintf(buf + pos, (size_t)remaining,
                             "%s[%.4f,%.4f]",
                             (i > 0) ? "," : "",
                             mn, mx);
            if (n < 0 || n >= remaining) {
                /* Buffer full - truncate gracefully */
                buf[pos] = ']';
                buf[pos + 1] = '\0';
                return pos + 1;
            }
            pos += n;
            remaining -= n;
        }

        if (remaining >= 1) {
            buf[pos++] = ']';
        }
        buf[pos] = '\0';
        return pos;
    }

    /* --- Seam waveform (loop junction view) --- */
    if (strncmp(param, "seam_waveform:", 14) == 0) {
        int sw_start = 0, sw_end = 0, half_samples = 0;
        if (sscanf(param + 14, "%d,%d,%d", &sw_start, &sw_end, &half_samples) != 3
            || half_samples <= 0) {
            int n = snprintf(buf, (size_t)buf_len, "[]");
            return (n >= 0 && n < buf_len) ? n : -1;
        }
        if (!t->audio_data || t->audio_frames <= 0) {
            int n = snprintf(buf, (size_t)buf_len, "[]");
            return (n >= 0 && n < buf_len) ? n : -1;
        }

        /* Clamp */
        if (sw_start < 0) sw_start = 0;
        if (sw_end > t->audio_frames) sw_end = t->audio_frames;
        if (sw_start > t->audio_frames) sw_start = t->audio_frames;
        if (sw_end < 0) sw_end = 0;

        /* First half: approaching loop end (end - half_samples .. end) */
        int left_start = sw_end - half_samples;
        if (left_start < 0) left_start = 0;
        int left_end = sw_end;
        if (left_end > t->audio_frames) left_end = t->audio_frames;

        /* Build seam waveform from precomputed full-file data — no raw scan */
        int16_t seam_min[128], seam_max[128];
        resample_waveform_range(t, seam_min, seam_max, 64, left_start, left_end);

        /* Second half: continuing from loop start (start .. start + half_samples) */
        int right_start = sw_start;
        int right_end = sw_start + half_samples;
        if (right_end > t->audio_frames) right_end = t->audio_frames;

        resample_waveform_range(t, seam_min + 64, seam_max + 64,
                                64, right_start, right_end);

        /* Build JSON array: [[min,max],[min,max],...] — 128 pairs */
        int pos = 0;
        int remaining = buf_len - 1;
        if (remaining < 1) return -1;
        buf[pos++] = '[';
        remaining--;

        for (int i = 0; i < 128; i++) {
            float mn = (float)seam_min[i] / 32768.0f;
            float mx = (float)seam_max[i] / 32768.0f;
            if (mn < -1.0f) mn = -1.0f;
            if (mn > 1.0f) mn = 1.0f;
            if (mx < -1.0f) mx = -1.0f;
            if (mx > 1.0f) mx = 1.0f;

            int n = snprintf(buf + pos, (size_t)remaining,
                             "%s[%.4f,%.4f]",
                             (i > 0) ? "," : "",
                             mn, mx);
            if (n < 0 || n >= remaining) {
                buf[pos] = ']';
                buf[pos + 1] = '\0';
                return pos + 1;
            }
            pos += n;
            remaining -= n;
        }

        if (remaining >= 1) {
            buf[pos++] = ']';
        }
        buf[pos] = '\0';
        return pos;
    }

    /* --- Integer params as string --- */
    if (strcmp(param, "start_sample") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->start_sample);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "end_sample") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->end_sample);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "play_pos") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->play_pos);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "playing") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->playing ? 1 : 0);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "has_undo") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->has_undo ? 1 : 0);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "has_clipboard") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d",
                         (t->clipboard_data && t->clipboard_frames > 0) ? 1 : 0);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "dirty") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->dirty ? 1 : 0);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "slice_state") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%s", t->slice_state);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "scene_state") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%s", t->scene_state);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "psx_cpu") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%.1f", t->psx.cpu_percent);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "peak_db") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%.1f", t->peak_db);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "gain_db") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%.1f", t->gain_db);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "play_loop") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->play_loop ? 1 : 0);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "play_whole") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d",
                         t->play_whole ? 1 : 0);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "copy_result") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%s", t->copy_result);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "mode") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->mode);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "audio_frames") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->audio_frames);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "zoom_level") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->zoom_level);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "zoom_center") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->zoom_center);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "sample_rate") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->sample_rate);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "gate_mode") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->gate_mode);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "muted") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->muted);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "pitch") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%.2f", t->pitch_semitones);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    if (strcmp(param, "tempo") == 0) {
        int n = snprintf(buf, (size_t)buf_len, "%d", t->tempo_percent);
        return (n >= 0 && n < buf_len) ? n : -1;
    }

    return -1; /* Unknown parameter */
}

/* ---- get_error ---- */

static int v2_get_error(void *instance, char *buf, int buf_len) {
    if (!instance || !buf || buf_len <= 0) return 0;
    instance_t *inst = (instance_t *)instance;

    /* Report error from active track */
    track_t *t = &inst->tracks[inst->active_track];
    if (t->load_error[0] == '\0') return 0;

    int n = snprintf(buf, (size_t)buf_len, "%s", t->load_error);
    return (n >= 0 && n < buf_len) ? n : 0;
}

/* ---- pitch/tempo helpers ---- */

static void recompute_ratios(track_t *t) {
    t->pitch_ratio = pow(2.0, (double)t->pitch_semitones / 12.0);
    t->tempo_ratio = (double)t->tempo_percent / 100.0;
}

/* ---- Bungee time-stretch helpers ---- */

/* Feed a grain to the stretcher from int16 audio_data (converts to float).
 * Handles out-of-bounds frames via muteHead/muteTail. */
static void bng_feed_grain(track_t *t, const Bungee::InputChunk &chunk) {
    int len = chunk.end - chunk.begin;
    int stride = t->bng_max_grain;

    memset(t->bng_grain_input, 0, stride * 2 * sizeof(float));

    for (int i = 0; i < len; i++) {
        int src = chunk.begin + i;
        if (src >= 0 && src < t->audio_frames) {
            t->bng_grain_input[i]          = (float)t->audio_data[src * 2 + 0] / 32768.0f;
            t->bng_grain_input[stride + i] = (float)t->audio_data[src * 2 + 1] / 32768.0f;
        }
    }

    int muteHead = std::max(0, -chunk.begin);
    int muteTail = std::max(0, chunk.end - t->audio_frames);

    t->stretcher->analyseGrain(t->bng_grain_input, stride, muteHead, muteTail);
}

/* Reset Bungee stretcher for playback start or loop wrap. */
static void bng_reset_stretcher(track_t *t, double position) {
    if (!t->stretcher) return;
    t->bng_req.position = position;
    t->bng_req.speed = (t->tempo_speed_precise > 0.0)
                        ? t->tempo_speed_precise
                        : (double)t->tempo_percent / 100.0;
    t->bng_req.pitch = pow(2.0, (double)t->pitch_semitones / 12.0);
    t->bng_req.reset = true;
    t->bng_req.resampleMode = resampleMode_autoOut;
    t->stretcher->preroll(t->bng_req);
}

/*
 * Destructively apply current pitch/tempo settings to the selected region
 * [start_sample, end_sample]. Audio outside the region is preserved unchanged.
 * After processing, pitch and tempo are reset to defaults.
 */
static void do_apply_pitch_tempo(track_t *t) {
    if (!t->audio_data || t->audio_frames <= 0) return;
    if (!t->stretcher) return;

    int start = t->start_sample;
    int end   = t->end_sample;
    if (start < 0) start = 0;
    if (end > t->audio_frames) end = t->audio_frames;
    if (start >= end) {
        plugin_log("apply_pitch_tempo: invalid selection");
        return;
    }

    if (fabsf(t->pitch_ratio - 1.0f) < 0.001f &&
        fabsf(t->tempo_ratio  - 1.0f) < 0.001f) {
        plugin_log("apply_pitch_tempo: pitch/tempo already at default, nothing to do");
        return;
    }

    save_undo(t);

    /* Save play state so we can resume after buffer swap */
    int was_playing = t->playing;
    double old_play_pos_frac = t->play_pos_frac;

    /* Stop playback during buffer swap to prevent render_track
     * from accessing freed memory. */
    t->playing = 0;
    t->bng_out_count = 0;

    int region_frames = end - start;

    /* Offline stretcher — separate instance so we don't disrupt playback state.
     * Use the file's actual sample rate as input rate for correct conversion. */
    Bungee::Stretcher<Bungee::Basic> *str =
        new Bungee::Stretcher<Bungee::Basic>(
            Bungee::SampleRates{t->sample_rate, MOVE_SAMPLE_RATE}, 2, 0);
    int max_grain = str->maxInputFrameCount();

    float *grain_buf = (float *)calloc(max_grain * 2, sizeof(float));
    if (!grain_buf) {
        delete str;
        plugin_log("apply_pitch_tempo: grain buffer alloc failed");
        return;
    }

    /* Estimate output capacity (tempo < 1 → longer output) */
    int out_capacity = (int)((double)region_frames / t->tempo_ratio) + 8192;
    float *out_float = (float *)calloc(out_capacity * 2, sizeof(float));
    if (!out_float) {
        free(grain_buf);
        delete str;
        plugin_log("apply_pitch_tempo: output buffer alloc failed");
        return;
    }
    int out_count = 0;

    /* Initialise request */
    Bungee::Request req;
    req.position     = (double)start;
    req.speed        = (double)t->tempo_percent / 100.0;
    req.pitch        = pow(2.0, (double)t->pitch_semitones / 12.0);
    req.reset        = true;
    req.resampleMode = resampleMode_autoOut;
    str->preroll(req);

    /* Safety limit: generous upper bound on grain iterations */
    int safety = region_frames * 8 + 16384;

    while (req.position < (double)end && safety-- > 0) {
        Bungee::InputChunk chunk = str->specifyGrain(req);
        int len = chunk.end - chunk.begin;

        memset(grain_buf, 0, max_grain * 2 * sizeof(float));
        for (int i = 0; i < len; i++) {
            int src = chunk.begin + i;
            if (src >= 0 && src < t->audio_frames) {
                grain_buf[i]             = (float)t->audio_data[src * 2 + 0] / 32768.0f;
                grain_buf[max_grain + i] = (float)t->audio_data[src * 2 + 1] / 32768.0f;
            }
        }
        /* Mute grain samples that fall outside [start, end] */
        int muteHead = std::max(0, start - chunk.begin);
        int muteTail = std::max(0, chunk.end - end);
        str->analyseGrain(grain_buf, max_grain, muteHead, muteTail);

        Bungee::OutputChunk output{};
        str->synthesiseGrain(output);

        /* Grow output buffer if needed */
        if (out_count + output.frameCount > out_capacity) {
            out_capacity = (out_count + output.frameCount) * 2;
            float *nb = (float *)realloc(out_float, out_capacity * 2 * sizeof(float));
            if (!nb) {
                free(out_float);
                free(grain_buf);
                delete str;
                plugin_log("apply_pitch_tempo: realloc failed");
                return;
            }
            out_float = nb;
        }

        for (int i = 0; i < output.frameCount; i++) {
            out_float[(out_count + i) * 2 + 0] = output.data[i];
            out_float[(out_count + i) * 2 + 1] = output.data[i + output.channelStride];
        }
        out_count += output.frameCount;

        str->next(req);
        req.reset = false;
    }

    free(grain_buf);
    delete str;

    /* Build new audio buffer: prefix + processed region + suffix */
    int prefix_frames = start;
    int suffix_frames = t->audio_frames - end;
    int new_total     = prefix_frames + out_count + suffix_frames;

    int16_t *new_data = (int16_t *)malloc((size_t)new_total * SAMPLES_PER_FRAME * sizeof(int16_t));
    if (!new_data) {
        free(out_float);
        plugin_log("apply_pitch_tempo: new audio buffer alloc failed");
        return;
    }

    if (prefix_frames > 0)
        memcpy(new_data, t->audio_data, (size_t)prefix_frames * SAMPLES_PER_FRAME * sizeof(int16_t));

    for (int i = 0; i < out_count; i++) {
        float L = out_float[i * 2 + 0] * 32768.0f;
        float R = out_float[i * 2 + 1] * 32768.0f;
        if (L >  32767.0f) L =  32767.0f;
        if (L < -32768.0f) L = -32768.0f;
        if (R >  32767.0f) R =  32767.0f;
        if (R < -32768.0f) R = -32768.0f;
        new_data[(prefix_frames + i) * 2 + 0] = (int16_t)L;
        new_data[(prefix_frames + i) * 2 + 1] = (int16_t)R;
    }

    if (suffix_frames > 0)
        memcpy(new_data + (prefix_frames + out_count) * SAMPLES_PER_FRAME,
               t->audio_data + end * SAMPLES_PER_FRAME,
               (size_t)suffix_frames * SAMPLES_PER_FRAME * sizeof(int16_t));

    free(out_float);
    free(t->audio_data);
    t->audio_data    = new_data;
    t->audio_frames  = new_total;
    t->total_frames  = new_total;
    t->duration_secs = (float)new_total / (float)t->sample_rate;

    /* Selection covers the processed region in the new buffer */
    t->start_sample = prefix_frames;
    t->end_sample   = prefix_frames + out_count;

    /* The stretcher output is at MOVE_SAMPLE_RATE (44100).  Update sample_rate
     * so that render_track's rate_inc = 1.0 and export writes the correct header. */
    t->sample_rate = MOVE_SAMPLE_RATE;
    t->duration_secs = (float)new_total / (float)MOVE_SAMPLE_RATE;

    /* Reset pitch/tempo to defaults */
    t->pitch_semitones = 0.0f;
    t->tempo_percent   = 100;
    recompute_ratios(t);
    /* Sync Bungee playback request too */
    t->bng_req.pitch = 1.0;
    t->bng_req.speed = 1.0;
    /* Rebuild stretcher for new sample rate */
    if (t->stretcher) {
        delete t->stretcher;
        t->stretcher = new Bungee::Stretcher<Bungee::Basic>(
            Bungee::SampleRates{MOVE_SAMPLE_RATE, MOVE_SAMPLE_RATE}, 2, 0);
        t->bng_max_grain = t->stretcher->maxInputFrameCount();
        free(t->bng_grain_input);
        t->bng_grain_input = (float *)calloc(t->bng_max_grain * 2, sizeof(float));
    }

    t->dirty = 1;
    compute_waveform(t, 128);
    t->peak_db = compute_peak_db(t->audio_data, t->audio_frames);

    /* Resume playback with mapped position */
    if (was_playing) {
        double new_pos;
        if (old_play_pos_frac < (double)start) {
            /* Before processed region — unchanged */
            new_pos = old_play_pos_frac;
        } else if (old_play_pos_frac < (double)end) {
            /* Inside processed region — scale proportionally */
            double frac = (old_play_pos_frac - (double)start) / (double)region_frames;
            new_pos = (double)prefix_frames + frac * (double)out_count;
        } else {
            /* After processed region — shift by size delta */
            new_pos = old_play_pos_frac + (double)(out_count - region_frames);
        }
        if (new_pos < 0) new_pos = 0;
        if (new_pos >= (double)new_total) new_pos = 0;
        t->play_pos_frac = new_pos;
        t->play_pos = (int)new_pos;
        t->psx.read_pos = new_pos;
        t->playing = 1;
        if (t->psx.enabled) { t->psx.needs_reinit = 1; }
    }

    {
        char log_buf[128];
        snprintf(log_buf, sizeof(log_buf),
                 "apply_pitch_tempo: region [%d,%d] -> %d frames (was %d)",
                 start, end, out_count, region_frames);
        plugin_log(log_buf);
    }
}

/* ---- render_block ---- */

/*
 * Render one track into a temporary buffer.
 * Returns 1 if audio was produced, 0 if silent.
 */
static int render_track(track_t *t, int16_t *out, int frames) {
    if (!t->playing || !t->audio_data || t->audio_frames <= 0) return 0;
    if (t->gate_mode == 1 && !t->gate_held) return 0;  /* gate: only play while held */

    /* PaulXStretch: bypass normal playback — PSX reads directly from audio_data
     * at its own pace, controlled by the spectral stretcher's state machine. */
    if (t->psx.enabled) {
        /* Lazy reinit must happen here, before the NULL-pointer guard,
         * so stretchers get allocated on the first render after enable. */
        if (t->psx.needs_reinit) {
            ps_reinit_track(t);
        }
        if (!t->psx.stretch_l || !t->psx.stretch_r) goto normal_playback;
        {
            struct timespec _t0, _t1;
            clock_gettime(CLOCK_MONOTONIC, &_t0);
            ps_process_track(t, out, frames);
            clock_gettime(CLOCK_MONOTONIC, &_t1);
            float elapsed_us = (float)(_t1.tv_sec - _t0.tv_sec) * 1e6f
                             + (float)(_t1.tv_nsec - _t0.tv_nsec) / 1e3f;
            float budget_us = (float)frames / (float)MOVE_SAMPLE_RATE * 1e6f;
            float pct = (budget_us > 0.0f) ? (elapsed_us / budget_us * 100.0f) : 0.0f;
            /* EMA smoothing: ~100 blocks → ~0.3s time constant */
            t->psx.cpu_percent = t->psx.cpu_percent * 0.97f + pct * 0.03f;
        }
        if (t->muted) {
            memset(out, 0, (size_t)frames * 2 * sizeof(int16_t));
            return 0;
        }
        return 1;
    }

normal_playback:
    /* Determine playback boundaries */
    int play_start, play_end;
    if (t->play_whole) {
        play_start = 0;
        play_end = t->audio_frames;
    } else {
        play_start = t->start_sample;
        play_end = t->end_sample;
        if (play_start < 0) play_start = 0;
        if (play_end > t->audio_frames) play_end = t->audio_frames;
        if (play_start >= play_end) return 0;
    }

    /* Precompute gain for preview */
    float linear_gain = 1.0f;
    if (t->gain_db != 0.0f) {
        linear_gain = powf(10.0f, t->gain_db / 20.0f);
    }

    /* Varispeed mode: skip Bungee entirely — tempo+pitch both applied via rate_inc.
     * Stretch mode: use Bungee when pitch or tempo differ from default. */
    int use_bungee;
    if (t->play_mode == 1) {
        use_bungee = 0; /* varispeed: always direct path */
    } else {
        use_bungee = !(fabsf(t->pitch_ratio - 1.0f) < 0.001f &&
                       fabsf(t->tempo_ratio - 1.0f) < 0.001f);
    }

    /* Sample-rate conversion ratio: advance faster for files with higher
     * sample rates (e.g. 48000/44100 ≈ 1.0884 for 48 kHz files).
     * In varispeed mode, tempo_ratio and pitch_ratio are baked in here. */
    double rate_inc = (double)t->sample_rate / (double)MOVE_SAMPLE_RATE;
    if (t->play_mode == 1) {
        rate_inc *= (double)t->tempo_ratio * (double)t->pitch_ratio;
    }

    int region_len = play_end - play_start;
    int produced = 0;

    /* Exact loop endpoint: when loop_len_exact is set, the musical loop length
     * may be fractionally longer or shorter than the integer region_len.
     * Use the exact endpoint for wrap checks to avoid drift and prevent
     * infinite-reset loops when loop_len_exact > region_len. */
    double loop_end = (t->loop_len_exact > 0.0)
                      ? (double)play_start + t->loop_len_exact
                      : (double)play_end;

    if (!use_bungee) {
        /* Direct playback path (no pitch/tempo change) */
        for (int i = 0; i < frames; i++) {
            if (!t->playing) {
                out[i * 2]     = 0;
                out[i * 2 + 1] = 0;
                continue;
            }

            while (t->play_pos_frac >= loop_end) {
                if (t->play_loop && region_len > 0) {
                    t->play_pos_frac -= (loop_end - (double)play_start);
                } else {
                    t->playing = 0;
                    break;
                }
            }
            if (!t->playing) {
                out[i * 2]     = 0;
                out[i * 2 + 1] = 0;
                t->play_pos = (int)t->play_pos_frac;
                continue;
            }

            int pos_int = (int)t->play_pos_frac;
            if (pos_int >= play_end) pos_int = play_end - 1; /* clamp for exact loop overshoot */
            float frac = (float)(t->play_pos_frac - (double)pos_int);
            int next = pos_int + 1;
            if (next >= play_end) {
                next = t->play_loop ? play_start : pos_int;
            }

            float sample_l = (float)t->audio_data[pos_int * 2]
                            + frac * ((float)t->audio_data[next * 2] - (float)t->audio_data[pos_int * 2]);
            float sample_r = (float)t->audio_data[pos_int * 2 + 1]
                            + frac * ((float)t->audio_data[next * 2 + 1] - (float)t->audio_data[pos_int * 2 + 1]);

            t->play_pos_frac += rate_inc;
            t->play_pos = (int)t->play_pos_frac;
            produced = 1;

            if (linear_gain != 1.0f) {
                sample_l *= linear_gain;
                sample_r *= linear_gain;
            }
            if (sample_l > 32767.0f) sample_l = 32767.0f;
            if (sample_l < -32768.0f) sample_l = -32768.0f;
            if (sample_r > 32767.0f) sample_r = 32767.0f;
            if (sample_r < -32768.0f) sample_r = -32768.0f;

            out[i * 2]     = (int16_t)sample_l;
            out[i * 2 + 1] = (int16_t)sample_r;
        }
    } else if (t->stretcher) {
        /* Bungee time-stretch path */

        /* Fill output accumulator from Bungee grains until we have enough */
        int safety = 256;
        while (t->bng_out_count < frames && safety-- > 0) {
            /* Handle loop wrap */
            if (t->bng_req.position >= loop_end) {
                if (t->play_loop && region_len > 0) {
                    double wrap_len = loop_end - (double)play_start;
                    double wrapped = (double)play_start +
                        fmod(t->bng_req.position - (double)play_start, wrap_len);
                    if (wrapped < (double)play_start) wrapped = (double)play_start;
                    /* Fade out the tail of any buffered pre-wrap output so the
                     * old audio smoothly decays to silence.  The new position's
                     * first grain is pre-rolled by bng_reset_stretcher (preroll),
                     * which naturally fades in via Bungee's analysis window —
                     * together this forms a click-free crossfade at the loop point. */
                    {
                        int fade_len = t->bng_out_count < BNG_LOOP_FADE_SAMPLES
                                       ? t->bng_out_count : BNG_LOOP_FADE_SAMPLES;
                        int fade_start = t->bng_out_count - fade_len;
                        for (int _fi = 0; _fi < fade_len; _fi++) {
                            float _ramp = 1.0f - (float)(_fi + 1) / (float)(fade_len + 1);
                            t->bng_out_buf[(fade_start + _fi) * 2 + 0] *= _ramp;
                            t->bng_out_buf[(fade_start + _fi) * 2 + 1] *= _ramp;
                        }
                    }
                    bng_reset_stretcher(t, wrapped);
                } else {
                    t->playing = 0;
                    break;
                }
            }

            Bungee::InputChunk chunk = t->stretcher->specifyGrain(t->bng_req);
            bng_feed_grain(t, chunk);

            Bungee::OutputChunk output{};
            t->stretcher->synthesiseGrain(output);

            /* Append output to accumulator */
            int space = BNG_OUT_BUF_CAPACITY - t->bng_out_count;
            int toCopy = std::min(output.frameCount, space);
            for (int i = 0; i < toCopy; i++) {
                int idx = (t->bng_out_count + i) * 2;
                t->bng_out_buf[idx + 0] = output.data[i];
                t->bng_out_buf[idx + 1] = output.data[i + output.channelStride];
            }
            t->bng_out_count += toCopy;

            t->stretcher->next(t->bng_req);
            t->bng_req.reset = false;
        }

        /* Drain frames from accumulator to output */
        int avail = std::min(t->bng_out_count, frames);
        for (int i = 0; i < avail; i++) {
            float L = t->bng_out_buf[i * 2 + 0] * 32768.0f;
            float R = t->bng_out_buf[i * 2 + 1] * 32768.0f;

            if (linear_gain != 1.0f) {
                L *= linear_gain;
                R *= linear_gain;
            }
            if (L > 32767.0f) L = 32767.0f;
            if (L < -32768.0f) L = -32768.0f;
            if (R > 32767.0f) R = 32767.0f;
            if (R < -32768.0f) R = -32768.0f;

            out[i * 2 + 0] = (int16_t)L;
            out[i * 2 + 1] = (int16_t)R;
        }

        /* Zero-fill if we ran short */
        for (int i = avail; i < frames; i++) {
            out[i * 2 + 0] = 0;
            out[i * 2 + 1] = 0;
        }

        /* Shift remaining frames to front of accumulator */
        int remaining = t->bng_out_count - avail;
        if (remaining > 0) {
            memmove(t->bng_out_buf, t->bng_out_buf + avail * 2,
                    remaining * 2 * sizeof(float));
        }
        t->bng_out_count = remaining;

        /* Update play_pos for UI cursor from Bungee position */
        t->play_pos = (int)t->bng_req.position;
        if (t->play_pos < 0) t->play_pos = 0;
        if (t->play_pos >= t->audio_frames) t->play_pos = t->audio_frames - 1;
        t->play_pos_frac = t->bng_req.position;

        produced = (avail > 0) ? 1 : 0;
    }

    /* Muted: position has already advanced (for re-sync on unmute), but
     * silence the output so the mix receives nothing. */
    if (t->muted) {
        memset(out, 0, (size_t)frames * 2 * sizeof(int16_t));
        return 0;
    }
    return produced;
}

/*
 * Multi-track render: mix all active tracks into output buffer.
 */
static void v2_render_block(void *instance, int16_t *out_interleaved_lr,
                            int frames) {
    if (!instance || !out_interleaved_lr) return;
    instance_t *inst = (instance_t *)instance;

    /* Clear output */
    memset(out_interleaved_lr, 0, (size_t)frames * 2 * sizeof(int16_t));

    /* -----------------------------------------------------------------------
     * Clock sync: compute bar triggers for this block.
     *
     * SYNC_CLOCK path: drain pending_bar_triggers queued by v2_on_midi.
     * Internal path: count samples against samples_per_bar_f when the MIDI
     * clock is not running (no Start received yet, or sync_to_clock off).
     * ----------------------------------------------------------------------- */
    if (inst->sync_to_clock) {
        int bar_triggers = 0;
        int sr = (g_host && g_host->sample_rate > 0) ? g_host->sample_rate : MOVE_SAMPLE_RATE;

        if (inst->clock_running) {
            /* Drain MIDI-clock triggered bar starts */
            bar_triggers = inst->pending_bar_triggers;
            inst->pending_bar_triggers = 0;
        } else {
            /* Internal BPM fallback — count samples, fire on bar boundary */
            if (inst->timing_dirty || inst->cached_sample_rate != sr)
                recalc_timing(inst, sr);
            if (inst->samples_per_bar_f > 0.0) {
                inst->samples_until_bar_f -= (double)frames;
                while (inst->samples_until_bar_f <= 0.0) {
                    bar_triggers++;
                    inst->samples_until_bar_f += inst->samples_per_bar_f;
                    if (inst->samples_until_bar_f < 1.0) inst->samples_until_bar_f = 1.0;
                }
            }
        }

        /* Launch queued tracks on each bar trigger */
        if (bar_triggers > 0) {
            for (int ti = 0; ti < NUM_TRACKS; ti++) {
                track_t *t = &inst->tracks[ti];
                if (!t->queued_play) continue;
                if (!t->audio_data || t->audio_frames <= 0) { t->queued_play = 0; continue; }
                t->queued_play = 0;
                t->play_whole = t->queued_whole;
                t->playing = 1;
                if (t->queued_whole) {
                    t->play_pos = 0;
                    t->play_pos_frac = 0.0;
                    t->psx.read_pos = 0.0;
                    t->bng_play_start = 0;
                    t->bng_play_end = t->audio_frames;
                } else {
                    t->play_pos = t->start_sample;
                    t->play_pos_frac = (double)t->start_sample;
                    t->psx.read_pos = (double)t->start_sample;
                    t->bng_play_start = t->start_sample;
                    t->bng_play_end = (t->sync_play_end > 0) ? t->sync_play_end : t->end_sample;
                }
                t->bng_out_count = 0;
                bng_reset_stretcher(t, t->play_pos_frac);
                if (t->psx.enabled) { t->psx.needs_reinit = 1; }
            }
        }
    }

    /* Temporary buffer for each track */
    int16_t tmp[MOVE_FRAMES_PER_BLOCK * 2];
    int any_audio = 0;

    for (int ti = 0; ti < NUM_TRACKS; ti++) {
        track_t *t = &inst->tracks[ti];

        /* Always render to tmp (silence if not playing) so ring buffer stays in sync */
        int produced = render_track(t, tmp, frames);
        if (produced) any_audio = 1;

        /* Apply stereo pan (balance law: center = both at 1.0, hard pan cuts one side) */
        if (produced && t->pan != 0.0f) {
            float pan_l = (t->pan <= 0.0f) ? 1.0f : 1.0f - t->pan;
            float pan_r = (t->pan >= 0.0f) ? 1.0f : 1.0f + t->pan;
            for (int f = 0; f < frames; f++) {
                float sl = (float)tmp[f * 2]     * pan_l;
                float sr = (float)tmp[f * 2 + 1] * pan_r;
                if (sl >  32767.0f) sl =  32767.0f;
                if (sl < -32768.0f) sl = -32768.0f;
                if (sr >  32767.0f) sr =  32767.0f;
                if (sr < -32768.0f) sr = -32768.0f;
                tmp[f * 2]     = (int16_t)sl;
                tmp[f * 2 + 1] = (int16_t)sr;
            }
        }

        /* Write per-track output to skipback ring buffer (silence if not playing) */
        if (inst->track_rb[ti]) {
            int wp = inst->track_rb_write[ti];
            for (int f = 0; f < frames; f++) {
                int ri = (wp + f) % RB_FRAMES;
                inst->track_rb[ti][ri * 2]     = produced ? tmp[f * 2]     : 0;
                inst->track_rb[ti][ri * 2 + 1] = produced ? tmp[f * 2 + 1] : 0;
            }
            inst->track_rb_write[ti] = (wp + frames) % RB_FRAMES;
        }

        if (!produced) continue;

        /* Mix into output with saturating addition */
        for (int i = 0; i < frames * 2; i++) {
            int32_t sum = (int32_t)out_interleaved_lr[i] + (int32_t)tmp[i];
            if (sum > 32767) sum = 32767;
            if (sum < -32768) sum = -32768;
            out_interleaved_lr[i] = (int16_t)sum;
        }
    }

    /* Apply master volume */
    if (inst->master_volume_db != 0.0f) {
        float master_gain = powf(10.0f, inst->master_volume_db / 20.0f);
        for (int i = 0; i < frames * 2; i++) {
            float s = (float)out_interleaved_lr[i] * master_gain;
            if (s >  32767.0f) s =  32767.0f;
            if (s < -32768.0f) s = -32768.0f;
            out_interleaved_lr[i] = (int16_t)s;
        }
    }

    /* Write master mix to skipback ring buffer */
    if (inst->master_rb) {
        int mwp = inst->master_rb_write;
        for (int f = 0; f < frames; f++) {
            int ri = (mwp + f) % RB_FRAMES;
            inst->master_rb[ri * 2]     = out_interleaved_lr[f * 2];
            inst->master_rb[ri * 2 + 1] = out_interleaved_lr[f * 2 + 1];
        }
        inst->master_rb_write = (mwp + frames) % RB_FRAMES;
    }

    (void)any_audio;
}

/* ============================================================================
 * V2 API registration
 * ============================================================================ */

static plugin_api_v2_t s_plugin_api = {
    .api_version = MOVE_PLUGIN_API_VERSION_2,
    .create_instance = v2_create,
    .destroy_instance = v2_destroy,
    .on_midi = v2_on_midi,
    .set_param = v2_set_param,
    .get_param = v2_get_param,
    .get_error = v2_get_error,
    .render_block = v2_render_block,
};

extern "C" __attribute__((visibility("default")))
void *move_plugin_init_v2(const host_api_v1_t *host_api) {
    if (!host_api) return NULL;
    g_host = host_api;
    plugin_log("Plugin initialized (V2 4-track stereo, Bungee)");
    return &s_plugin_api;
}
