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
#include <unistd.h>
#include <pwd.h>

#include <bungee/Bungee.h>

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

    int16_t waveform_min[MAX_WAVEFORM_COLS];
    int16_t waveform_max[MAX_WAVEFORM_COLS];
    int waveform_cols;

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

    int mode;                   /* 0=trim, 1=gain */

    int gate_mode;              /* 0=trigger (toggle), 1=gate (hold-to-play) */
    int gate_held;              /* 1 while gate is held (pad/key down) */
    int muted;                  /* 1=track muted (non-destructive, skips render) */

    float pitch_semitones;      /* -12.0 to +12.0 (fractional = cents) */
    int   tempo_percent;        /* 50 to 200 */
    float pitch_ratio;          /* cached: pow(2, pitch_semitones/12) */
    float tempo_ratio;          /* cached: tempo_percent / 100.0 */
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

    char copy_result[256];
    char load_error[256];
    char slice_state[2048];     /* JSON blob for UI slice state persistence */
    char scene_state[1024];     /* JSON blob for UI scene state persistence */
} track_t;

/* ============================================================================
 * Instance structure — container for all tracks
 * ============================================================================ */

typedef struct {
    track_t tracks[NUM_TRACKS];
    int active_track;           /* 0-3, which track is being edited */
    int sync_to_clock;          /* Global clock sync flag */
    char module_dir[512];

    /* Skipback ring buffers — filled continuously during render_block */
    int16_t *master_rb;              /* stereo circular buffer of master mix */
    int      master_rb_write;        /* write position in frames */
    int16_t *track_rb[NUM_TRACKS];   /* per-track stereo circular buffers */
    int      track_rb_write[NUM_TRACKS];
    char     skipback_result[512];   /* path of last successfully written skipback WAV */
} instance_t;

/* ============================================================================
 * Globals
 * ============================================================================ */

static const host_api_v1_t *g_host = NULL;

static void plugin_log(const char *msg) {
    if (g_host && g_host->log) {
        char buf[512];
        snprintf(buf, sizeof(buf), "[waveform-editor] %s", msg);
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

/*
 * Compute waveform overview (min/max per column) for an explicit sample range.
 * Uses L channel only for display.
 */
static void compute_waveform_range(track_t *t, int num_cols,
                                   int vis_start, int vis_end) {
    if (num_cols <= 0) num_cols = 1;
    if (num_cols > MAX_WAVEFORM_COLS) num_cols = MAX_WAVEFORM_COLS;

    t->waveform_cols = num_cols;

    if (!t->audio_data || t->audio_frames <= 0) {
        memset(t->waveform_min, 0, (size_t)num_cols * sizeof(int16_t));
        memset(t->waveform_max, 0, (size_t)num_cols * sizeof(int16_t));
        return;
    }

    /* Clamp range */
    if (vis_start < 0) vis_start = 0;
    if (vis_end > t->audio_frames) vis_end = t->audio_frames;
    int vis_frames = vis_end - vis_start;
    if (vis_frames <= 0) {
        memset(t->waveform_min, 0, (size_t)num_cols * sizeof(int16_t));
        memset(t->waveform_max, 0, (size_t)num_cols * sizeof(int16_t));
        return;
    }

    for (int col = 0; col < num_cols; col++) {
        int col_start = vis_start + (col * vis_frames) / num_cols;
        int col_end   = vis_start + ((col + 1) * vis_frames) / num_cols;
        if (col_end <= col_start) col_end = col_start + 1;
        if (col_end > vis_end) col_end = vis_end;

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
}

/*
 * Compute waveform overview using internal zoom state.
 */
static void compute_waveform(track_t *t, int num_cols) {
    int vis_start, vis_end;
    get_visible_range(t, &vis_start, &vis_end);
    compute_waveform_range(t, num_cols, vis_start, vis_end);
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
}

static void recompute_ratios(track_t *t);
static void bng_reset_stretcher(track_t *t, double position);
static void do_apply_pitch_tempo(track_t *t);

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
    /* Waveform editor does not process MIDI (yet — clock sync TODO) */
    (void)instance;
    (void)msg;
    (void)len;
    (void)source;
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
                    t->play_whole = 0;
                    t->playing = 1;
                    t->play_pos = t->start_sample;
                    t->play_pos_frac = (double)t->start_sample;
                    /* Reset Bungee state */
                    t->bng_out_count = 0;
                    t->bng_play_start = t->start_sample;
                    t->bng_play_end = t->end_sample;
                    bng_reset_stretcher(t, (double)t->start_sample);
                }
            } else {
                t->playing = 0;
            }
        }
        return;
    }

    if (strcmp(key, "sync_clock") == 0) {
        if (!val) return;
        inst->sync_to_clock = atoi(val) ? 1 : 0;
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
        if (sscanf(val, "%15[^,],%d,%511s", source, &extract_frames, path) != 3) return;
        if (extract_frames <= 0 || extract_frames > RB_FRAMES) extract_frames = RB_FRAMES;

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

    /* --- Route to track --- */
    const char *param;
    int track_idx = parse_track_prefix(key, &param);
    if (track_idx < 0) track_idx = inst->active_track;
    if (track_idx < 0 || track_idx >= NUM_TRACKS) return;

    track_t *t = &inst->tracks[track_idx];

    /* --- File loading --- */
    if (strcmp(param, "file_path") == 0) {
        if (!val || val[0] == '\0') return;

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
            return;
        }

        /* Value encodes play mode: "whole" = entire file, anything else = selection */
        int whole = (val && strcmp(val, "whole") == 0) ? 1 : 0;
        t->play_whole = whole;
        t->playing = 1;
        if (whole) {
            t->play_pos = 0;
            t->play_pos_frac = 0.0;
            t->bng_play_start = 0;
            t->bng_play_end = t->audio_frames;
            t->bng_out_count = 0;
            bng_reset_stretcher(t, 0.0);
        } else {
            t->play_pos = t->start_sample;
            t->play_pos_frac = (double)t->start_sample;
            t->bng_play_start = t->start_sample;
            t->bng_play_end = t->end_sample;
            t->bng_out_count = 0;
            bng_reset_stretcher(t, (double)t->start_sample);
        }
        {
            char dbg[256];
            snprintf(dbg, sizeof(dbg), "PLAY t%d: whole=%d start=%d end=%d pos=%d total=%d",
                     track_idx, whole, t->start_sample, t->end_sample,
                     t->play_pos, t->audio_frames);
            plugin_log(dbg);
        }
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
        /* Reset Bungee state */
        t->bng_play_start = t->start_sample;
        t->bng_play_end = t->end_sample;
        t->bng_out_count = 0;
        bng_reset_stretcher(t, (double)pos);
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
        t->gate_mode = atoi(val) ? 1 : 0;
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
        if (v < -12.0f) v = -12.0f;
        if (v > 12.0f) v = 12.0f;
        t->pitch_semitones = v;
        recompute_ratios(t);
        /* Sync Bungee state to current playback position so the transition
         * from direct path to Bungee path is seamless (no position jump,
         * correct sample-rate-aware resampling from the start). */
        if (t->playing) {
            t->bng_out_count = 0;
            bng_reset_stretcher(t, t->play_pos_frac);
        } else {
            t->bng_req.pitch = pow(2.0, (double)v / 12.0);
        }
        return;
    }

    if (strcmp(param, "tempo") == 0) {
        if (!val) return;
        int v = atoi(val);
        if (v < 50) v = 50;
        if (v > 200) v = 200;
        t->tempo_percent = v;
        recompute_ratios(t);
        /* Sync Bungee state to current playback position so the transition
         * from direct path to Bungee path is seamless. */
        if (t->playing) {
            t->bng_out_count = 0;
            bng_reset_stretcher(t, t->play_pos_frac);
        } else {
            t->bng_req.speed = (double)v / 100.0;
        }
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
            t->playing = 1;
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
        int vis_start = -1, vis_end = -1;
        if (param[8] == ':') {
            sscanf(param + 9, "%d,%d", &vis_start, &vis_end);
        }
        if (vis_start >= 0 && vis_end > vis_start) {
            compute_waveform_range(t, 128, vis_start, vis_end);
        } else {
            compute_waveform(t, 128);
        }

        if (t->waveform_cols <= 0 || !t->audio_data) {
            int n = snprintf(buf, (size_t)buf_len, "[]");
            return (n >= 0 && n < buf_len) ? n : -1;
        }

        /* Build JSON array: [[min,max],[min,max],...] */
        int pos = 0;
        int remaining = buf_len - 1;

        if (remaining < 1) return -1;
        buf[pos++] = '[';
        remaining--;

        for (int i = 0; i < t->waveform_cols; i++) {
            float mn = (float)t->waveform_min[i] / 32768.0f;
            float mx = (float)t->waveform_max[i] / 32768.0f;

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

        compute_waveform_range(t, 64, left_start, left_end);

        /* Save first-half results to stack before second call overwrites */
        int16_t seam_min[128], seam_max[128];
        for (int i = 0; i < 64; i++) {
            seam_min[i] = (i < t->waveform_cols) ? t->waveform_min[i] : 0;
            seam_max[i] = (i < t->waveform_cols) ? t->waveform_max[i] : 0;
        }

        /* Second half: continuing from loop start (start .. start + half_samples) */
        int right_start = sw_start;
        int right_end = sw_start + half_samples;
        if (right_end > t->audio_frames) right_end = t->audio_frames;

        compute_waveform_range(t, 64, right_start, right_end);

        for (int i = 0; i < 64; i++) {
            seam_min[64 + i] = (i < t->waveform_cols) ? t->waveform_min[i] : 0;
            seam_max[64 + i] = (i < t->waveform_cols) ? t->waveform_max[i] : 0;
        }

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
    t->pitch_ratio = powf(2.0f, t->pitch_semitones / 12.0f);
    t->tempo_ratio = (float)t->tempo_percent / 100.0f;
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
    t->bng_req.speed = (double)t->tempo_percent / 100.0;
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
        t->playing = 1;
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
    if (t->muted) return 0;
    if (t->gate_mode && !t->gate_held) return 0;

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

    /* Use Bungee when pitch or tempo differ from default */
    int use_bungee = !(fabsf(t->pitch_ratio - 1.0f) < 0.001f &&
                       fabsf(t->tempo_ratio - 1.0f) < 0.001f);

    /* Sample-rate conversion ratio: advance faster for files with higher
     * sample rates (e.g. 48000/44100 ≈ 1.0884 for 48 kHz files). */
    double rate_inc = (double)t->sample_rate / (double)MOVE_SAMPLE_RATE;

    int region_len = play_end - play_start;
    int produced = 0;

    if (!use_bungee) {
        /* Direct playback path (no pitch/tempo change) */
        for (int i = 0; i < frames; i++) {
            if (!t->playing) {
                out[i * 2]     = 0;
                out[i * 2 + 1] = 0;
                continue;
            }

            while (t->play_pos_frac >= (double)play_end) {
                if (t->play_loop && region_len > 0) {
                    t->play_pos_frac -= (double)region_len;
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
            if (t->bng_req.position >= (double)play_end) {
                if (t->play_loop && region_len > 0) {
                    double wrapped = (double)play_start +
                        fmod(t->bng_req.position - (double)play_start, (double)region_len);
                    if (wrapped < (double)play_start) wrapped = (double)play_start;
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

    /* Temporary buffer for each track */
    int16_t tmp[MOVE_FRAMES_PER_BLOCK * 2];
    int any_audio = 0;

    for (int ti = 0; ti < NUM_TRACKS; ti++) {
        track_t *t = &inst->tracks[ti];

        /* Always render to tmp (silence if not playing) so ring buffer stays in sync */
        int produced = render_track(t, tmp, frames);
        if (produced) any_audio = 1;

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
