// =============================================================================
//  audio_pipeline.h
// =============================================================================
#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ── Shared buffer sizes ──────────────────────────────────────────────────────
#define AUDIO_CHUNK_SAMPLES  2048   // stereo pairs per raw/resampled chunk
#define FFT_CHUNK_SAMPLES     512   // stereo pairs per FFT chunk (power of 2)

// ── Raw / resampled audio chunk (used by SD reader, resampler, BT) ───────────
struct AudioChunk {
    int16_t samples[AUDIO_CHUNK_SAMPLES * 2];  // L,R interleaved
    int     count;                              // valid stereo pairs
};

// ── FFT chunk — a COPY of audio data, owned entirely by the FFT consumer ─────
//  Sized to FFT_CHUNK_SAMPLES so it's much smaller than AudioChunk and
//  doesn't waste PSRAM. The resampler copies into this before handing it off.
struct FFTChunk {
    int16_t samples[FFT_CHUNK_SAMPLES * 2];   // L,R interleaved
    int     count;                             // valid stereo pairs
};

// ── Queue handles exposed to main.cpp ────────────────────────────────────────
extern QueueHandle_t g_fftQueue;      // resampler → FFT consumer (main loop)
extern QueueHandle_t g_fftFreePool;   // recycled FFTChunks

// ── Entry point ──────────────────────────────────────────────────────────────
void startAudioPipeline();
