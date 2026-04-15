// audio_pipeline.h
// Include this in ANY .cpp file that needs to talk to the audio pipeline.
// The #pragma once ensures it's only processed once per compilation.
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ─────────────────────────────────────────────────────────────────
//  AudioChunk — the shared data structure
//  Both audio_pipeline.cpp and your main.cpp need to agree on
//  what an AudioChunk looks like. Defining it here means both
//  files see exactly the same definition automatically.
// ─────────────────────────────────────────────────────────────────
#define AUDIO_CHUNK_SAMPLES 2048  // Must match the value in audio_pipeline.cpp

struct AudioChunk {
    int16_t samples[AUDIO_CHUNK_SAMPLES*2 ]; // Stereo interleaved: L,R,L,R,...
    int     count;                             // How many stereo pairs are valid
};

// ─────────────────────────────────────────────────────────────────
//  extern declarations
//  These tell main.cpp "these variables exist in audio_pipeline.cpp"
//  The keyword extern means: "don't allocate memory for this here,
//  just trust that it exists somewhere and the linker will find it."
// ─────────────────────────────────────────────────────────────────
extern QueueHandle_t g_fftQueue;  // Your spectrogram reads from this

// ─────────────────────────────────────────────────────────────────
//  Function declaration
//  Tells main.cpp that startAudioPipeline() exists and can be called.
// ─────────────────────────────────────────────────────────────────
void startAudioPipeline();
