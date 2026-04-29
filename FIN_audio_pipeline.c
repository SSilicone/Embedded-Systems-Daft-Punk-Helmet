// =============================================================================
//  audio_pipeline.cpp  (SD card → resampler → Bluetooth A2DP)
// =============================================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <SPI.h>
#include <SD.h>
#include "BluetoothA2DPSource.h"
#include "audio_pipeline.h"
#include "esp_task_wdt.h"

// =============================================================================
//  PIN DEFINITIONS
// =============================================================================
#define SD_CS_PIN    5
#define SD_MOSI_PIN  23
#define SD_MISO_PIN  19
#define SD_SCK_PIN   18

// ★ POT moved to GPIO 25 (ADC2 channel 8).
//   ADC1 (GPIOs 32-39) is monopolised by I2S when i2s_adc_enable() is active.
//   ADC2 is free for normal analogRead() as long as WiFi is not in use.
#define POT_PIN      25
#define SPEED_MIN    0.5f
#define SPEED_MAX    2.0f

// =============================================================================
//  AUDIO CONFIGURATION
// =============================================================================
#define WAV_FILE_PATH   "/audio.wav"
#define WAV_SAMPLE_RATE 44100
#define SD_SPI_FREQ     4000000

#define AUDIO_QUEUE_DEPTH     16
#define RESAMPLED_QUEUE_DEPTH 24
#define FFT_QUEUE_DEPTH        8   // small — main loop drains it every ~150 ms

// =============================================================================
//  GLOBALS
// =============================================================================
QueueHandle_t g_rawQueue          = nullptr;
QueueHandle_t g_resampledQueue    = nullptr;
QueueHandle_t g_fftQueue          = nullptr;      // → main.cpp FFT consumer
QueueHandle_t g_rawFreePool       = nullptr;
QueueHandle_t g_resampledFreePool = nullptr;
QueueHandle_t g_fftFreePool       = nullptr;      // recycled FFTChunks

static AudioChunk* g_rawChunks        = nullptr;
static AudioChunk* g_resampledChunks  = nullptr;
static FFTChunk*   g_fftChunks        = nullptr;  // PSRAM-allocated FFT pool

volatile float     g_playbackSpeed    = 1.0f;

BluetoothA2DPSource a2dpSource;

static File              g_wavFile;
static SemaphoreHandle_t g_fileMutex     = nullptr;
static uint32_t          g_audioDataSize  = 0;
static uint32_t          g_audioDataStart = 0;

// =============================================================================
//  WAV HEADER
// =============================================================================
struct __attribute__((packed)) WavHeader {
    char     chunkID[4];
    uint32_t chunkSize;
    char     format[4];
    char     subchunk1ID[4];
    uint32_t subchunk1Size;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    char     subchunk2ID[4];
    uint32_t subchunk2Size;
};

// =============================================================================
//  SD CARD HEALTH CHECK
// =============================================================================
bool verifySdConnection() {
    Serial.println("[SD] Running SPI integrity check...");
    File testFile = SD.open("/spi_test.tmp", FILE_WRITE);
    if (!testFile) {
        Serial.println("[SD] Could not create test file");
        return false;
    }
    uint8_t writePattern[256];
    for (int i = 0; i < 256; i++) writePattern[i] = (uint8_t)i;
    testFile.write(writePattern, 256);
    testFile.close();

    testFile = SD.open("/spi_test.tmp", FILE_READ);
    if (!testFile) { Serial.println("[SD] Could not re-open test file"); return false; }
    uint8_t readBack[256];
    testFile.read(readBack, 256);
    testFile.close();
    SD.remove("/spi_test.tmp");

    int errors = 0;
    for (int i = 0; i < 256; i++) if (readBack[i] != writePattern[i]) errors++;
    if (errors > 0) {
        Serial.printf("[SD] *** CORRUPTION: %d/256 bytes wrong ***\n", errors);
        return false;
    }
    Serial.println("[SD] SPI check PASSED");
    return true;
}

// =============================================================================
//  OPEN AND VALIDATE WAV FILE
// =============================================================================
bool openWavFile(const char* path) {
    g_wavFile = SD.open(path, FILE_READ);
    if (!g_wavFile) {
        Serial.printf("[SD] ERROR: Cannot open %s\n", path);
        return false;
    }
    WavHeader hdr;
    if (g_wavFile.read((uint8_t*)&hdr, sizeof(hdr)) != sizeof(hdr)) {
        Serial.println("[SD] ERROR: Could not read WAV header");
        g_wavFile.close(); return false;
    }
    if (strncmp(hdr.chunkID,     "RIFF", 4) != 0 ||
        strncmp(hdr.format,      "WAVE", 4) != 0 ||
        strncmp(hdr.subchunk1ID, "fmt ", 4) != 0) {
        Serial.println("[SD] ERROR: Not a valid WAV file");
        g_wavFile.close(); return false;
    }
    if (hdr.audioFormat != 1 || hdr.bitsPerSample != 16) {
        Serial.println("[SD] ERROR: Need PCM 16-bit WAV");
        g_wavFile.close(); return false;
    }
    g_audioDataStart = sizeof(WavHeader);
    g_audioDataSize  = hdr.subchunk2Size;

    Serial.printf("[SD] %s | %d Hz | %dch | %d-bit | %.1fs\n",
        path, hdr.sampleRate, hdr.numChannels, hdr.bitsPerSample,
        (float)g_audioDataSize / (hdr.sampleRate * hdr.numChannels * 2));
    return true;
}

// =============================================================================
//  TASK 1: SD CARD READER  (Core 1, Priority 3)
// =============================================================================
void taskSDReader(void* param) {
    Serial.printf("[SDReader] Core %d\n", xPortGetCoreID());
    int chunkBytes = AUDIO_CHUNK_SAMPLES * 2 * sizeof(int16_t);
    int chunkCount = 0, errorCount = 0;

    while (true) {
        AudioChunk* chunk = nullptr;
        xQueueReceive(g_rawFreePool, &chunk, portMAX_DELAY);

        xSemaphoreTake(g_fileMutex, portMAX_DELAY);
        uint32_t pos = g_wavFile.position();
        if (!g_wavFile.available() || pos >= g_audioDataStart + g_audioDataSize) {
            g_wavFile.seek(g_audioDataStart);
            Serial.println("[SDReader] Looping");
        }
        int bytesRead = g_wavFile.read((uint8_t*)chunk->samples, chunkBytes);
        xSemaphoreGive(g_fileMutex);

        if (bytesRead <= 0) {
            if (++errorCount > 10) Serial.println("[SDReader] Too many errors");
            xQueueSend(g_rawFreePool, &chunk, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        chunk->count = bytesRead / (2 * sizeof(int16_t));
        if (++chunkCount % 500 == 0)
            Serial.printf("[SDReader] %d chunks, %d errors\n", chunkCount, errorCount);

        if (xQueueSend(g_rawQueue, &chunk, pdMS_TO_TICKS(50)) != pdTRUE)
            xQueueSend(g_rawFreePool, &chunk, 0);

        if (uxQueueSpacesAvailable(g_rawQueue) == 0)
            vTaskDelay(pdMS_TO_TICKS(1));
        else
            taskYIELD();
    }
}

// =============================================================================
//  TASK 2: RESAMPLER  (Core 1, Priority 4)
//
//  After producing each resampled chunk for Bluetooth it also copies the
//  first FFT_CHUNK_SAMPLES stereo pairs into a dedicated FFTChunk and
//  sends the pointer to g_fftQueue.  Because g_fftQueue holds a *different*
//  pointer to a *different* buffer, the BT callback and the FFT consumer
//  are completely independent — no use-after-free.
// =============================================================================
void taskResampler(void* param) {
    Serial.printf("[Resampler] Core %d\n", xPortGetCoreID());
    float       readPos  = 0.0f;
    AudioChunk* rawChunk = nullptr;

    while (true) {
        // ── Get a resampled output buffer ────────────────────────────────
        AudioChunk* outChunk = nullptr;
        xQueueReceive(g_resampledFreePool, &outChunk, portMAX_DELAY);

        int outIdx = 0;
        while (outIdx < AUDIO_CHUNK_SAMPLES) {
            if (rawChunk == nullptr || readPos >= (float)(rawChunk->count - 1)) {
                if (rawChunk != nullptr) {
                    readPos -= (float)(rawChunk->count - 1);
                    xQueueSend(g_rawFreePool, &rawChunk, 0);
                    rawChunk = nullptr;
                }
                if (readPos < 0.0f) readPos = 0.0f;
                if (xQueueReceive(g_rawQueue, &rawChunk, pdMS_TO_TICKS(50)) != pdTRUE) {
                    // Underrun — fill with silence
                    while (outIdx < AUDIO_CHUNK_SAMPLES) {
                        outChunk->samples[outIdx * 2]     = 0;
                        outChunk->samples[outIdx * 2 + 1] = 0;
                        outIdx++;
                    }
                    break;
                }
            }

            int   idx0 = (int)readPos;
            int   idx1 = min(idx0 + 1, rawChunk->count - 1);
            float frac = readPos - (float)idx0;
            idx0 = constrain(idx0, 0, rawChunk->count - 1);

            float L = rawChunk->samples[idx0*2]   *(1.0f-frac)
                    + rawChunk->samples[idx1*2]   * frac;
            float R = rawChunk->samples[idx0*2+1] *(1.0f-frac)
                    + rawChunk->samples[idx1*2+1] * frac;

            outChunk->samples[outIdx*2]   = (int16_t)constrain(L, -32768.f, 32767.f);
            outChunk->samples[outIdx*2+1] = (int16_t)constrain(R, -32768.f, 32767.f);
            outIdx++;
            readPos += g_playbackSpeed;
        }
        outChunk->count = outIdx;

        // ── Send to BT ───────────────────────────────────────────────────
        if (xQueueSend(g_resampledQueue, &outChunk, pdMS_TO_TICKS(20)) != pdTRUE) {
            xQueueSend(g_resampledFreePool, &outChunk, 0);
            continue;   // don't attempt FFT copy on a dropped chunk
        }

        // ── Copy first FFT_CHUNK_SAMPLES pairs into an FFTChunk ──────────
        //  Try non-blocking — if the pool is empty the spectrogram just
        //  skips this chunk, which is fine.
        FFTChunk* fftChunk = nullptr;
        if (xQueueReceive(g_fftFreePool, &fftChunk, 0) == pdTRUE) {
            int toCopy = min(outChunk->count, FFT_CHUNK_SAMPLES);
            memcpy(fftChunk->samples, outChunk->samples,
                   toCopy * 2 * sizeof(int16_t));
            fftChunk->count = toCopy;
            if (xQueueSend(g_fftQueue, &fftChunk, 0) != pdTRUE)
                xQueueSend(g_fftFreePool, &fftChunk, 0);  // FFT queue full, drop
        }
    }
}

// =============================================================================
//  TASK 3: POTENTIOMETER READER  (Core 1, Priority 1)
//  ★ Uses GPIO 25 (ADC2) — safe while I2S is NOT active on this project.
//    If you later re-enable the mic I2S path, move pot to an ADC2 pin and
//    call analogRead() only when WiFi is off (ADC2 / WiFi conflict).
// =============================================================================
void taskPotReader(void* param) {
    Serial.printf("[PotReader] Core %d\n", xPortGetCoreID());
    const int FILTER_LEN = 8;
    int filterBuf[FILTER_LEN] = {0};
    int filterIdx = 0;
    long filterSum = 0;

    while (true) {
        int raw = analogRead(POT_PIN);
        filterSum -= filterBuf[filterIdx];
        filterBuf[filterIdx] = raw;
        filterSum += raw;
        filterIdx = (filterIdx + 1) % FILTER_LEN;

        float speed = SPEED_MIN +
            ((float)filterSum / FILTER_LEN / 4095.0f) * (SPEED_MAX - SPEED_MIN);
        g_playbackSpeed = speed;

        // Periodic log so you can verify the pot is being read correctly
        static int potLogCount = 0;
        if (++potLogCount % 100 == 0)
            Serial.printf("[Pot] raw=%d  speed=%.3f\n", raw, speed);

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// =============================================================================
//  BLUETOOTH AUDIO CALLBACK
// =============================================================================
int32_t btAudioCallback(Frame* data, int32_t frame_count) {
    static AudioChunk* chunk  = nullptr;
    static int         offset = 0;
    int filled = 0;

    while (filled < frame_count) {
        if (chunk == nullptr || offset >= chunk->count) {
            if (chunk != nullptr) {
                xQueueSend(g_resampledFreePool, &chunk, 0);
                chunk  = nullptr;
                offset = 0;
            }
            if (xQueueReceive(g_resampledQueue, &chunk, 0) != pdTRUE) {
                memset(&data[filled], 0, (frame_count - filled) * sizeof(Frame));
                return frame_count;
            }
        }
        int canCopy = min(frame_count - filled, chunk->count - offset);
        for (int i = 0; i < canCopy; i++) {
            data[filled+i].channel1 = chunk->samples[(offset+i)*2];
            data[filled+i].channel2 = chunk->samples[(offset+i)*2+1];
        }
        filled += canCopy;
        offset += canCopy;
    }
    return frame_count;
}

// =============================================================================
//  BLUETOOTH CONNECTION STATE CALLBACK
// =============================================================================
void btConnectionCallback(esp_a2d_connection_state_t connState, void* obj) {
    const char* s = "Unknown";
    switch (connState) {
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:  s = "Disconnected";  break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:    s = "Connecting";    break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED:     s = "Connected";     break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING: s = "Disconnecting"; break;
    }
    Serial.printf("[BT] %s\n", s);
}

// =============================================================================
//  startAudioPipeline()
// =============================================================================
void startAudioPipeline() {

    // ── PSRAM allocation ─────────────────────────────────────────────────
    Serial.printf("[MEM] DRAM free:  %d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    Serial.printf("[MEM] PSRAM free: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    g_rawChunks       = (AudioChunk*)ps_malloc(AUDIO_QUEUE_DEPTH     * sizeof(AudioChunk));
    g_resampledChunks = (AudioChunk*)ps_malloc(RESAMPLED_QUEUE_DEPTH * sizeof(AudioChunk));
    g_fftChunks       = (FFTChunk*)  ps_malloc(FFT_QUEUE_DEPTH       * sizeof(FFTChunk));

    if (!g_rawChunks || !g_resampledChunks || !g_fftChunks) {
        Serial.println("[Pipeline] FATAL: PSRAM allocation failed!"); return;
    }
    Serial.printf("[Pipeline] Buffers in PSRAM: %d bytes\n",
        (int)(AUDIO_QUEUE_DEPTH     * sizeof(AudioChunk) +
              RESAMPLED_QUEUE_DEPTH * sizeof(AudioChunk) +
              FFT_QUEUE_DEPTH       * sizeof(FFTChunk)));

    // ── SD card ──────────────────────────────────────────────────────────
    SPIClass* spi = new SPIClass(HSPI);
    spi->begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN, *spi, SD_SPI_FREQ)) {
        Serial.println("[SD] MOUNT FAILED"); return;
    }
    verifySdConnection();
    g_fileMutex = xSemaphoreCreateMutex();
    if (!openWavFile(WAV_FILE_PATH)) {
        Serial.println("[SD] Cannot start — no valid WAV."); return;
    }

    // ── Queues and free pools ─────────────────────────────────────────────
    g_rawFreePool = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(AudioChunk*));
    for (int i = 0; i < AUDIO_QUEUE_DEPTH; i++) {
        AudioChunk* p = &g_rawChunks[i];
        xQueueSend(g_rawFreePool, &p, 0);
    }
    g_resampledFreePool = xQueueCreate(RESAMPLED_QUEUE_DEPTH, sizeof(AudioChunk*));
    for (int i = 0; i < RESAMPLED_QUEUE_DEPTH; i++) {
        AudioChunk* p = &g_resampledChunks[i];
        xQueueSend(g_resampledFreePool, &p, 0);
    }
    g_fftFreePool = xQueueCreate(FFT_QUEUE_DEPTH, sizeof(FFTChunk*));
    for (int i = 0; i < FFT_QUEUE_DEPTH; i++) {
        FFTChunk* p = &g_fftChunks[i];
        xQueueSend(g_fftFreePool, &p, 0);
    }

    g_rawQueue       = xQueueCreate(AUDIO_QUEUE_DEPTH,     sizeof(AudioChunk*));
    g_resampledQueue = xQueueCreate(RESAMPLED_QUEUE_DEPTH, sizeof(AudioChunk*));
    g_fftQueue       = xQueueCreate(FFT_QUEUE_DEPTH,       sizeof(FFTChunk*));

    if (!g_rawFreePool || !g_resampledFreePool || !g_fftFreePool ||
        !g_rawQueue    || !g_resampledQueue    || !g_fftQueue) {
        Serial.println("[Pipeline] FATAL: Queue creation failed!"); return;
    }

    // ── ADC / pot ─────────────────────────────────────────────────────────
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(POT_PIN, INPUT);

    // ── Tasks ─────────────────────────────────────────────────────────────
    Serial.println("[Pipeline] Starting tasks...");
    xTaskCreatePinnedToCore(taskSDReader,  "SDReader",  8192,  nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(taskResampler, "Resampler", 16384, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(taskPotReader, "PotReader", 2048,  nullptr, 1, nullptr, 1);

    // ── Pre-fill then start BT ────────────────────────────────────────────
    Serial.println("[Pipeline] Pre-filling buffer...");
    int waitCount = 0;
    while (uxQueueMessagesWaiting(g_resampledQueue) < 16) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (++waitCount > 50) { Serial.println("[Pipeline] Fill timeout."); break; }
    }
    Serial.println("[Pipeline] Buffer ready — starting Bluetooth.");
    a2dpSource.set_on_connection_state_changed(btConnectionCallback);
    a2dpSource.set_pin_code("0000");
    a2dpSource.set_auto_reconnect(false);
    a2dpSource.start("J22", btAudioCallback);
}
