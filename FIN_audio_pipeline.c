// =============================================================================
//  audio_pipeline.cpp  (SD card version)
//  Data flow: SD card → resampler → Bluetooth A2DP speaker
//
//  AUDIO QUALITY FIXES APPLIED IN THIS VERSION:
//    Fix 1 — AUDIO_QUEUE_DEPTH increased 8 → 16 (deeper raw buffer)
//    Fix 2 — RESAMPLED_QUEUE_DEPTH added at 32 slots (larger BT feed buffer)
//    Fix 3 — SD reader yield is now conditional, not a fixed 1ms sleep
//    Fix 4 — Resampler priority raised above SD reader (4 vs 3)
//    Fix 5 — Resampler stack size increased to 8192
//    Fix 6 — Pre-fill threshold raised from 4 to 8 chunks before BT starts
//
//  WIRING:
//    SD MODULE    →   ESP32
//    CS           →   GPIO 5
//    MOSI         →   GPIO 23
//    MISO         →   GPIO 19
//    SCK          →   GPIO 18
//    VCC          →   3.3V
//    GND          →   GND
//
//  SD CARD PREPARATION:
//    - Format as FAT32
//    - File must be named exactly: audio.wav (lowercase, no spaces)
//    - WAV format: PCM, 16-bit, stereo, 44100Hz
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

#define POT_PIN      34
#define SPEED_MIN    0.5f
#define SPEED_MAX    2.0f

// =============================================================================
//  AUDIO CONFIGURATION
// =============================================================================
#define WAV_FILE_PATH   "/audio.wav"
#define WAV_SAMPLE_RATE 44100
#define SD_SPI_FREQ     4000000  // 4 MHz — reliable on breadboards

// ★ Fix 1: Increased from 8 to 16.
//   More raw chunks buffered = fewer gaps when SD reads are briefly slow.
#define AUDIO_QUEUE_DEPTH     16

// ★ Fix 2: Separate deeper queue for the resampled (BT-ready) audio.
//   The BT callback pulls directly from this queue so it needs to be the
//   largest buffer in the chain to absorb any timing variation.
#define RESAMPLED_QUEUE_DEPTH 24

// =============================================================================
//  GLOBALS
// =============================================================================
QueueHandle_t g_rawQueue          = nullptr;
QueueHandle_t g_resampledQueue    = nullptr;
QueueHandle_t g_fftQueue          = nullptr;
QueueHandle_t g_rawFreePool       = nullptr;
QueueHandle_t g_resampledFreePool = nullptr;

// REMOVE these two static arrays:
// static AudioChunk g_rawChunks[AUDIO_QUEUE_DEPTH];
// static AudioChunk g_resampledChunks[RESAMPLED_QUEUE_DEPTH];

// REPLACE with pointers, initialized to nullptr:
static AudioChunk* g_rawChunks       = nullptr;
static AudioChunk* g_resampledChunks = nullptr;

volatile float    g_playbackSpeed  = 1.0f;

BluetoothA2DPSource a2dpSource;

static File              g_wavFile;
static SemaphoreHandle_t g_fileMutex    = nullptr;
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
        Serial.println("[SD] Could not create test file — check CS pin and wiring");
        return false;
    }

    uint8_t writePattern[256];
    for (int i = 0; i < 256; i++) writePattern[i] = (uint8_t)i;
    testFile.write(writePattern, 256);
    testFile.close();

    testFile = SD.open("/spi_test.tmp", FILE_READ);
    if (!testFile) {
        Serial.println("[SD] Could not re-open test file");
        return false;
    }

    uint8_t readBack[256];
    testFile.read(readBack, 256);
    testFile.close();
    SD.remove("/spi_test.tmp");

    int errors = 0;
    for (int i = 0; i < 256; i++) {
        if (readBack[i] != writePattern[i]) errors++;
    }

    if (errors > 0) {
        Serial.printf("[SD] *** CORRUPTION DETECTED: %d/256 bytes wrong ***\n", errors);
        Serial.println("[SD] Try: shorter wires, 3.3V power, lower SPI frequency");
        return false;
    }

    Serial.println("[SD] SPI integrity check PASSED — no corruption detected");
    return true;
}

// =============================================================================
//  OPEN AND VALIDATE WAV FILE
// =============================================================================
bool openWavFile(const char* path) {
    g_wavFile = SD.open(path, FILE_READ);
    if (!g_wavFile) {
        Serial.printf("[SD] ERROR: Cannot open %s\n", path);
        Serial.println("[SD] Check the file exists and is named exactly 'audio.wav'");
        return false;
    }

    WavHeader hdr;
    size_t bytesRead = g_wavFile.read((uint8_t*)&hdr, sizeof(hdr));
    if (bytesRead != sizeof(hdr)) {
        Serial.println("[SD] ERROR: Could not read WAV header");
        g_wavFile.close();
        return false;
    }

    if (strncmp(hdr.chunkID,     "RIFF", 4) != 0 ||
        strncmp(hdr.format,      "WAVE", 4) != 0 ||
        strncmp(hdr.subchunk1ID, "fmt ", 4) != 0) {
        Serial.println("[SD] ERROR: Not a valid WAV file");
        g_wavFile.close();
        return false;
    }

    if (hdr.audioFormat != 1) {
        Serial.printf("[SD] ERROR: WAV must be PCM. Got audioFormat=%d\n",
                      hdr.audioFormat);
        Serial.println("[SD] Open in Audacity → Export → WAV → PCM 16-bit");
        g_wavFile.close();
        return false;
    }

    if (hdr.bitsPerSample != 16) {
        Serial.printf("[SD] ERROR: WAV must be 16-bit. Got %d-bit\n",
                      hdr.bitsPerSample);
        g_wavFile.close();
        return false;
    }

    if (hdr.numChannels != 2) {
        Serial.printf("[SD] WARNING: Expected stereo (2ch), got %d ch\n",
                      hdr.numChannels);
        Serial.println("[SD] Code assumes stereo — audio may sound wrong");
    }

    g_audioDataStart = sizeof(WavHeader);
    g_audioDataSize  = hdr.subchunk2Size;

    Serial.println("[SD] ──────────────────────────────────");
    Serial.printf( "[SD] File:        %s\n",   path);
    Serial.printf( "[SD] Sample rate: %d Hz\n", hdr.sampleRate);
    Serial.printf( "[SD] Channels:    %d\n",    hdr.numChannels);
    Serial.printf( "[SD] Bit depth:   %d\n",    hdr.bitsPerSample);
    Serial.printf( "[SD] Audio data:  %lu bytes\n",
                   (unsigned long)g_audioDataSize);
    Serial.printf( "[SD] Duration:    ~%.1f seconds\n",
                   (float)g_audioDataSize /
                   (hdr.sampleRate * hdr.numChannels * 2));
    Serial.println("[SD] ──────────────────────────────────");

    return true;
}

// =============================================================================
//  TASK 1: SD CARD READER  (Core 0, Priority 3)
//
//  ★ Fix 3: Fixed 1ms delay replaced with a conditional yield.
//    taskYIELD() gives other tasks a chance to run without sleeping,
//    so the raw queue fills as fast as the SD card allows.
//    Only sleeps 1ms if the queue is completely full and there is
//    genuinely nothing useful to do right now.
// =============================================================================
void taskSDReader(void* param) {
    Serial.printf("[SD Reader] Task started on Core %d\n", xPortGetCoreID());

    int  chunkBytes = AUDIO_CHUNK_SAMPLES * 2 * sizeof(int16_t);
    int  chunkCount = 0;
    int  errorCount = 0;
    static bool firstChunkPrinted = false;

    while (true) {
        AudioChunk* chunk = nullptr;
        xQueueReceive(g_rawFreePool, &chunk, portMAX_DELAY);

        xSemaphoreTake(g_fileMutex, portMAX_DELAY);

        uint32_t currentPos = g_wavFile.position();
        if (!g_wavFile.available() ||
            currentPos >= g_audioDataStart + g_audioDataSize) {
            g_wavFile.seek(g_audioDataStart);
            Serial.println("[SD Reader] Looping audio file");
        }

        int bytesRead = g_wavFile.read((uint8_t*)chunk->samples, chunkBytes);

        xSemaphoreGive(g_fileMutex);

        // One-time diagnostic at startup
        if (!firstChunkPrinted && bytesRead > 0) {
            firstChunkPrinted = true;
            Serial.println("[SD] First 8 sample values from WAV data:");
            for (int i = 0; i < 8; i++) {
                Serial.printf("  [%d] L=%d  R=%d\n",
                    i,
                    chunk->samples[i * 2],
                    chunk->samples[i * 2 + 1]);
            }
        }

        if (bytesRead <= 0) {
            errorCount++;
            Serial.printf("[SD Reader] Read error #%d — zero bytes returned\n",
                          errorCount);
            if (errorCount > 10) {
                Serial.println("[SD Reader] Too many errors — check SD card.");
            }
            xQueueSend(g_rawFreePool, &chunk, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        chunk->count = bytesRead / (2 * sizeof(int16_t));

        chunkCount++;
        if (chunkCount % 500 == 0) {
            Serial.printf("[SD Reader] %d chunks read, %d errors\n",
                          chunkCount, errorCount);
        }

        if (xQueueSend(g_rawQueue, &chunk, pdMS_TO_TICKS(50)) != pdTRUE) {
            xQueueSend(g_rawFreePool, &chunk, 0);
        }

        // ★ Fix 3: Conditional yield — only sleep if the queue is full.
        //   This keeps the watchdog fed without artificially slowing down
        //   the read rate when there is still queue space available.
        if (uxQueueSpacesAvailable(g_rawQueue) == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            taskYIELD();
        }
    }
}

// =============================================================================
//  TASK 2: RESAMPLER  (Core 0, Priority 4)
//
//  ★ Fix 4: Priority raised to 4 (above SD reader at 3).
//    The resampler must stay ahead of the BT callback rate.
//    Giving it higher priority ensures it processes raw chunks
//    immediately rather than waiting for the SD reader to finish.
//
//  ★ Fix 5: Stack size raised to 8192 words (set at task creation below).
// =============================================================================
void taskResampler(void* param) {
    Serial.printf("[Resampler] Task started on Core %d\n", xPortGetCoreID());

    float       readPos  = 0.0f;
    AudioChunk* rawChunk = nullptr;

    while (true) {
        AudioChunk* outChunk = nullptr;
        xQueueReceive(g_resampledFreePool, &outChunk, portMAX_DELAY);

        int outIdx = 0;
        while (outIdx < AUDIO_CHUNK_SAMPLES) {

            if (rawChunk == nullptr ||
                readPos >= (float)(rawChunk->count - 1)) {
                if (rawChunk != nullptr) {
                    readPos -= (float)(rawChunk->count - 1);
                    xQueueSend(g_rawFreePool, &rawChunk, 0);
                    rawChunk = nullptr;
                }
                if (readPos < 0.0f) readPos = 0.0f;

                if (xQueueReceive(g_rawQueue, &rawChunk,
                                  pdMS_TO_TICKS(50)) != pdTRUE) {
                    while (outIdx < AUDIO_CHUNK_SAMPLES) {
                        outChunk->samples[outIdx * 2]     = 0;
                        outChunk->samples[outIdx * 2 + 1] = 0;
                        outIdx++;
                    }
                    break;
                }
            }

            int   idx0 = (int)readPos;
            int   idx1 = idx0 + 1;
            float frac = readPos - (float)idx0;

            idx0 = constrain(idx0, 0, rawChunk->count - 1);
            idx1 = constrain(idx1, 0, rawChunk->count - 1);

            float L = rawChunk->samples[idx0 * 2]     * (1.0f - frac)
                    + rawChunk->samples[idx1 * 2]     * frac;
            float R = rawChunk->samples[idx0 * 2 + 1] * (1.0f - frac)
                    + rawChunk->samples[idx1 * 2 + 1] * frac;

            outChunk->samples[outIdx * 2]     =
                (int16_t)constrain(L, -32768.0f, 32767.0f);
            outChunk->samples[outIdx * 2 + 1] =
                (int16_t)constrain(R, -32768.0f, 32767.0f);
            outIdx++;
            readPos += g_playbackSpeed;
        }

        outChunk->count = outIdx;

        if (xQueueSend(g_resampledQueue, &outChunk,
                       pdMS_TO_TICKS(20)) != pdTRUE) {
            xQueueSend(g_resampledFreePool, &outChunk, 0);
            continue;
        }

        xQueueSend(g_fftQueue, &outChunk, 0);
    }
}

// =============================================================================
//  TASK 3: POTENTIOMETER READER  (Core 0, Priority 1)
// =============================================================================
void taskPotReader(void* param) {
    Serial.printf("[PotReader] Task started on Core %d\n", xPortGetCoreID());

    const int FILTER_LEN = 8;
    int       filterBuf[FILTER_LEN] = {0};
    int       filterIdx = 0;
    long      filterSum = 0;

    while (true) {
        int raw = analogRead(POT_PIN);

        filterSum -= filterBuf[filterIdx];
        filterBuf[filterIdx] = raw;
        filterSum += raw;
        filterIdx = (filterIdx + 1) % FILTER_LEN;

        g_playbackSpeed = SPEED_MIN +
            ((float)filterSum / FILTER_LEN / 4095.0f) *
            (SPEED_MAX - SPEED_MIN);

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// =============================================================================
//  BLUETOOTH AUDIO CALLBACK
// =============================================================================
int32_t btAudioCallback(Frame* data, int32_t frame_count) {
    static AudioChunk* chunk  = nullptr;
    static int         offset = 0;   // stereo pairs already consumed from chunk

    int filled = 0;

    while (filled < frame_count) {
        // Get a new chunk if we don't have one or finished the last one
        if (chunk == nullptr || offset >= chunk->count) {
            if (chunk != nullptr) {
                xQueueSend(g_resampledFreePool, &chunk, 0);
                chunk  = nullptr;
                offset = 0;
            }
            if (xQueueReceive(g_resampledQueue, &chunk, 0) != pdTRUE) {
                // No data available — fill remainder with silence
                memset(&data[filled], 0, (frame_count - filled) * sizeof(Frame));
                return frame_count;
            }
        }

        // Copy as many frames as we can from the current chunk
        int canCopy = min(frame_count - filled, chunk->count - offset);
        for (int i = 0; i < canCopy; i++) {
            data[filled + i].channel1 = chunk->samples[(offset + i) * 2];
            data[filled + i].channel2 = chunk->samples[(offset + i) * 2 + 1];
        }
        filled += canCopy;
        offset += canCopy;
    }

    return frame_count;
}

// =============================================================================
//  BLUETOOTH CONNECTION STATE CALLBACK (A2DP v1.8.4 compatible)
// =============================================================================
void btConnectionCallback(esp_a2d_connection_state_t connState,
                          void* obj) {

    const char* connStr = "Unknown";
    switch (connState) {
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:  connStr = "Disconnected";  break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:    connStr = "Connecting";    break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED:     connStr = "Connected";     break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING: connStr = "Disconnecting"; break;
    }


    Serial.printf("[BT] Connection: %s\n", connStr);
}

// =============================================================================
//  startAudioPipeline()
//  Call once from setup().
// =============================================================================
void startAudioPipeline() {

    // ── STEP 0: Verify and allocate PSRAM ───────────────────
    Serial.printf("[MEM] Free DRAM:  %d bytes\n",
                  heap_caps_get_free_size(MALLOC_CAP_8BIT));
    Serial.printf("[MEM] Free PSRAM: %d bytes\n",
                  heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    g_rawChunks = (AudioChunk*)ps_malloc(
                      AUDIO_QUEUE_DEPTH * sizeof(AudioChunk));
    g_resampledChunks = (AudioChunk*)ps_malloc(
                      RESAMPLED_QUEUE_DEPTH * sizeof(AudioChunk));

    if (!g_rawChunks || !g_resampledChunks) {
        Serial.println("[Pipeline] FATAL: PSRAM allocation failed!");
        return;
    }

    Serial.printf("[Pipeline] Audio buffers allocated in PSRAM: %d bytes\n",
        (AUDIO_QUEUE_DEPTH + RESAMPLED_QUEUE_DEPTH) * (int)sizeof(AudioChunk));
    

    // ── STEP 1: SPI and SD card ──────────────────────────────
    SPIClass* spi = new SPIClass(HSPI);
    spi->begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    if (!SD.begin(SD_CS_PIN, *spi, SD_SPI_FREQ)) {
        Serial.println("[SD] MOUNT FAILED");
        return;
    }
    verifySdConnection();

    g_fileMutex = xSemaphoreCreateMutex();

    if (!openWavFile(WAV_FILE_PATH)) {
        Serial.println("[SD] Cannot start — no valid WAV file.");
        return;
    }

    // ── STEP 2: Create ALL queues and free pools ─────────────
    // (Nothing can run until these exist)
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

    g_rawQueue       = xQueueCreate(AUDIO_QUEUE_DEPTH,     sizeof(AudioChunk*));
    g_resampledQueue = xQueueCreate(RESAMPLED_QUEUE_DEPTH, sizeof(AudioChunk*));
    g_fftQueue       = xQueueCreate(4,                     sizeof(AudioChunk*));

    // Sanity check — catch null queues before tasks try to use them
    if (!g_rawFreePool || !g_resampledFreePool ||
        !g_rawQueue    || !g_resampledQueue    || !g_fftQueue) {
        Serial.println("[Pipeline] FATAL: Queue creation failed — out of memory!");
        return;
    }

    // ── STEP 3: ADC setup ────────────────────────────────────
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(POT_PIN, INPUT);

    // ── STEP 4: Spawn tasks LAST, after everything is ready ──
    Serial.println("[Pipeline] Starting FreeRTOS tasks...");
    xTaskCreatePinnedToCore(taskSDReader,  "SDReader",  8192, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(taskResampler, "Resampler", 16384, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(taskPotReader, "PotReader", 2048, nullptr, 1, nullptr, 1);

    // ── STEP 5: Pre-fill buffer, then start Bluetooth ────────
    Serial.println("[Pipeline] Pre-filling audio buffer...");
    int waitCount = 0;
    while (uxQueueMessagesWaiting(g_resampledQueue) < 16) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (++waitCount > 50) {
            Serial.println("[Pipeline] WARNING: Buffer fill timeout.");
            break;
        }
    }

    Serial.println("[Pipeline] Buffer ready — starting Bluetooth now.");
    a2dpSource.set_on_connection_state_changed(btConnectionCallback);
    a2dpSource.set_pin_code("0000");
    a2dpSource.set_auto_reconnect(false);
    a2dpSource.start("J22", btAudioCallback);
}
