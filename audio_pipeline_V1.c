// =============================================================================
//  audio_pipeline.cpp  (SD card version)
//  Data flow: SD card → resampler → Bluetooth A2DP speaker
//
//  SD card reads over SPI — completely independent of the Bluetooth radio.
//  No WiFi/BT coexistence issues.
//
//  WIRING (use SHORT wires, under 10cm if possible):
//    SD MODULE    →   ESP32
//    CS           →   GPIO 5
//    MOSI         →   GPIO 23
//    MISO         →   GPIO 19
//    SCK          →   GPIO 14   ← NOTE: using GPIO14 not GPIO18
//    VCC          →   3.3V      ← important: 3.3V not 5V
//    GND          →   GND
//
//  WHY GPIO14 FOR SCK:
//    GPIO18 is your LED strip data pin. Using GPIO14 for SD clock
//    keeps the two signals completely separate and avoids crosstalk.
//
//  SD CARD PREPARATION:
//    - Format as FAT32
//    - File must be named exactly:  audio.wav  (lowercase, no spaces)
//    - WAV format: PCM, 16-bit, stereo, 44100Hz
//    - Recommended max file size: 50MB (about 10 minutes at 44100Hz stereo)
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

// =============================================================================
//  PIN DEFINITIONS
// =============================================================================
#define SD_CS_PIN    5
#define SD_MOSI_PIN  23
#define SD_MISO_PIN  19
#define SD_SCK_PIN   18   // GPIO14 — avoids conflict with LED strip on GPIO18

#define POT_PIN      34    // Potentiometer wiper — ADC1_CH6
#define SPEED_MIN    0.5f
#define SPEED_MAX    2.0f

// =============================================================================
//  AUDIO CONFIGURATION
// =============================================================================
#define WAV_FILE_PATH       "/audio.wav"
#define WAV_SAMPLE_RATE     44100
#define AUDIO_QUEUE_DEPTH   8

// SD SPI clock speed — this is the key corruption fix.
// Default is 4MHz which is very reliable on breadboards.
// If you get no corruption at 4MHz you can try raising to 8MHz.
// Do NOT go above 8MHz on a breadboard.
#define SD_SPI_FREQ   4000000   // 4 MHz

// =============================================================================
//  GLOBALS
// =============================================================================
QueueHandle_t g_rawQueue          = nullptr;
QueueHandle_t g_resampledQueue    = nullptr;
QueueHandle_t g_fftQueue          = nullptr;
QueueHandle_t g_rawFreePool       = nullptr;
QueueHandle_t g_resampledFreePool = nullptr;

static AudioChunk g_rawChunks[AUDIO_QUEUE_DEPTH];
static AudioChunk g_resampledChunks[AUDIO_QUEUE_DEPTH];

volatile float g_playbackSpeed = 1.0f;

BluetoothA2DPSource a2dpSource;

// SD file handle and mutex to protect it between tasks
static File           g_wavFile;
static SemaphoreHandle_t g_fileMutex = nullptr;

// Total audio data bytes in the file (learned from WAV header)
static uint32_t g_audioDataSize  = 0;
// Byte offset where audio data starts (after the WAV header)
static uint32_t g_audioDataStart = 0;

// =============================================================================
//  WAV HEADER
// =============================================================================
struct __attribute__((packed)) WavHeader {
    char     chunkID[4];        // "RIFF"
    uint32_t chunkSize;
    char     format[4];         // "WAVE"
    char     subchunk1ID[4];    // "fmt "
    uint32_t subchunk1Size;
    uint16_t audioFormat;       // 1 = PCM
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    char     subchunk2ID[4];    // "data"
    uint32_t subchunk2Size;
};

// =============================================================================
//  SD CARD HEALTH CHECK
//  Reads back what it just wrote to verify the SPI connection is clean.
//  Call once at startup before playing audio.
//  Returns true if card is healthy, false if data is being corrupted.
// =============================================================================
bool verifySdConnection() {
    Serial.println("[SD] Running SPI integrity check...");

    // Write a small test file
    File testFile = SD.open("/spi_test.tmp", FILE_WRITE);
    if (!testFile) {
        Serial.println("[SD] Could not create test file — check CS pin and wiring");
        return false;
    }

    // Write a known pattern of 256 bytes
    uint8_t writePattern[256];
    for (int i = 0; i < 256; i++) writePattern[i] = (uint8_t)i;
    testFile.write(writePattern, 256);
    testFile.close();

    // Read it back and compare
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

    // Validate RIFF/WAVE format
    if (strncmp(hdr.chunkID,     "RIFF", 4) != 0 ||
        strncmp(hdr.format,      "WAVE", 4) != 0 ||
        strncmp(hdr.subchunk1ID, "fmt ", 4) != 0) {
        Serial.println("[SD] ERROR: Not a valid WAV file");
        g_wavFile.close();
        return false;
    }

    if (hdr.audioFormat != 1) {
        Serial.printf("[SD] ERROR: WAV must be PCM (uncompressed). "
                      "Got audioFormat=%d\n", hdr.audioFormat);
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

    // Store where audio data starts and how large it is
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
//  TASK 1: SD CARD READER  (Core 0, Priority 2)
//
//  Reads raw audio from the WAV file in fixed chunks and posts them
//  to g_rawQueue. Loops the file seamlessly when it ends.
//
//  Corruption troubleshooting built in: counts how many chunks have
//  been read and prints a heartbeat every 500 chunks so you can see
//  it's running even when there are no errors.
// =============================================================================
void taskSDReader(void* param) {
    Serial.printf("[SD Reader] Task started on Core %d\n", xPortGetCoreID());

    int chunkBytes  = AUDIO_CHUNK_SAMPLES * 2 * sizeof(int16_t);
    int chunkCount  = 0;
    int errorCount  = 0;
   
    // ★ ADD THIS LINE at the top of the function, before the while loop
    static bool firstChunkPrinted = false;

    while (true) {
        // Grab a free chunk from the memory pool
        AudioChunk* chunk = nullptr;
        xQueueReceive(g_rawFreePool, &chunk, portMAX_DELAY);

        // Lock the file for this read
        xSemaphoreTake(g_fileMutex, portMAX_DELAY);

        // Loop back to start of audio data if at end of file
        uint32_t currentPos = g_wavFile.position();
        if (!g_wavFile.available() ||
            currentPos >= g_audioDataStart + g_audioDataSize) {
            g_wavFile.seek(g_audioDataStart);
            Serial.println("[SD Reader] Looping audio file");
        }

        int bytesRead = g_wavFile.read((uint8_t*)chunk->samples, chunkBytes);

        xSemaphoreGive(g_fileMutex);

         // ★ ADD THIS ENTIRE BLOCK immediately after the read and semaphore release,
        //   before the error check below
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
                Serial.println("[SD Reader] Too many read errors.");
                Serial.println("[SD Reader] Check SD card seating and wiring.");
            }
            xQueueSend(g_rawFreePool, &chunk, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        chunk->count = bytesRead / (2 * sizeof(int16_t));

        // Heartbeat: print every 500 chunks so you know the reader is alive
        chunkCount++;
        if (chunkCount % 500 == 0) {
            Serial.printf("[SD Reader] %d chunks read, %d errors\n",
                          chunkCount, errorCount);
        }

        // Post to raw queue
        if (xQueueSend(g_rawQueue, &chunk, pdMS_TO_TICKS(50)) != pdTRUE) {
            xQueueSend(g_rawFreePool, &chunk, 0);
        }
         vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
//  TASK 2: RESAMPLER  (Core 0, Priority 3)
//  Linear interpolation speed control — unchanged from previous versions
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
                    // No data — pad with silence
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

        // Bluetooth queue first — audio output is the priority
        if (xQueueSend(g_resampledQueue, &outChunk,
                       pdMS_TO_TICKS(20)) != pdTRUE) {
            xQueueSend(g_resampledFreePool, &outChunk, 0);
            continue;
        }
        // FFT queue non-blocking — visualization can miss frames safely
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
//  Called by the A2DP library on Core 0 when it needs audio frames.
// =============================================================================
int32_t btAudioCallback(Frame* data, int32_t frame_count) {
    AudioChunk* chunk = nullptr;

    if (xQueueReceive(g_resampledQueue, &chunk, 0) != pdTRUE) {
        // Queue empty — send silence rather than noise
        memset(data, 0, frame_count * sizeof(Frame));
        return frame_count;
    }

    int toFill = min(frame_count, chunk->count);
    for (int i = 0; i < toFill; i++) {
        data[i].channel1 = chunk->samples[i * 2];
        data[i].channel2 = chunk->samples[i * 2 + 1];
    }
    if (toFill < frame_count) {
        memset(&data[toFill], 0, (frame_count - toFill) * sizeof(Frame));
    }

    xQueueSend(g_resampledFreePool, &chunk, 0);
    return frame_count;
}

// =============================================================================
//  BLUETOOTH CONNECTION STATE CALLBACK
//  Prints exactly what state the Bluetooth connection is in so you
//  can see in the Serial Monitor whether audio streaming has started.
// =============================================================================
// =============================================================================
//  BLUETOOTH CONNECTION STATE CALLBACK (compatible with A2DP v1.8.4)
// =============================================================================
void btConnectionCallback(esp_a2d_connection_state_t connState,
                          esp_a2d_audio_state_t audioState) {

    const char* connStr = "Unknown";
    switch (connState) {
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:  connStr = "Disconnected";  break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:    connStr = "Connecting";    break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED:     connStr = "Connected";     break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING: connStr = "Disconnecting"; break;
    }

    const char* audioStr = "Unknown";
    switch (audioState) {
        case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND: audioStr = "Suspended (no audio)"; break;
        case ESP_A2D_AUDIO_STATE_STOPPED:        audioStr = "Stopped";              break;
        case ESP_A2D_AUDIO_STATE_STARTED:        audioStr = "STREAMING — audio playing now"; break;
    }

    Serial.printf("[BT] Connection: %s | Audio: %s\n", connStr, audioStr);
}
// =============================================================================
//  startAudioPipeline()
//  Call once from setup(). Mounts SD, validates WAV, starts all tasks,
//  pre-fills buffer, then starts Bluetooth.
// =============================================================================
void startAudioPipeline() {
    // Erase any stored Bluetooth pairing data from previous runs
    // This forces a completely fresh connection every time
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    Serial.println("[Pipeline] ══════════════════════════════════");
    Serial.println("[Pipeline] Starting SD card audio pipeline");
    Serial.println("[Pipeline] ══════════════════════════════════");

    // ── Initialize SPI bus on safe pins ───────────────────────────────────
    // Using HSPI on GPIO14 for SCK to avoid conflict with LED strip on GPIO18
    SPIClass* spi = new SPIClass(HSPI);
    spi->begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    // ── Mount SD card at reduced SPI frequency ────────────────────────────
    Serial.println("[SD] Mounting SD card...");
    Serial.printf("[SD] SPI pins: SCK=%d MISO=%d MOSI=%d CS=%d\n",
                  SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    Serial.printf("[SD] SPI frequency: %d Hz (%.1f MHz)\n",
                  SD_SPI_FREQ, SD_SPI_FREQ / 1000000.0f);

    if (!SD.begin(SD_CS_PIN, *spi, SD_SPI_FREQ)) {
        Serial.println("[SD] *** MOUNT FAILED ***");
        Serial.println("[SD] Checklist:");
        Serial.println("[SD]   1. Is the card inserted?");
        Serial.println("[SD]   2. Is it formatted as FAT32?");
        Serial.println("[SD]   3. Check wiring against pin definitions above");
        Serial.println("[SD]   4. Is the SD module powered from 3.3V (not 5V)?");
        Serial.println("[SD]   5. Are your jumper wires under 15cm?");
        Serial.println("[SD] Pipeline cannot start without SD card.");
        return;
    }

    Serial.println("[SD] Card mounted successfully.");
    Serial.printf("[SD] Card size: %llu MB\n",
                  SD.cardSize() / (1024 * 1024));
    Serial.printf("[SD] Card type: %d (1=MMC 2=SD 3=SDHC 4=UNKNOWN)\n",
                  SD.cardType());

    // ── SPI integrity check ────────────────────────────────────────────────
    if (!verifySdConnection()) {
        Serial.println("[SD] *** SPI INTEGRITY CHECK FAILED ***");
        Serial.println("[SD] Data will be corrupted. Fix wiring before continuing.");
        Serial.println("[SD] Most common fix: shorten all SD jumper wires to < 8cm");
        // Continue anyway — audio may still work, just distorted
    }

    // ── Create file mutex ──────────────────────────────────────────────────
    g_fileMutex = xSemaphoreCreateMutex();

    // ── Open and validate WAV file ─────────────────────────────────────────
    if (!openWavFile(WAV_FILE_PATH)) {
        Serial.println("[SD] Pipeline cannot start without a valid WAV file.");
        return;
    }

    // ── Initialize memory pools ────────────────────────────────────────────
    g_rawFreePool = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(AudioChunk*));
    for (int i = 0; i < AUDIO_QUEUE_DEPTH; i++) {
        AudioChunk* p = &g_rawChunks[i];
        xQueueSend(g_rawFreePool, &p, 0);
    }
    g_resampledFreePool = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(AudioChunk*));
    for (int i = 0; i < AUDIO_QUEUE_DEPTH; i++) {
        AudioChunk* p = &g_resampledChunks[i];
        xQueueSend(g_resampledFreePool, &p, 0);
    }

    // ── Initialize data queues ─────────────────────────────────────────────
    g_rawQueue       = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(AudioChunk*));
    g_resampledQueue = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(AudioChunk*));
    g_fftQueue       = xQueueCreate(4, sizeof(AudioChunk*));

    // ── ADC for potentiometer ──────────────────────────────────────────────
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(POT_PIN, INPUT);

    // ── Start FreeRTOS tasks on Core 0 ─────────────────────────────────────
    Serial.println("[Pipeline] Starting FreeRTOS tasks on Core 0...");
    xTaskCreatePinnedToCore(taskSDReader,  "SDReader",  4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(taskResampler, "Resampler", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(taskPotReader, "PotReader", 2048, nullptr, 1, nullptr, 0);

    // ── Pre-fill the audio buffer before Bluetooth starts ─────────────────
    Serial.println("[Pipeline] Pre-filling audio buffer...");
    int waitCount = 0;
    while (uxQueueMessagesWaiting(g_resampledQueue) < 4) {
        vTaskDelay(pdMS_TO_TICKS(100));
        waitCount++;
        if (waitCount % 5 == 0) {
            Serial.printf("[Pipeline] Buffer: %d/4 chunks ready...\n",
                          (int)uxQueueMessagesWaiting(g_resampledQueue));
        }
        if (waitCount > 50) {
            Serial.println("[Pipeline] WARNING: Buffer pre-fill is taking too long.");
            Serial.println("[Pipeline] SD reader may be stuck. Check SD card.");
            break;
        }
    }
    Serial.println("[Pipeline] Buffer ready — starting Bluetooth now.");

    // ── Start Bluetooth A2DP ───────────────────────────────────────────────
    Serial.println("[BT] Initializing Bluetooth A2DP source...");
    Serial.println("[BT] Make sure your speaker is in PAIRING MODE");
    Serial.println("[BT] (hold speaker Bluetooth button until it flashes)");

  // Handle PIN requests from the speaker during pairing
  a2dpSource.set_pin_code("0000");
  a2dpSource.set_auto_reconnect(false);
  a2dpSource.start("J22", btAudioCallback);

    Serial.println("[BT] Scanning for speaker — this may take 5-15 seconds...");
    Serial.println("[Pipeline] ══════════════════════════════════");
    Serial.println("[Pipeline] Startup complete");
    Serial.println("[Pipeline] ══════════════════════════════════");
}
