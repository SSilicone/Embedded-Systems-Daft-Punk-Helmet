// =============================================================================
//  audio_pipeline.cpp  (WiFi streaming version)
// =============================================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include "BluetoothA2DPSource.h"
#include "audio_pipeline.h"

// =============================================================================
//  CONFIGURATION
// =============================================================================
#define WIFI_SSID           "ESP32_Audio"
#define WIFI_PASSWORD       "password1234"
#define LAPTOP_IP           "192.168.4.2"
#define SERVER_PORT         8080
#define WAV_FILE_PATH       "/audio.wav"
#define AUDIO_URL           "http://" LAPTOP_IP ":" XSTR(SERVER_PORT) WAV_FILE_PATH
#define XSTR(s)             STR(s)
#define STR(s)              #s

#define POT_PIN             34
#define SPEED_MIN           0.5f
#define SPEED_MAX           2.0f
#define AUDIO_QUEUE_DEPTH   8

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
//  TASK 1: WiFi AUDIO FETCHER  (Core 0, Priority 2)
// =============================================================================
void taskWiFiReader(void* param) {
    Serial.printf("[WiFi Reader] Task started on Core %d\n", xPortGetCoreID());

    const int WAV_HEADER_SIZE = sizeof(WavHeader);
    uint32_t  fileSize        = 0;
    uint32_t  readPosition    = WAV_HEADER_SIZE;

    while (true) {
        HTTPClient http;
        http.begin(AUDIO_URL);

        int chunkBytes = AUDIO_CHUNK_SAMPLES * 2 * sizeof(int16_t);
        char rangeHeader[64];
        snprintf(rangeHeader, sizeof(rangeHeader),
                 "bytes=%u-%u", readPosition, readPosition + chunkBytes - 1);
        http.addHeader("Range", rangeHeader);

        const char* headerKeys[] = {"Content-Range"};
        http.collectHeaders(headerKeys, 1);

        int httpCode = http.GET();

        if (httpCode == 206 || httpCode == 200) {

            if (fileSize == 0) {
                String contentRange = http.header("Content-Range");
                if (contentRange.length() > 0) {
                    int slashPos = contentRange.indexOf('/');
                    if (slashPos >= 0) {
                        fileSize = contentRange.substring(slashPos + 1).toInt();
                        Serial.printf("[WiFi Reader] File size: %u bytes\n", fileSize);
                    }
                }
                if (fileSize == 0) {
                    fileSize = http.getSize();
                    Serial.printf("[WiFi Reader] File size (fallback): %u bytes\n", fileSize);
                }
            }

            WiFiClient* stream = http.getStreamPtr();
            AudioChunk* chunk  = nullptr;

            if (xQueueReceive(g_rawFreePool, &chunk, pdMS_TO_TICKS(100)) == pdTRUE) {

                int      bytesToRead = chunkBytes;
                int      bytesRead   = 0;
                uint8_t* destPtr     = (uint8_t*)chunk->samples;

                unsigned long timeout = millis() + 2000;
                while (bytesRead < bytesToRead && millis() < timeout) {
                    if (stream->available()) {
                        int got = stream->readBytes(
                            destPtr + bytesRead,
                            bytesToRead - bytesRead);
                        bytesRead += got;
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                }

                chunk->count = bytesRead / (2 * sizeof(int16_t));

                if (xQueueSend(g_rawQueue, &chunk, pdMS_TO_TICKS(50)) != pdTRUE) {
                    xQueueSend(g_rawFreePool, &chunk, 0);
                }

                readPosition += bytesRead;
            }

        } else {
            Serial.printf("[WiFi Reader] HTTP error: %d — retrying in 1s\n", httpCode);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        http.end();

        if (fileSize > 0 && readPosition >= fileSize) {
            Serial.println("[WiFi Reader] Looping audio file...");
            readPosition = WAV_HEADER_SIZE;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =============================================================================
//  TASK 2: RESAMPLER  (Core 0, Priority 3)
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
            if (rawChunk == nullptr || readPos >= (float)(rawChunk->count - 1)) {
                if (rawChunk != nullptr) {
                    readPos -= (float)(rawChunk->count - 1);
                    xQueueSend(g_rawFreePool, &rawChunk, 0);
                    rawChunk = nullptr;
                }
                if (readPos < 0.0f) readPos = 0.0f;

                if (xQueueReceive(g_rawQueue, &rawChunk, pdMS_TO_TICKS(50)) != pdTRUE) {
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

            outChunk->samples[outIdx * 2]     = (int16_t)constrain(L, -32768.0f, 32767.0f);
            outChunk->samples[outIdx * 2 + 1] = (int16_t)constrain(R, -32768.0f, 32767.0f);
            outIdx++;

            readPos += g_playbackSpeed;
        }

        outChunk->count = outIdx;

        // ★ Fixed order: Bluetooth send FIRST, FFT send only if BT succeeded.
        //   This prevents the double-free crash from the previous version.
        if (xQueueSend(g_resampledQueue, &outChunk, pdMS_TO_TICKS(20)) != pdTRUE) {
            // BT queue full — return chunk to free pool and skip this frame entirely
            xQueueSend(g_resampledFreePool, &outChunk, 0);
            continue;
        }

        // BT send succeeded — now it's safe to also notify the FFT queue.
        // Timeout=0 means non-blocking: if FFT queue is full, we just skip it.
        // The BT pipeline is never affected by the FFT queue being slow.
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
            ((float)filterSum / FILTER_LEN / 4095.0f) * (SPEED_MAX - SPEED_MIN);

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// =============================================================================
//  BLUETOOTH CALLBACK
// =============================================================================
int32_t btAudioCallback(Frame* data, int32_t frame_count) {
    AudioChunk* chunk = nullptr;

    if (xQueueReceive(g_resampledQueue, &chunk, 0) != pdTRUE) {
        Serial.println("[BT] Callback fired but queue empty — sending silence");
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
//  startAudioPipeline()
// =============================================================================
void startAudioPipeline() {
    Serial.println("[Pipeline] Initializing WiFi audio pipeline...");

    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("[WiFi] Access point started. ESP32 IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.printf("[WiFi] Connect your laptop to WiFi: \"%s\"\n", WIFI_SSID);
    Serial.println("[WiFi] Then run: python3 -m http.server 8080");
    Serial.println("[WiFi] Waiting 15 seconds for laptop to connect...");

    vTaskDelay(pdMS_TO_TICKS(15000));

    Serial.printf("[WiFi] Fetching audio from: %s\n", AUDIO_URL);

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

    g_rawQueue       = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(AudioChunk*));
    g_resampledQueue = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(AudioChunk*));
    g_fftQueue       = xQueueCreate(4, sizeof(AudioChunk*));

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(POT_PIN, INPUT);

    xTaskCreatePinnedToCore(taskWiFiReader, "WiFiReader", 8192, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(taskResampler,  "Resampler",  4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(taskPotReader,  "PotReader",  2048, nullptr, 1, nullptr, 0);

    // ★ NEW: Wait until the resampled queue has at least 4 chunks ready
//   before starting Bluetooth. This ensures the callback never starves.
Serial.println("[Pipeline] Pre-filling audio buffer before BT start...");
UBaseType_t queueCount = 0;
while (queueCount < 4) {
    vTaskDelay(pdMS_TO_TICKS(100));
    queueCount = uxQueueMessagesWaiting(g_resampledQueue);
    Serial.printf("[Pipeline] Buffer level: %d / 4 chunks\n", queueCount);
}
Serial.println("[Pipeline] Buffer ready — starting Bluetooth.");

    a2dpSource.set_auto_reconnect(false);
    a2dpSource.start("Cancan", btAudioCallback);

    Serial.println("[Pipeline] All tasks started.");
}
