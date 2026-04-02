// =============================================================================
//  main.cpp
//  ESP32 + MAX4466 → Real-Time Audio Spectrogram on WS2812B LED Matrix
// =============================================================================

#include <Arduino.h>
#include <driver/i2s.h>
#include <FastLED.h>
#include <arduinoFFT.h>
#include "audio_pipeline.h"
// removed: extern QueueHandle_t g_resampledFreePool — not used here

// =============================================================================
//  MATRIX CONFIGURATION
// =============================================================================
#define LED_PIN         22
#define NUM_COLS        32
#define NUM_ROWS        8
#define NUM_LEDS        (NUM_COLS * NUM_ROWS)
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
#define MAX_BRIGHTNESS  150

// =============================================================================
//  ADC / I2S (MICROPHONE) CONFIGURATION
// =============================================================================
#define I2S_PORT        I2S_NUM_0
#define ADC_CHANNEL     ADC1_CHANNEL_7
#define SAMPLE_RATE     40000
#define FFT_SAMPLES     512
#define DMA_BUF_LEN     256
#define DMA_BUF_COUNT   4

// =============================================================================
//  SPECTROGRAM VISUAL TUNING
// =============================================================================
#define NOISE_FLOOR     300.0f
#define GAIN            2.0f
#define MAX_MAGNITUDE   4000.0f
#define DECAY           0.78f

// =============================================================================
//  GLOBALS
// =============================================================================
CRGB leds[NUM_LEDS];
float vReal[FFT_SAMPLES];
float vImag[FFT_SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE);
float barHeights[NUM_COLS] = {0};

// =============================================================================
//  I2S / MICROPHONE SETUP
// =============================================================================
void setupI2S() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX
                                             | I2S_MODE_ADC_BUILT_IN),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };
    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
    i2s_adc_enable(I2S_PORT);
}

// =============================================================================
//  AUDIO CAPTURE FROM MICROPHONE
// =============================================================================
void captureAudio() {
    uint16_t rawBuf[DMA_BUF_LEN];
    int samplesRead = 0;

    while (samplesRead < FFT_SAMPLES) {
        size_t bytesRead = 0;
        i2s_read(I2S_PORT, rawBuf, sizeof(rawBuf), &bytesRead, portMAX_DELAY);
        int count = bytesRead / sizeof(uint16_t);

        for (int i = 0; i < count && samplesRead < FFT_SAMPLES; i++) {
            // ★ Fixed: cast to float (not double) to match ArduinoFFT<float>
            vReal[samplesRead] = (float)((rawBuf[i] & 0x0FFF) - 2048);
            vImag[samplesRead] = 0.0f;
            samplesRead++;
        }
    }
}

// =============================================================================
//  LED INDEX MAPPING
// =============================================================================
int getLEDIndex(int col, int row) {
    int displayRow = (NUM_ROWS - 1) - row;
    if (displayRow % 2 == 0) {
        return displayRow * NUM_COLS + col;
    } else {
        return displayRow * NUM_COLS + (NUM_COLS - 1 - col);
    }
}

// =============================================================================
//  DRAW SPECTROGRAM BARS
// =============================================================================
void drawBars() {
    FastLED.clear();
    for (int col = 0; col < NUM_COLS; col++) {
        int barTop = constrain(
            (int)(barHeights[col] * (NUM_ROWS - 1) + 0.5f), 0, NUM_ROWS - 1);
        for (int row = 0; row <= barTop; row++) {
            uint8_t hue = map(row, 0, NUM_ROWS - 1, 160, 0);
            leds[getLEDIndex(col, row)] = CHSV(hue, 255, 255);
        }
    }
    FastLED.show();
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Spectrogram init...");

    // ── Commented out while testing WiFi+BT without mic and LED matrix ──
    // FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    // FastLED.setBrightness(MAX_BRIGHTNESS);
    // FastLED.clear();
    // FastLED.show();
    // setupI2S();

    startAudioPipeline();

    Serial.println("Ready — listening for audio.");
}

// =============================================================================
//  MAIN LOOP
// =============================================================================
void loop() {

    // ── Commented out while testing without microphone ──
    // captureAudio();
    // FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
    // FFT.compute(FFTDirection::Forward);
    // FFT.complexToMagnitude();
    //
    // int halfSamples = FFT_SAMPLES / 2;
    // for (int col = 0; col < NUM_COLS; col++) {
    //     float freqStart = powf((float)halfSamples, (float)col       / NUM_COLS);
    //     float freqEnd   = powf((float)halfSamples, (float)(col + 1) / NUM_COLS);
    //     int binStart = max(1, (int)freqStart);
    //     int binEnd   = max(binStart + 1, (int)freqEnd);
    //     binEnd       = min(binEnd, halfSamples);
    //     float sum = 0;
    //     for (int b = binStart; b < binEnd; b++) sum += vReal[b];
    //     float avg        = sum / (float)(binEnd - binStart);
    //     float level      = max(0.0f, avg - NOISE_FLOOR) * GAIN;
    //     float normalized = constrain(level / MAX_MAGNITUDE, 0.0f, 1.0f);
    //     if (normalized > barHeights[col]) barHeights[col] = normalized;
    //     else barHeights[col] = barHeights[col] * DECAY + normalized * (1.0f - DECAY);
    // }
    // drawBars();
    // double peakHz = FFT.majorPeak();
    // Serial.printf("Peak: %.1f Hz\n", peakHz);

    // Keep loop() alive without burning CPU while audio runs on Core 0
    vTaskDelay(pdMS_TO_TICKS(100));
}
