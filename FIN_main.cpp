// =============================================================================
//  main.cpp
//  ESP32 WROVER + MAX4466 → real-time audio spectrogram on WS2812B 8×32 matrix
//
//  SERIAL COMMANDS (115200 baud):
//    gain   <float>   amplification after floor             (default 2.5)
//    floor  <float>   per-bin magnitude noise gate          (default 250.0)
//    max    <float>   per-bin peak that fills a full bar    (default 22000.0)
//    decay  <0–1>     bar fall speed  (0=instant, 1=never)  (default 0.85)
//    attack <0–1>     bar rise speed  (0=instant, 1=never)  (default 0.70)
//    bright <0–255>   global LED brightness                 (default 160)
//    pdrop  <float>   peak-hold dot fall speed per frame    (default 0.025)
//    phold  <int>     frames to hold peak dot before drop   (default 18)
//    status           print all current values
//    help             print command list
// =============================================================================

#include <Arduino.h>
#include <driver/i2s.h>
#include <FastLED.h>
#include <arduinoFFT.h>
#include "audio_pipeline.h"

// =============================================================================
//  MATRIX
// =============================================================================
#define LED_PIN      13
#define NUM_COLS     32
#define NUM_ROWS      8
#define NUM_LEDS     (NUM_COLS * NUM_ROWS)
#define LED_TYPE     WS2812B
#define COLOR_ORDER  GRB

// =============================================================================
//  I2S / ADC  (MAX4466 → GPIO 35 / ADC1_CH7)
// =============================================================================
#define I2S_PORT      I2S_NUM_0
#define ADC_CHANNEL   ADC1_CHANNEL_7
#define SAMPLE_RATE   40000
#define FFT_SAMPLES   1024
#define DMA_BUF_LEN   512
#define DMA_BUF_COUNT 4

// =============================================================================
//  FREQUENCY WINDOW
//  bin resolution = 40000/1024 ≈ 39 Hz per bin
//  BIN_LOW   3  →  ~117 Hz  (skip DC + rumble)
//  BIN_HIGH 200 → ~7800 Hz
// =============================================================================
#define BIN_LOW    3
#define BIN_HIGH  200

// =============================================================================
//  DEFAULTS
// =============================================================================
#define DEFAULT_GAIN      2.5f
#define DEFAULT_FLOOR   250.0f
#define DEFAULT_MAX   22000.0f   // peak-bin values run higher than averages
#define DEFAULT_DECAY     0.85f
#define DEFAULT_ATTACK    0.70f
#define DEFAULT_BRIGHT   160
#define DEFAULT_PDROP     0.025f
#define DEFAULT_PHOLD     18

// =============================================================================
//  RUNTIME PARAMETERS
// =============================================================================
float   g_gain   = DEFAULT_GAIN;
float   g_floor  = DEFAULT_FLOOR;
float   g_max    = DEFAULT_MAX;
float   g_decay  = DEFAULT_DECAY;
float   g_attack = DEFAULT_ATTACK;
uint8_t g_bright = DEFAULT_BRIGHT;
float   g_pdrop  = DEFAULT_PDROP;
int     g_phold  = DEFAULT_PHOLD;

// =============================================================================
//  GLOBALS
// =============================================================================
CRGB  leds[NUM_LEDS];
float vReal[FFT_SAMPLES];
float vImag[FFT_SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE);

float   barHeights[NUM_COLS] = {0};   // smoothed bar height, 0.0–1.0
float   peakHold[NUM_COLS]   = {0};   // floating peak-dot position, 0.0–1.0
uint8_t peakTimer[NUM_COLS]  = {0};   // frames remaining in hold phase

int binStart[NUM_COLS];
int binEnd[NUM_COLS];

// =============================================================================
//  BIN BOUNDARIES  (logarithmic)
// =============================================================================
void computeBinBoundaries() {
    float logLow  = logf((float)BIN_LOW);
    float logHigh = logf((float)BIN_HIGH);

    for (int col = 0; col <= NUM_COLS; col++) {
        float t   = (float)col / (float)NUM_COLS;
        float bin = expf(logLow + t * (logHigh - logLow));
        if (col < NUM_COLS) binStart[col] = (int)bin;
        if (col > 0)        binEnd[col-1] = (int)bin;
    }
    binEnd[NUM_COLS - 1] = BIN_HIGH;

    for (int col = 1; col < NUM_COLS; col++) {
        if (binStart[col] <= binStart[col-1])
            binStart[col] = binStart[col-1] + 1;
        if (binEnd[col-1] <= binStart[col-1])
            binEnd[col-1] = binStart[col-1] + 1;
    }
    for (int col = 0; col < NUM_COLS; col++) {
        binStart[col] = constrain(binStart[col], BIN_LOW,           BIN_HIGH);
        binEnd[col]   = constrain(binEnd[col],   binStart[col] + 1, BIN_HIGH + 1);
    }

    float hz = (float)SAMPLE_RATE / (float)FFT_SAMPLES;
    Serial.println("[BinMap] col | Hz range | bins");
    for (int col = 0; col < NUM_COLS; col++) {
        Serial.printf("  col %2d | %4.0f–%4.0f Hz | bins %d–%d\n",
            col,
            binStart[col] * hz, binEnd[col] * hz,
            binStart[col], binEnd[col]);
    }
}

// =============================================================================
//  I2S INIT
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
    Serial.println("[I2S] ADC ready on GPIO 35.");
}

// =============================================================================
//  AUDIO CAPTURE + DC REMOVAL
//
//  After reading raw samples the actual mean is subtracted so a true
//  zero-mean signal enters the FFT. This prevents ADC DC bias from leaking
//  energy across all bins and causing the false low-frequency peaks seen
//  previously (peak freq: 4 Hz etc).
// =============================================================================
void captureAudio() {
    uint16_t raw[DMA_BUF_LEN];
    int n = 0;
    while (n < FFT_SAMPLES) {
        size_t bytes = 0;
        i2s_read(I2S_PORT, raw, sizeof(raw), &bytes, portMAX_DELAY);
        int count = bytes / sizeof(uint16_t);
        for (int i = 0; i < count && n < FFT_SAMPLES; i++, n++) {
            vReal[n] = (float)((raw[i] & 0x0FFF) - 2048);
            vImag[n] = 0.0f;
        }
    }
    // Remove residual DC offset
    float mean = 0.0f;
    for (int i = 0; i < FFT_SAMPLES; i++) mean += vReal[i];
    mean /= FFT_SAMPLES;
    for (int i = 0; i < FFT_SAMPLES; i++) vReal[i] -= mean;
}

// =============================================================================
//  LED INDEX  (serpentine, data enters top-left)
// =============================================================================
#define FLIP_ROWS true
#define FLIP_COLS false

int getLEDIndex(int col, int row) {
    int r = FLIP_ROWS ? (NUM_ROWS - 1 - row) : row;
    int c = FLIP_COLS ? (NUM_COLS - 1 - col) : col;
    return (r % 2 == 0) ? (r * NUM_COLS + c)
                        : (r * NUM_COLS + (NUM_COLS - 1 - c));
}

// =============================================================================
//  COLOR MAPPING
//
//  Bar body: per-row gradient, blue at bottom → red near top.
//  Peak dot: bright white so it always stands out above the bar body.
// =============================================================================
CRGB rowColor(int row) {
    // Row 0 = bottom (blue), Row 7 = top (red)
    float frac   = (float)row / (float)(NUM_ROWS - 1);  // 0.0–1.0
    uint8_t hue  = (uint8_t)((1.0f - frac) * 170.0f);  // 170=blue, 0=red
    uint8_t val  = (uint8_t)(130 + frac * 125.0f);      // brighter toward top
    return CHSV(hue, 255, val);
}

// =============================================================================
//  DRAW SPECTROGRAM
//
//  Each column draws:
//    1. A solid bar from row 0 up to barHeight, colored by row position.
//    2. A single bright white peak-hold dot above the bar that hangs briefly
//       then falls — gives the eye a clear anchor for musical transients.
// =============================================================================
void drawSpectrogram() {
    FastLED.clear();

    for (int col = 0; col < NUM_COLS; col++) {
        // ── Bar body ─────────────────────────────────────────────────────
        int barH = constrain((int)(barHeights[col] * NUM_ROWS + 0.5f), 0, NUM_ROWS);
        for (int row = 0; row < barH; row++) {
            leds[getLEDIndex(col, row)] = rowColor(row);
        }

        // ── Peak hold dot ─────────────────────────────────────────────────
        // Only draw if the dot is above the current bar top and has
        // meaningful height, so it never overlaps the bar body.
        int peakRow = (int)(peakHold[col] * NUM_ROWS - 0.5f);
        peakRow = constrain(peakRow, 0, NUM_ROWS - 1);
        if (peakHold[col] > 0.05f && peakRow >= barH) {
            leds[getLEDIndex(col, peakRow)] = CRGB(255, 255, 200);
        }
    }

    FastLED.setBrightness(g_bright);
    FastLED.show();
}

// =============================================================================
//  SERIAL COMMAND HANDLER
// =============================================================================
void printHelp() {
    Serial.println("─────────────────────────────────────────────────────");
    Serial.println("  gain   <float>   amplification          default 2.5");
    Serial.println("  floor  <float>   per-bin noise gate     default 250");
    Serial.println("  max    <float>   full-bar peak value    default 22000");
    Serial.println("  decay  <0–1>     bar fall speed         default 0.85");
    Serial.println("  attack <0–1>     bar rise speed         default 0.70");
    Serial.println("  bright <0–255>   LED brightness         default 160");
    Serial.println("  pdrop  <float>   peak-dot fall/frame    default 0.025");
    Serial.println("  phold  <int>     frames to hold peak    default 18");
    Serial.println("  status           show current values");
    Serial.println("  help             this list");
    Serial.println("─────────────────────────────────────────────────────");
    Serial.println("  QUICK TUNING:");
    Serial.println("  peak bar < 0.3  →  max 10000");
    Serial.println("  peak bar = 1.00 →  max 35000");
    Serial.println("  noise flicker   →  floor 500");
    Serial.println("  too responsive  →  decay 0.92");
    Serial.println("  too sluggish    →  decay 0.72, attack 0.90");
    Serial.println("─────────────────────────────────────────────────────");
}

void printStatus() {
    Serial.println("─────────────────────────────────────────────────────");
    Serial.printf ("  gain   = %.2f\n", g_gain);
    Serial.printf ("  floor  = %.1f\n", g_floor);
    Serial.printf ("  max    = %.0f\n", g_max);
    Serial.printf ("  decay  = %.3f\n", g_decay);
    Serial.printf ("  attack = %.3f\n", g_attack);
    Serial.printf ("  bright = %d\n",   g_bright);
    Serial.printf ("  pdrop  = %.3f\n", g_pdrop);
    Serial.printf ("  phold  = %d frames\n", g_phold);
    Serial.println("─────────────────────────────────────────────────────");
}

void handleSerial() {
    static String buf = "";
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            buf.trim();
            if (buf.length() == 0) { buf = ""; return; }
            int sp     = buf.indexOf(' ');
            String cmd = (sp >= 0) ? buf.substring(0, sp) : buf;
            String arg = (sp >= 0) ? buf.substring(sp + 1) : "";
            cmd.toLowerCase();
            float val  = arg.toFloat();

            if      (cmd == "help")   { printHelp(); }
            else if (cmd == "status") { printStatus(); }
            else if (cmd == "gain")   { g_gain   = max(0.1f, val);
                                        Serial.printf("[OK] gain = %.2f\n", g_gain); }
            else if (cmd == "floor")  { g_floor  = max(0.0f, val);
                                        Serial.printf("[OK] floor = %.1f\n", g_floor); }
            else if (cmd == "max")    { g_max    = max(100.0f, val);
                                        Serial.printf("[OK] max = %.0f\n", g_max); }
            else if (cmd == "decay")  { g_decay  = constrain(val, 0.0f, 0.99f);
                                        Serial.printf("[OK] decay = %.3f\n", g_decay); }
            else if (cmd == "attack") { g_attack = constrain(val, 0.01f, 1.0f);
                                        Serial.printf("[OK] attack = %.3f\n", g_attack); }
            else if (cmd == "bright") { g_bright = (uint8_t)constrain((int)val, 0, 255);
                                        Serial.printf("[OK] bright = %d\n", g_bright); }
            else if (cmd == "pdrop")  { g_pdrop  = constrain(val, 0.001f, 0.5f);
                                        Serial.printf("[OK] pdrop = %.3f\n", g_pdrop); }
            else if (cmd == "phold")  { g_phold  = constrain((int)val, 0, 120);
                                        Serial.printf("[OK] phold = %d\n", g_phold); }
            else { Serial.printf("[??] Unknown: '%s'  (type 'help')\n", cmd.c_str()); }
            buf = "";
        } else {
            if (buf.length() < 64) buf += c;
        }
    }
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n==========================================");
    Serial.println("  Spectrogram — ESP32 + MAX4466 + WS2812B");
    Serial.println("==========================================");

    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(g_bright);
    fill_solid(leds, NUM_LEDS, CRGB::White);
    FastLED.show();
    Serial.println("[LED] Matrix white — power OK.");

    computeBinBoundaries();

    // startAudioPipeline() calls analogSetAttenuation() internally;
    // setupI2S() must run after so i2s_adc_enable() owns ADC1 last.
    startAudioPipeline();
    setupI2S();

    delay(500);
    FastLED.clear();
    FastLED.show();

    printHelp();
    Serial.println("[Ready] Type 'status' to see current settings.");
}

// =============================================================================
//  MAIN LOOP
// =============================================================================
void loop() {
    handleSerial();

    // ── 1. Capture + DC removal ──────────────────────────────────────────
    captureAudio();

    // ── 2. Hann window + FFT ────────────────────────────────────────────
    FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
    // vReal[0..FFT_SAMPLES/2] now holds per-bin magnitudes

    // ── 3. Map bins → columns ────────────────────────────────────────────
    float peakThisFrame  = 0.0f;
    float peakFreqHz     = 0.0f;
    float peakFreqMag    = 0.0f;

    for (int col = 0; col < NUM_COLS; col++) {

        // Use the PEAK bin within this column's frequency range rather than
        // the average. Music has narrow harmonic peaks; averaging dilutes them
        // and makes all columns look similar. Peak gives sharp column-to-column
        // contrast so individual notes and harmonics are visible.
        float colPeak = 0.0f;
        for (int b = binStart[col]; b < binEnd[col]; b++) {
            if (vReal[b] > colPeak) colPeak = vReal[b];

            // Track the loudest bin in the whole display range for reporting.
            // This replaces FFT.majorPeak() which searched below BIN_LOW and
            // returned spurious low-frequency values (4 Hz, 27 Hz etc).
            if (vReal[b] > peakFreqMag) {
                peakFreqMag = vReal[b];
                peakFreqHz  = b * ((float)SAMPLE_RATE / (float)FFT_SAMPLES);
            }
        }

        // Floor → gain → normalise
        float level      = max(0.0f, colPeak - g_floor) * g_gain;
        float normalised = constrain(level / g_max, 0.0f, 1.0f);

        // Apply a gentle square-root curve so mid-level signals spread across
        // more of the 8-row height instead of all sitting at the bottom.
        normalised = sqrtf(normalised);

        // Asymmetric smoothing: fast attack, slow exponential decay
        if (normalised > barHeights[col]) {
            barHeights[col] += min(normalised - barHeights[col], g_attack);
        } else {
            barHeights[col] = barHeights[col] * g_decay
                            + normalised      * (1.0f - g_decay);
        }

        // ── Peak hold logic ───────────────────────────────────────────────
        if (normalised >= peakHold[col]) {
            // New peak reached: reset the hold timer
            peakHold[col]  = normalised;
            peakTimer[col] = (uint8_t)min(g_phold, 255);
        } else {
            if (peakTimer[col] > 0) {
                // Still in hold phase: dot stays put
                peakTimer[col]--;
            } else {
                // Hold expired: dot falls at g_pdrop per frame
                peakHold[col] = max(0.0f, peakHold[col] - g_pdrop);
            }
        }

        if (barHeights[col] > peakThisFrame)
            peakThisFrame = barHeights[col];
    }

    // ── 4. Render ────────────────────────────────────────────────────────
    drawSpectrogram();

    // ── 5. Diagnostics (every second) ────────────────────────────────────
    static uint32_t lastLog = 0;
    if (millis() - lastLog >= 1000) {
        lastLog = millis();
        Serial.printf("[FFT] peak bar: %.2f | peak freq: %.0f Hz | "
                      "gain=%.1f floor=%.0f max=%.0f\n",
                      peakThisFrame, peakFreqHz,
                      g_gain, g_floor, g_max);
    }

    vTaskDelay(pdMS_TO_TICKS(8));
}
