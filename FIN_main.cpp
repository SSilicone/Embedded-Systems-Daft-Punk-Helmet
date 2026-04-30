// =============================================================================
//  main.cpp
//  ESP32 WROVER + MAX4466 → real-time audio spectrogram on WS2812B 8×32 matrix
//
//  IMPROVED VERSION  ─  noise-robust, perceptually balanced, intuitive
//                       attack / decay semantics.
//
//  PROCESSING PIPELINE (per frame, ~30 Hz):
//    1.  Capture 1024 ADC samples from the MAX4466 → remove DC.
//    2.  Hann window + FFT → 512 magnitude bins.
//    3.  For each of 32 display columns:
//          a. Combine the column's bins (70 % peak / 30 % mean) so a single
//             noisy bin can't dominate, but harmonics still pop visibly.
//          b. Convert to dB — human hearing is logarithmic; the display
//             should be too. This stops loud signals from saturating the
//             top while quiet signals stay invisible at the bottom.
//          c. Apply a gentle high-frequency tilt: music is naturally pink
//             (low-freq heavy), so without compensation the left side of
//             the matrix always wins regardless of what's actually playing.
//          d. Subtract a per-column ADAPTIVE noise floor that slowly tracks
//             whatever steady background level each column sees. This kills
//             60 Hz hum, fan noise, mic self-noise, and PSU coupling
//             without any manual tuning.
//          e. Hard gate anything within `margin` dB of the floor → 0
//             (eliminates residual flicker on quiet bars).
//          f. Normalize against a fixed dB `range` so bar height represents
//             "how much louder than this column's own background".
//          g. Asymmetric attack / decay smoothing — BOTH are now smoothing
//             coefficients in 0..1 with the same meaning ("fraction of the
//             way to target per frame"). Higher = more responsive.
//    4.  Render per-row colored bar plus a peak-hold dot.
//
//  SERIAL COMMANDS (115200 baud):
//    attack <0–1>    rise responsiveness     default 0.45  (higher = snappier)
//    decay  <0–1>    fall responsiveness     default 0.08  (lower  = bars hang)
//    range  <dB>     dB above floor = full   default 28.0
//    tilt   <dB>     high-freq boost (total) default 8.0
//    margin <dB>     gate above noise floor  default 4.0
//    bright <0–255>  LED brightness          default 160
//    pdrop  <float>  peak dot fall/frame     default 0.020
//    phold  <int>    frames to hold peak     default 18
//    reset           re-learn the noise floor from current input
//    status / help
// =============================================================================

#include <Arduino.h>
#include <driver/i2s.h>
#include <FastLED.h>
#include <arduinoFFT.h>
#include "audio_pipeline.h"

// =============================================================================
//  HARDWARE
// =============================================================================
#define LED_PIN      13
#define NUM_COLS     32
#define NUM_ROWS      8
#define NUM_LEDS     (NUM_COLS * NUM_ROWS)
#define LED_TYPE     WS2812B
#define COLOR_ORDER  GRB

#define I2S_PORT      I2S_NUM_0
#define ADC_CHANNEL   ADC1_CHANNEL_7      // GPIO 35
#define SAMPLE_RATE   40000
#define FFT_SAMPLES   1024
#define DMA_BUF_LEN   512
#define DMA_BUF_COUNT 4

// 40000 / 1024 ≈ 39 Hz/bin.  3 → ~117 Hz, 200 → ~7800 Hz.
#define BIN_LOW    3
#define BIN_HIGH  200

// =============================================================================
//  RUNTIME PARAMETERS  (all live-tunable over serial)
// =============================================================================
float   g_attack   = 0.45f;    // 0..1, fraction of remaining gap closed per frame on rise
float   g_decay    = 0.08f;    // 0..1, ditto on fall
float   g_rangeDb  = 28.0f;    // dB above floor that fills the full bar
float   g_tiltDb   = 8.0f;     // total high-frequency boost across the spectrum (dB)
float   g_marginDb = 4.0f;     // hard gate: any value within this many dB of floor → 0
uint8_t g_bright   = 160;
float   g_pdrop    = 0.020f;
int     g_phold    = 18;

// =============================================================================
//  STATE
// =============================================================================
CRGB  leds[NUM_LEDS];
float vReal[FFT_SAMPLES];
float vImag[FFT_SAMPLES];
ArduinoFFT<float> FFT(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE);

float   barHeights[NUM_COLS]   = {0};   // smoothed display height, 0..1
float   peakHold[NUM_COLS]     = {0};   // floating peak-dot position, 0..1
uint8_t peakTimer[NUM_COLS]    = {0};   // hold-phase frame counter
float   noiseFloorDb[NUM_COLS];         // per-column adaptive noise floor (dB)
bool    noiseFloorInit = false;         // false → seed on first frame

int binStart[NUM_COLS];
int binEnd[NUM_COLS];

// =============================================================================
//  LOGARITHMIC BIN BOUNDARIES
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
    // Subtract the actual mean so the FFT sees a true zero-mean signal.
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
//  COLORS — blue at the bottom, red at the top, white peak dot.
// =============================================================================
CRGB rowColor(int row) {
    float frac   = (float)row / (float)(NUM_ROWS - 1);   // 0..1, bottom→top
    uint8_t hue  = (uint8_t)((1.0f - frac) * 170.0f);    // 170 blue → 0 red
    uint8_t val  = (uint8_t)(130 + frac * 125.0f);       // brighter near top
    return CHSV(hue, 255, val);
}

void drawSpectrogram() {
    FastLED.clear();
    for (int col = 0; col < NUM_COLS; col++) {
        // Bar body
        int barH = constrain((int)(barHeights[col] * NUM_ROWS + 0.5f), 0, NUM_ROWS);
        for (int row = 0; row < barH; row++) {
            leds[getLEDIndex(col, row)] = rowColor(row);
        }
        // Peak-hold dot — only above the bar so it never gets buried.
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
//  SERIAL COMMANDS
// =============================================================================
void printHelp() {
    Serial.println("─────────────────────────────────────────────────────");
    Serial.println("  attack <0–1>    rise speed             default 0.45");
    Serial.println("  decay  <0–1>    fall speed             default 0.08");
    Serial.println("  range  <dB>     dB → full bar          default 28.0");
    Serial.println("  tilt   <dB>     high-freq boost        default 8.0");
    Serial.println("  margin <dB>     gate above noise floor default 4.0");
    Serial.println("  bright <0–255>  LED brightness         default 160");
    Serial.println("  pdrop  <float>  peak-dot fall/frame    default 0.020");
    Serial.println("  phold  <int>    frames to hold peak    default 18");
    Serial.println("  reset           relearn noise floor");
    Serial.println("  status          show current values");
    Serial.println("  help            this list");
    Serial.println("─────────────────────────────────────────────────────");
    Serial.println("  TUNING TIPS");
    Serial.println("  too jumpy / noisy   →  margin 7,  decay 0.05");
    Serial.println("  bars look frozen    →  attack 0.7, decay 0.20");
    Serial.println("  only left side lit  →  tilt 14");
    Serial.println("  bars never reach 8  →  range 22");
    Serial.println("  bars always pegged  →  range 36");
    Serial.println("  moved to new room   →  reset");
    Serial.println("─────────────────────────────────────────────────────");
}

void printStatus() {
    Serial.println("─────────────────────────────────────────────────────");
    Serial.printf ("  attack = %.3f\n", g_attack);
    Serial.printf ("  decay  = %.3f\n", g_decay);
    Serial.printf ("  range  = %.1f dB\n", g_rangeDb);
    Serial.printf ("  tilt   = %.1f dB\n", g_tiltDb);
    Serial.printf ("  margin = %.1f dB\n", g_marginDb);
    Serial.printf ("  bright = %d\n",     g_bright);
    Serial.printf ("  pdrop  = %.3f\n",   g_pdrop);
    Serial.printf ("  phold  = %d frames\n", g_phold);
    Serial.println("─────────────────────────────────────────────────────");
}

void resetNoiseFloor() {
    for (int i = 0; i < NUM_COLS; i++) noiseFloorDb[i] = 0.0f;
    noiseFloorInit = false;
    Serial.println("[OK] Noise floor reset; will re-adapt over the next ~2 seconds.");
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

            if      (cmd == "help")   printHelp();
            else if (cmd == "status") printStatus();
            else if (cmd == "reset")  resetNoiseFloor();
            else if (cmd == "attack") { g_attack   = constrain(val, 0.01f, 1.0f);
                                        Serial.printf("[OK] attack = %.3f\n", g_attack); }
            else if (cmd == "decay")  { g_decay    = constrain(val, 0.001f, 1.0f);
                                        Serial.printf("[OK] decay  = %.3f\n", g_decay); }
            else if (cmd == "range")  { g_rangeDb  = constrain(val, 5.0f, 80.0f);
                                        Serial.printf("[OK] range  = %.1f dB\n", g_rangeDb); }
            else if (cmd == "tilt")   { g_tiltDb   = constrain(val, 0.0f, 30.0f);
                                        Serial.printf("[OK] tilt   = %.1f dB\n", g_tiltDb); }
            else if (cmd == "margin") { g_marginDb = constrain(val, 0.0f, 30.0f);
                                        Serial.printf("[OK] margin = %.1f dB\n", g_marginDb); }
            else if (cmd == "bright") { g_bright   = (uint8_t)constrain((int)val, 0, 255);
                                        Serial.printf("[OK] bright = %d\n", g_bright); }
            else if (cmd == "pdrop")  { g_pdrop    = constrain(val, 0.001f, 0.5f);
                                        Serial.printf("[OK] pdrop  = %.3f\n", g_pdrop); }
            else if (cmd == "phold")  { g_phold    = constrain((int)val, 0, 120);
                                        Serial.printf("[OK] phold  = %d frames\n", g_phold); }
            else { Serial.printf("[??] Unknown: '%s'  (type 'help')\n", cmd.c_str()); }
            buf = "";
        } else if (buf.length() < 64) {
            buf += c;
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
    resetNoiseFloor();

    // Pipeline first; setupI2S afterwards so i2s_adc_enable() is the last
    // thing to claim ADC1 (matches the working order from the original code).
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

    // 1. Capture + DC remove
    captureAudio();

    // 2. Hann + FFT
    FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    // 3. Per-column processing
    float frameMaxDb  = -1e9f;
    int   frameMaxCol = 0;

    for (int col = 0; col < NUM_COLS; col++) {

        // 3a. Combine bins: 70% peak + 30% mean.
        //     Peak alone is too jittery (one noisy bin spikes the column);
        //     mean alone smears harmonics. The blend keeps transients sharp
        //     but stabilises the floor.
        float peak = 0.0f;
        float sum  = 0.0f;
        int   nb   = binEnd[col] - binStart[col];
        for (int b = binStart[col]; b < binEnd[col]; b++) {
            float v = vReal[b];
            if (v > peak) peak = v;
            sum += v;
        }
        float mag = peak * 0.7f + (sum / max(nb, 1)) * 0.3f;

        // 3b. dB scale. The +1 floor inside log keeps it finite when mag→0.
        float magDb = 20.0f * log10f(mag + 1.0f);

        // 3c. Linear frequency tilt. col 0 → +0 dB, col 31 → +g_tiltDb dB.
        magDb += g_tiltDb * ((float)col / (float)(NUM_COLS - 1));

        // 3d. Adaptive per-column noise floor.
        //     Track minima quickly (so we lock onto silence within a second
        //     of starting up or after `reset`), but leak upward extremely
        //     slowly so a sustained loud signal can't drag the floor with
        //     it. Time constant on the upward leak is ~20–25 s at 30 fps.
        if (!noiseFloorInit) {
            noiseFloorDb[col] = magDb;
        } else if (magDb < noiseFloorDb[col]) {
            noiseFloorDb[col] = noiseFloorDb[col] * 0.6f   + magDb * 0.4f;
        } else {
            noiseFloorDb[col] = noiseFloorDb[col] * 0.9985f + magDb * 0.0015f;
        }

        // 3e + 3f. Subtract floor + margin, normalise against fixed range.
        float relDb  = magDb - noiseFloorDb[col] - g_marginDb;
        float target = (relDb <= 0.0f) ? 0.0f
                                       : constrain(relDb / g_rangeDb, 0.0f, 1.0f);

        // 3g. Asymmetric attack/decay using consistent smoothing-coefficient
        //     semantics. Both knobs mean the same thing now: "fraction of
        //     the way to target per frame". Try attack=0.9 vs 0.2 and you
        //     will actually see the difference.
        float diff = target - barHeights[col];
        if (diff > 0.0f)
            barHeights[col] += diff * g_attack;
        else
            barHeights[col] += diff * g_decay;

        // Peak-hold dot (operates on the pre-smoothed target so it stays
        // sharp regardless of decay setting).
        if (target >= peakHold[col]) {
            peakHold[col]  = target;
            peakTimer[col] = (uint8_t)min(g_phold, 255);
        } else if (peakTimer[col] > 0) {
            peakTimer[col]--;
        } else {
            peakHold[col] = max(0.0f, peakHold[col] - g_pdrop);
        }

        if (magDb > frameMaxDb) { frameMaxDb = magDb; frameMaxCol = col; }
    }
    if (!noiseFloorInit) noiseFloorInit = true;

    // 4. Render
    drawSpectrogram();

    // 5. Diagnostics every second
    static uint32_t lastLog = 0;
    if (millis() - lastLog >= 1000) {
        lastLog = millis();
        float maxBar = 0.0f;
        for (int c = 0; c < NUM_COLS; c++)
            if (barHeights[c] > maxBar) maxBar = barHeights[c];
        float floorAvg = 0.0f;
        for (int c = 0; c < NUM_COLS; c++) floorAvg += noiseFloorDb[c];
        floorAvg /= NUM_COLS;
        float hzCenter = ((binStart[frameMaxCol] + binEnd[frameMaxCol]) * 0.5f)
                       * ((float)SAMPLE_RATE / (float)FFT_SAMPLES);
        Serial.printf("[FFT] maxBar=%.2f  loudestCol=%d (%.0f Hz, %.1f dB)  floor≈%.1f dB\n",
            maxBar, frameMaxCol, hzCenter, frameMaxDb, floorAvg);
    }

    vTaskDelay(pdMS_TO_TICKS(8));
}
