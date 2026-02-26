#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <FastLED.h>

// =============================================================================
//  HARDWARE CONFIGURATION
//  Change these if you rewire your setup.
// =============================================================================
#define I2S_PORT       I2S_NUM_0
#define ADC_CHANNEL    ADC1_CHANNEL_7   // GPIO35
#define LED_PIN        18               // FastLED data pin
#define MATRIX_WIDTH   32               // Number of frequency columns
#define MATRIX_HEIGHT  8                // Number of amplitude rows (1–8)
#define NUM_LEDS       (MATRIX_WIDTH * MATRIX_HEIGHT)

// =============================================================================
//  FFT / SAMPLING CONFIGURATION
//  SAMPLE_RATE and SAMPLES control frequency resolution.
//
//  Frequency resolution  = SAMPLE_RATE / SAMPLES
//  Max detectable freq   = SAMPLE_RATE / 2  (Nyquist)
//
//  At 20480 Hz sample rate and 512 samples:
//    - Each raw FFT bin is 40 Hz wide (20480 / 512)
//    - Max frequency is ~10240 Hz  ← good for music
//    - Total usable FFT bins: 256  (SAMPLES / 2)
//
//  If you raise SAMPLE_RATE you capture higher frequencies but lose
//  low-frequency detail. If you raise SAMPLES you get finer resolution
//  but each FFT frame takes longer to fill.
// =============================================================================
#define SAMPLE_RATE    20480
#define SAMPLES        512


// =============================================================================
//  ★ FREQUENCY BIN TUNING — EDIT THIS SECTION TO EXPERIMENT ★
//
//  These two values define the slice of the audio spectrum displayed
//  across the 32 columns of your matrix.
//
//  Recommended ranges for music:
//    - Sub-bass / kick drum:  20 –  80 Hz
//    - Bass / bass guitar:    80 – 300 Hz
//    - Midrange / vocals:    300 – 3000 Hz
//    - Presence / harmonics: 3000 – 6000 Hz
//    - Air / cymbals:        6000 – 10000 Hz
//
//  Setting MIN_FREQ = 60 and MAX_FREQ = 8000 covers the musically
//  interesting range well for a speaker in a room.
//
//  The mapping between columns and frequencies is LOGARITHMIC, meaning
//  low frequencies are spread across more columns (more detail in bass),
//  while high frequencies are compressed into fewer columns — matching
//  how human hearing actually works.
// =============================================================================
#define MIN_FREQ       60        // Hz — lowest frequency shown (leftmost column)
                                 // Lower this (e.g. 40) to capture more sub-bass.
                                 // Raise it (e.g. 100) to reduce rumble / mic noise.

#define MAX_FREQ       8000      // Hz — highest frequency shown (rightmost column)
                                 // Lower this (e.g. 5000) to zoom in on mids.
                                 // Raise it (e.g. 10000) to include more highs
                                 //   (capped by SAMPLE_RATE / 2 = 10240 Hz).


// =============================================================================
//  ★ AMPLITUDE TUNING — EDIT THIS SECTION TO EXPERIMENT ★
//
//  These values control how the FFT magnitude maps onto the 8 LED rows.
//
//  NOISE_FLOOR: Magnitude values below this are treated as silence and
//    clamped to zero. Raise it if idle noise lights up the bottom rows.
//    Lower it if quiet passages disappear entirely.
//    For a microphone picking up a nearby speaker, 500–1200 is typical.
//
//  AMPLITUDE_SCALE: Divides the magnitude to fit within 8 rows.
//    Raise this if the bars are always hitting the top (too sensitive).
//    Lower this if the bars rarely get above row 3 (not sensitive enough).
//    Start at 1500 and tune in increments of 200.
//
//  DECAY_RATE: How many rows the bar drops per frame when the signal falls.
//    Lower values (0.1) = slow, smooth fall — good for ambient / slow music.
//    Higher values (0.5) = fast, punchy fall — good for EDM / percussion.
// =============================================================================
#define NOISE_FLOOR       800    // Raw FFT magnitude — silence below this
#define AMPLITUDE_SCALE   1500   // Divisor: magnitude / AMPLITUDE_SCALE = rows
#define DECAY_RATE        0.2f   // Rows dropped per frame during decay


// =============================================================================
//  DERIVED CONSTANTS  (do not edit — calculated from values above)
// =============================================================================

// Width of one raw FFT bin in Hz
#define HZ_PER_BIN  ((float)SAMPLE_RATE / (float)SAMPLES)

// Convert a frequency in Hz to its FFT bin index
// Clamp to [1, SAMPLES/2-1] so we never read the DC bin (bin 0) or out-of-bounds
inline int freqToBin(float hz) {
  int bin = (int)(hz / HZ_PER_BIN);
  if (bin < 1)             bin = 1;
  if (bin >= SAMPLES / 2)  bin = (SAMPLES / 2) - 1;
  return bin;
}


// =============================================================================
//  GLOBALS
// =============================================================================
CRGB leds[NUM_LEDS];

float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLE_RATE);

// Smoothed visual heights for each column (float for sub-row decay precision)
float peakHeights[MATRIX_WIDTH] = {0};

// Shared amplitude array written by Core 0, read by Core 1.
// volatile prevents the compiler from caching stale values across cores.
volatile uint8_t columnAmplitudes[MATRIX_WIDTH] = {0};

TaskHandle_t AudioTaskHandle;


// =============================================================================
//  I2S SETUP
// =============================================================================
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 2,
    .dma_buf_len          = SAMPLES,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  i2s_adc_enable(I2S_PORT);
}


// =============================================================================
//  CORE 0 TASK — Audio sampling, FFT, bin mapping
// =============================================================================
void audioTask(void *pvParameters) {
  uint16_t i2sBuffer[SAMPLES];
  size_t   bytesRead;

  // ── Pre-compute the FFT bin boundaries for each column ──────────────────
  //
  //  We calculate these once at startup rather than every frame to save CPU.
  //
  //  The mapping is logarithmic: column c spans frequencies
  //    f_start(c) = MIN_FREQ * (MAX_FREQ / MIN_FREQ) ^ (c / MATRIX_WIDTH)
  //    f_end(c)   = MIN_FREQ * (MAX_FREQ / MIN_FREQ) ^ ((c+1) / MATRIX_WIDTH)
  //
  //  This is the standard "octave band" approach used in real spectrum
  //  analysers and matches the logarithmic nature of musical pitch.
  //
  //  Example at default settings (60–8000 Hz, 32 columns):
  //    Col  0:   60 –   70 Hz  (sub-bass, kick drum fundamental)
  //    Col  4:   87 –  102 Hz  (bass guitar open E)
  //    Col  8:  126 –  148 Hz  (bass guitar upper range)
  //    Col 12:  183 –  215 Hz  (lower midrange)
  //    Col 16:  266 –  312 Hz  (vocal fundamentals begin)
  //    Col 20:  386 –  453 Hz  (vocal formants)
  //    Col 24:  560 –  657 Hz  (guitar body resonance)
  //    Col 28: 1290 – 1513 Hz  (upper mids, presence)
  //    Col 31: 6834 – 8000 Hz  (air, cymbals, sibilance)
  //
  //  To print these boundaries to Serial for inspection, temporarily
  //  uncomment the Serial.printf lines in the loop below.
  // ────────────────────────────────────────────────────────────────────────

  int binStart[MATRIX_WIDTH];
  int binEnd[MATRIX_WIDTH];

  float freqRatio = (float)MAX_FREQ / (float)MIN_FREQ;

  for (int col = 0; col < MATRIX_WIDTH; col++) {
    float fStart = MIN_FREQ * pow(freqRatio, (float)col        / MATRIX_WIDTH);
    float fEnd   = MIN_FREQ * pow(freqRatio, (float)(col + 1)  / MATRIX_WIDTH);

    binStart[col] = freqToBin(fStart);
    binEnd[col]   = freqToBin(fEnd);

    // Guard: each column must span at least one FFT bin
    if (binEnd[col] <= binStart[col]) {
      binEnd[col] = binStart[col] + 1;
    }

    // Uncomment to debug boundaries on first boot:
    // Serial.printf("Col %2d: %.0f–%.0f Hz → bins %d–%d\n",
    //               col, fStart, fEnd, binStart[col], binEnd[col]);
  }

  // ── Main audio loop ──────────────────────────────────────────────────────
  for (;;) {

    // 1. Fill the sample buffer from the ADC via I2S DMA
    i2s_read(I2S_PORT, &i2sBuffer, sizeof(i2sBuffer), &bytesRead, portMAX_DELAY);

    // 2. Copy to FFT input array and remove DC offset
    //    DC offset (the non-zero average of a resting ADC signal) would
    //    dominate bin 0 and bleed into bin 1, so we subtract the mean.
    float mean = 0;
    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] = (float)i2sBuffer[i];
      vImag[i] = 0.0f;
      mean += vReal[i];
    }
    mean /= SAMPLES;
    for (int i = 0; i < SAMPLES; i++) vReal[i] -= mean;

    // 3. Apply Hamming window to reduce spectral leakage
    //    (prevents energy from one frequency "leaking" into adjacent bins)
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);

    // 4. Compute FFT and convert complex output to magnitudes
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
    // After this, vReal[0..SAMPLES/2-1] holds the magnitude of each bin.
    // vReal[0] is DC (skip it). vReal[1] is HZ_PER_BIN Hz, etc.

    // 5. Map FFT magnitudes onto the 32 matrix columns
    for (int col = 0; col < MATRIX_WIDTH; col++) {

      // Average the magnitudes across all raw FFT bins in this column's
      // frequency range. Averaging (vs. peak) makes the display more
      // stable and less twitchy for music content.
      float avg = 0;
      for (int b = binStart[col]; b < binEnd[col]; b++) {
        avg += vReal[b];
      }
      avg /= (binEnd[col] - binStart[col]);

      // ── Noise gate ──────────────────────────────────────────────────────
      //  Discard anything below NOISE_FLOOR. This prevents idle mic hiss
      //  and electrical noise from showing faint bars when music stops.
      //  Adjust NOISE_FLOOR in the tuning section above.
      if (avg < NOISE_FLOOR) avg = 0;

      // ── Scale magnitude to LED rows (0.0 – 8.0) ─────────────────────────
      //  Divide by AMPLITUDE_SCALE so that typical music peaks hit
      //  somewhere in the upper half of the matrix.
      //  Adjust AMPLITUDE_SCALE in the tuning section above.
      float targetHeight = avg / AMPLITUDE_SCALE;
      if (targetHeight > MATRIX_HEIGHT) targetHeight = MATRIX_HEIGHT;  // hard cap at 8

      // ── Temporal smoothing (gravity / decay) ─────────────────────────────
      //  If the new reading is higher → snap up immediately (instant attack).
      //  If the new reading is lower  → fall gradually at DECAY_RATE rows/frame.
      //  This gives the classic "bouncing bar" look.
      if (targetHeight >= peakHeights[col]) {
        peakHeights[col] = targetHeight;
      } else {
        peakHeights[col] -= DECAY_RATE;
        if (peakHeights[col] < 0) peakHeights[col] = 0;
      }

      // Write the integer height (1–8) to the shared array.
      // Cast truncates toward zero, so 0.9 → 0 (no LED), 1.0 → 1 (one LED).
      columnAmplitudes[col] = (uint8_t)peakHeights[col];
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}


// =============================================================================
//  SETUP (Core 1)
// =============================================================================
void setup() {
  Serial.begin(115200);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(50);
  FastLED.clear();
  FastLED.show();

  setupI2S();

  // Pin the audio/FFT task to Core 0 so it doesn't compete with FastLED on Core 1
  xTaskCreatePinnedToCore(
    audioTask,
    "Audio_FFT",
    10000,           // Stack depth in words — increase if you see stack overflows
    NULL,
    1,               // Task priority (1 = normal; raise to 2 if audio stutters)
    &AudioTaskHandle,
    0                // Core 0
  );
}


// =============================================================================
//  LOOP / DISPLAY (Core 1) — reads shared amplitudes, drives LEDs
// =============================================================================
void loop() {
  FastLED.clear();

  for (int x = 0; x < MATRIX_WIDTH; x++) {
    int height = columnAmplitudes[x];   // 0–8

    for (int y = 0; y < height; y++) {

      // ── Serpentine (snake) layout ───────────────────────────────────────
      //  WESIRI matrices wire columns in alternating directions:
      //    Even columns (0, 2, 4 …): LED 0 is at the BOTTOM, count upward
      //    Odd  columns (1, 3, 5 …): LED 0 is at the TOP,    count downward
      //
      //  If your matrix is wired differently (e.g. all columns bottom-up),
      //  remove the if/else and just use:
      //    pixelIndex = (x * MATRIX_HEIGHT) + y;
      int pixelIndex;
      if (x % 2 == 0) {
        pixelIndex = (x * MATRIX_HEIGHT) + y;
      } else {
        pixelIndex = (x * MATRIX_HEIGHT) + (MATRIX_HEIGHT - 1 - y);
      }

      // ── Colour: hue sweeps left-to-right across the full spectrum ────────
      //  Low-frequency columns (left)  → red / orange  (hue near 0)
      //  High-frequency columns (right) → blue / violet (hue near 255)
      //  Change the hue formula to experiment with other colour schemes, e.g.:
      //    Height-based:  CHSV(y * (255 / MATRIX_HEIGHT), 255, 255)
      //    Single colour: CHSV(160, 255, 255)  // solid blue
      leds[pixelIndex] = CHSV(x * (255 / MATRIX_WIDTH), 255, 255);
    }
  }

  FastLED.show();

  vTaskDelay(16 / portTICK_PERIOD_MS);  // ~60 FPS
}
