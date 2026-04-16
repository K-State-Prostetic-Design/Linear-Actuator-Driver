// EMG Filter - BioAmp EXG Pill
// https://github.com/upsidedownlabs/BioAmp-EXG-Pill

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2021 - 2024 Upside Down Labs - contact@upsidedownlabs.tech

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sub-license, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Samples per second
#define SAMPLE_RATE 500 // try 60 for less sampling, default was 500

// Make sure to set the same baud rate on your Serial Monitor/Plotter
#define BAUD_RATE 115200

#define WAIT_UNTIL_RETRACTION_MILLI_SECONDS 50

// Change if not using A0 analog pin
#define INPUT_PIN A0
#define SERVO A7 // unused for now

#define INDEX_CONTRACTION 2
#define INDEX_RELAXATION 3

#define MIDDLE_CONTRACTION 4
#define MIDDLE_RELAXATION 5

#define RING_CONTRACTION 6
#define RING_RELAXATION 7

#define PINKY_CONTRACTION 8
#define PINKY_RELAXATION 9

#define THUMB_CONTRACTION 10
#define THUMB_RELAXATION 11

enum State {
    ExtendHand,
    WaitToRetract,
    RetractHand,
    Wait,
} STATE;

typedef enum Finger {
  Index,
  Middle,
  Ring,
  Pinky,
  Thumb
} Finger;

const Finger fingers[5] = {
    Index,
    Middle,
    Ring,
    Pinky,
    Thumb
};

const int contractionPins[5] = {
  INDEX_CONTRACTION,
  MIDDLE_CONTRACTION,
  RING_CONTRACTION,
  PINKY_CONTRACTION,
  THUMB_CONTRACTION
};

const int relaxationPins[5] = {
  INDEX_RELAXATION,
  MIDDLE_RELAXATION,
  RING_RELAXATION,
  PINKY_RELAXATION,
  THUMB_RELAXATION
};

void Extend(Finger finger) {
  digitalWrite(contractionPins[finger], HIGH);
  digitalWrite(relaxationPins[finger], LOW);
}

void Retract(Finger finger) {
  digitalWrite(contractionPins[finger], LOW);
  digitalWrite(relaxationPins[finger], HIGH);
}

void Stop(Finger finger) {
    digitalWrite(contractionPins[finger], LOW);
    digitalWrite(relaxationPins[finger], LOW);
}

void ExtendAll() {
    for (int i = 0; i < 5; i++) {
        Extend(fingers[i]);
    }
}

void RetractAll() {
    for (int i = 0; i < 5; i++) {
        Retract(fingers[i]);
    }
}

void StopAll() {
    for (int i = 0; i < 5; i++) {
        Stop(fingers[i]);
    }
}

/// Smoothing for a chaotic system
struct ExponentialMovingAverage {
  private:
    float ema = 0.0;
    float alpha;
    float last_input = 0.0;

  public:
    /// EMA Constructor
    ExponentialMovingAverage(float alpha) {
      this->alpha = alpha;
    }

    /// Get next smoothed value based on the last average
    float update(float input) {
      float input_f = (float)input;
      float max_slope = abs(last_input - ema);

      float go_to = alpha * input_f + (1.0f - alpha) * ema;
      float slope = go_to - ema;

      if  (slope > max_slope) { slope = max_slope; }
      else if  (slope < -max_slope / 2) { slope = -max_slope / 2; }

      ema += slope;
      last_input = input_f;

      return ema;
    }
} ema(0.15);

unsigned long past = 0;
float angle = 0;
float highest = 0;
bool closing = false;
unsigned long last_switched_states = 0;
unsigned long total_extension_time = 0;

unsigned long sequenceStartTime = 0;
int sequenceStep = 0; // 0=Idle, 1=Retract, 2=Idle, 3=Extend
unsigned long sequenceDuration = 5000; // Time spent Extending/Retracting
int EMGThreshold = 15; // Threshold for Linear Actuator Activation

void setup()
{
  STATE = Wait;
  // Serial connection begin
  Serial.begin(BAUD_RATE);
  Serial.println("Serial Setup!");
  total_extension_time = 0;
  Serial.println("Starting Loop!");
}

void loop()
{
  // Calculate elapsed time
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  // Run timer
  static long timer = 0;
  timer -= interval;

  // Sample
  if (timer < 0)
  {
    timer += 1000000 / SAMPLE_RATE;

    // Get analog input value (Raw EMG)
    float sensor_value = analogRead(INPUT_PIN);

    // Apply the band-stop filter (48 Hz to 52 Hz)
    float bandstop_filtered = BandStopFilter(sensor_value);
    Serial.print("Base:100");
    //Serial.print(", BandStop:");
    //Serial.print(bandstop_filtered);

    // Apply the high-pass filter (70 Hz)
    float highpass_filtered = HighPassFilter(bandstop_filtered);

    // Print the final filtered signal
    Serial.print(", HighPass:");
    Serial.print(highpass_filtered);

    Serial.print(", Abs:");
    Serial.print(abs(highpass_filtered));

    float smoothed = ema.update(abs(highpass_filtered));
    Serial.print(", Smoothed:");
    Serial.print(smoothed);

   //Checks for EMG signal
    if (smoothed >= EMGThreshold && sequenceStep == 0) {
      sequenceStep = 1;               // Switch to step 1
      sequenceStartTime = millis();   // Start stopwatch
    }

    // Retraction Step
    if (sequenceStep == 1) {
      RetractAll();
      if (millis() - sequenceStartTime >= sequenceDuration) {
        sequenceStep = 2;             // Switch to step 3
        StopAll();                    // Turn off actuators
        sequenceStartTime = millis(); // Restart stopwatch
      }
    } 
    // Checks for EMG signal
    else if (smoothed >= EMGThreshold && sequenceStep == 2) {
      sequenceStep = 3;               // Switch to step 3
      sequenceStartTime = millis();   // Reset stopwatch
    } 
    else if (sequenceStep == 3) {
      // Step 3: Extend for 6 seconds
      ExtendAll();
      if (millis() - sequenceStartTime >= sequenceDuration) {
        sequenceStep = 0;             // Switch to step 0
        StopAll();                    // Turn off actuators
      }
    } 
    else {
      // Step 0: Idle state (waiting for the next EMG spike)
      StopAll();
    }
    
  } // <-- This is the closing bracket for your if (timer < 0) block

  Serial.println();
}
// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: 70.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float HighPassFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -0.82523238 * z1 - 0.29463653 * z2;
    output = 0.52996723 * x + -1.05993445 * z1 + 0.52996723 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float BandStopFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.56858163 * z1 - 0.96424138 * z2;
    output = 0.96508099 * x + -1.56202714 * z1 + 0.96508099 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.61100358 * z1 - 0.96592171 * z2;
    output = 1.00000000 * x + -1.61854514 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
