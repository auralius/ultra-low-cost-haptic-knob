# TelU-Knob
## Potentiometer for position and speed measurements
- The potentiometer wiper is read through the [__ADS1115__](https://www.ti.com/lit/ds/symlink/ads1115.pdf) on channel `A0`.
- The ADC is configured with `setGain(GAIN_FOUR)`, so the conversion from ADC code to voltage is: `voltage = adc0 * 0.00003125f`.
- In our setup, the potentiometer output spans approximately 0 to 1 V over about two turns of the shaft.

## Motor driver with current sensing
To drive the motor, we use [Arduino Motor Shield Rev3](https://docs.arduino.cc/tutorials/motor-shield-rev3/msr3-controlling-dc-motor/) which includes __current sensing__. The shield provides two motor channels (A and B), but in this setup we only use channel A.

Channel A:

- `D12` - Direction
- `D3` - PWM (work duty)
- `D9` - Brake
- `A0` - current sensing.

Channel B (not used):

- `D13` - Direction
- `D11` - PWM (work duty)
- `D8` - Brake
- `A1` - current sensing.

## Fast PWM
The Arduino Uno’s fastest PWM frequency is about 62.5 kHz. We’ll use this setting to push the motor noise out of the audible range and to smooth the current measurements.

```cpp
void setupHighFreqPwmOnTimer2() {
  // Pin 3 is OC2B, we need it as output
  pinMode(PWM_A, OUTPUT);

  // Stop Timer2
  TCCR2A = 0;
  TCCR2B = 0;

  // Fast PWM mode, TOP = 0xFF
  // WGM22:0 = 0b011 -> Fast PWM, 8-bit
  TCCR2A |= (1 << WGM20) | (1 << WGM21);
  // No prescaling: CS22:0 = 0b001
  TCCR2B |= (1 << CS20);

  // Non-inverting PWM on OC2B (pin 3):
  // Clear OC2B on compare match, set at BOTTOM
  TCCR2A |= (1 << COM2B1);

  // Start with 0% duty
  OCR2B = 0;
}
```

## DC motor deadzone
Since the DC motor is very low-cost, we can expect some imperfections in its behavior.

```cpp
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;   // ADS1115 instance

// Motor Shield Rev3, Channel A pins
const int PWM_A   = 3;   // OC2B
const int DIR_A   = 12;
const int BRAKE_A = 9;

// Pot / ADS config
// GAIN_FOUR → ~0.00003125 V per LSB
const float LSB_V = 0.00003125f;

// Deadzone detection
const float MOVE_THRESHOLD_V = 0.01f;   // 10 mV change = movement
const unsigned long STEP_DELAY_MS = 50; // delay between PWM steps

void setupHighFreqPwmOnTimer2() {
  pinMode(PWM_A, OUTPUT);
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << WGM20) | (1 << WGM21);
  TCCR2B |= (1 << CS20);
  TCCR2A |= (1 << COM2B1);
  OCR2B = 0;
}

// Read pot voltage with a small average to reduce noise
float readPotVoltage() {
  const int N = 4;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    int16_t raw = ads.readADC_SingleEnded(0);
    sum += raw;
  }
  float avgRaw = (float)sum / (float)N;
  return avgRaw * LSB_V;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Motor deadzone ramp test");

  // Motor setup
  pinMode(DIR_A, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  digitalWrite(BRAKE_A, LOW);  // release brake
  setupHighFreqPwmOnTimer2();   // High-frequency PWM on pin 3

  // ADS1115 init
  if (!ads.begin()) {
    Serial.println("Failed to find ADS1115. Check wiring!");
    while (1);
  }
  ads.setGain(GAIN_FOUR);  // ±1.024 V range

  Serial.println("Setup complete. Running ramps in CW then CCW.");
  delay(1000);
}

void loop() {
  // --- Forward ramp ---
  Serial.println("\n=== CW ramp ===");
  digitalWrite(DIR_A, HIGH);   // forward direction (flip if needed)
  digitalWrite(BRAKE_A, LOW);

  float baseline = readPotVoltage();
  bool movementDetected = false;

  for (int pwm = 0; pwm <= 255; pwm++) {
    OCR2B = pwm;  // set PWM duty on pin 3
    delay(STEP_DELAY_MS);

    float v = readPotVoltage();
    float dv = v - baseline;
    Serial.println(pwm);

    if (!movementDetected && fabs(dv) > MOVE_THRESHOLD_V) {
      Serial.print(">>> Movement detected (CW) at PWM = ");
      Serial.print(pwm);
      movementDetected = true;
      break;  // stop ramp once we detect motion
    }
  }

  // Stop motor
  OCR2B = 0;
  delay(2000);

  // --- Reverse ramp ---
  Serial.println("\n=== CCW ramp ===");
  digitalWrite(DIR_A, LOW);    // reverse direction (flip if needed)
  digitalWrite(BRAKE_A, LOW);

  baseline = readPotVoltage();
  movementDetected = false;

  for (int pwm = 0; pwm <= 255; pwm++) {
    OCR2B = pwm;
    delay(STEP_DELAY_MS);

    float v = readPotVoltage();
    float dv = v - baseline;
    Serial.println(pwm);

    if (!movementDetected && fabs(dv) > MOVE_THRESHOLD_V) {
      Serial.print(">>> Movement detected (CCW) at PWM = ");
      Serial.print(pwm);
      movementDetected = true;
      break;
    }
  }

  OCR2B = 0; // Stop motor
  Serial.println("\nDONE...");
  delay(1000);
}
```

And the results are:
```
>>> Movement detected (CW) at PWM = 119
>>> Movement detected (CCW) at PWM = 143
```
## Motor response

```
u range: -200.0 to 200.0
Estimated sample time Ts ≈ 11.00 ms
Identified discrete model: y[k+1] = 0.6796 * y[k] + 0.02367 * u[k]
Approx continuous model PWM -> speed:
  G(s) = 0.074 / (0.028 s + 1)
  DC gain ≈ 0.074 (V/s per PWM count)
  Time constant τ ≈ 28.5 ms
```
<img src="./images/systemid.png" width="400">

Therefore, the motor is close to :

$$G(s) \approx \frac{18.8}{0.03s + 1}$$

Motor time constant is 3 ms, thus, its approximate -3dB bandwidt will be 5.3 Hz.

## PID control

```cpp
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;   // default I2C address 0x48

// Motor Shield Rev3, Channel A pins
const int PWM_A   = 3;   // speed (PWM)
const int DIR_A   = 12;  // direction
const int BRAKE_A = 9;   // brake

// PID target: 1.00 V on the potentiometer
const float TARGET_V = 0.50f;

// PID gains (start here, then tune)
const float Kp = 400.0f;
const float Ki = 0.0f;
const float Kd = 0.0f;

// PID state
float integral   = 0.0f;
float lastError  = 0.0f;
unsigned long lastTimeMs = 0;

void setupHighFreqPwmOnTimer2() {
  // Pin 3 is OC2B, we need it as output
  pinMode(PWM_A, OUTPUT);

  // Stop Timer2
  TCCR2A = 0;
  TCCR2B = 0;

  // Fast PWM mode, TOP = 0xFF
  // WGM22:0 = 0b011 -> Fast PWM, 8-bit
  TCCR2A |= (1 << WGM20) | (1 << WGM21);
  // No prescaling: CS22:0 = 0b001
  TCCR2B |= (1 << CS20);

  // Non-inverting PWM on OC2B (pin 3):
  // Clear OC2B on compare match, set at BOTTOM
  TCCR2A |= (1 << COM2B1);

  // Start with 0% duty
  OCR2B = 0;
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);

  // Disable brake initially, motor free to move
  digitalWrite(BRAKE_A, LOW);
  setupHighFreqPwmOnTimer2();  // <-- NEW: high-frequency PWM

  analogWrite(PWM_A, 0);

  // ADS1115 init
  if (!ads.begin()) {
    Serial.println("Failed to find ADS1115. Check wiring!");
    while (1);
  }

  // Gain: ±1.024 V, ~0.00003125 V/LSB
  ads.setGain(GAIN_FOUR);

  // Optional: data rate (128 SPS is fine for position control)
  // ads.setDataRate(RATE_ADS1115_128SPS);

  Serial.println("System ready. Moving to 1.00 V position...");
}

void loop() {
  unsigned long nowMs = millis();

  // First loop: initialize time and skip PID math
  if (lastTimeMs == 0) {
    lastTimeMs = nowMs;
    return;
  }

  float dt = (nowMs - lastTimeMs) / 1000.0f;  // seconds
  if (dt <= 0.0f) dt = 0.001f;
  lastTimeMs = nowMs;

  // --- Read potentiometer via ADS1115 ---
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltage = adc0 * 0.00003125f;   // V per bit at GAIN_FOUR

  // --- PID math ---
  float error = TARGET_V - voltage;     // positive => need to increase voltage

  // Integrator with simple anti-windup clamp
  integral += error * dt;
  const float I_MAX = 5.0f;             // clamp integral term
  if (integral > I_MAX)  integral = I_MAX;
  if (integral < -I_MAX) integral = -I_MAX;

  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // --- Convert PID output to motor command ---
  // Output range: roughly -255 .. +255
  if (output > 255.0f)  output = 255.0f;
  if (output < -255.0f) output = -255.0f;

  // Small deadband around target to avoid jitter
  const float ERROR_DEADBAND = 0.005f;    // 5 mV
  int speedCmd = 0;

  if (fabs(error) < ERROR_DEADBAND) {
    // Close enough: stop and brake
    speedCmd = 0;
    digitalWrite(BRAKE_A, HIGH);
  } else {
    digitalWrite(BRAKE_A, LOW);

    // Choose direction based on sign of output
    if (output >= 0.0f) {
      digitalWrite(DIR_A, HIGH); // If this moves away from target, flip HIGH/LOW
      speedCmd = (int)fabs(output);
    } else {
      digitalWrite(DIR_A, LOW);
      speedCmd = (int)fabs(output);
    }

    if (speedCmd > 255) speedCmd = 255;
  }

  if ((speedCmd > 0) && (speedCmd < 160)) speedCmd = 160;
  analogWrite(PWM_A, speedCmd);

  // --- Optional debug output (comment out if it slows things too much) ---
  static unsigned long lastPrint = 0;
  if (nowMs - lastPrint > 100) { // print at ~10 Hz
    lastPrint = nowMs;
    Serial.print("V = ");
    Serial.print(voltage, 4);
    Serial.print(" V, err = ");
    Serial.print(error, 4);
    Serial.print(", out = ");
    Serial.print(output, 1);
    Serial.print(", pwm = ");
    Serial.println(speedCmd);
  }
}
```

# Programmable detent positions

```cpp
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;   // default I2C address 0x48

// Motor Shield Rev3, Channel A pins
const int PWM_A   = 3;   // speed (PWM)
const int DIR_A   = 12;  // direction
const int BRAKE_A = 9;   // brake

// Detent config: one detent every 0.2 V
const float DETENT_STEP_V      = 0.02f;
// With GAIN_FOUR we care about ~0..1.0 V range
const float MAX_RANGE_V        = 1.0f;
const int   MAX_DETENT_INDEX   = (int)(MAX_RANGE_V / DETENT_STEP_V); // 5 -> 0..1.0 V

// PID gains (start here, then tune)
const float Kp = 400.0f;
const float Ki = 0.0f;
const float Kd = 0.0f;

// PID state
float integral   = 0.0f;
float lastError  = 0.0f;
unsigned long lastTimeMs = 0;

void setupHighFreqPwmOnTimer2() {
  // Pin 3 is OC2B, we need it as output
  pinMode(PWM_A, OUTPUT);

  // Stop Timer2
  TCCR2A = 0;
  TCCR2B = 0;

  // Fast PWM mode, TOP = 0xFF
  // WGM22:0 = 0b011 -> Fast PWM, 8-bit
  TCCR2A |= (1 << WGM20) | (1 << WGM21);
  // No prescaling: CS22:0 = 0b001
  TCCR2B |= (1 << CS20);

  // Non-inverting PWM on OC2B (pin 3):
  // Clear OC2B on compare match, set at BOTTOM
  TCCR2A |= (1 << COM2B1);

  // Start with 0% duty
  OCR2B = 0;
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);

  // Disable brake initially, motor free to move
  digitalWrite(BRAKE_A, LOW);
  setupHighFreqPwmOnTimer2();  // high-frequency PWM

  analogWrite(PWM_A, 0);       // harmless

  // ADS1115 init
  if (!ads.begin()) {
    Serial.println("Failed to find ADS1115. Check wiring!");
    while (1);
  }

  // Gain: ±1.024 V, ~0.00003125 V/LSB
  ads.setGain(GAIN_FOUR);

  // Optional: data rate (128 SPS is fine for position control)
  // ads.setDataRate(RATE_ADS1115_128SPS);

  Serial.println("System ready. Detents every 0.2 V (0.0 .. 1.0 V)...");
}

void loop() {
  unsigned long nowMs = millis();

  // First loop: initialize time and skip PID math
  if (lastTimeMs == 0) {
    lastTimeMs = nowMs;
    return;
  }

  float dt = (nowMs - lastTimeMs) / 1000.0f;  // seconds
  if (dt <= 0.0f) dt = 0.001f;
  lastTimeMs = nowMs;

  // --- Read potentiometer via ADS1115 ---
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltage = adc0 * 0.00003125f;   // V per bit at GAIN_FOUR

  // --- Compute detent target ---
  // Clamp into 0..MAX_RANGE_V first
  float vClamped = voltage;
  if (vClamped < 0.0f)         vClamped = 0.0f;
  if (vClamped > MAX_RANGE_V)  vClamped = MAX_RANGE_V;

  // Find nearest detent index
  float detentIndexF = roundf(vClamped / DETENT_STEP_V);
  if (detentIndexF < 0.0f) detentIndexF = 0.0f;
  if (detentIndexF > (float)MAX_DETENT_INDEX) detentIndexF = (float)MAX_DETENT_INDEX;

  int detentIndex = (int)detentIndexF;
  float targetV = detentIndex * DETENT_STEP_V;  // snapped target

  // --- PID math ---
  float error = targetV - voltage;     // positive => need to increase voltage

  // Integrator with simple anti-windup clamp (Ki is 0 for now)
  integral += error * dt;
  const float I_MAX = 5.0f;             // clamp integral term
  if (integral > I_MAX)  integral = I_MAX;
  if (integral < -I_MAX) integral = -I_MAX;

  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // --- Convert PID output to motor command ---
  // Output range: roughly -255 .. +255
  if (output > 255.0f)  output = 255.0f;
  if (output < -255.0f) output = -255.0f;

  // Small deadband around target to avoid jitter
  const float ERROR_DEADBAND = 0.005f;    // 5 mV
  int speedCmd = 0;

  if (fabs(error) < ERROR_DEADBAND) {
    // Close enough: stop and brake
    speedCmd = 0;
    digitalWrite(BRAKE_A, HIGH);
  } else {
    digitalWrite(BRAKE_A, LOW);

    // Choose direction based on sign of output
    if (output >= 0.0f) {
      digitalWrite(DIR_A, HIGH); // If this moves away from target, flip HIGH/LOW
      speedCmd = (int)fabs(output);
    } else {
      digitalWrite(DIR_A, LOW);
      speedCmd = (int)fabs(output);
    }

    if (speedCmd > 255) speedCmd = 255;
  }

  // Minimum PWM to overcome static friction
  if ((speedCmd > 0) && (speedCmd < 160)) speedCmd = 160;

  analogWrite(PWM_A, speedCmd);

  // --- Debug output (~10 Hz) ---
  static unsigned long lastPrint = 0;
  if (nowMs - lastPrint > 100) {
    lastPrint = nowMs;
    Serial.print("V = ");
    Serial.print(voltage, 4);
    Serial.print(" V, targetV = ");
    Serial.print(targetV, 4);
    Serial.print(" V, detent #");
    Serial.print(detentIndex);
    Serial.print(", err = ");
    Serial.print(error, 4);
    Serial.print(", out = ");
    Serial.print(output, 1);
    Serial.print(", pwm = ");
    Serial.println(speedCmd);
  }
}

```