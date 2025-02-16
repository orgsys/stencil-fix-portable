#include <Arduino.h>
// #include <Adafruit_NeoPixel.h>  // Commented out NeoPixel
#include <Servo.h>

class LowPass
{
private:
  float _previousPrefilteredAnalog;
  unsigned long _previousMicros = 0;

public:
  float RCus;
  LowPass(float _RCus) : RCus(_RCus) {}
  float step(float prefilteredAnalog)
  {
    if (_previousMicros == 0)
    {
      _previousMicros = micros();
      _previousPrefilteredAnalog = prefilteredAnalog;
      return prefilteredAnalog;
    }
    unsigned long now = micros();
    unsigned long dt = now - _previousMicros;
    float alpha = dt / (RCus * dt);
    _previousMicros = now;
    _previousPrefilteredAnalog += alpha * (prefilteredAnalog - _previousPrefilteredAnalog);
    return _previousPrefilteredAnalog;
  }
};

// Commented out all NeoPixel definitions
// #define NUMPIXELS 1
// Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

Servo esc; // renamed from servo to esc for clarity

// low pass filter with an RC time constant of 40us
LowPass lowPass(40);

// Constants for ESC control
const int ARM_PULSE = 1000;   // ESC arming pulse
const int START_PULSE = 1200; // Motor starts spinning
const int MAX_PULSE = 2000;   // Maximum speed

void setup()
{
  Serial.begin(115200);
  Serial.println("System starting...");

  // Removed NeoPixel power control code
  // #if defined(NEOPIXEL_POWER)
  //   pinMode(NEOPIXEL_POWER, OUTPUT);
  //   digitalWrite(NEOPIXEL_POWER, HIGH);
  // #endif

  // Removed NeoPixel initialization
  // pixels.begin();
  // pixels.setBrightness(127);

  pinMode(A7, INPUT);
  // Removed analogReadResolution as it's not available on Nano
  // analogReadResolution(12);

  esc.attach(11, ARM_PULSE, MAX_PULSE);
  // esc.writeMicroseconds(ARM_PULSE);
  // Serial.println("ESC initialized");

  delay(1000);
}

void loop()
{
  int pot = analogRead(A7);

  // Direct mapping from pot to ESC range
  int escValue = map(pot, 0, 1023, START_PULSE, MAX_PULSE);
  float filteredValue = lowPass.step(escValue);

  // Simple debug output
  Serial.print(pot);
  Serial.print(" -> ");
  Serial.println(filteredValue);

  esc.writeMicroseconds(filteredValue);

  delay(20);
}
