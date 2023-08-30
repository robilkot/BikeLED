#include <FastLED.h>
#include <TimerMs.h>

//#define DEBUG

#ifdef DEBUG
const uint8_t LedCount = 4;
#else
const uint8_t LedCount = 120;
#endif
const uint8_t LedPin = 9;
const uint8_t TransitionSpeed = 30;

const uint8_t TachoPin = 4;
const uint16_t SpeedMeasureInterval = 900;
const float WheelRadius = 0.255; // Diameter ~510 mm

const uint8_t RedAmplitude = 170;

CRGB leds[LedCount];

void setup()
{
  pinMode(LedPin, OUTPUT);
  pinMode(TachoPin, INPUT_PULLUP);

  // Disable the ADC by setting the ADEN bit (bit 7) to 0
  ADCSRA &= B01111111;

  // Disable the analog comparator by setting the ACD bit
  // (bit 7) of the ACSR register to 1.
  ACSR |= B10000000;

  // Disable digital input buffers on all analog input pins
  // by setting bits 0-5 of the DIDR0 register to 1.
  DIDR0 |= B00111111;

  FastLED.addLeds<WS2812B, LedPin, GRB>(leds, LedCount).setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInVoltsAndMilliamps(5,1700); 
  FastLED.setBrightness(35);

  //random16_set_seed(analogRead(A1)); // Get noise from not connected A1

  Serial.begin(57600);
  Serial.setTimeout(10);
}

void updateLights(uint16_t speed)
{
  static uint8_t waveArg1 = 0;
  static uint8_t transitionCounter = 0;

  if(speed > 20) {
    if(transitionCounter < 254) transitionCounter += 2;
  } else {
    if(transitionCounter > 1) transitionCounter -= 2;
  }

  uint8_t sin1 = sin8(waveArg1);
  uint16_t redValue = (uint16_t)sin1 * RedAmplitude / 255  + 255 - RedAmplitude;
   
  for(uint8_t i = 0; i < LedCount; i++) {
    if(transitionCounter > 150) {
      leds[i] = CHSV(waveArg1 + i, 255, (transitionCounter - 150) * 2.42);
    } else
    if (transitionCounter < 90) {       
      leds[i] = CHSV(0, 255, redValue * (90 - transitionCounter) / 90.);
    }
  }

#ifndef DEBUG
  for(uint8_t i = 0; i <= 1; i++) // 1st bottom light
    leds[i] = CHSV(0,0,255);
  for(uint8_t i = 118; i <= 119; i++) // 2nd bottom light
    leds[i] = CHSV(0,0,255);

  for(uint8_t i = 57; i <= 61; i++) // headlight
    leds[i] = CHSV(0,0,255);
#endif

  waveArg1++;

  FastLED.show();
}

int getSpeed()
{
  static bool trigger = 0;
  static bool triggerPrev = 0;
  static uint16_t rotations = 0;
  static uint16_t rotationsPrev = 0;

  static uint16_t v1 = 0,
                  v2 = 0,
                  v3 = 0,
                  v4 = 0,
                  v = 0;

  triggerPrev = trigger;
  trigger = digitalRead(TachoPin) == 0;
  
  if(trigger && !triggerPrev) { // When falling from 1 (pull-up resistor) to 0
    rotations++;
  }

  static TimerMs updateSpeedTimer(SpeedMeasureInterval, 1, 1);

  if(updateSpeedTimer.elapsed()) {
    uint16_t dRotations = rotations - rotationsPrev;

    v4 = v3;
    v3 = v2;
    v2 = v1;
    v1 = 2 * PI * WheelRadius * dRotations * 1000 / SpeedMeasureInterval * 36; // In km/h
    v = (v1 + v2 + v3 + v4) / 4;
    if(v < 15) v4 = v3 = v2 = v1 = v = 0;

    #ifdef DEBUG
    // Serial.print(v1);
    // Serial.print(" ");
    // Serial.print(v2);
    // Serial.print(" ");
    // Serial.print(v3);
    // Serial.print(" ");
    // Serial.print(v4);
    // Serial.print(" ");
    // Serial.println(v);
    #endif

    if(rotations > 65000) { // Overflow protection for correct maths on dRotations
      rotations = dRotations;
      rotationsPrev = 0;
    } else  {
      rotationsPrev = rotations; 
    }

    updateSpeedTimer.start();
  }

  return v;
}

void loop()
{
  static TimerMs updateLightsTimer(10, 1, 1);
  static uint16_t speed = 0;

  #ifdef DEBUG
  if(Serial.available()) {
    speed = Serial.parseInt();
    Serial.print("speed ");
    Serial.println(speed);  
  }
  #else
  speed = getSpeed();
  #endif

  if(updateLightsTimer.elapsed()) {
    updateLights(speed);
    updateLightsTimer.start();
  }
}