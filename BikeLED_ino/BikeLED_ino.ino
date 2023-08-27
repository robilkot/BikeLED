#include <FastLED.h>
#include <TimerMs.h>

const uint8_t LedPin = 9;
const uint8_t LedCount = 120;
const uint8_t TransitionSpeed = 30;

const uint8_t TachoPin = 4;
const uint16_t SpeedMeasureInterval = 900;
const float WheelRadius = 0.255; // Diameter ~510 mm

const uint8_t RedAmplitude = 200;

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
  DIDR0 = DIDR0 | B00111111;

  FastLED.addLeds<WS2812B, LedPin, GRB>(leds, LedCount).setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInVoltsAndMilliamps(5,1700); 
  FastLED.setBrightness(35);

  random16_set_seed(analogRead(A1)); // Get noise from not connected A1

  // Serial.begin(57600);
  // Serial.setTimeout(10);
}

void shiftLeds()
{
  for(uint8_t i = LedCount - 1; i >= 1; i--) {
    leds[i] = leds[i-1];
  }
}

void updateLights(uint16_t speed)
{
  static bool idleMode = true;
  static uint8_t waveArg1 = 0;
  static CRGB transitionCounter(255,255,255);

  if(speed > 20) {
    if(idleMode) transitionCounter = CRGB(255,255,255);
    idleMode = false;
  } else {
    if(!idleMode) transitionCounter = CRGB(255,255,255);
    idleMode = true;
  }

  uint8_t sin1 = sin8(waveArg1);

  if(transitionCounter) {
    shiftLeds();
    for(auto& x : leds) x.fadeToBlackBy(TransitionSpeed);
    transitionCounter.fadeToBlackBy(TransitionSpeed);
  }
    // STASH
    // CRGB color1 = Red1 + CRGB(0, 50 * fabs((sin8(waveArg += 2)) / 255.), 0);
    // fill_gradient_RGB(leds, LedCount, color1, color2);
    // fill_solid(leds, LedCount, CHSV( 0, sin8(waveArg) / 2 + 100, sin8(waveArg) / 2 + 100 ));
    // blur1d(leds,LedCount, 120);

  if(idleMode)
  { 
  // Serial.println("idle");
    //for(uint8_t i = 0; i < 3; i++) {
    // shiftLeds();
      uint16_t value = sin1 * RedAmplitude / 255  + 255 - RedAmplitude;
      for(auto& x : leds)
        x = CHSV(0, 255, value);
    //}
    //float k = 255 - transitionCounter.r;
    //for(auto& x : leds) x = toFill % k;
  }
  else {
    // Serial.println("ride");
    // shiftLeds();
    // uint8_t hue = sin8(waveArg1 * (float) speed / 500) * 80 / 255 + 70;
    // leds[0] = CHSV(hue, 255, 180);
    // leds[0] = CHSV( sin8(waveArg1) * 80 / 255 + 70, 255, 180 );

    // Splashes
    // leds[random16(0, LedCount)] = CHSV(random16(0, 256), 255, random16(150, 256) * (255 - transitionCounter.r) / 255.);
    // blur1d(leds, LedCount, 160);
    // for(auto& x : leds) x.fadeToBlackBy(2);
    fill_rainbow(leds, LedCount, waveArg1);
  }

  for(uint8_t i = 0; i <= 2; i++)
    leds[i] = CHSV(0,0,255);
  for(uint8_t i = 117; i <= 119; i++)
    leds[i] = CHSV(0,0,255);
  for(uint8_t i = 57; i <= 61; i++)
    leds[i] = CHSV(0,0,255);

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
    // Serial.print(v1);
    // Serial.print(" ");
    // Serial.print(v2);
    // Serial.print(" ");
    // Serial.print(v3);
    // Serial.print(" ");
    // Serial.print(v4);
    // Serial.print(" ");
    // Serial.println(v);

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
  // if(Serial.available()) {
  //   speed = Serial.parseInt();
  //   Serial.print("speed ");
  //   Serial.println(speed);  
  // }

  static TimerMs updateLightsTimer(10, 1, 1);
  static uint16_t speed = 0;

  speed = getSpeed();

  if(updateLightsTimer.elapsed()) {
    updateLights(speed);
    updateLightsTimer.start();
  }
}