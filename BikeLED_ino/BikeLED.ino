#include <FastLED.h>
#include <TimerMs.h>

const uint8_t LedPin = 9;
const uint8_t LedCount = 120;
const uint8_t TransitionSpeed = 30;

const uint8_t TachoPin = A0;
const uint16_t SpeedMeasureInterval = 1000;
const uint8_t WheelRadius = 255; // Diameter ~510 mm

CRGB leds[LedCount];

void setup()
{
  pinMode(LedPin, OUTPUT);
  pinMode(TachoPin, INPUT_PULLUP);

  FastLED.addLeds<WS2812B, LedPin, GRB>(leds, LedCount).setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInVoltsAndMilliamps(5,2100); 
  FastLED.setBrightness(15);

  // Serial.begin(57600);
  // Serial.setTimeout(10);
}

void shiftLeds()
{
  for(uint8_t i = LedCount - 1; i >= 1; i--) {
    leds[i] = leds[i-1];
  }
}

void updateLights(uint8_t speed)
{
  static bool idleMode = true;
  static uint8_t waveArg1 = 0;
  static CRGB transitionCounter(255,255,255);

  if(speed > 0) {
    if(idleMode) transitionCounter = CRGB(255,255,255);
    idleMode = false;
  } else {
    if(!idleMode) transitionCounter = CRGB(255,255,255);
    idleMode = true;
  }

  if(transitionCounter) {
    shiftLeds();
    leds[0].fadeToBlackBy(TransitionSpeed);
    transitionCounter.fadeToBlackBy(TransitionSpeed);
  }
    // STASH
    // CRGB color1 = Red1 + CRGB(0, 50 * fabs((sin8(waveArg += 2)) / 255.), 0);
    // fill_gradient_RGB(leds, LedCount, color1, color2);
    // fill_solid(leds, LedCount, CHSV( 0, sin8(waveArg) / 2 + 100, sin8(waveArg) / 2 + 100 ));
    // blur1d(leds,LedCount, 120);

  uint8_t sin1 = sin8(waveArg1);

  if(idleMode)
  { 
    shiftLeds();
    leds[0] = CHSV( 0, 255, sin1 / 2 + 100 );
  }
  else {
    shiftLeds();
    leds[0] = CHSV( sin1 * 80 / 255 + 70, 255, 200 );

    // Splashes
    // leds[random(0, LedCount)] = CHSV(random(0, 256), 255, random(100, 256));
    // blur1d(leds, LedCount, 120);
    // for(auto& x : leds) x.fadeToBlackBy(10);
  }

  waveArg1++;

  FastLED.show();
}

// float updateSpeed()
// {
//   static bool trigger = 0;
//   static bool triggerPrev = 0;
//   static uint16_t rotations = 0;
//   static uint16_t rotationsPrev = 0;
//   static float v1 = 0, v2 = 0, v3 = 0, v = 0;

//   triggerPrev = trigger;
//   trigger = analogRead(TachoPin) < 100;
  
//   if(trigger && !triggerPrev) { // When falling from 1 (pull-up resistor) to 0
//     rotations++;
//   }

//   static TimerMs updateSpeedTimer(SpeedMeasureInterval, 1, 1);

//   if(updateSpeedTimer.elapsed()) {
//     uint16_t dRotations = rotations - rotationsPrev;

//     v3 = v2;
//     v2 = v1;
//     v1 = 2 * PI * WheelRadius * dRotations * 1000 / SpeedMeasureInterval * 3.6; // In km/h
//     v = (v1 + v2 + v3) / 3.;
//     if(v < 1) v3 = v2 = v1 = v = 0;

//     if(rotations > 65000) { // Overflow protection for correct maths on dRotations
//       rotations = dRotations;
//       rotationsPrev = 0;
//     } else  {
//       rotationsPrev = rotations; 
//     }

//     updateSpeedTimer.start();
//   }

//   return v;
// }

void loop()
{
  static uint8_t speed = 0;

  if(Serial.available()) {
    speed = Serial.read() - '0';
    Serial.print("speed ");
    Serial.println(speed);  
  }

  static TimerMs updateLightsTimer(10, 1, 1);

  if(updateLightsTimer.elapsed()) {
    updateLights(speed);
    updateLightsTimer.start();
  }
}