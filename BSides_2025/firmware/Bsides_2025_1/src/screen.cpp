#include "screen.h"

#include <Arduino.h>

#include "leds.h"
#include "encoder.h"
#include "phototrans.h"
#include "utils.h"
#include "i2c_mini.h"
#include "ssd1306_mini.h"

void ScreenBase::enter() {}
void ScreenBase::execute() {}
void ScreenBase::leave() {}

ScreenBase *ScreenBase::current = nullptr;
ScreenBase *ScreenBase::defaultScreen = nullptr;
ButtonState ScreenBase::buttonState;

void DefaultScreen::enter()
{
  lastEncoder = getEncoderValue();
}

void DefaultScreen::execute()
{
  uint16_t enc = getEncoderValue();
  int delta = -int16_t(enc - lastEncoder) / 2;
  if (delta != 0)
  {
    lastEncoder = enc;
    level = max(1, min(11, level + delta));
  }
  int pattern = (1 << level) - 1;
  pattern = (pattern << 2) | (pattern >> 10);
  setLitValue(pattern);
}

void TestScreen::execute()
{
  int v1 = getEncoderValue(); // millis() / 100;
  uint32_t display = getPhase() ? 0x03F : 0xFC0;
  display |= (display << 12);
  setLitValue(0xFFF & (display >> (v1 % 12)));
}

void AccelerometerScreen::enter()
{
  last_frame_t = millis();
}

void AccelerometerScreen::execute()
{
  if (millis() - last_frame_t < 10)
  {
    return; // let mpu6050 refreshd
  }
  last_frame_t = millis();

  int16_t data[3];
  if (!readAccel(data))
  {
    defaultScreen->select();
    return;
  }
  const int DECAY = 192;
  ax = (DECAY * ax + (256 - DECAY) * data[0]) / 256;
  ay = (DECAY * ay + (256 - DECAY) * data[1]) / 256;
  az = (DECAY * az + (256 - DECAY) * data[2]) / 256;

  // angle of lowest part of badge
  int a = patan2(ay, -ax);
  // esimate size of the curve based on incline of the badge
  int s = 2 * min(3, patan2(2 * abs(az), abs(ax) + abs(ay))) + 1;
  int v = (1 << (s + 1)) - 1; // curve binary
  int o = (s / 2 + a) % 12;   // ... and how to move it
  setLitValue(0xFFF & (v << (12 - o) | (v >> o)));
}

void AnimationScreen::enter()
{
  last_frame_t = millis();
  next_frame = 0;
}

void AnimationScreen::execute()
{
  if (millis() - last_frame_t < FRAME_RATE)
  {
    return;
  }
  last_frame_t = millis();
  setLitValue(framesData.frames[next_frame]);
  next_frame = (next_frame + 1) % framesData.count;
}

void AnimationScreen::setFrames(uint16_t *frames, int count)
{
  framesData.count = count;
  memcpy(framesData.frames, frames, count * sizeof(frames[0]));
  next_frame = 0;
}

const FramesData AnimationScreen::pattern1{
    2,
    {0b1010, 0b0101}};

void TextScreen::enter()
{
  oledDrawText(0, 0, "Hello,", 1, 1);
  oledDrawText(20, 16, "World!", 1, 1);
  oledRefresh();
  setLitValue(0b111000111000);
}