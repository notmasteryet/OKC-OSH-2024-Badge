#include "ch32fun.h"

#include "leds.h"
#include "phototrans.h"
#include "encoder.h"
#include "utils.h"

void setup()
{
  SystemInit();  
  funGpioInitAll();

  systickInit();
  setupLit();
  setupPhoto();
  setupEncoder();
}


int frames_len = 0;
uint16_t frames[20];

int next_frame;
uint32_t last_frame_t;
const uint32_t FRAME_RATE = 333;

void animateFrames()
{
  if (millis() - last_frame_t < FRAME_RATE)
  {
    return;
  }
  last_frame_t = millis();
  setLitValue(frames[next_frame]);
  next_frame = (next_frame + 1) % frames_len;
}

void loop()
{
  if (getPreamble())
  {
    if (!receive())
    {
      Delay_Ms(1000);
      return;
    }
    frames_len = receivedFrames();
    getFrames(frames);
    last_frame_t = millis();
    next_frame = 0;
  }
  if (frames_len > 0)
  {
    animateFrames();
    return;
  }
  int v1 = getEncoderValue();
  ///int v1 = millis() / 200;
  uint32_t display = getPhase() ? 0x03F : 0xFC0;
  display |= (display << 12);
  setLitValue(0xFFF & (display >> (v1 % 12)));
}

int main() {
  setup();
  while(1) { 
    loop();
  }
}