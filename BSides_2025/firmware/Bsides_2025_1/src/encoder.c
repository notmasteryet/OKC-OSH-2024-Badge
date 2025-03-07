#include "encoder.h"

#include "ch32fun.h"

#define ROT_A PD0
#define ROT_B PD2

void setLitValue(int n); // TODO for debug, refactor

#define ENCODER_POS ((funDigitalRead(ROT_A) * 3) ^ funDigitalRead(ROT_B))

volatile uint8_t encoderPos;
volatile uint16_t encoderValue = 0;

uint16_t getEncoderValue()
{
  return encoderValue;
}

__attribute__((always_inline))
void encoderISR()
{
  uint8_t pos = ENCODER_POS;
  switch ((pos - encoderPos) & 3)
  {
  case 0:
    return;
  case 1:
    encoderValue++;
    break;
  case 3:
    encoderValue--;
    break;
  }
  encoderPos = pos;
}

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
  if ((EXTI->INTFR & EXTI_Line0) != RESET) {
    encoderISR();
    EXTI->INTFR = EXTI_Line0;
  }
  if ((EXTI->INTFR & EXTI_Line2) != RESET) {
    encoderISR();
    EXTI->INTFR = EXTI_Line2;
  }
}

void setupEncoder()
{
  funPinMode(ROT_A, GPIO_CFGLR_IN_FLOAT);
  funPinMode(ROT_B, GPIO_CFGLR_IN_FLOAT);

  encoderPos = ENCODER_POS;

  AFIO->EXTICR |= AFIO_EXTICR_EXTI0_PD | AFIO_EXTICR_EXTI2_PD;
  EXTI->INTENR = EXTI_INTENR_MR0 | EXTI_INTENR_MR2;
  EXTI->RTENR = EXTI_RTENR_TR0 | EXTI_RTENR_TR2;
  EXTI->FTENR = EXTI_FTENR_TR0 | EXTI_FTENR_TR2;

  NVIC_SetPriority(EXTI7_0_IRQn, 0xFF);
  NVIC_EnableIRQ(EXTI7_0_IRQn);
}
