#include "leds.h"

#include "ch32fun.h"

#define X1 PC4
#define X2 PC5
#define X3 PC6
#define X4 PC7

const uint32_t UPDATE_INTERVAL = 300;

void show_value();

void TIM2_IRQHandler( void ) __attribute__((interrupt()));
void TIM2_IRQHandler(void) {
  uint16_t status = TIM2->INTFR & TIM_IT_Update;
  if (status != RESET) {
    show_value();
      
    TIM2->INTFR = ~TIM_IT_Update;
  }
}


void setupLit()
{
  funPinMode( X1, GPIO_CFGLR_IN_FLOAT );
  funPinMode( X2, GPIO_CFGLR_IN_FLOAT );
  funPinMode( X3, GPIO_CFGLR_IN_FLOAT );
  funPinMode( X4, GPIO_CFGLR_IN_FLOAT );

  RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

  uint32_t period_cyc = UPDATE_INTERVAL * (FUNCONF_SYSTEM_CORE_CLOCK / 1000000);
  uint32_t prescaler_factor = (period_cyc / 0x10000) + 1;
  uint32_t period_ticks = period_cyc / prescaler_factor;

  TIM2->RPTCR = 0;
	TIM2->CTLR1 |= TIM_CKD_DIV1;
	TIM2->PSC = prescaler_factor;
	TIM2->ATRLR = period_ticks - 1;
	TIM2->SWEVGR |= TIM_UG;
	TIM2->DMAINTENR |= TIM_IT_Update;
	TIM2->CTLR1 |= TIM_CEN;

  NVIC_EnableIRQ(TIM2_IRQn);
}

int pins[] = {X1, X2, X3, X4};

int l1 = 0, l2 = 0;

void noLit()
{
  if (!l1 && !l2)
    return;
  funPinMode(pins[l1], GPIO_CFGLR_IN_FLOAT);
  funPinMode(pins[l2], GPIO_CFGLR_IN_FLOAT);
  l1 = l2 = 0;
}

void lit(int i)
{
  if (l1 || l2)
  {
    funPinMode(pins[l1], GPIO_CFGLR_IN_FLOAT);
    funPinMode(pins[l2], GPIO_CFGLR_IN_FLOAT);
  }
  l1 = (i / 3) % 4;
  l2 = i % 3;
  if (l2 >= l1)
    l2++;
  funPinMode(pins[l1], GPIO_CFGLR_OUT_10Mhz_PP);
  funDigitalWrite(pins[l1], FUN_LOW);
  funPinMode(pins[l2], GPIO_CFGLR_OUT_10Mhz_PP);
  funDigitalWrite(pins[l2], FUN_HIGH);
}

volatile uint16_t lit_n = 0;
volatile uint16_t lit_m = 1;

void show_value()
{
  volatile uint16_t n = lit_n;
  volatile uint16_t m = lit_m;
  if (n < m)
  {
    m = 1;
    if (n < m)
    {
      lit_m = m;
      noLit();
      return;
    }
  }
  while (!(n & m)) {
    m <<= 1;
  }
  lit(__builtin_popcount(m - 1));
  lit_m = m << 1;
}

void setLitValue(int n)
{
  lit_n = n;
}
