#include "phototrans.h"

#include "ch32fun.h"
#include "utils.h"

#include <memory.h>

static inline int abs(int a) { return a < 0 ? -a : a; }

static inline int min(int a, int b) { return a < b ? a : b; }

void setLitValue(int n); // TODO for debug, refactor

void setupAdc(void)
{
	// Enable ADC
	RCC->APB2PCENR |= RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA;
  RCC->CFGR0 = (RCC->CFGR0 & CFGR0_ADCPRE_Reset_Mask) | RCC_PCLK2_Div4;

	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1; // RCC_APB2Periph_GPIOA;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

  funPinMode(PA1, GPIO_CFGLR_IN_ANALOG );
  funPinMode(PA2, GPIO_CFGLR_IN_ANALOG );

	// Setup channel conversion situation
	ADC1->ISQR=
     ADC_JL_0 |     // 2 Total channels
     0 |            // Channel 0 (PA2)
     ADC_JSQ3_0;   // Channel 1 (PA1)

  // Once we read the analog values will be populated in the following:
  // ADC1->JSQ1 = Channel 0 (PA2)
  // ADC1->JSQ2 = Channel 1 (PA1)

  // Setup sampling time 241 cycles
  // 0:7 => 3/9/15/30/43/57/73/241 cycles
  ADC1->SAMPTR2 |= 
        ADC_SMP0_0 | ADC_SMP0_1 | ADC_SMP0_2 | \
        ADC_SMP1_0 | ADC_SMP1_1 | ADC_SMP1_2 ;

  // turn on ADC and set rule group to sw trig
  ADC1->CTLR2 = ADC_JEXTSEL | ADC_JEXTTRIG;
  ADC1->CTLR1 = ADC_IT_JEOC | ADC_SCAN;
  ADC1->STATR &= ~(ADC_JEOC | ADC_JSTRT);

  #ifdef ADC_CALIBRATION_ENABLED
  // Reset calibration
  ADC1->CTLR2 |= ADC_RSTCAL;
  while(ADC1->CTLR2 & ADC_RSTCAL);

  // Calibrate
  ADC1->CTLR2 |= ADC_CAL;
  while(ADC1->CTLR2 & ADC_CAL);
  #endif

  // Tell the ADC to start converting, continously.
  ADC1->CTLR2 |= ADC_ADON | ADC_JSWSTART;

  NVIC_SetPriority(ADC_IRQn, 3);
  NVIC_EnableIRQ(ADC_IRQn);
}

volatile int a1 =  10, a2 = 0;
volatile int phase = 0;
volatile int preamble = 0;

int getPhase() { return phase; }
int getPreamble() { return preamble; }

uint32_t last_t;
int16_t ds[4];
uint8_t di = 0;
int c = 0;
const int accept_diff = 10;
const int min_duration = accept_diff * 5;
const int max_duration = 500;

void ADC1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void ADC1_IRQHandler(void)
{
  if ((ADC1->STATR & ADC_JEOC))
  {
    const int DECAY = 192;
    int v1 = ADC1->IDATAR1 & 0xFFFF;
    int v2 = ADC1->IDATAR2 & 0xFFFF;
    a1 = (a1 * DECAY + (256 - DECAY) * (int)v1) >> 8;
    a2 = (a2 * DECAY + (256 - DECAY) * (int)v2) >> 8;
    int newPhase = a1 < a2;
    if (phase != newPhase)
    {
      uint32_t t = millis();
      ds[di] = t - last_t;
      di = (di + 1) & 3;
      last_t = t;

      int sum = abs(ds[0] - ds[1]) + abs(ds[2] - ds[1]) + abs(ds[2] - ds[3]) + abs(ds[0] - ds[3]);
      preamble = ds[di] >= min_duration && ds[di] <= max_duration && sum <= 4 * accept_diff;

      phase = newPhase;
    }

    //ADC1->CTLR2 |= ADC_JSWSTART;
    ADC1->STATR &= ~ADC_JEOC;
  }
}

uint8_t buf[31];
const uint8_t MAX_RECV_BUF = 30; // max buf[0] value

void init_decoder()
{
  preamble = 0;
  memset(ds, 0, sizeof ds);
  di = 0;
}

void setupPhoto()
{
  setupAdc();

  init_decoder();
  last_t = millis();
}

inline void indicateStatus(int value)
{
  setLitValue(value << 6);
}

int receive()
{
  uint32_t t = last_t;
  int16_t d = (ds[0] + ds[1]) * 3 / 8; // 3/4 of d

  indicateStatus(0b001);

  int p, p0;
  uint32_t next = t + d;
  while (next <= millis())
    next += d;
  next -= millis();
  do
  {
    p = phase;
    Delay_Ms(next);

    p0 = phase;
    t = millis();
    while (p0 == phase)
    {
      if ((long)(millis() - t) > d)
      {
        indicateStatus(0b011);
        return 0;
      }
    }

    next = d;
  } while (p == p0);
  indicateStatus(0b101);
  int k = 0;
  int km = 1;
  int len = 1;
  while (k < len)
  {
    Delay_Ms(d);

    p0 = phase;
    t = millis();
    while (p0 == phase)
    {
      if ((long)(millis() - t) > d)
      {
        indicateStatus(0b011);
        return 0;
      }
    }

    if (p0 != p)
    {
      buf[k] |= km;
    }
    else
    {
      buf[k] &= ~km;
    }
    km <<= 1;
    if (km > 128)
    {
      if (k == 0)
        len = min(MAX_RECV_BUF, buf[0]) + 1;
      k++;
      km = 1;
    }
  }
  indicateStatus(0b111);
  init_decoder();
  return 1;
}

int receivedFrames()
{
  return buf[0] / 2;
}

void getFrames(uint16_t *frames)
{
  int len = receivedFrames();
  for (int i = 0; i < len; i++)
  {
    int j = 1 + (i << 1);
    frames[i] = (buf[j] | ((int)(buf[j + 1]) << 8)) & 0xFFF;
  }
}