// Copyright (c) 2023 Cesanta Software Limited
// https://www.st.com/resource/en/reference_manual/dm00151940-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
// SPDX-License-Identifier: MIT

/*       NUCLEO-L412KB PIN assignment
 *          
 *          CN3 [USB conn]  CN4
 *          -------------------------------
 *          PA9      1      VIN     
 *          PA10     2      GND     
 *          NRST     3      NRST    
 *          GND      4      +5V     
 *          PA12     5      PA2     
 *          PB0      6      PA7     
 *          PB7      7      PA6     
 *          PB6      8      PA5     
 *          PB1      9      PA4     
 *          PC14    10      PA3     
 *          PC15    11      PA1     
 *          PA8     12      PA0     
 *          PA11    13      AREF    
 *          PB5     14      +3V3    
 *          PB4     15      PB3(led)
 
 */


#define DEBOUNCE_MS 32  // LED blinking period in millis
// Keyboard matrix with 9 rows x 7 cols (mapping 62keys out of 63)
#define NCOLS 7  // Cols low. Pins with diode cathode exiting
#define NROWS 9  // Rows high 
#define ROWMASK (1<<NROWS)-1
#define ROW8_PIN PIN('A', 12) 
#define ROW0_PIN PIN('B', 0)  // first key
#define ROW1_PIN PIN('B', 7)  
#define ROW7_PIN PIN('B', 6)  
#define ROW2_PIN PIN('B', 1)  
#define ROW6_PIN PIN('A', 8)  
#define ROW3_PIN PIN('A', 11) 
#define ROW5_PIN PIN('B', 5)  
#define ROW4_PIN PIN('B', 4)  

#define COL0_PIN PIN('A', 0)  // first key
#define COL1_PIN PIN('A', 1)  
#define COL2_PIN PIN('A', 3)  
#define COL3_PIN PIN('A', 4)  
#define COL4_PIN PIN('A', 5)  
#define COL5_PIN PIN('A', 6)  
#define COL6_PIN PIN('A', 7)  

#define ROWSA BIT(8) | BIT(11) | BIT(12)
#define ROWSB BIT(0) | BIT(1) | BIT(4) | BIT(5) | BIT(6) | BIT(7)
#define COLBITS BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7)

#pragma once
#include <stm32l412xx.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BIT(x) (1UL << (x))
#define SETBITS(R, CLEARMASK, SETMASK) (R) = ((R) & ~(CLEARMASK)) | (SETMASK)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static const uint16_t ROWS[NROWS] = {ROW0_PIN, ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN, \
                               ROW5_PIN, ROW6_PIN, ROW7_PIN, ROW8_PIN};

static const uint16_t COLS[NCOLS] = {COL6_PIN, COL5_PIN, COL4_PIN, COL3_PIN, COL2_PIN, \
                               COL1_PIN, COL0_PIN};

static const uint8_t Hmask = 0b10000000;  // holds the key status
static const uint8_t Lmask = 0b01111111;  // holds the history
// System clock
enum { AHB_DIV = 1, APB1_DIV = 1, APB2_DIV = 1 };
enum { PLL_HSI = 16, PLL_M = 1, PLL_N = 10, PLL_R = 2 };  // 80 Mhz
//#define SYS_FREQUENCY ((PLL_HSI * PLL_N / PLL_M / PLL_R) * 1000000)
#define SYS_FREQUENCY 16000000
#define APB2_FREQUENCY (SYS_FREQUENCY / APB2_DIV)
#define APB1_FREQUENCY (SYS_FREQUENCY / APB1_DIV)

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum { GPIO_OTYPE_PUSH_PULL, GPIO_OTYPE_OPEN_DRAIN };
enum { GPIO_SPEED_LOW, GPIO_SPEED_MEDIUM, GPIO_SPEED_HIGH, GPIO_SPEED_INSANE };
enum { GPIO_PULL_NONE, GPIO_PULL_UP, GPIO_PULL_DOWN };
#define GPIO(N) ((GPIO_TypeDef *) (GPIOA_BASE + 0x400 * (N)))

static GPIO_TypeDef *gpio_bank(uint16_t pin) {
  return GPIO(PINBANK(pin));
}
static inline void gpio_toggle(uint16_t pin) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  uint32_t mask = BIT(PINNO(pin));
  gpio->BSRR = mask << (gpio->ODR & mask ? 16 : 0);
}
static inline uint8_t gpio_read(uint16_t pin) {
  return gpio_bank(pin)->IDR & BIT(PINNO(pin)) ? 1 : 0;
}
static inline int gpio_read_out(uint16_t pin) {
  return gpio_bank(pin)->ODR & BIT(PINNO(pin)) ? 1 : 0;
}
static inline void gpio_write(uint16_t pin, bool val) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  gpio->BSRR = BIT(PINNO(pin)) << (val ? 0 : 16);
}

static inline void gpio_init(uint16_t pin, uint8_t mode, uint8_t type,
                             uint8_t speed, uint8_t pull, uint8_t af) {
  GPIO_TypeDef *gpio = gpio_bank(pin);
  uint8_t n = (uint8_t) (PINNO(pin));
  RCC->AHB2ENR |= BIT(PINBANK(pin));  // Enable GPIO clock
  SETBITS(gpio->OTYPER, 1UL << n, ((uint32_t) type) << n);
  SETBITS(gpio->OSPEEDR, 3UL << (n * 2), ((uint32_t) speed) << (n * 2));
  SETBITS(gpio->PUPDR, 3UL << (n * 2), ((uint32_t) pull) << (n * 2));
  SETBITS(gpio->AFR[n >> 3], 15UL << ((n & 7) * 4),
          ((uint32_t) af) << ((n & 7) * 4));
  SETBITS(gpio->MODER, 3UL << (n * 2), ((uint32_t) mode) << (n * 2));
}
static inline void gpio_input(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}
static inline void gpio_output(uint16_t pin) {
  gpio_init(pin, GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_HIGH,
            GPIO_PULL_NONE, 0);
}

static inline void gpio_matrix_init(void) {
  for (int m=0; m< NCOLS; m++) {
    gpio_init(COLS[m], GPIO_MODE_INPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_DOWN, 0);
  }
  for (int m=0; m< NROWS; m++) {
  gpio_init(ROWS[m], GPIO_MODE_OUTPUT, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE, 0);
  }
}

static inline void col_trigger_init(void) {
  // EXTI0 to EXTI3
  SYSCFG->EXTICR[0] &= ~((1<<16)-1); // clear first 16 bits
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA | SYSCFG_EXTICR1_EXTI1_PA | SYSCFG_EXTICR1_EXTI3_PA;
  // EXTI4 to EXTI7
  SYSCFG->EXTICR[1] &= ~((1<<16)-1); // clear first 16 bits
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PA | SYSCFG_EXTICR2_EXTI5_PA | SYSCFG_EXTICR2_EXTI6_PA | SYSCFG_EXTICR2_EXTI7_PA;
  // Rising AND falling edge trigger for each line
  //EXTI->RTSR1 |=  (1 << 0);
  //EXTI->FTSR1 |=  (1 << 0);
  EXTI->RTSR1 |=  COLBITS;
  EXTI->FTSR1 |=  COLBITS;
  NVIC_SetPriority(EXTI0_IRQn, 2);
  NVIC_SetPriority(EXTI1_IRQn, 2);
  NVIC_SetPriority(EXTI3_IRQn, 2);
  NVIC_SetPriority(EXTI4_IRQn, 2);
  NVIC_SetPriority(EXTI9_5_IRQn, 2);
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static inline void set_triggers(bool val) {
  EXTI->IMR1 &= ~((1<<16)-1); // clear first 16 bits
  if (val) EXTI->IMR1 |= COLBITS;
}

static inline void set_all_rows(bool val) {
  //uint32_t bitsa = ROWSA;
  //uint32_t bitsb = ROWSB;
  //GPIO(0)->BSRR = bitsa << (val ? 0 : 16);
  //GPIO(1)->BSRR = bitsb << (val ? 0 : 16);
  GPIO(0)->BSRR = ROWSA << (val ? 0 : 16);
  GPIO(1)->BSRR = ROWSB << (val ? 0 : 16);
  //return 1; // check the ODR register to guaranty all set
}

static inline void set_row(int val) { // here incorporate the readout of the column and return
  GPIO_TypeDef *gpio = gpio_bank(ROWS[val]);
  gpio->BSRR = BIT(PINNO(ROWS[val])) <<  0 ;
  //return 1; // check the ODR register to guaranty all set
}


#ifndef UART_DEBUG
#define UART_DEBUG USART2
#endif

static inline bool uart_init(USART_TypeDef *uart, unsigned long baud) {
  // https://www.st.com/resource/en/datasheet/stm32l432kc.pdf
  uint8_t aftx = 7, afrx = 7;  // Alternate function
  uint16_t rx = 0, tx = 0;     // pins
  uint32_t freq = 0;           // Bus frequency. UART1 is on APB2, rest on APB1

  if (uart == USART1) {
    freq = APB2_FREQUENCY, RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    tx = PIN('A', 9), rx = PIN('A', 10);
  } else if (uart == USART2) {
    freq = APB1_FREQUENCY, RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    tx = PIN('A', 2), rx = PIN('A', 15), afrx = 3;
  } else {
    return false;
  }

  gpio_init(tx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_MEDIUM, 0, aftx);
  gpio_init(rx, GPIO_MODE_AF, GPIO_OTYPE_PUSH_PULL, GPIO_SPEED_MEDIUM, 0, afrx);
  uart->CR1 = 0;                          // Disable this UART
  uart->BRR = freq / baud;                // Set baud rate
  uart->CR1 |= BIT(0) | BIT(2) | BIT(3);  // Set UE, RE, TE
  //uart->CR3 |= BIT(7);  // Set DMA Transmit
  //uart->CR3 |= BIT(8) | BIT(9);  // Set CTSE and RTSE, for RS-232
  return true;
}
static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte) {
  uart->TDR = byte;
  while ((uart->ISR & BIT(6)) == 0) spin(1); // vik: changed to bit 6 instead of 7 (Trans. completed instead of shift reg. emptied)
}
static inline void uart_write_buf(USART_TypeDef *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}
static inline int uart_read_ready(USART_TypeDef *uart) {
  return uart->ISR & BIT(5);  // If RXNE bit is set, data is ready
}
static inline uint8_t uart_read_byte(USART_TypeDef *uart) {
  return (uint8_t) (uart->RDR & 255);
}

// t: expiration time, prd: period, now: current time. Return true if expired
static inline bool timer_expired(volatile uint64_t *t, uint64_t prd,
                                 uint64_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}
// Fill in stack with markers, in order to calculate stack usage later
extern unsigned char _estack, _end;
static inline void stack_fill(void) {
  uint32_t dummy, *p = (uint32_t *) &_end;
  while (p < &dummy) *p++ = 0xa5a5a5a5;
}

static inline long stack_usage(void) {
  uint32_t *sp = (uint32_t *) &_estack, *end = (uint32_t *) &_end, *p = sp - 1;
  while (p > end && *p != 0xa5a5a5a5) p--;
  return (sp - p) * sizeof(*p);
}

static inline void init_debouncer(void) {
    TIM2->CR1 &= ~(1<<0);    // Disable timer, for now
    TIM2->CR1 |=  (1<<2);    // set source for events to only overflow, not UG
    //TIM2->CR1 |=  (1<<3);    // set one pulse (stops after reaching limit)
    RCC->APB1ENR1 |= (1<<0); // Enable clock for TIM2
    //RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; 
    TIM2->PSC = 16000-1;    // Set PSC+1 = 160000  pre-scaler, if freq is 16MHz, that is 1ms
    TIM2->ARR = DEBOUNCE_MS;        // Set timer to reset after CNT = 100 (that is 100 ms)
    TIM2->DIER |= (1<<0);   // Enable timer interrupt generation
    //TIM2->SR &= ~(1<<0);
    TIM2->EGR |= (1<<0);
    // Enable timer interrupt and set priority
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
}


static inline void clock_init(void) {
  FLASH->ACR |= FLASH_ACR_LATENCY_0WS | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY)) spin(1);
  RCC->CFGR &= ~(RCC_CFGR_SW);
  RCC->CFGR |= (RCC_CFGR_SW_HSI);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) spin(1);

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;    // Enable SYSCFG
  SystemCoreClock = SYS_FREQUENCY;         // Required by CMSIS
  SysTick_Config(SystemCoreClock / 1000);  // Sys tick every 1ms
}

/* STOP2 
 * VDD: to what distribute power
 * Voltage regulator: MR is OFF and LPR is ON
 * All clocks OFF except LSI and LSE
 * Wakeup system clock: Either HSI16: if STOPWUCK=1 or MSI if STOPWUCK=0
 * LPMS="010"  SLEEPDEEP=1 and WFI
 * Disabled: TIM2, SysTick timer, USARTs, Flash memory, HSI16,
 */
static inline void stopkbd(void) {
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // disable SysTicks      
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; //enable clock to the PWR peripheral to access it

  //TIM2->CR1 &= ~(1UL<<0);    // Disable timer counter
			     //
  //RCC->AHB2ENR &= ~3;  // disable all GPIO clock
  RCC->AHB2ENR = 0UL;  // disable all GPIO clock
  //USART2->CR1 &= ~(BIT(0) | BIT(2) | BIT(3));  // Set UE, RE, TE
  //RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;  // disable USART2
  //RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM2EN;  // disable tim2

  //RCC->AHB2SMENR |= BIT(0) | BIT(1);  
  // Need to clear the pending flags here, before entering...
  //PWR->CR &= ~PWR_CR_PVDE;                       // disable PVD
  //PWR->CR |= PWR_CR_ULP;                         // ultra-low power (VREFINT off)
  //PWR->CR |= PWR_CR_LPSDSR;                      // Switch voltage regulator to low power mode
  /*  USART: 
   *  see pag.1236 and 1241: In STOP2 mode USART must be diabled (bit UE), but before the 
   *  TE bit bust be reset, and wait until the TC bit in USART_ISR is set, then set UE.
   *  Also, the DMA channel must be disabled before resetting the UE bit.
   */

  __WFI();

  __disable_irq(); // execute this before the interrupt ISR
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY)) spin(1);
  RCC->CFGR &= ~(RCC_CFGR_SW);
  RCC->CFGR |= (RCC_CFGR_SW_HSI);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) spin(1);

  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // re-enable USART2
  USART1->CR1 |= BIT(0) | BIT(2) | BIT(3);  // Set UE, RE, TE
  //RCC->APB1ENR1 |= (1<<0); // Enable clock for TIM2
  RCC->AHB2ENR |= 3;  // enable GPIO clocks for bit 0 and 1 (A and B banks)
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;        
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  __enable_irq();

}

static inline void stop2_init(void) {
  PWR->CR1 &= ~PWR_CR1_LPMS_STOP2_Msk;
  PWR->CR1 |= PWR_CR1_LPMS_STOP2;
  SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;  // disable re-entering in sleep mode after ISR
  RCC->CFGR |= RCC_CFGR_STOPWUCK;  // select the HSI16 clock for wakeup
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;  // set Cortex SLEEPDEEP bit
  PWR->CR3 |= PWR_CR3_ENULP;  // enable ultra low power sampling
}
