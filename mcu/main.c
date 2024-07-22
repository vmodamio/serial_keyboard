// Copyright (c) 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include "hal.h"
#include "input-event-codes.h"

#define TKEY0  4
#define TKEY1  60
#define TKEY2  61

static uint8_t keycode[256] = { \
     KEY_ESC, KEY_TAB, KEY_CAPSLOCK, KEY_LEFTSHIFT, KEY_TYPE, KEY_102ND, KEY_A, KEY_Q, KEY_1, \
       KEY_2, KEY_W, KEY_S, KEY_Z, KEY_LEFTCTRL, KEY_X, KEY_D, KEY_E, KEY_3, \
       KEY_4, KEY_R, KEY_F, KEY_C, KEY_LEFTALT, KEY_V, KEY_G, KEY_T, KEY_5, \
       KEY_6, KEY_Y, KEY_H, KEY_B, KEY_SPACE, KEY_N, KEY_J, KEY_U, KEY_7, \
       KEY_8, KEY_I, KEY_K, KEY_M, KEY_RESERVED, KEY_COMMA, KEY_L, KEY_O, KEY_9, \
       KEY_0, KEY_P, KEY_SEMICOLON, KEY_DOT, KEY_RIGHTALT, KEY_SLASH, KEY_APOSTROPHE, KEY_LEFTBRACE, KEY_MINUS, \
       KEY_EQUAL, KEY_RIGHTBRACE, KEY_BACKSLASH, KEY_RIGHTSHIFT, KEY_RIGHTCTRL, KEY_TYPE, KEY_TYPE,KEY_ENTER, KEY_BACKSPACE, \
       KEY_RESERVED, \
     KEY_ESC, KEY_TAB, KEY_CAPSLOCK, KEY_LEFTSHIFT, KEY_TYPE, KEY_102ND, KEY_A, KEY_Q, KEY_F1, \
       KEY_F2, KEY_W, KEY_S, KEY_Z, KEY_LEFTCTRL, KEY_X, KEY_D, KEY_E, KEY_F3, \
       KEY_F4, KEY_R, KEY_F, KEY_C, KEY_LEFTALT, KEY_V, KEY_G, KEY_T, KEY_F5, \
       KEY_F6, KEY_Y, KEY_H, KEY_B, KEY_SPACE, KEY_N, KEY_J, KEY_U, KEY_F7, \
       KEY_F8, KEY_I, KEY_K, KEY_M, KEY_RESERVED, KEY_COMMA, KEY_LEFT, KEY_O, KEY_F9, \
       KEY_F10, KEY_UP, KEY_DOWN, KEY_DOT, KEY_RIGHTALT, KEY_SLASH, KEY_RIGHT, KEY_UP, KEY_MINUS, \
       KEY_EQUAL, KEY_PAGEUP, KEY_PAGEDOWN, KEY_RIGHTSHIFT, KEY_RIGHTCTRL, KEY_TYPE, KEY_TYPE,KEY_ENTER, KEY_BACKSPACE, \
       KEY_RESERVED, \
     KEY_ESC, KEY_TAB, KEY_CAPSLOCK, KEY_LEFTSHIFT, KEY_TYPE, KEY_102ND, KEY_A, KEY_Q, KEY_1, \
       KEY_2, KEY_W, KEY_S, KEY_Z, KEY_LEFTCTRL, KEY_X, KEY_D, KEY_E, KEY_3, \
       KEY_4, KEY_R, KEY_F, KEY_C, KEY_LEFTALT, KEY_V, KEY_G, KEY_T, KEY_5, \
       KEY_6, KEY_Y, KEY_H, KEY_B, KEY_SPACE, KEY_N, KEY_J, KEY_U, KEY_7, \
       KEY_8, KEY_I, KEY_K, KEY_M, KEY_RESERVED, KEY_COMMA, KEY_L, KEY_O, KEY_9, \
       KEY_0, KEY_P, KEY_SEMICOLON, KEY_DOT, KEY_RIGHTALT, KEY_SLASH, KEY_APOSTROPHE, KEY_LEFTBRACE, KEY_MINUS, \
       KEY_EQUAL, KEY_RIGHTBRACE, KEY_BACKSLASH, KEY_RIGHTSHIFT, KEY_RIGHTCTRL, KEY_TYPE, KEY_TYPE,KEY_ENTER, KEY_BACKSPACE, \
       KEY_RESERVED, \
     KEY_ESC, KEY_TAB, KEY_CAPSLOCK, KEY_LEFTSHIFT, KEY_TYPE, KEY_102ND, KEY_A, KEY_Q, KEY_1, \
       KEY_2, KEY_W, KEY_S, KEY_Z, KEY_LEFTCTRL, KEY_X, KEY_D, KEY_E, KEY_3, \
       KEY_4, KEY_R, KEY_F, KEY_C, KEY_LEFTALT, KEY_V, KEY_G, KEY_T, KEY_5, \
       KEY_6, KEY_Y, KEY_H, KEY_B, KEY_SPACE, KEY_N, KEY_J, KEY_U, KEY_7, \
       KEY_8, KEY_I, KEY_K, KEY_M, KEY_RESERVED, KEY_COMMA, KEY_L, KEY_O, KEY_9, \
       KEY_0, KEY_P, KEY_SEMICOLON, KEY_DOT, KEY_RIGHTALT, KEY_SLASH, KEY_APOSTROPHE, KEY_LEFTBRACE, KEY_MINUS, \
       KEY_EQUAL, KEY_RIGHTBRACE, KEY_BACKSLASH, KEY_RIGHTSHIFT, KEY_RIGHTCTRL, KEY_TYPE, KEY_TYPE,KEY_ENTER, KEY_BACKSPACE, \
       KEY_RESERVED, \
};


uint32_t SystemCoreClock;  // Required by CMSIS. Holds system core cock value
void SystemInit(void) {    // Called automatically by startup code
  clock_init();            // Sets SystemCoreClock
  stop2_init();
}

static volatile uint64_t s_ticks;  // Milliseconds since boot
static volatile uint64_t tick_init;  // Milliseconds since boot
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static volatile uint8_t scanning;  
static volatile uint8_t setready;  
static volatile uint8_t keyn; // holds the key that triggered
static volatile uint8_t keymod; // holds the layer offset
static volatile uint8_t keylock; // holds the key locks
volatile uint8_t keyboard[NROWS*NCOLS]; // holds the status of the keyboard
static volatile uint64_t keydown; // holds the key locks

//void TIM2_IRQHandler(void) {
//   if (TIM2->SR & (1<<0)) {
//     TIM2->SR &= ~(1<<0); // Clear UIF update interrupt flag
//     //scanning = 0;
//     //EXTI->IMR1 |= (1 << 0); // re-enable the EXTI interrupt, HERE: need to map to the coln EXTI
//     TIM2->CR1 &= ~(1<<0);    // Disable timer
//   }
//}

void kbd_layer(void) {
    if (keyboard[TKEY0]>>7) {
      keymod = 64;  // Layer-1: While holding TKEY0
      keylock = 0;
      if (keyboard[TKEY1]>>7) {
        keymod = 128;  // Layer-2: Once TKEY0+TKEY1
        keylock = 1;
      }
      else if (keyboard[TKEY2]>>7) {
        keymod = 192;  // Layer-3: Once TKEY0+TKEY2
        keylock = 2;
      }
    }
    else if (!keylock) {
      keymod = 0;  // Back to Layer-0: Releasing TKEY0
    }
}

void col_IRQ(void) {
    if (scanning == 0) {
      scanning = 1;
      set_triggers(0);
      //TIM2->CNT = 0; 
      //TIM2->CR1 |= (1<<0);    // Enable timer
      set_all_rows(0);
    }
}


void EXTI0_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 0)) {
    EXTI->PR1 |= (1 << 0);
    //col_IRQ(6);
    col_IRQ();
  }
}
void EXTI1_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 1)) {
    EXTI->PR1 |= (1 << 1);
    //col_IRQ(5);
    col_IRQ();
  }
}
void EXTI3_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 3)) {
    EXTI->PR1 |= (1 << 3);
    //col_IRQ(4);
    col_IRQ();
  }
}
void EXTI4_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 4)) {
    EXTI->PR1 |= (1 << 4);
    //col_IRQ(3);
    col_IRQ();
  }
}
void EXTI9_5_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 5)) {
    EXTI->PR1 |= (1 << 5);
    //col_IRQ(2);
    col_IRQ();
  }
  else if (EXTI->PR1 & (1 << 6)) {
    EXTI->PR1 |= (1 << 6);
    //col_IRQ(1);
    col_IRQ();
  }
  else if (EXTI->PR1 & (1 << 7)) {
    EXTI->PR1 |= (1 << 7);
    //col_IRQ(0);
    col_IRQ();
  }
}

int main(void) {
  gpio_matrix_init();
  set_all_rows(1);
  col_trigger_init();

  //uart_init(USART1, 115200);
  uart_init(USART1, 500000);
  //init_debouncer();

  scanning = 0;
  setready = 0;
  keymod = 0;
  keylock = 0;

  // Initialize the keyboard matrix
  for (int m=0; m<NROWS*NCOLS; m++) keyboard[m] = 0x00;  
  keydown = 0;

  set_triggers(1);
  //while (s_ticks < 10000) spin(1);
  uart_write_buf(USART1, "Init ",5 );
  
  while (1) {
    while (scanning) {   // expires in 32ms because of TIM2
       if (s_ticks > tick_init) {
	   //set_all_rows(0); //already in the interrupt
           tick_init = s_ticks;
           for (int m=0 ; m < NROWS ; m++) {
	       gpio_write(ROWS[m], 1);
	       for (int k=0; k < NCOLS ; k++) {
	           keyn = NROWS*k +m;
                   keyboard[keyn] = (keyboard[keyn] & Hmask) |
	                             ((keyboard[keyn] <<1 ) & Lmask) | 
	                             gpio_read(COLS[k]) ;
	           if (keyboard[keyn] == Hmask) {
	               keyboard[keyn] = 0x00;
		       keydown &= ~( 1<< keyn);
	               scanning = keydown ? 1 : 0;
		       setready = 1;
                       if (keycode[keyn]>>7) kbd_layer(); // process special KEY
		       else uart_write_byte(USART1, keycode[keyn+keymod] );
                       //uart_write_buf(USART1, "o] ",3 );
	           }
	           else if (keyboard[keyn] == Lmask) {
	               keyboard[keyn] = 0xFF;
		       keydown |= ( 1<< keyn);
                       if (keycode[keyn]>>7) kbd_layer(); // process special KEY
		       else uart_write_byte(USART1, keycode[keyn+keymod] + 128);
                       //uart_write_buf(USART1, "[x",2 );
	           }
	       }
	       gpio_write(ROWS[m], 0);
           }
	   
       }
       else spin(1);
    }

    if (setready) {
        set_all_rows(1);
        for (int m=0; m<NROWS*NCOLS; m++) keyboard[m] = (keyboard[m]>>7) ? 0xFF: 0x00;  
        spin(8);
        set_triggers(1);
        //TIM2->CR1 &= ~(1<<0);    // Disable timer
	setready = 0;
    }

    if ((USART1->ISR & BIT(6)) == 0) spin(1);
    else stopkbd();
  }  
  return 0;
}
