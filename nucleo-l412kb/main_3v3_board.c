// Copyright (c) 2022-2023 Cesanta Software Limited
// SPDX-License-Identifier: MIT

#include "hal.h"
#include "input-event-codes.h"

#define TKEY0  1LL<<4
#define TKEY1  1LL<<60
#define TKEY2  1LL<<61

static uint8_t keycode[256] = { \
     KEY_ESC, KEY_TAB, KEY_CAPSLOCK, KEY_LEFTSHIFT, KEY_TYPE, KEY_MINUS, KEY_A, KEY_Q, KEY_1, \
       KEY_2, KEY_W, KEY_S, KEY_Z, KEY_LEFTCTRL, KEY_X, KEY_D, KEY_E, KEY_3, \
       KEY_4, KEY_R, KEY_F, KEY_C, KEY_LEFTALT, KEY_V, KEY_G, KEY_T, KEY_5, \
       KEY_6, KEY_Y, KEY_H, KEY_B, KEY_SPACE, KEY_N, KEY_J, KEY_U, KEY_7, \
       KEY_8, KEY_I, KEY_K, KEY_M, KEY_RESERVED, KEY_COMMA, KEY_L, KEY_O, KEY_9, \
       KEY_0, KEY_P, KEY_SEMICOLON, KEY_DOT, KEY_RIGHTALT, KEY_SLASH, KEY_APOSTROPHE, KEY_LEFTBRACE, KEY_MINUS, \
       KEY_EQUAL, KEY_RIGHTBRACE, KEY_BACKSLASH, KEY_RIGHTSHIFT, KEY_RIGHTCTRL, KEY_TYPE, KEY_TYPE,KEY_ENTER, KEY_BACKSPACE, \
       KEY_RESERVED, \
     KEY_ESC, KEY_TAB, KEY_CAPSLOCK, KEY_LEFTSHIFT, KEY_TYPE, KEY_MINUS, KEY_A, KEY_Q, KEY_F1, \
       KEY_F2, KEY_W, KEY_S, KEY_Z, KEY_LEFTCTRL, KEY_X, KEY_D, KEY_E, KEY_F3, \
       KEY_F4, KEY_R, KEY_F, KEY_C, KEY_LEFTALT, KEY_V, KEY_G, KEY_T, KEY_F5, \
       KEY_F6, KEY_Y, KEY_H, KEY_B, KEY_SPACE, KEY_N, KEY_J, KEY_U, KEY_F7, \
       KEY_F8, KEY_I, KEY_K, KEY_M, KEY_RESERVED, KEY_COMMA, KEY_LEFT, KEY_O, KEY_F9, \
       KEY_F10, KEY_UP, KEY_DOWN, KEY_DOT, KEY_RIGHTALT, KEY_SLASH, KEY_RIGHT, KEY_UP, KEY_MINUS, \
       KEY_EQUAL, KEY_PAGEUP, KEY_PAGEDOWN, KEY_RIGHTSHIFT, KEY_RIGHTCTRL, KEY_TYPE, KEY_TYPE,KEY_ENTER, KEY_BACKSPACE, \
       KEY_RESERVED, \
     KEY_ESC, KEY_TAB, KEY_CAPSLOCK, KEY_LEFTSHIFT, KEY_TYPE, KEY_MINUS, KEY_A, KEY_Q, KEY_1, \
       KEY_2, KEY_W, KEY_S, KEY_Z, KEY_LEFTCTRL, KEY_X, KEY_D, KEY_E, KEY_3, \
       KEY_4, KEY_R, KEY_F, KEY_C, KEY_LEFTALT, KEY_V, KEY_G, KEY_T, KEY_5, \
       KEY_6, KEY_Y, KEY_H, KEY_B, KEY_SPACE, KEY_N, KEY_J, KEY_U, KEY_7, \
       KEY_8, KEY_I, KEY_K, KEY_M, KEY_RESERVED, KEY_COMMA, KEY_L, KEY_O, KEY_9, \
       KEY_0, KEY_P, KEY_SEMICOLON, KEY_DOT, KEY_RIGHTALT, KEY_SLASH, KEY_APOSTROPHE, KEY_LEFTBRACE, KEY_MINUS, \
       KEY_EQUAL, KEY_RIGHTBRACE, KEY_BACKSLASH, KEY_RIGHTSHIFT, KEY_RIGHTCTRL, KEY_TYPE, KEY_TYPE,KEY_ENTER, KEY_BACKSPACE, \
       KEY_RESERVED, \
     KEY_ESC, KEY_TAB, KEY_CAPSLOCK, KEY_LEFTSHIFT, KEY_TYPE, KEY_MINUS, KEY_A, KEY_Q, KEY_1, \
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
void SysTick_Handler(void) {       // SyStick IRQ handler, triggered every 1ms
  s_ticks++;
}

static volatile uint8_t pendingkey;  
static volatile uint8_t bouncing; // global bouncing   
static volatile uint8_t coln; // holds the column that triggered
static volatile uint8_t keyn; // holds the key that triggered
static volatile uint8_t keymod; // holds the layer offset
static volatile uint8_t keylock; // holds the key locks
static volatile uint16_t rowscan; // holds the status of the NROWS rows in the active coln
static volatile uint64_t keydown; // holds the status of the 62 keys   

void TIM2_IRQHandler(void) {
   if (TIM2->SR & (1<<0)) {
     TIM2->SR &= ~(1<<0); // Clear UIF update interrupt flag
     bouncing = 0;
     //EXTI->IMR1 |= (1 << 0); // re-enable the EXTI interrupt, HERE: need to map to the coln EXTI
     TIM2->CR1 &= ~(1<<0);    // Disable timer
   }
}

void kbd_layer(void) {
    if (keydown & TKEY0) {
      keymod = 64;  // Layer-1: While holding TKEY0
      keylock = 0;
      if (keydown & TKEY1) {
        keymod = 128;  // Layer-2: Once TKEY0+TKEY1
        keylock = 1;
      }
      else if (keydown & TKEY2) {
        keymod = 192;  // Layer-3: Once TKEY0+TKEY2
        keylock = 2;
      }
    }
    else if (!keylock) {
      keymod = 0;  // Back to Layer-0: Releasing TKEY0
    }
}

void col_IRQ(uint8_t col) {
    //startkbd();
    if (pendingkey == 0) {
      pendingkey = 1;
      bouncing = 1;
      coln = col;
      set_triggers(0);
      TIM2->CNT = 0; 
      TIM2->CR1 |= (1<<0);    // Enable timer
      set_all_rows(0);
    }
}


void EXTI0_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 0)) {
    EXTI->PR1 |= (1 << 0);
    col_IRQ(0);
  }
}
void EXTI1_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 1)) {
    EXTI->PR1 |= (1 << 1);
    col_IRQ(1);
  }
}
void EXTI3_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 3)) {
    EXTI->PR1 |= (1 << 3);
    col_IRQ(2);
  }
}
void EXTI4_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 4)) {
    EXTI->PR1 |= (1 << 4);
    col_IRQ(3);
  }
}
void EXTI9_5_IRQHandler(void) {
  if (EXTI->PR1 & (1 << 5)) {
    EXTI->PR1 |= (1 << 5);
    col_IRQ(4);
  }
  else if (EXTI->PR1 & (1 << 6)) {
    EXTI->PR1 |= (1 << 6);
    col_IRQ(5);
  }
  else if (EXTI->PR1 & (1 << 7)) {
    EXTI->PR1 |= (1 << 7);
    col_IRQ(6);
  }
}

int main(void) {
  //stop2_init();
  gpio_matrix_init();
  set_all_rows(1);
  col_trigger_init();

  //uart_init(UART_DEBUG, 115200);
  uart_init(USART1, 115200);
  init_debouncer();

  pendingkey = 0;
  bouncing = 0;
  keydown = 0;
  rowscan = 0;
  coln = 0;
  keymod = 0;
  keylock = 0;

  set_triggers(1);
  //while (s_ticks < 10000) spin(1);
  
  while (1) {
    if (!bouncing && pendingkey) {
       rowscan = 0;
       for (int m=0 ; m < NROWS ; m++) {
           cycle_to_row(ROWS[(m+1)%NROWS]); 
           while (!gpio_read_out(ROWS[(m+1)%NROWS])) spin(1); // wait until pin is set
	   rowscan |= (gpio_read(COLS[coln]) << (m+1)%NROWS );
       }

       rowscan ^= ((keydown & ((ROWMASK*1ULL)<<(NROWS*coln)))>>(NROWS*coln));  // bits changed 
       if (rowscan && !(rowscan & (rowscan-1)) ) {  // one and only one bit changed
         keydown ^= rowscan << (NROWS*coln);
	 // check the bit that changed
         keyn = NROWS*coln;
         while(rowscan-1){
             rowscan = rowscan >> 1;
             ++keyn;
         }

	 if (keycode[keyn]>>7) kbd_layer(); // process special KEY
	 //else uart_write_byte(UART_DEBUG, keycode[keyn+keymod] + 128*((keydown & (1<<keyn)) >> keyn));
	 else uart_write_byte(USART1, keycode[keyn+keymod] + 128*((keydown & (1<<keyn)) >> keyn));
       }

       set_all_rows(1);
       spin(4);
       pendingkey = 0; // Now EXTIs can trigger again
       set_triggers(1);
       //uart_write_buf(UART_DEBUG, "Entering STOP2 mode", 19);
       stopkbd();
    }
  }
  return 0;
}
