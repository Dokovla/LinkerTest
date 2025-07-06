# LinkerTest
Simple program for linker modification

1. Practical steps for  STM32CubeIDE:

🧩 Step 1: modify .ld fajl manually
Example: For STM32F103 if the app needs to start from 0x08004000, in STM32F103C8Tx_FLASH.ld find:

/* original */
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 64K
/* replace with: */
FLASH (rx)      : ORIGIN = 0x08004000, LENGTH = 48K


🧩 Step 2: Check  VECT_TAB_OFFSET
In system_stm32f1xx.c:
#define VECT_TAB_OFFSET  0x4000  // 16KB if app starts on 0x08004000
Or in main.c, before HAL_Init:
SCB->VTOR = 0x08004000;
