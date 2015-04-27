Welcome to CooCox CoIDE (front screen)
Click on Create a New Project
****************************************************************
New Project (screen)
PROJECT
  Project name
  printf_tutorial
    Next

MODEL
  Chip
    Next

CHIP
  ST
    Scoll down to STM32F4
  STM32F407VG (The chip on my STM32F4 discovery board)
    Finished
****************************************************************
REPOSITORY COMMON
Check the following boxes
  M4 CMSIS Core
  Retarget printf, syscalls
     Click Yes in the message box


REPOSITORY BOOT
  CMSIS BOOT


REPOSITORY PERIPHERAL.ST
  RCC
  GPIO
  USART
  MISC

VIEW (menu button)

CONFIGURATION COMPLIE
  FPU
    Not use FPU

CONFIGURATION LINK
  Library
    Use Base C library
  Linked Libraries
    Add
  Add Library
    Labrary path: m
      OK

****************************************************************

FILE: startup_stm32f3xx.c
  On line 24, change the STACK_SIZE value from 0x00000200  to  0x00002000   
  On line 143, remove the // from in front of extern void SystemInit(void); 
  On line 157, replace (void *)&pulStack[STACK_SIZE], with (void (*)(void))((unsigned long)pulStack + sizeof(pulStack)), 
  On line 299,insert new SystemInit(); above main();

****************************************************************

FILE: system_stm32f4xx.c
On line 59, change 25000000 to 8000000
On line 60, change 25 to 8
On line 149, change 25 to 8

****************************************************************

FILE: stm32f4xx.h 
On line 92, change the HSE_VALUE to ((uint32_t)8000000) 

****************************************************************

FILE: serial.h
On line 32, Replace board.h with stm32f4xx_conf.h

****************************************************************

FILE: syscalls.c
On lines 4 - 9 replce
 0
 1 
 2
 3 
 4 #include "board.h"
 5
 6 #include <stdio.h>
 7 #include <stdarg.h>
 8 #include <sys/types.h>
 9 #include <sys/stat.h>

With this
 1 #include "syscalls.h"
 2 #include "stm32f4xx_rcc.h"
 3 #include "stm32f4xx_usart.h"
 4 #include <stdio.h>
 5 #include <string.h>
 6 #include <stdarg.h>
 7 #include <sys/types.h>
 8 #include <sys/stat.h>

On (approx.) line 74 - 88, replace the following 

extern int _write( int file, char *ptr, int len )
{
   int iIndex = 0;
    
   #ifdef SERIAL_DEBUG
   for ( iIndex=0 ; iIndex < len ; iIndex++, ptr++ )
   {
      USART_SendData( USART_SERIAL_DEBUG, *ptr ) ;
      while (USART_GetFlagStatus(USART_SERIAL_DEBUG, USART_FLAG_TC) == RESET)
      {}
   }
   #endif

   return iIndex ;
}

--- with this ---
int _write(int file, char *ptr, int len)
{
   int counter;

   counter = len;
   for (; counter > 0; counter--)
   {
      if (*ptr == 0) break;
      USART_SendData(USART2, (uint16_t) (*ptr));

      // Loop until the end of transmission
      while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
      ptr++;
   }
   return len;
}

****************************************************************
FILE: stm32f4xx_conf.h

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
//#include "stm32f4xx_adc.h"
//#include "stm32f4xx_can.h"
//#include "stm32f4xx_crc.h"
//#include "stm32f4xx_cryp.h"
//#include "stm32f4xx_dac.h"
//#include "stm32f4xx_dbgmcu.h"
//#include "stm32f4xx_dcmi.h"
//#include "stm32f4xx_dma.h"
//#include "stm32f4xx_exti.h"
//#include "stm32f4xx_flash.h"
//#include "stm32f4xx_fsmc.h"
//#include "stm32f4xx_hash.h"
  #include "stm32f4xx_gpio.h"
//#include "stm32f4xx_i2c.h"
//#include "stm32f4xx_iwdg.h"
//#include "stm32f4xx_pwr.h"
  #include "stm32f4xx_rcc.h"
//#include "stm32f4xx_rng.h"
//#include "stm32f4xx_rtc.h"
//#include "stm32f4xx_sdio.h"
//#include "stm32f4xx_spi.h"
//#include "stm32f4xx_syscfg.h"
//#include "stm32f4xx_tim.h"
  #include "stm32f4xx_usart.h"
//#include "stm32f4xx_wwdg.h"
  #include "misc.h"

Add
#include "syscalls.h"
