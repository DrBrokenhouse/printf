Welcome to CooCox CoIDE (screen)
- Click on Create a New Project
****************************************************************
New Project (screen)
PROJECT
- Project name
  printf_tutorial
- Next

MODEL
-Chip
-Next

CHIP
- ST
  scoll down to STM32F4
- STM32F407VG (The chip on my STM32F4 discovery board)
- Finished
****************************************************************
REPOSITORY COMMON
Check the following boxes
- M4 CMSIS Core
- Retarget printf, syscalls
     Click Yes in the message box


REPOSITORY BOOT
- CMSIS BOOT


REPOSITORY PERIPHERAL.ST
- RCC
- GPIO
- USART
- MISC

VIEW (menu button)

CONFIGURATION COMPLIE
- FPU
  Not use FPU

CONFIGURATION LINK
- Library
  Use Base C library
-Linked Libraries
  Add
- Add Library
  Labrary path: m
  OK

****************************************************************

FILE: startup_stm32f3xx.c
- Change line 24
from
- #define STACK_SIZE       0x00000200      /*!< The Stack size suggest using even number    */
to 
- #define STACK_SIZE       0x00002000      /*!< The Stack size suggest using even number    */

- Change line 143
from 
  //extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */
to
  extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */

- Change line 157 
from
   (void *)&pulStack[STACK_SIZE],     /*!< The initial stack pointer         */
to
   (void (*)(void))((unsigned long)pulStack + sizeof(pulStack)), /*!< The initial stack pointer */

- Change line 299 (insert new line. line 300 is main();)
from  
  ( a blank line)
to
  SystemInit(); 

****************************************************************
 FILE: system_stm32f4xx.c
- Change line 59
from
  25000000
to 
  8000000

- Change line 60
from
   25
to 
   8
- Change line 149
from
   #define PLL_M      25
to
   #define PLL_M      8
****************************************************************
FILE: stm32f4xx.h 
- Change line 92/
from
#define HSE_VALUE    ((uint32_t)25000000)    
to
#define HSE_VALUE    ((uint32_t)8000000) 
****************************************************************
FILE: serial.h

- Change line 32
from
#include "board.h"
to 
#include "stm32f4xx_conf.h"

****************************************************************

FILE: syscalls.c

- Replace this lines 4 - 9
#include "board.h"
 
  #include <stdio.h>
  #include <stdarg.h>
  #include <sys/types.h>
  #include <sys/stat.h>


-With this
  #include "syscalls.h"
  #include "stm32f4xx_rcc.h"
  #include "stm32f4xx_usart.h"
  #include <stdio.h>
  #include <string.h>
  #include <stdarg.h>
  #include <sys/types.h>
  #include <sys/stat.h>

- Change (approx.) line 74 - 88
from
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

to
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
#include "syscalls.h"
