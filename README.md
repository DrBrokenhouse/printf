****************************************************************
****************************************************************
SPECIAL NOTE:

For some reason the format is better when viewed in RAW format.
****************************************************************
****************************************************************


Welcome to CooCox CoIDE (screen)
- Click on Create a New Project
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
  Change line 24
from
  #define STACK_SIZE       0x00000200      /*!< The Stack size suggest using even number    */
to 
  #define STACK_SIZE       0x00002000      /*!< The Stack size suggest using even number    */

Change line 143
from 
  //extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */
to
  extern void SystemInit(void);    /*!< Setup the microcontroller system(CMSIS) */

Change line 157 
from
  (void *)&pulStack[STACK_SIZE],     /*!< The initial stack pointer         */
to
  (void (*)(void))((unsigned long)pulStack + sizeof(pulStack)), /*!< The initial stack pointer */

Change line 299 (insert new line. line 300 is main();)
from  
 main();
to
  SystemInit();
  main(); 

****************************************************************
FILE: system_stm32f4xx.c
Change line 59
from
  25000000
to 
  8000000

Change line 60
from
  25
to 
  8

Change line 149
from
  #define PLL_M      25
to
  #define PLL_M      8
****************************************************************
FILE: stm32f4xx.h 
Change line 92/
from
#define HSE_VALUE    ((uint32_t)25000000)    
to
#define HSE_VALUE    ((uint32_t)8000000) 
****************************************************************
FILE: serial.h

Change line 32
from
  #include "board.h"
to 
  #include "stm32f4xx_conf.h"

****************************************************************

FILE: syscalls.c

Replace this lines 4 - 9
  #include "board.h"
 
  #include <stdio.h>
  #include <stdarg.h>
  #include <sys/types.h>
  #include <sys/stat.h>


With this
  #include "stm32f4xx_conf.h"


Change (approx.) line 74 - 88
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
  #include "stm32f4xx.h"
  #include "serial.h"
  #include "misc.h"
  #include "usart.h"
  #include <stdio.h>
  #include <string.h>
  #include <stdarg.h>
  #include <sys/types.h>
  #include <sys/stat.h>



****************************************************************
FILE: main.c
****************************************************************



#include "stm32f4xx_conf.h"


extern void USART_Config(void);
void GPIO_Setup(void);
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
extern __IO uint32_t TimingDelay;


int main(void)
{
	int ii= 0;
	// Set unbuffered mode for stdout (newlib) 
	setvbuf( stdout, 0, _IONBF, 0 );
	// Set up LEDs
	GPIO_Setup();
	// Set up USART2
	USART_Config();


	 if (SysTick_Config(SystemCoreClock / 1000))
	{
		 /* Capture error */
		while (1);
	}

    while(1)
    {
    	printf("STM32F4 Discovery Board Project. printf() %d: Green LED \r\n",ii);
    	GPIO_SetBits(GPIOD, GPIO_Pin_12);
    	Delay(500);
    	GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    	Delay(500);
    	ii++;
    	printf("STM32F4 Discovery Board Project. printf() %d: Amber LED \r\n",ii);
    	GPIO_SetBits(GPIOD, GPIO_Pin_13);
    	Delay(500);
    	GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    	Delay(500);
    	ii++;
    	printf("STM32F4 Discovery Board Project. printf() %d: Red LED \r\n",ii);
    	GPIO_SetBits(GPIOD, GPIO_Pin_14);
    	Delay(500);
    	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    	Delay(500);
    	ii++;
    	printf("STM32F4 Discovery Board Project. printf() %d: Blue LED \r\n",ii);
    	GPIO_SetBits(GPIOD, GPIO_Pin_15);
    	Delay(500);
    	GPIO_ResetBits(GPIOD, GPIO_Pin_15);
    	Delay(500);
    	ii++;
    }
}

void GPIO_Setup(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void Delay(__IO uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}


*****************************************************************
FILE: usart.c
*****************************************************************
#include "usart.h"

void USART_Config(void)
{ 						
	/*
	PuTTY Configuration:
	- Port = COM3
	- Speed (baud) = 115200
	- Data bits = 8
	- Stop Bit = 1
	- Parity = None
	- Flow control = XON/XOFF
	 */

  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  // USART2 TX -> PA2 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // USART2 RX -PA3 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // USART2 setup
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  
  // iterrups when USART2 Tx data register is empty
  USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

  USART_Cmd(USART2, ENABLE);
  USART_NVIC_Config();
}

void USART_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable the USARTx Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



*****************************************************************
FILE: usart.h
*****************************************************************

#ifndef _USART_H
#define _USART_H

#include "stm32f4xx_conf.h"



void USART_Config(void);
void USART_NVIC_Config(void);

#endif /*_USART_H*/

*****************************************************************
FILE: stm32f4xx_it.h
*****************************************************************

/**
  ******************************************************************************
  * @file   stm32f4xx_it.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


*****************************************************************
FILE: stm32f4xx_it.c
*****************************************************************

/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup I2C_EEPROM
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
__IO uint32_t TimingDelay;
void SysTick_Handler(void)
{
  if(TimingDelay !=0)
  {
	  TimingDelay --;
  }
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
void USART2_IRQHandler(void)
{

  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  { 
  		//USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		printf("\n\rUSART Hyperterminal Interrupts Receive a word: %c\n\r",USART_ReceiveData(USART2));
  }
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

