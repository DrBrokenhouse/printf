/**********************************************************************
 *    						serial.c
 *
 *
 * Author:    Julien Thaon Feb. 01, 2013
 * Copyright: DEMTACH SAS
 *
 * This file is part of YaRTOS.
 *
 * YaRTOS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * YaRTOS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 *
 **********************************************************************/
/**
 * 	\file	serial.c
 * 	\brief  
 */

#include "serial.h"


#ifdef SERIAL_DEBUG
	#ifdef __GNUC__
		#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE{
	#ifdef SERIAL_DEBUG
		USART_SendData(USART_SERIAL_DEBUG, (uint8_t) ch);
	
		// Loop until the end of transmission.
		while (USART_GetFlagStatus(USART_SERIAL_DEBUG, USART_FLAG_TC) == RESET)
		{}
	
		return ch;
	#endif
}

#endif
