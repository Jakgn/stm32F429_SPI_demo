/**
 ******************************************************************************
 * @file    Template/main.c 
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    20-September-2013
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/** @addtogroup Template
 * @{
 */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#define LED_ON 0xF0
#define LED_OFF 0x0F
void init_SPI(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
#ifdef MASTER
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// enable SPI2 peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	// connect SPI2 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	/* configure SPI1 in Mode 0
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_I2S_DeInit(SPI1);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master; // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low; // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge; // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft|SPI_NSSInternalSoft_Set ; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE); // enable SPI1
#else //Slave mode
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// enable SPI2 peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/* configure pins used by SPI2
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	// connect SPI2 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	/* configure SPI1 in Mode 0
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_I2S_DeInit(SPI1);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Slave; // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low; // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge; // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft ; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE); // make SPI1 receive interrupt enable	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn; // Configure SPI1 interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // Set the priority group of SPI1 interrupt
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // Set the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Globally enable SPI1 interrupt
	NVIC_Init(&NVIC_InitStructure);

	SPI_Cmd(SPI1, ENABLE);
#endif	
}


void init_LED()
{
	/* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
	//STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED3);
}

//Init Push Button
void init_PB()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Here the GPIOA module is initialized.
	 * We want to use PA0 as an input because
	 * the USER button on the board is connected
	 * between this pin and VCC.
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; // we want to configure PA0
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // we want it to be an input
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN; // this enables the pulldown resistor --> we want to detect a high level
	GPIO_Init(GPIOA, &GPIO_InitStruct); // this passes the configuration to the Init function which takes care of the low level stuff
}
#ifdef MASTER
// FOR MASTER Used
uint8_t RemoteLED_OnOff(uint8_t action)
{
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
	if(action == 1) //Led On
		SPI_I2S_SendData(SPI1,LED_ON);
	else //Led Off
		SPI_I2S_SendData(SPI1,LED_OFF);
	return 1;
}
void pb_task(void *pvParameters)
{
	uint8_t LedOnOff = 0;
	uint8_t previous = 0;
	while(1)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)== Bit_SET) //User Button Pressed
		{
			if(previous == 0)
			{
				if(LedOnOff)
					RemoteLED_OnOff(0);
				else
					RemoteLED_OnOff(1);
				LedOnOff ^= 1;
				STM_EVAL_LEDToggle(LED3);
			}
			previous = 1;
		}
		else
			previous = 0;
	}
}
#else //slave

void SPI1_IRQHandler(void)
{
	uint8_t rcv_tmp = 0;
	while(1)
	{
		while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)== RESET);
		rcv_tmp = (uint8_t)SPI_I2S_ReceiveData(SPI1);
		if(rcv_tmp == LED_ON)
		{
			STM_EVAL_LEDOff(LED4);
			STM_EVAL_LEDOff(LED3);
		}
		else if(rcv_tmp == LED_OFF)
		{
			STM_EVAL_LEDOn(LED4);
			STM_EVAL_LEDOn(LED3);
		}
	}
}
#endif


int main(void)
{
	init_SPI();
	init_LED();
	init_PB();
#ifdef MASTER
	xTaskCreate(pb_task,
			"Push Button Task",
			512 /* stack size */, NULL, tskIDLE_PRIORITY + 2, NULL);
#else	

#endif
	/* Start running the tasks. */
	vTaskStartScheduler();
	return 0;	

}
