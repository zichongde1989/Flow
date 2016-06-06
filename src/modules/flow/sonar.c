/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Laurens Mackay <mackayl@student.ethz.ch>
 *   		 Dominik Honegger <dominik.honegger@inf.ethz.ch>
 *   		 Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "utils.h"
#include "usart.h"
#include "settings.h"
#include "sonar.h"
#include "sonar_mode_filter.h"

#ifdef SONAR_USING_MB1240
#define SONAR_VCC (5.0)
#define ADC_VREF (3.3)
#define ADCBITWIDTH  (12)
#define ADCDATA2VOLTAGE_FACTOR  (ADC_VREF/(2^ADCBITWIDTH))
#define VOLTAGE2DISTENCE_FACTOR (SONAR_VCC/1024.0) /** analog voltage with a scaling factor,mV->cm */
#define COMPENSATION  (1.1235)	/* The Compensation factor */

#define SONAR_SCALE	100.0f
#define SONAR_MIN	0.20f		/** 0.20m sonar minimum distance */
#define SONAR_MAX	7.0f		/** 5.0m sonar maximum distance */
#else
#define SONAR_SCALE	1000.0f
#define SONAR_MIN	0.15f		/** 0.15m sonar minimum distance */
#define SONAR_MAX	3.5f		/** 3.5m sonar maximum distance */
#endif
#define atoi(nptr)  strtol((nptr), NULL, 10)
extern uint32_t get_boot_time_us(void);

#ifndef SONAR_USING_MB1240
static char data_buffer[5]; // array for collecting decoded data
#endif
static volatile uint32_t last_measure_time = 0;
static volatile uint32_t measure_time = 0;
static volatile float dt = 0.0f;
static volatile int valid_data;

#ifndef SONAR_USING_MB1240
static volatile int data_counter = 0;
static volatile int data_valid = 0;
#endif
static volatile int new_value = 0;

static volatile uint32_t sonar_measure_time_interrupt = 0;
static volatile uint32_t sonar_measure_time = 0;

/* kalman filter states */
float x_pred = 0.0f; // m
float v_pred = 0.0f;
float x_post = 0.0f; // m
float v_post = 0.0f; // m/s

float sonar_raw = 0.0f;  // m

float sonar_mode = 0.0f;
bool sonar_valid = false;				/**< the mode of all sonar measurements */



/**
  * @brief  Triggers the sonar to measure the next value
  *
  * see datasheet for more info
  */
void sonar_trigger(){
	GPIO_SetBits(GPIOE, GPIO_Pin_8);
	
#ifdef	SONAR_USING_MB1240
	/* Start the ADC Convesation to get the last value */ 
	ADC_SoftwareStartConv( ADC1 );
#endif

}




#ifdef	SONAR_USING_MB1240

/**
  * @brief  Sonar adc interrupt handler
  * @author zcd
  *
  */
void ADC_IRQHandler(void)

{

	if ( ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
	{
		/* Read one byte from the receive ADC register */
		uint16_t data = ADC_GetConversionValue( ADC1 ) ;

		/* set sonar pin 4 to low -> we want triggered mode */
		GPIO_ResetBits(GPIOE, GPIO_Pin_8);

		/* caculate the distence, V -- > cm */
		double temp1 = data * ADC_VREF/SONAR_VCC/4; //data * ADCDATA2VOLTAGE_FACTOR/ VOLTAGE2DISTENCE_FACTOR;
		int temp = temp1 * COMPENSATION;			//do the caliburation(or compensation?) 

//		float temp1 = data * 3.3/4096;
//		int temp = (temp1 * 204 );

		
		/* use real-world maximum ranges to cut off pure noise */
		if ((temp > SONAR_MIN*SONAR_SCALE) && (temp < SONAR_MAX*SONAR_SCALE))
		{
			/* it is in normal sensor range, take it */
			last_measure_time = measure_time;
			measure_time = get_boot_time_us();
			sonar_measure_time_interrupt = measure_time;
			dt = ((float)(measure_time - last_measure_time)) / 1000000.0f;
		
			valid_data = temp;
			// the mode filter turned out to be more problematic
			// than using the raw value of the sonar
			//insert_sonar_value_and_get_mode_value(valid_data / SONAR_SCALE);
			sonar_mode = valid_data / SONAR_SCALE;
			new_value = 1;
			sonar_valid = true;
		} else {
			sonar_valid = false;
		}

		ADC_ClearITPendingBit( ADC1, ADC_IT_EOC );
	}
	else if ( ADC_GetITStatus(ADC1, ADC_IT_AWD) != RESET)
	{
		ADC_ClearITPendingBit( ADC1, ADC_IT_AWD );
	}
	else if ( ADC_GetITStatus(ADC1, ADC_IT_JEOC) != RESET)
	{
		ADC_ClearITPendingBit( ADC1, ADC_IT_JEOC );
	}
	else if ( ADC_GetITStatus(ADC1, ADC_IT_OVR) != RESET)
	{
		ADC_ClearITPendingBit( ADC1, ADC_IT_OVR );
	}
}

#else

/**
  * @brief  Sonar interrupt handler
  */
void UART4_IRQHandler(void)
{
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		uint8_t data = (USART_ReceiveData(UART4));

		if (data == 'R')
		{
			/* this is the first char (start of transmission) */
			data_counter = 0;
			data_valid = 1;

			/* set sonar pin 4 to low -> we want triggered mode */
			GPIO_ResetBits(GPIOE, GPIO_Pin_8);
		}
		else if (0x30 <= data && data <= 0x39)
		{
			if (data_valid)
			{
				data_buffer[data_counter] = data;
				data_counter++;
			}
		}
		else if (data == 0x0D)
		{
			if (data_valid && data_counter == 4)
			{
				data_buffer[4] = 0;
				int temp = atoi(data_buffer);

				/* use real-world maximum ranges to cut off pure noise */
				if ((temp > SONAR_MIN*SONAR_SCALE) && (temp < SONAR_MAX*SONAR_SCALE))
				{
					/* it is in normal sensor range, take it */
					last_measure_time = measure_time;
					measure_time = get_boot_time_us();
					sonar_measure_time_interrupt = measure_time;
					dt = ((float)(measure_time - last_measure_time)) / 1000000.0f;

					valid_data = temp;
					// the mode filter turned out to be more problematic
					// than using the raw value of the sonar
					//insert_sonar_value_and_get_mode_value(valid_data / SONAR_SCALE);
					sonar_mode = valid_data / SONAR_SCALE;
					new_value = 1;
					sonar_valid = true;
				} else {
					sonar_valid = false;
				}
			}

			data_valid = 0;
		}
		else
		{
			data_valid = 0;
		}
	}
}

#endif


/**
  * @brief  Basic Kalman filter
  */
static void sonar_filter(void)
{
	/* no data for long time */
	if (dt > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt * v_pred;
	v_pred = v_post;

	float x_new = sonar_mode;
	sonar_raw = x_new;
	x_post = x_pred + global_data.param[PARAM_SONAR_KALMAN_L1] * (x_new - x_pred);
	v_post = v_pred + global_data.param[PARAM_SONAR_KALMAN_L2] * (x_new - x_pred);

}


/**
  * @brief  Read out newest sonar data
  *
  * @param  sonar_value_filtered Filtered return value
  * @param  sonar_value_raw Raw return value
  */
bool sonar_read(float* sonar_value_filtered, float* sonar_value_raw)
{
	/* getting new data with only around 10Hz */
	if (new_value) {
		sonar_filter();
		new_value = 0;
		sonar_measure_time = get_boot_time_us();
	}

	/* catch post-filter out of band values */
	if (x_post < SONAR_MIN || x_post > SONAR_MAX) {
		sonar_valid = false;
	}

	*sonar_value_filtered = x_post;
	*sonar_value_raw = sonar_raw;

	return sonar_valid;
}

/**
 * @brief  Configures the sonar sensor Peripheral.
 */
void sonar_config(void)
{
	valid_data = 0;

	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef	SONAR_USING_MB1240
	/* Enable the ADC interface clock using */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 

	/* ADC pins configuration,Enable the clock for the ADC GPIOs  */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	
	/* Configure these ADC pins in analog mode using */
	/* Configure ADC123_IN10  pin in anlog mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);



	/* ADC1 common configuration */
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	ADC_CommonStructInit( &ADC_CommonInitStruct );
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;						/* Initialize the ADC_Mode member */
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;					/* initialize the ADC_Prescaler member */
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;		/* Initialize the ADC_DMAAccessMode member */
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;	/* Initialize the ADC_TwoSamplingDelay member */
	ADC_CommonInit( &ADC_CommonInitStruct );	

	ADC_InitTypeDef ADC_InitStructure;
	/*  Configure the ADC Prescaler, conversion resolution and data  */
	ADC_StructInit( &ADC_InitStructure );
	ADC_InitStructure.ADC_Resolution =  ADC_Resolution_12b;  
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ;  
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; 
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	
	/* configure with ADC channel 10 */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);

	/* Configures the nested vectored interrupt controller. */
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the ADCx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enables the End of  ADC interrupts conversion interrupt mask . */
	ADC_ITConfig( ADC1, ADC_IT_EOC  , ENABLE) ;

	/* Activate the ADC peripheral */
	ADC_Cmd(ADC1, ENABLE);

	/* Start the ADC Convesation */ 
	ADC_SoftwareStartConv( ADC1 );

#else
	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure l3gd20 CS pin in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Configures the nested vectored interrupt controller. */
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Connect UART pins to AF7 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	GPIO_InitTypeDef GPIO_InitStructure_Serial2;
	GPIO_InitStructure_Serial2.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure_Serial2.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure_Serial2.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_Serial2.GPIO_PuPd = GPIO_PuPd_UP;

	/* USART RX pin configuration */
	GPIO_InitStructure_Serial2.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure_Serial2);

	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;

	/* Configure the UART4 */
	USART_Init(UART4, &USART_InitStructure);

	/* Enable UART4 interrupt */
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	USART_Cmd(UART4, ENABLE);

#endif
	

}

uint32_t get_sonar_measure_time()
{
    return sonar_measure_time;
}

uint32_t get_sonar_measure_time_interrupt()
{
    return sonar_measure_time_interrupt;
}

