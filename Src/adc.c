/* USER CODE BEGIN Header  */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "usbd_cdc_if.h"

uint16_t ad_value1[12];
uint16_t ad_value2[12];
uint16_t adnum1=0;
uint16_t adnum2=0;
uint16_t hallvalue_x1[100];
uint16_t hallvalue_x2[100];
uint16_t hallvalue_y[100];
uint16_t hallvalue_z[100];
uint32_t hallvalue32_x1;
uint32_t hallvalue32_x2;
uint32_t hallvalue32_y;
uint32_t hallvalue32_z;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC4     ------> ADC1_INP4
    PC5     ------> ADC1_INP8
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PF13     ------> ADC2_INP2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PC4     ------> ADC1_INP4
    PC5     ------> ADC1_INP8
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PF13     ------> ADC2_INP2
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_13);

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void ad_conv(void)
{
						for(char n=0;n<12;n++) 
						{
						HAL_ADC_Start(&hadc1);
            HAL_ADC_Start(&hadc2);
						if((HAL_ADC_PollForConversion(&hadc1, 10)==HAL_OK)&&(HAL_ADC_PollForConversion(&hadc2, 10)==HAL_OK))
            {
 						ad_value1[n]=HAL_ADC_GetValue(&hadc1);
						ad_value2[n]=HAL_ADC_GetValue(&hadc2);
						adnum1=adnum1+ad_value1[n];
						adnum2=adnum2+ad_value2[n];
						}
					  }
						uint16_t max1=ad_value1[0];
						uint16_t min1=ad_value1[0];
						uint16_t max2=ad_value2[0];
						uint16_t min2=ad_value2[0];
						for(char n=0;n<12;n++) 
						{
						max1=(ad_value1[n]<max1)?max1:ad_value1[n];    
            min1=(min1<ad_value1[n])?min1:ad_value1[n];
						max2=(ad_value2[n]<max2)?max2:ad_value2[n];    
            min2=(min2<ad_value2[n])?min2:ad_value2[n];
						}
						usbcdc_sendbuf[0]=((adnum1-max1-min1)/10+4095-(adnum2-max2-min2)/10) >> 8;
						usbcdc_sendbuf[1]=(adnum1-max1-min1)/10+4095-(adnum2-max2-min2)/10;
						CDC_Transmit_HS((uint8_t*)usbcdc_sendbuf,2);	
};

uint16_t hall_dect_z(void)
{ 
	
	
		        HAL_ADC_Start(&hadc2);
						if(HAL_ADC_PollForConversion(&hadc2, 10)==HAL_OK)
						{
						ad_value1[0]=HAL_ADC_GetValue(&hadc2);
						}
						CDC_Transmit_HS((uint8_t*)ad_value1,2);	
						return ad_value1[0];

};

uint32_t hall_dect_y(void)
{
						ADC_ChannelConfTypeDef sConfig = {0};
						  sConfig.Channel = ADC_CHANNEL_4;
							sConfig.Rank = ADC_REGULAR_RANK_1;
							sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
							sConfig.SingleDiff = ADC_SINGLE_ENDED;
							sConfig.OffsetNumber = ADC_OFFSET_NONE;
							sConfig.Offset = 0;
							sConfig.OffsetSignedSaturation = DISABLE;
//						for(char n=0;n<2;n++) 
//						{
						if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
						{
							Error_Handler();
						}
					
		        HAL_ADC_Start(&hadc1);
						if(HAL_ADC_PollForConversion(&hadc1,10)==HAL_OK)
						{
						hallvalue_y[0]=HAL_ADC_GetValue(&hadc1);
						hallvalue32_y+=hallvalue_y[0];
						}
//				  	}
						
//						CDC_Transmit_HS((uint8_t*)ad_value1,2);	
						hallvalue32_y=hallvalue32_y/2;
						return hallvalue32_y;

};



uint16_t hall_dect_x1(void)
{
						ADC_ChannelConfTypeDef sConfig = {0};
						sConfig.Channel = ADC_CHANNEL_8;
						sConfig.Rank = ADC_REGULAR_RANK_1;
						sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
							sConfig.SingleDiff = ADC_SINGLE_ENDED;
							sConfig.OffsetNumber = ADC_OFFSET_NONE;
							sConfig.Offset = 0;
							sConfig.OffsetSignedSaturation = DISABLE;
						if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
						{
							Error_Handler();
						}
											
						HAL_ADC_Start(&hadc1);
						if(HAL_ADC_PollForConversion(&hadc1, 10)==HAL_OK)
						{
						hallvalue_x1[0]=HAL_ADC_GetValue(&hadc1);
						}
						
						CDC_Transmit_HS((uint8_t*)hallvalue_y,2);	
						return hallvalue_x1[0];
};

uint16_t hall_dect_x2(void)
{
						HAL_ADC_Start(&hadc2);
						if(HAL_ADC_PollForConversion(&hadc2, 10)==HAL_OK)
						{
						ad_value1[0]=HAL_ADC_GetValue(&hadc2);
						}
						CDC_Transmit_HS((uint8_t*)ad_value1,2);	
						return ad_value1[0];
};



/* USER CODE END 1 */
