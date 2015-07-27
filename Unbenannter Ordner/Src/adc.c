/**
  ******************************************************************************
  * File Name          : ADC.c
  * Date               : 13/07/2015 16:14:04
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 10;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 9;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 8;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 7;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc2);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 6;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 7;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc3.Init.Resolution = ADC_RESOLUTION12b;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc3);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

}
/* ADC4 init function */
void MX_ADC4_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc4.Init.Resolution = ADC_RESOLUTION12b;
  hadc4.Init.ScanConvMode = ENABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 2;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc4);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc4, &sConfig);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc4, &sConfig);

}

static int ADC12_CLK_ENABLED=0;
static int ADC34_CLK_ENABLED=0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    ADC12_CLK_ENABLED++;
    if(ADC12_CLK_ENABLED==1){
      __ADC12_CLK_ENABLE();
    }
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN6
    PC1     ------> ADC1_IN7
    PC2     ------> ADC1_IN8
    PC3     ------> ADC1_IN9
    PF2     ------> ADC1_IN10
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    PF4     ------> ADC1_IN5 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(hadc->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* Peripheral clock enable */
    ADC12_CLK_ENABLED++;
    if(ADC12_CLK_ENABLED==1){
      __ADC12_CLK_ENABLE();
    }
  
    /**ADC2 GPIO Configuration    
    PA4     ------> ADC2_IN1
    PA5     ------> ADC2_IN2
    PA6     ------> ADC2_IN3
    PA7     ------> ADC2_IN4
    PC4     ------> ADC2_IN5
    PC5     ------> ADC2_IN11
    PB2     ------> ADC2_IN12 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc2.Instance = DMA2_Channel1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc2);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc2);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(hadc->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* Peripheral clock enable */
    ADC34_CLK_ENABLED++;
    if(ADC34_CLK_ENABLED==1){
      __ADC34_CLK_ENABLE();
    }
  
    /**ADC3 GPIO Configuration    
    PB1     ------> ADC3_IN1
    PE9     ------> ADC3_IN2
    PE13     ------> ADC3_IN3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc3.Instance = DMA2_Channel5;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc3);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc3);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
  else if(hadc->Instance==ADC4)
  {
  /* USER CODE BEGIN ADC4_MspInit 0 */

  /* USER CODE END ADC4_MspInit 0 */
    /* Peripheral clock enable */
    ADC34_CLK_ENABLED++;
    if(ADC34_CLK_ENABLED==1){
      __ADC34_CLK_ENABLE();
    }
  
    /**ADC4 GPIO Configuration    
    PE14     ------> ADC4_IN1
    PE15     ------> ADC4_IN2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc4.Instance = DMA2_Channel2;
    hdma_adc4.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc4.Init.Mode = DMA_CIRCULAR;
    hdma_adc4.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&hdma_adc4);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc4);

  /* USER CODE BEGIN ADC4_MspInit 1 */

  /* USER CODE END ADC4_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    ADC12_CLK_ENABLED--;
    if(ADC12_CLK_ENABLED==0){
      __ADC12_CLK_DISABLE();
    }
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN6
    PC1     ------> ADC1_IN7
    PC2     ------> ADC1_IN8
    PC3     ------> ADC1_IN9
    PF2     ------> ADC1_IN10
    PA0     ------> ADC1_IN1
    PA1     ------> ADC1_IN2
    PA2     ------> ADC1_IN3
    PA3     ------> ADC1_IN4
    PF4     ------> ADC1_IN5 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_2|GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hadc->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    ADC12_CLK_ENABLED--;
    if(ADC12_CLK_ENABLED==0){
      __ADC12_CLK_DISABLE();
    }
  
    /**ADC2 GPIO Configuration    
    PA4     ------> ADC2_IN1
    PA5     ------> ADC2_IN2
    PA6     ------> ADC2_IN3
    PA7     ------> ADC2_IN4
    PC4     ------> ADC2_IN5
    PC5     ------> ADC2_IN11
    PB2     ------> ADC2_IN12 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hadc->DMA_Handle);
  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    ADC34_CLK_ENABLED--;
    if(ADC34_CLK_ENABLED==0){
      __ADC34_CLK_DISABLE();
    }
  
    /**ADC3 GPIO Configuration    
    PB1     ------> ADC3_IN1
    PE9     ------> ADC3_IN2
    PE13     ------> ADC3_IN3 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9|GPIO_PIN_13);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hadc->DMA_Handle);
  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC4)
  {
  /* USER CODE BEGIN ADC4_MspDeInit 0 */

  /* USER CODE END ADC4_MspDeInit 0 */
    /* Peripheral clock disable */
    ADC34_CLK_ENABLED--;
    if(ADC34_CLK_ENABLED==0){
      __ADC34_CLK_DISABLE();
    }
  
    /**ADC4 GPIO Configuration    
    PE14     ------> ADC4_IN1
    PE15     ------> ADC4_IN2 
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_14|GPIO_PIN_15);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hadc->DMA_Handle);
  /* USER CODE BEGIN ADC4_MspDeInit 1 */

  /* USER CODE END ADC4_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
