/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 26/05/2015 10:05:21
  * Description        : Main program body
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
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "uartmessagebuffer.h"
#include "midi.h"
#include "voltdrum_interface.h"
#include "mcp4251.h"
#include "triggerchannel.h"
#include "hihatchannel.h"
#include "performance.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define headChannelCount 10
#define rimChannelCount 10
#define hihatChannelCount 2

volatile int irqflag_timer1 = 0;
volatile int irqflag_timer2 = 0;
volatile int irqflag_timer3 = 0;
volatile int irqflag_btns[3] = {0,0,0};

volatile uint32_t adc1Samples[10*4];
volatile uint32_t adc2Samples[7*4];
volatile uint32_t adc3Samples[3*4];
volatile uint32_t adc4Samples[2];

TriggerChannel triggerChannelsHead[headChannelCount];
TriggerChannel triggerChannelsRim[rimChannelCount];
HiHatChannel hiHatChannels[hihatChannelCount];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void initHeadChannels(){

	int i;
	for(i = 0; i < headChannelCount; i++){
		triggerChannelsHead[i].potiWiperId = MCP4261_WIPER0;
		triggerchannel_init(&triggerChannelsHead[i], &(adc1Samples[i]), &(adc1Samples[i+10]), &(adc1Samples[i+20]), &(adc1Samples[i+30]));

	}

	triggerChannelsHead[0].potiCsPort = GPIOC;
	triggerChannelsHead[0].potiCsPin = GPIO_PIN_11;
	triggerChannelsHead[0].midinote = 36;

	triggerChannelsHead[1].potiCsPort = GPIOC;
	triggerChannelsHead[1].potiCsPin = GPIO_PIN_12;
	triggerChannelsHead[1].midinote = 38;

	triggerChannelsHead[2].potiCsPort = GPIOD;
	triggerChannelsHead[2].potiCsPin = GPIO_PIN_0;
	triggerChannelsHead[2].midinote = 38;

	triggerChannelsHead[3].potiCsPort = GPIOD;
	triggerChannelsHead[3].potiCsPin = GPIO_PIN_1;
	triggerChannelsHead[3].midinote = 38;

	triggerChannelsHead[4].potiCsPort = GPIOD;
	triggerChannelsHead[4].potiCsPin = GPIO_PIN_2;
	triggerChannelsHead[4].midinote = 38;

	triggerChannelsHead[5].potiCsPort = GPIOD;
	triggerChannelsHead[5].potiCsPin = GPIO_PIN_3;
	triggerChannelsHead[5].midinote = 38;

	triggerChannelsHead[6].potiCsPort = GPIOD;
	triggerChannelsHead[6].potiCsPin = GPIO_PIN_4;
	triggerChannelsHead[6].midinote = 38;

	triggerChannelsHead[7].potiCsPort = GPIOD;
	triggerChannelsHead[7].potiCsPin = GPIO_PIN_5;
	triggerChannelsHead[7].midinote = 38;

	triggerChannelsHead[8].potiCsPort = GPIOD;
	triggerChannelsHead[8].potiCsPin = GPIO_PIN_6;
	triggerChannelsHead[8].midinote = 38;

	triggerChannelsHead[9].potiCsPort = GPIOD;
	triggerChannelsHead[9].potiCsPin = GPIO_PIN_7;
	triggerChannelsHead[9].midinote = 38;

	for(i = 0; i < headChannelCount; i++){
		HAL_GPIO_WritePin(triggerChannelsHead[i].potiCsPort,triggerChannelsHead[i].potiCsPin,GPIO_PIN_SET);
	}

	HAL_Delay(10);

	//set initial wiper positions
	triggerchannel_setWiper(&triggerChannelsHead[0], 16);
	triggerchannel_setWiper(&triggerChannelsHead[1], 30);
	triggerchannel_setWiper(&triggerChannelsHead[2], 30);
	triggerchannel_setWiper(&triggerChannelsHead[3], 30);
	triggerchannel_setWiper(&triggerChannelsHead[4], 30);
	triggerchannel_setWiper(&triggerChannelsHead[5], 30);
	triggerchannel_setWiper(&triggerChannelsHead[6], 30);
	triggerchannel_setWiper(&triggerChannelsHead[7], 30);
	triggerchannel_setWiper(&triggerChannelsHead[8], 30);
	triggerchannel_setWiper(&triggerChannelsHead[9], 30);

}

void initRimChannels(){

	int i;
	for(i = 0; i < headChannelCount; i++){
		triggerChannelsRim[i].potiWiperId = MCP4261_WIPER1;
		if(i < 7) triggerchannel_init(&triggerChannelsRim[i], &(adc2Samples[i]),&(adc2Samples[i+7]),&(adc2Samples[i+14]),&(adc2Samples[i+21]));
		else triggerchannel_init(&triggerChannelsRim[i], &(adc3Samples[i-7]),&(adc3Samples[i-7+3]),&(adc3Samples[i-7+6]),&(adc3Samples[i-7+9]));
	}

	triggerChannelsRim[0].potiCsPort = GPIOC;
	triggerChannelsRim[0].potiCsPin = GPIO_PIN_11;
	triggerChannelsRim[0].midinote = 36;

	triggerChannelsRim[1].potiCsPort = GPIOC;
	triggerChannelsRim[1].potiCsPin = GPIO_PIN_12;
	triggerChannelsRim[1].midinote = 38;

	triggerChannelsRim[2].potiCsPort = GPIOD;
	triggerChannelsRim[2].potiCsPin = GPIO_PIN_0;
	triggerChannelsRim[2].midinote = 38;

	triggerChannelsRim[3].potiCsPort = GPIOD;
	triggerChannelsRim[3].potiCsPin = GPIO_PIN_1;
	triggerChannelsRim[3].midinote = 38;

	triggerChannelsRim[4].potiCsPort = GPIOD;
	triggerChannelsRim[4].potiCsPin = GPIO_PIN_2;
	triggerChannelsRim[4].midinote = 38;

	triggerChannelsRim[5].potiCsPort = GPIOD;
	triggerChannelsRim[5].potiCsPin = GPIO_PIN_3;
	triggerChannelsRim[5].midinote = 38;

	triggerChannelsRim[6].potiCsPort = GPIOD;
	triggerChannelsRim[6].potiCsPin = GPIO_PIN_4;
	triggerChannelsRim[6].midinote = 38;

	triggerChannelsRim[7].potiCsPort = GPIOD;
	triggerChannelsRim[7].potiCsPin = GPIO_PIN_5;
	triggerChannelsRim[7].midinote = 38;

	triggerChannelsRim[8].potiCsPort = GPIOD;
	triggerChannelsRim[8].potiCsPin = GPIO_PIN_6;
	triggerChannelsRim[8].midinote = 38;

	triggerChannelsRim[9].potiCsPort = GPIOD;
	triggerChannelsRim[9].potiCsPin = GPIO_PIN_7;
	triggerChannelsRim[9].midinote = 38;


	for(i = 0; i < rimChannelCount; i++){

		HAL_GPIO_WritePin(triggerChannelsRim[i].potiCsPort,triggerChannelsRim[i].potiCsPin,GPIO_PIN_SET);

	}

	HAL_Delay(10);

	triggerchannel_setWiper(&triggerChannelsRim[0], 50);
	triggerchannel_setWiper(&triggerChannelsRim[1], 50);
	triggerchannel_setWiper(&triggerChannelsRim[2], 50);
	triggerchannel_setWiper(&triggerChannelsRim[3], 50);
	triggerchannel_setWiper(&triggerChannelsRim[4], 50);
	triggerchannel_setWiper(&triggerChannelsRim[5], 50);
	triggerchannel_setWiper(&triggerChannelsRim[6], 50);
	triggerchannel_setWiper(&triggerChannelsRim[7], 50);
	triggerchannel_setWiper(&triggerChannelsRim[8], 50);
	triggerchannel_setWiper(&triggerChannelsRim[9], 50);

}

void initHiHatChannels(){
		hihatchannel_init(&hiHatChannels[0], &adc4Samples[0], &triggerChannelsHead [0]);
		hihatchannel_init(&hiHatChannels[1], &adc4Samples[1], &triggerChannelsRim [9]);

}


//
//int getHiHatResistance(HiHatChannel *chan){
//	int16_t adcValue = -1; // -1 is Errorvalue
//
//	// setup adc for channel
//	HAL_ADC_ConfigChannel(chan->adcHandle, &chan->adcSConfig);
//
//
//	HAL_ADC_Start(chan->adcHandle);									//start sampling
//	HAL_ADC_PollForConversion(chan->adcHandle, 100);					//wait for conversion
//
//	if(HAL_ADC_GetState(chan->adcHandle) == HAL_ADC_STATE_EOC_REG){ 	//check ADC error state
//		adcValue = HAL_ADC_GetValue(chan->adcHandle);
//
//		HAL_ADC_Stop(chan->adcHandle);
//	}
//
//	int resistance = 10000/( (3.3 * 4096) / (adcValue * 3.3) -1 );
//
//	return resistance;
//}



volatile int btnDebounceIRQTimer[3] = {0,0,0};
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_10) btnDebounceIRQTimer[0] = 1;	// Btn 0 clicked
	if(GPIO_Pin == GPIO_PIN_11) btnDebounceIRQTimer[1] = 1;	// Btn 1 clicked
	if(GPIO_Pin == GPIO_PIN_12) btnDebounceIRQTimer[2] = 1;	// Btn 2 clicked
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart){
	if(midiMessagebuffer != 0 && midiMessagebuffer->uarthandle == huart) uartmessagebuffer_onTxComplete(midiMessagebuffer);
}
void HAL_UART_ErrorCallback (UART_HandleTypeDef *huart){
	if(midiMessagebuffer != 0 && midiMessagebuffer->uarthandle == huart) uartmessagebuffer_onTxComplete(midiMessagebuffer);

}



/*
 * Triggered every 200us
 *
 * Processing triggerchannel samples taken
 */
void onTimer1Triggered(){
	int i;
	//triggerchannel_process(&triggerChannelsHead[0]);

	for(i = 0; i < headChannelCount; i++){
		triggerchannel_process(&triggerChannelsHead[i]);
	}

	for(i = 0; i < rimChannelCount; i++){
		triggerchannel_process(&triggerChannelsRim[i]);
	}

}
/*
 * Triggered every 1ms
 *
 * processing hihat-samples taken
 */
void onTimer2Triggered(){
	int i;

	for(i = 0; i < hihatChannelCount; i++){
		hihatchannel_process(&hiHatChannels[i]);
	}

}

/*
 * Triggered on btn0 click with debounce delay
 */
void onBtn0Triggered(){
	midi_sendNote(10,36,120);
}

/*
 * Triggered on btn1 click with debounce delay
 */
void onBtn1Triggered(){

		triggerChannelsHead[0].midinote = 38;

}

/*
 * Triggered on btn2 click with debounce delay
 */
void onBtn2Triggered(){


}




/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  //initializes the datastructure and amplifiing of the triggerchannels
  midi_init();
  voltdrum_init();
  initHeadChannels();
  initRimChannels();
  initHiHatChannels();

  HAL_UART_Transmit_IT(&huart1, "init...",7);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {

	  if(irqflag_timer1){
			irqflag_timer1 = 0;
			onTimer1Triggered();
	  } else if(irqflag_timer2){
		  irqflag_timer2 = 0;
		  onTimer2Triggered();
	  } else if(irqflag_btns[0]){
		  irqflag_btns[0] = 0;
		  onBtn0Triggered();
	  } else if(irqflag_btns[1]){
		  irqflag_btns[1] = 0;
		  onBtn1Triggered();
	  } else if(irqflag_btns[2]){
		  irqflag_btns[2] = 0;
		  onBtn2Triggered();
	  }

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_RCC_EnableCSS();

  __SYSCFG_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
