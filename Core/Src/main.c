/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "LoRa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define HIGH 1
#define LOW 0
#define size_of_buffer 8
#define BITLENGTH 8 //1 Byte
//#define FIRSTBIT pow(2,(BITLENGTH-1))//MSB Value
#define FIRSTBIT (1 << (BITLENGTH - 1))
#define TRUE 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
SPI_HandleTypeDef hspi3;
/* USER CODE BEGIN PV */
LoRa myLoRa;
uint8_t read_data[128];
uint8_t send_data[128];
int			RSSI;
/* USER CODE BEGIN PV */
uint16_t v_R1=0;
uint16_t v_R2=0;
uint16_t v_R3=0;
uint16_t v_R=0;
uint16_t rawValues[3];
uint8_t token=0;
uint16_t Th_voltages[8];
uint16_t Th_voltage=0;
uint16_t Delay=500;
uint8_t CT_ID_EGC=0;
uint8_t CT_ID_MLC=0;
uint8_t CT_ID_SC=0;
uint8_t CT_ID=0;
uint8_t RecPacket=0;
uint16_t Avg=0;
uint8_t countB1=0;
uint8_t countB0=0;
int input=0;
int cntR1_1=0,cntR2_1=0,cntR3_1=0;
int cntR1_0=0,cntR2_0=0,cntR3_0=0;
int BestRx=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void DiversityGainEngineInit();
uint16_t ThresholdVoltageEstimator(uint16_t v1, uint16_t v2, uint16_t v3);
void CombiningTEchniqueSelection();
void LoRa_Module_Setting();
void LoRa_Transceiver_Init();
uint8_t EGC_Engine(uint16_t voltage_R1, uint16_t voltage_R2, uint16_t voltage_R3,uint8_t tkn,uint8_t num);
uint8_t MLC_Engine(uint16_t voltage_R1, uint16_t voltage_R2, uint16_t voltage_R3,uint8_t tkn,uint8_t num);
uint8_t SC_Engine(uint16_t voltage_R1, uint16_t voltage_R2, uint16_t voltage_R3,uint8_t num);
uint8_t SC_EngineReception(uint16_t voltage,uint8_t num);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t convCompleted=0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	convCompleted=1;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  LoRa_Module_Setting();
  /* USER CODE BEGIN 2 */
   HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  DiversityGainEngineInit();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DiversityGainEngineInit()
{
	for(int j=0;j<BITLENGTH;j++)
	{
	while(!convCompleted);

	for(uint8_t i=0; i<hadc1.Init.NbrOfConversion;i++){
		 v_R1=rawValues[0];
		 v_R2=rawValues[1];
		v_R3=rawValues[2];
	}
	Th_voltages[j]=ThresholdVoltageEstimator(v_R1,v_R2,v_R3);
	HAL_Delay(Delay);
	}
	Th_voltage=(Th_voltages[0]+Th_voltages[1]+Th_voltages[2]+Th_voltages[3]+Th_voltages[4]+Th_voltages[5]+Th_voltages[6]+Th_voltages[7])/8;
	CombiningTEchniqueSelection();
}

uint16_t  ThresholdVoltageEstimator (uint16_t v1, uint16_t v2, uint16_t v3)
{
	if (v1>v2&&v1>v3){return v1;}
	else if(v2>v1&&v2>v3){return v2;}
	else{return v3;}
}

void CombiningTEchniqueSelection()
{
	for(uint8_t j=0;j<BITLENGTH;j++)
		{
		while(!convCompleted);

		for(uint8_t i=0; i<hadc1.Init.NbrOfConversion;i++){
			 v_R1=rawValues[0];
			 v_R2=rawValues[1];
			v_R3=rawValues[2];
		}
		CT_ID_EGC=EGC_Engine(v_R1,v_R2,v_R3,HIGH,j);
		CT_ID_MLC=MLC_Engine(v_R1,v_R2,v_R3,HIGH,j);
		CT_ID_SC=SC_Engine(v_R1,v_R2,v_R3,j);
		HAL_Delay(Delay);
		convCompleted = 0; // Reset flag for next conversion
		}
	if (CT_ID_EGC > CT_ID_MLC && CT_ID_EGC > CT_ID_SC)
	{
	    for (uint8_t j = 0; j < BITLENGTH; j++)
	    {
	        while (!convCompleted);

	        for (uint8_t i = 0; i < hadc1.Init.NbrOfConversion; i++)
	        {
	            v_R1 = rawValues[0];
	            v_R2 = rawValues[1];
	            v_R3 = rawValues[2];
	        }

	        // Assuming RecPacket is a single byte and that EGC_Engine returns a single byte
	        uint8_t RecPacket = EGC_Engine(v_R1, v_R2, v_R3, LOW,j);
	        HAL_UART_Transmit(&huart1, &RecPacket, 1, 100);  // Transmit one byte

	        convCompleted = 0;  // Reset the flag for the next conversion
	    }
	}
	else if (CT_ID_MLC > CT_ID_EGC && CT_ID_MLC > CT_ID_SC)
	{
	    // Perform the necessary actions for this condition
		for (uint8_t j = 0; j < BITLENGTH; j++)
			    {
			        while (!convCompleted);

			        for (uint8_t i = 0; i < hadc1.Init.NbrOfConversion; i++)
			        {
			            v_R1 = rawValues[0];
			            v_R2 = rawValues[1];
			            v_R3 = rawValues[2];
			        }

			        // Assuming RecPacket is a single byte and that EGC_Engine returns a single byte
			        uint8_t RecPacket = MLC_Engine(v_R1, v_R2, v_R3, LOW,j);
			        HAL_UART_Transmit(&huart1, &RecPacket, 1, 100);  // Transmit one byte

			        convCompleted = 0;  // Reset the flag for the next conversion
			    }
	}
	else if (CT_ID_SC > CT_ID_EGC && CT_ID_SC > CT_ID_MLC)
	{
	    // Perform the necessary actions for this condition
		for (uint8_t j = 0; j < BITLENGTH; j++)
			    {
			        while (!convCompleted);

			        for (uint8_t i = 0; i < hadc1.Init.NbrOfConversion; i++)
			        {
			            if(BestRx==0)v_R = rawValues[0];
			            else if(BestRx==1)v_R = rawValues[1];
			            else if(BestRx==2)v_R = rawValues[2];
			        }

			        // Assuming RecPacket is a single byte and that EGC_Engine returns a single byte
			        uint8_t RecPacket = SC_EngineReception(v_R,j);
			        HAL_UART_Transmit(&huart1, &RecPacket, 1, 100);  // Transmit one byte

			        convCompleted = 0;  // Reset the flag for the next conversion
			    }
	}

	// The function should return some value, adjust as needed
	//return 0;
}
uint8_t EGC_Engine(uint16_t voltage_R1, uint16_t voltage_R2, uint16_t voltage_R3,uint8_t tkn,uint8_t num)
{

	Avg =(voltage_R1+voltage_R2+voltage_R3)/3;
	if(tkn==HIGH){
	    if(Avg>=Th_voltage){countB1++;if(num==7){return countB1;}}
	    else{countB0++;}
	}
		  else if (tkn==LOW){
			  if(Avg>=Th_voltage)
			      {input++;
			      //Binary shift to store another bit
			      input=input<<1;
			      }
			  	    if(num==7){input=input>>1;
			  	  return input;}
		  }
	 return 0; // Default return value
}
uint8_t MLC_Engine(uint16_t voltage_R1, uint16_t voltage_R2, uint16_t voltage_R3,uint8_t tkn,uint8_t num)
{
	int cnt0=0,cnt1=0;
	if(voltage_R1>=Th_voltage){cnt1++;}else{cnt0++;}
	if(voltage_R2>=Th_voltage){cnt1++;}else{cnt0++;}
	if(voltage_R3>=Th_voltage){cnt1++;}else{cnt0++;}
	if(tkn==HIGH){
	    if(cnt1>cnt0){countB1++;if(num==7){return countB1;countB1=0;}}
	    else{countB0++;}
	}
		  else if (tkn==LOW){
			  if(cnt1>=cnt0)
			      {input++;
			      //Binary shift to store another bit
			      input=input<<1;
			      }
			  	    if(num==7){input=input>>1;
			  	  return input;input=0;}
		  }
	 return 0; // Default return value
}
uint8_t SC_Engine(uint16_t voltage_R1, uint16_t voltage_R2, uint16_t voltage_R3,uint8_t num)
{

		if(voltage_R1>=Th_voltage){cntR1_1++;}else{cntR1_0++;}
			if(voltage_R2>=Th_voltage){cntR2_1++;}else{cntR2_0++;}
			if(voltage_R3>=Th_voltage){cntR3_1++;}else{cntR3_0++;}
			if(num==7)
			        {if(cntR1_1>=cntR2_1 && cntR1_1>=cntR3_1){return cntR1_1;BestRx=0;cntR1_1=0;cntR2_1=0;cntR3_1=0;}
			           if(cntR2_1>=cntR1_1 && cntR2_1>=cntR3_1){return cntR2_1;BestRx=1;cntR1_1=0;cntR2_1=0;cntR3_1=0;}
			           if(cntR3_1>=cntR2_1 && cntR3_1>=cntR1_1){return cntR3_1;BestRx=2;cntR1_1=0;cntR2_1=0;cntR3_1=0;}
			          }
			return cntR1_1;
}
uint8_t SC_EngineReception(uint16_t voltage,uint8_t num)
{
	if(voltage> Th_voltage)
	{input++;
    //Binary shift to store another bit
    input=input<<1;
    }
	    if(num==7){input=input>>1;
	  return input;input=0;}
	    return 0; // Default return statement in case none of the conditions are met
}

void LoRa_Module_Setting()
{
	// MODULE SETTINGS ----------------------------------------------
		myLoRa = newLoRa();

		myLoRa.hSPIx                 = &hspi3;
	//	myLoRa.CS_port               = NSS_GPIO_Port;
		myLoRa.CS_pin                = NSS_Pin;
	//	myLoRa.reset_port            = RESET_GPIO_Port;
		myLoRa.reset_pin             = RESET_Pin;
	//	myLoRa.DIO0_port						 = DIO0_GPIO_Port;
		myLoRa.DIO0_pin							 = DIO0_Pin;

		myLoRa.frequency             = 433;							  // default = 433 MHz
		myLoRa.spredingFactor        = SF_7;							// default = SF_7
		myLoRa.bandWidth			       = BW_125KHz;				  // default = BW_125KHz
		myLoRa.crcRate				       = CR_4_5;						// default = CR_4_5
		myLoRa.power					       = POWER_20db;				// default = 20db
		myLoRa.overCurrentProtection = 120; 							// default = 100 mA
		myLoRa.preamble				       = 10;		  					// default = 8;

		LoRa_reset(&myLoRa);
		LoRa_init(&myLoRa);

		// START CONTINUOUS RECEIVING -----------------------------------
		LoRa_startReceiving(&myLoRa);
		//---------------------------------------------------------------

}

void LoRa_Transceiver_Init()
{
	// SENDING DATA - - - - - - - - - - - - - - - - - - - - - - - - -
			send_data[0] = 0x3B; // MY ADDRESS
			for(int i=0; i<26; i++)
				send_data[i+1] = 48+i;
			LoRa_transmit(&myLoRa, send_data, 128, 500);
			HAL_Delay(1500);

			// RECEIVING DATA - - - - - - - - - - - - - - - - - - - - - - - -
			LoRa_receive(&myLoRa, read_data, 128);
	}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
