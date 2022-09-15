/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
#include "button.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct
{
	uint8_t leftPlay   : 1;
	uint8_t leftPause  : 1;
	uint8_t leftCue    : 1;
	uint8_t leftStop   : 1;
	uint8_t leftHold   : 1;
	uint8_t rightPlay  : 1;
	uint8_t rightPause : 1;
	uint8_t rightCue   : 1;
	uint8_t rightStop  : 1;
	uint8_t rightHold  : 1;
} Buttons_t;

typedef struct
{
	Buttons_t  buttons;
	uint8_t    leftPitch;
	uint8_t    rightPitch;
	uint8_t    leftVolume;
	uint8_t    rightVolume;
	uint8_t    crossFader;
	uint8_t    leftJog;
	uint8_t    rightJog;
} UsbHidVdjController_t;

GPIO_PinState buttonB1State;
GPIO_PinState buttonOut1State;
GPIO_PinState buttonOut2State;
GPIO_PinState buttonOut3State;
GPIO_PinState buttonOut4State;

UsbHidVdjController_t VdjCtrlReport;
extern USBD_HandleTypeDef hUsbDeviceFS;

ButtonHandler_t ButtonPlayPause =
{
	.ActiveState = GPIO_PIN_SET,
	.DebounceCtr = 0u,
	.DebounceOff = 20u,
	.DebounceOn  = 20u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};

ButtonHandler_t ButtonCue =
{
	.ActiveState = GPIO_PIN_SET,
	.DebounceCtr = 0u,
	.DebounceOff = 20u,
	.DebounceOn  = 20u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};

ButtonHandler_t ButtonSearchLeft =
{
	.ActiveState = GPIO_PIN_SET,
	.DebounceCtr = 0u,
	.DebounceOff = 20u,
	.DebounceOn  = 20u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};

ButtonHandler_t ButtonSearchRight =
{
	.ActiveState = GPIO_PIN_SET,
	.DebounceCtr = 0u,
	.DebounceOff = 20u,
	.DebounceOn  = 20u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};

ButtonHandler_t ButtonHold =
{
	.ActiveState = GPIO_PIN_SET,
	.DebounceCtr = 0u,
	.DebounceOff = 20u,
	.DebounceOn  = 20u,
	.IsPressed   = false,
	.WasPressed  = false,
	.Padding     = 0u
};

ButtonHandler_t ButtonEject =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

ButtonHandler_t ButtonTrackBackward =
{
  .ActiveState = GPIO_PIN_SET,
  .DebounceCtr = 0u,
  .DebounceOff = 20u,
  .DebounceOn  = 20u,
  .IsPressed   = false,
  .WasPressed  = false,
  .Padding     = 0u
};

volatile uint16_t JogCntr = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void SegmentDisplayDriver(uint16_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);

  // Init
  VdjCtrlReport.buttons.leftPause = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Read buttons on S1: HOLD, TRKB and PLAY
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_SET );
    if( HAL_GPIO_ReadPin(KD0_GPIO_Port, KD0_Pin) == ButtonHold.ActiveState )
    {
      if( ButtonHold.IsPressed == false )
      {
        ButtonHold.IsPressed = true;
        if(VdjCtrlReport.buttons.leftHold == true)
        {
          VdjCtrlReport.buttons.leftHold = false;
        }
        else
        {
          VdjCtrlReport.buttons.leftHold = true;
        }
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
      }
    }
    else
    {
      if( ButtonHold.IsPressed == true )
      {
        ButtonHold.IsPressed = false;
      }
    }
    if( HAL_GPIO_ReadPin(KD1_GPIO_Port, KD1_Pin) == ButtonTrackBackward.ActiveState )
    {
      if( ButtonTrackBackward.IsPressed == false )
      {
        ButtonTrackBackward.IsPressed = true;
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
      }
    }
    else
    {
      if( ButtonTrackBackward.IsPressed == true )
      {
        ButtonTrackBackward.IsPressed = false;
      }
    }
    if( HAL_GPIO_ReadPin(KD2_GPIO_Port, KD2_Pin) == ButtonPlayPause.ActiveState )
    {
      if( ButtonPlayPause.IsPressed == false )
      {
        ButtonPlayPause.IsPressed = true;
        if(VdjCtrlReport.buttons.leftPlay == true)
        {
          VdjCtrlReport.buttons.leftPlay = false;
          VdjCtrlReport.buttons.leftPause = true;
        }
        else
        {
          VdjCtrlReport.buttons.leftPlay = true;
          VdjCtrlReport.buttons.leftPause = false;
        }
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
      }
    }
    else
    {
      if( ButtonPlayPause.IsPressed == true )
      {
        ButtonPlayPause.IsPressed = false;
      }
    }
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_RESET );
#if 1
    JogCntr = TIM4->CNT;
    // Jog value changed?
    if( VdjCtrlReport.leftJog != JogCntr )
    {
      VdjCtrlReport.leftJog = JogCntr;
      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&VdjCtrlReport, sizeof(VdjCtrlReport));
    }
    HAL_Delay(1u);
#else
    HAL_Delay(500u);
    SegmentDisplayDriver(0x0FFFu);
    HAL_Delay(500u);
#endif
    //SegmentDisplayDriver(0x0000u);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 47;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0xf;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0xf;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, S1_Pin|S2_Pin|S3_Pin|S4_Pin
                          |S5_Pin|S6_Pin|S7_Pin|S8_Pin
                          |S9_Pin|S10_Pin|S11_Pin|S12_Pin
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, G1_Pin|G2_Pin|G3_Pin|G9_Pin
                          |G10_Pin|G11_Pin|G4_Pin|G5_Pin
                          |G6_Pin|G7_Pin|G8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_Pin S2_Pin S3_Pin S4_Pin
                           S5_Pin S6_Pin S7_Pin S8_Pin
                           S9_Pin S10_Pin S11_Pin S12_Pin
                           PC12 */
  GPIO_InitStruct.Pin = S1_Pin|S2_Pin|S3_Pin|S4_Pin
                          |S5_Pin|S6_Pin|S7_Pin|S8_Pin
                          |S9_Pin|S10_Pin|S11_Pin|S12_Pin
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : G1_Pin G2_Pin G3_Pin G9_Pin
                           G10_Pin G11_Pin G4_Pin G5_Pin
                           G6_Pin G7_Pin G8_Pin */
  GPIO_InitStruct.Pin = G1_Pin|G2_Pin|G3_Pin|G9_Pin
                          |G10_Pin|G11_Pin|G4_Pin|G5_Pin
                          |G6_Pin|G7_Pin|G8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KD0_Pin KD1_Pin KD2_Pin */
  GPIO_InitStruct.Pin = KD0_Pin|KD1_Pin|KD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void SegmentDisplayDriver(uint16_t value)
{
  // segments connected to PortC 0-11
  uint32_t tempU32 = (uint32_t)value;
  // set the 1 segments
  GPIOC->BSRR = tempU32 & 0x00000FFFuL;
  // clear the 0 segments
  tempU32 = (~tempU32) & 0x00000FFFuL;
  tempU32 <<= 16u;
  //GPIOC->BSRR = tempU32;
  // Switch on the segment
  //GPIOB->BSRR = 0x0000373FuL;
  GPIOB->BSRR = 0x373F0000uL;
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
