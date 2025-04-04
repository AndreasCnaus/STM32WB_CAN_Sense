/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <logger.h>
#include <bme280.h>
#include <mcp2515.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct can_frame_data {
    uint16_t sid;                   // Standard identifier
    uint8_t dlc;                    // Data length code
    uint8_t data[MCP2515_MAXDL];    // Data buffer
};

// Structure for SID_String mapping
struct sid_string_mapping {
	uint16_t sid;		// Standard identifier of the CAN Message
	const char *name;	// Corresponding string
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// SIDs for BME280 sensor values transmitted via MCP2515
#define BME280_TEMPERATURE_SID    25
#define BME280_HUMIDITY_SID       26
#define BME280_PRESSURE_SID       27

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
volatile uint8_t tim16_flag = 0; // Timer flag to signal time events; declared volatile to prevent compiler optimization.
BME280 bme280;					 // BME280 instance

// SID Mappings
const struct sid_string_mapping sid_map[] = {
    {BME280_TEMPERATURE_SID, "Temperature"},
    {BME280_PRESSURE_SID, "Pressure"},
    {BME280_HUMIDITY_SID, "Humidity"}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch);	// Redirect printf to ITM (Instrumentation Trace Macrocell)
struct can_frame_data create_can_frame(uint16_t sid, void *data, uint8_t dlc);
uint8_t wait_for_tx2_buffer(uint8_t retries, uint16_t delay_ms);
uint32_t get_timestamp_milliseconds();

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  uint8_t error_count = 0;
  u8 opmode = 0x00;
  HAL_StatusTypeDef status = HAL_OK;

  // Configure the BME280 Sensor chip
  BME280_cmd_res cmd_res = BME280_init(&bme280, &hi2c1);
  if (cmd_res != BME280_CMD_OK) {
	  LOG_ERROR("Failed to initialize BME280 hardware (status: %d)", cmd_res);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);	// indicate error with blue LED on PA10
	  error_count++;
  }

  // Disable external interrupt line (EXTI3) connected to MCP2515 CAN controller
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);

  // Configure the MCP2515 hardware
  status = mcp2515_reset_hw(&hspi1);
  if (status != HAL_OK) {
	  LOG_ERROR("Failed to reset MCP2515 hardware (status: %d)", status);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);	// indicate error with blue LED on PA10
	  error_count++;
  }

  status = mcp2515_config_hw(&hspi1);
  if (status != HAL_OK) {
	  LOG_ERROR("Failed to configure MCP2515 hardware (status: %d)", status);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);	// indicate error with blue LED on PA10
	  error_count++;
  }

  status = mcp2515_set_opmode(&hspi1, MCP2515_NORMAL_MODE);
  if (status != HAL_OK) {
	  LOG_ERROR("Failed to set MCP2515 into loop-back operation mode (status: %d)", status);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);	// indicate error with blue LED on PA10
	  error_count++;
  }

  status = mcp2515_get_opmode(&hspi1, &opmode);
  if (status != HAL_OK) {
	  LOG_ERROR("Failed to read current MCP2515 operation mode (status: %d)", status);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);	// indicate error with blue LED on PA10
	  error_count++;
  }
  else {
	  LOG_INFO("Device is in %s", get_opmode_string(opmode));
  }

  // Enable external interrupt line (EXTI3) connected to MCP2515 CAN controller
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  // Start the timer16
  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  struct can_frame_data can_frames[3];

  while ((status == HAL_OK) && (error_count == 0))
  {
	  if (tim16_flag) {

		  //uint32_t timestamp = get_timestamp_milliseconds();
		  //LOG_INFO("Timestamp: %lu ms", timestamp);

		  // Disable the update interrupt for TIM16
		  __HAL_TIM_DISABLE_IT(&htim16, TIM_IT_UPDATE);

		  // Read environmental (temperature, humidity, pressure) data from the sensor
		  BME280_cmd_res cmd_res = BME280_read_env_data(&bme280);
		  if (cmd_res != BME280_CMD_OK) {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Indicate error with blue LED on PA10
			  error_count++;
		  }

		  // Convert integer values to float
		  float temp_deg = (float)bme280.T / (float)kT;  // Temperature in degrees
		  float hum_prh = (float)bme280.H / (float)kH;   // Relative Humidity in percent
		  float pres_pa = (float)bme280.P / (float)kP;  // Pressure in pascals

		  // Log the values
		  LOG_INFO("Temperature: %.1f °C", temp_deg);  // One decimal precision
		  LOG_INFO("Humidity: %.1f %%", hum_prh);      // One decimal precision
		  LOG_INFO("Pressure: %.1f Pa", pres_pa);     // One decimal precision

		  // Create CAN frame
		  can_frames[0] = create_can_frame(BME280_TEMPERATURE_SID, &temp_deg, sizeof(temp_deg));
		  can_frames[1] = create_can_frame(BME280_HUMIDITY_SID, &hum_prh, sizeof(hum_prh));
		  can_frames[2] = create_can_frame(BME280_PRESSURE_SID, &pres_pa, sizeof(pres_pa));

		  // Send CAN fames
		  for (int i = 0; i < sizeof(can_frames)/sizeof(can_frames[0]); i++) {
			  uint8_t retries = 3;
			  uint16_t delay_ms = 10;
			  struct can_frame_data frame = can_frames[i];

			  // Wait until the TX buffer is ready to accept new data
			  if (!wait_for_tx2_buffer(retries, delay_ms)) {
				  LOG_ERROR("Transmission buffer 2 not ready after %d retries", retries);
			      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Indicate error with blue LED on PA10
			      error_count++;
			      continue; // Skip the current frame
			  }

			  // Send the CAN frame
			  status = mcp2515_write_can_frame(&hspi1, frame.sid, frame.data, frame.dlc);
			  if (status != HAL_OK) {
				  LOG_ERROR("Failed to write %s CAN frame (status: %d)", sid_map[frame.sid], status);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Indicate error with blue LED on PA10
				  error_count++;
			  }
		  }

		  tim16_flag = 0; // Reset the timer flag

		  // Re-enable the update interrupt for TIM16
		  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  LOG_ERROR("Error count: %u", error_count);
  LOG_ERROR("Exiting...");
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1024-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 62500-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin|ERROR_INDICATOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MCP2515_INT_Pin */
  GPIO_InitStruct.Pin = MCP2515_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCP2515_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_NSS_Pin ERROR_INDICATOR_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin|ERROR_INDICATOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Timer16 callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check which version of the timer triggered this callback
	if (htim == &htim16 )
	{
		tim16_flag = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == MCP2515_INT_Pin) {

		HAL_StatusTypeDef status = HAL_OK;
		u8 canintf = 0x00;

		// Read MCP2515 interrupt flag register
		status = mcp2515_read_reg(&hspi1, MCP2515_CANINTF, &canintf);
		if (status != HAL_OK) {
			LOG_ERROR("Failed to read interrupt flag register (status: %d)", status);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);	// Indicate error with LED on PA10
			return;
		}

		// Process the interrupts
		for (int i = 0; i < IRQ_TABLE_SIZE; i++) {
			if (canintf & irq_table[i].flag) {
				status = irq_table[i].handler(&hspi1);
				if (status != HAL_OK) {
					LOG_ERROR("Failed to handle interrupt (flag: 0x%X, index: %d, status: %d)", irq_table[i].flag, i, status);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);	// Indicate error with LED on PA10
				}
			}
		}
	}
}

// Redirect printf to ITM (Instrumentation Trace Macrocell)
int __io_putchar(int ch) {
    ITM_SendChar(ch); // Send character to SWO
    return ch;
}

struct can_frame_data create_can_frame(uint16_t sid, void *data,  uint8_t dlc)
{
	 if (dlc > MCP2515_MAXDL) {
		 dlc = MCP2515_MAXDL; // Cap the length to avoid buffer overflows
	 }

	 struct can_frame_data frame;
	 frame.sid = sid;
	 frame.dlc = dlc;
	 memcpy(frame.data, (uint8_t *)data, frame.dlc);

	 return frame;
}

uint8_t wait_for_tx2_buffer(uint8_t retries, uint16_t delay_ms)
{
	while (retries-- > 0) {
		if (is_tx2_buf_ready()) {
			return 1;	 // TX buffer 2 is ready
		}
		HAL_Delay(delay_ms);	// Wait for buffer
	}

	return 0;	// // TX buffer 2 not ready after retries
}


uint32_t get_timestamp_milliseconds() {
    // Use HAL_GetTick() to get the system uptime in milliseconds
    uint32_t milliseconds = HAL_GetTick();
    return milliseconds;
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
