/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdbool.h>
#include <arm_math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CS_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BUFFER_SIZE 256

// Extracted features
float sensor_features[21];

// CMSIS DSP FFT Instance
arm_rfft_fast_instance_f32 fft_instance;

// Buffers for the DSP pipeline (using X-axis as our primary example)
float fft_input_buffer[BUFFER_SIZE];
float fft_output_buffer[BUFFER_SIZE];
float fft_magnitude_buffer[BUFFER_SIZE/2];


// Dual ping-pong buffers for X, Y, Z vibration data
float buffer_a[BUFFER_SIZE][4];
float buffer_b[BUFFER_SIZE][4];

volatile float(*write_ptr)[4] = buffer_a;  // ISR writes here
float (*processing_ptr)[4] = NULL;         // Main loop reads here

volatile uint16_t sample_index = 0;
volatile bool data_ready_flag = false;
volatile bool processing_buffer_full = false;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char* ptr, int len){
	//Transmit the string over USART2 in blocking mode
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

// ADXL345 Registers
#define ADXL345_DEVID_REG 0x00
#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_INT_ENABLE  0x2E
#define ADXL345_INT_MAP     0x2F
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32

//Function to read a single register from adxl345
uint8_t ADXL_readRegister(uint8_t regAddr){
	uint8_t txData[2];
	uint8_t rxData[2];

	//ADXL Rquires bit 7 to be 1 for read operation
	txData[0] = regAddr | 0x80;
	txData[1] = 0x00;         // dummy byte to push further clock cycles

	CS_LOW();

	HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, HAL_MAX_DELAY);

	CS_HIGH();

	// Sensor's response comes in the second byte.
	return rxData[1];
}

void ADXL_writeRegister(uint8_t regAddr, uint8_t value){
	uint8_t txData[2] = {regAddr & 0x7F, value};          // Clear bit 7 for write operation

	CS_LOW();
	HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
	CS_HIGH();

}

void ADXL_init(){
	ADXL_writeRegister(ADXL345_DATA_FORMAT, 0x0B); // Full resolution mode, +/-16g range
	ADXL_writeRegister(ADXL345_BW_RATE, 0x0D);     // Set Output Data Rate (ODR) to 800 Hz
    ADXL_writeRegister(ADXL345_INT_MAP, 0x00);     // Route DATA_READY interrupt to INT1 pin

    // Crucial : Switch to measurement mode
    ADXL_writeRegister(ADXL345_POWER_CTL, 0x08);   // Switch to Measurement Mode
    ADXL_writeRegister(ADXL345_INT_ENABLE, 0x80);  // Enable the DATA_READY interrupt

    // Crucial Step 2: Clear/Flush any pending boot-up interrupts
	uint8_t dummyTx[7] = { (ADXL345_DATAX0 | 0x80 | 0x40), 0, 0, 0, 0, 0, 0 };
	uint8_t dummyRx[7] = {0};

	CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, dummyTx, dummyRx, 7, HAL_MAX_DELAY);
	CS_HIGH();

}

void extractFeatures(uint8_t axis_idx, float(*raw_buffer)[4], float *output_array){

	float sum = 0.0f;
	float mean = 0.0f;

	// Zero-Centering the data
	for(int i=0; i<BUFFER_SIZE; i++){
		sum += processing_ptr[i][axis_idx];
	}

	mean = sum / (float) BUFFER_SIZE;

	//Subtracting the mean from every sample to eliminate gravity pull
	for(int i=0; i<BUFFER_SIZE; i++){
		fft_input_buffer[i] = processing_ptr[i][axis_idx] - mean;
	}

	// Hardware accelerated FFT & Magnitude calculation
	arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0);
	arm_cmplx_mag_f32(fft_output_buffer, fft_magnitude_buffer, BUFFER_SIZE/2);

	// Time-Domain feature extracton
	float max_val = 0.0f;
	float min_val = 0.0f;
	float rms = 0.0f;

	uint32_t max_idx, min_idx;

	// Calculating RMS
	arm_rms_f32(fft_input_buffer, BUFFER_SIZE, &rms);

	// Calculating peak to peak
	arm_max_f32(fft_input_buffer, BUFFER_SIZE, &max_val, &max_idx);
	arm_min_f32(fft_input_buffer, BUFFER_SIZE, &min_val, &min_idx);

	float peak_to_peak = max_val - min_val;
	float crest_factor = (rms > 0.0001f) ? (max_val / rms) : 0.0f;

	// Frequency Domain Feature Extraction
	float low_freq = 0.0f;
	float mid_freq = 0.0f;
	float high_freq = 0.0f;

	// Bin 0 is DC Offset
	// Bin 1-10 is low freq
	for(int i=1; i<=10; i++){
		low_freq += fft_magnitude_buffer[i];
	}

	// Bin 11-40 is mid frequency
	for(int i=11; i<=40; i++){
		mid_freq += fft_magnitude_buffer[i];
	}

	// Bin 41-127 is high frequency
	for(int i=41; i<=127; i++){
		high_freq += fft_magnitude_buffer[i];
	}

	// Pack this into feature array
	uint8_t offset = axis_idx * 6;
	output_array[offset + 0] = rms;
	output_array[offset + 1] = peak_to_peak;
	output_array[offset + 2] = crest_factor;
	output_array[offset + 3] = low_freq;
    output_array[offset + 4] = mid_freq;
	output_array[offset + 5] = high_freq;

}

void extractCurrentFeatures(float (*raw_buffer)[4], float *output_array){
	float sum = 0.0f;
	float max_val = 0.0f;
	float min_val = 3.3f;

	float current_1d_buffer[BUFFER_SIZE];

	for (int i = 0; i < BUFFER_SIZE; i++) {
	    float val = raw_buffer[i][3]; // Index 3 is the current data
	    sum += val;
	    current_1d_buffer[i] = val;

	    if (val > max_val) max_val = val;
	    if (val < min_val) min_val = val;
	}

	// 1. Calculate Mean (Average Voltage)
	float mean_voltage = sum / (float)BUFFER_SIZE;

	// 2. Calculate RMS Voltage using DSP hardware
	float rms_voltage = 0.0f;
	arm_rms_f32(current_1d_buffer, BUFFER_SIZE, &rms_voltage);

    // 3. Calculate Peak-to-Peak Ripple
	float peak_to_peak = max_val - min_val;

    // Pack into indices 18, 19, and 20 of our global feature array
    output_array[18] = mean_voltage;
    output_array[19] = rms_voltage;
    output_array[20] = peak_to_peak;

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  CS_HIGH();     // For Assurance , explicitly keep the cs line high
  ADXL_init();   // Initializes sensor for 800Hz readings

  // Initialize the FFT-Engine
  arm_status status = arm_rfft_fast_init_f32(&fft_instance, BUFFER_SIZE);
  if(status != ARM_MATH_SUCCESS){
	  printf("DSP INIT ERROR: FFT Initialization failed\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("System booting...\r\n");
  while (1)
  {
	if(data_ready_flag){

		data_ready_flag = false;

		uint8_t regAddr = ADXL345_DATAX0 | 0x80 | 0x40;
		uint8_t txData[7] = {regAddr, 0, 0, 0, 0, 0, 0};
		uint8_t rxData[7] = { 0 };

		//Fast burst read of all three axes
		CS_LOW();
		HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 7, HAL_MAX_DELAY);
		CS_HIGH();


		// Convert 2's complement data.
		int16_t x_raw = (int16_t)((rxData[2] << 8) | rxData[1]);
		int16_t y_raw = (int16_t)((rxData[4] << 8) | rxData[3]);
		int16_t z_raw = (int16_t)((rxData[6] << 8) | rxData[5]);

		write_ptr[sample_index][0] = x_raw * 0.0039f;
		write_ptr[sample_index][1] = y_raw * 0.0039f;
		write_ptr[sample_index][2] = z_raw * 0.0039f;

		// Poll ADC for current data
		HAL_ADC_Start(&hadc1);

		if(HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK){
			uint32_t adcValue = HAL_ADC_GetValue(&hadc1);

			float voltage = ((float)adcValue / 4095.0f) * 3.3f;

			write_ptr[sample_index][3] = voltage;
		}

		sample_index++;

		// Buffer management
		if(sample_index >= BUFFER_SIZE){
			sample_index = 0;

			if(write_ptr == buffer_a){
				write_ptr = buffer_b;
				processing_ptr = buffer_a;
			}else{
				write_ptr = buffer_a;
				processing_ptr = buffer_b;
			}

			processing_buffer_full = true;  // Signal DSP Phase to begin

		}
	}

	// Processing the full buffer

	if(processing_buffer_full){
		processing_buffer_full = false;

		// Executing the pipeline sequentionally
		extractFeatures(0, processing_ptr, sensor_features);
		extractFeatures(1, processing_ptr, sensor_features);
		extractFeatures(2, processing_ptr, sensor_features);

		// Extract the Current Features
		extractCurrentFeatures(processing_ptr, sensor_features);



		// Print out a summary to verify the 3-axis extraction
		printf("--- 3-Axis Extraction Complete ---\r\n");
		printf("X-Axis | RMS: %.2f | P-P: %.2f\r\n", sensor_features[0], sensor_features[1]);
		printf("Y-Axis | RMS: %.2f | P-P: %.2f\r\n", sensor_features[6], sensor_features[7]);
		printf("Z-Axis | RMS: %.2f | P-P: %.2f\r\n", sensor_features[12], sensor_features[13]);
		printf("----------------------------------\r\n\n");

		processing_ptr = NULL;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADXL_CS_Pin */
  GPIO_InitStruct.Pin = ADXL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADXL_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADXL_INT1_Pin */
  GPIO_InitStruct.Pin = ADXL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ADXL_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		data_ready_flag = true;
	}
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
