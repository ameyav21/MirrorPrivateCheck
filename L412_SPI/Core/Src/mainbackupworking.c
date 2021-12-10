///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2021 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "string.h"
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include <stdio.h>
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//#define REG_POWER_CTL 0x2D
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//SPI_HandleTypeDef hspi1;
//
//UART_HandleTypeDef huart2;
//
///* USER CODE BEGIN PV */
//// 25AA040A instructions
//uint8_t data[9]; /*Buffer to hold SPI data. Axis acceleration is a 20 bit value, and it's stored
//in consecutive registers, from the most significative to the least significative data
//and in left-justified mode. It is thus needed to read 3 bytes for each axis (i.e. 9 consecutive bytes
//for each axis). */
//int32_t x,y,z; /*variables that hold the binary acceleration reads. */
//float xg, yg, zg; /*variables that hold the data converted in g. */
//uint8_t data_tx[2];
//
//
//uint8_t temp;
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);
//static void MX_SPI1_Init(void);
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//void SPI_write (uint8_t address, uint8_t value) //function to write 1 byte in a register on the accelerometer  through SPI
//{
//
//    data_tx[0]= (address<<1) | 0x00; /* set write operation= to enter Write mode you have to set the 8th bit of the first byte sent to 0.*/
//    data_tx[1] = value; /*byte to write in the register*/
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);  // pull the CS pin (PA12) low (selects the slave)
//    HAL_SPI_Transmit (&hspi1, data_tx, 2, 100);  // write data to register specifying that it consists of 2 bytes (address+value)
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  // pull the CS pin high
//}
//
//void SPI_read (uint8_t address, int bytes) //function to read multiple bytes from a register on the accelerometer through SPI
//{
//    address = (address<<1) | 0x01;  /* set read operation= to enter Read mode you have to set the 8th bit of the first byte sent to 1.*/
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);   // pull the CS pin low
//    HAL_SPI_Transmit (&hspi1, &address, 1, 100);  // send address
//    HAL_SPI_Receive (&hspi1, data, bytes, 100);  // receive the data
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // pull the CS pin high
//}
//
//void adxl_init(void){
//    //Configuring the Range
//    SPI_write(0x2C, 0x02);  /*b00000001=0x01 in 0x2C register (Interrupt Polarity, Range register)
//    sets a 4g range. The ADXL also uses 20 bit RES and stores data in the left-justified mode.*/
//
//    //Configuring the Power Control or POWER_CTL register:
//    SPI_write(REG_POWER_CTL, 0x06); /* enters measurement mode and disables temperature reading */
//}
//
//
//void display_Data(double val, char axis){ //displays acceleration data using UART
//    char msg[20];
//    sprintf(msg, "%c: %f",axis, val);
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
//    char newline[2] = "\r\n";
//    HAL_UART_Transmit(&huart2, (uint8_t *) newline, 2, 10);
//}
//
//void readSensor(void){
//    SPI_read(0x08, 9); /*the full accelerometer information is held in 3 bytes for each axis, so you need to read 9 bytes
//    (also, acceleration data is stored from the most significant bit to the least significant one) */
//
//    // copy the 3 bytes received for each axis into a single variable
//    x=((data[0]<<16) | (data[1]<<8) | data[2]) >>4; /*moving the first received byte at the beginning (left-justified),
//    then using an OR to concatenate it with the second byte (shifted the same way) and lastly the third byte
//    Since this are 20-bits values, we then shift the copied value 4 bits to the right*/
//    y=((data[3]<<16) | (data[4] <<8) | data[5]) >>4;
//    z=((data[6]<<16) | (data[7] <<8) | data[8]) >>4;
//    //converting to g
//    /* From the datasheet we get that for a +-4g sensitivity the scale factor is 7.8 ug/LSB.
//     * To convert the data we just need to multiple with 7.8 and then divide by 1000000*/
//    xg=x*0.0000078;
//    yg=y*0.0000078;
//    zg=z*0.0000078;
//}
//
//int check_adxl(){
//	char uart_buf[50];
//	int uart_buf_len;
//    SPI_read(0x03, 1);
//    temp=(uint8_t)data[0];
//    // Print out status register
//      uart_buf_len = sprintf(uart_buf,
//                              "Status: 0xx\r\n",
//                              (unsigned int)temp);
//      HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//    return (temp==0x01);
//}
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//	char uart_buf[50];
//	int uart_buf_len;
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART2_UART_Init();
//  MX_SPI1_Init();
//  /* USER CODE BEGIN 2 */
//  // CS pin should default high
//  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//
//  // Say something
//  uart_buf_len = sprintf(uart_buf, "SPI Test\r\n");
//  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
//
//
//    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); //sets CS line high when not reading/writing
//    /* USER CODE BEGIN 2 */
//    HAL_Delay(1000);
//    adxl_init();
//    char *mex="Accelerometer...";
//    char line[2] = "\r\n";
//    HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
//    HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//	  int checknum = check_adxl();
//	  if (checknum==1){
//		  char *mex="True............";
//		  char line[2] = "\r\n";
//		  HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
//		  HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
//	  }
//      HAL_Delay(2000);
//      /*Reading and displaying sensor data*/
//      readSensor();
//      display_Data(xg, 'X');
//      display_Data(yg, 'Y');
//      display_Data(zg, 'Z');
//
//      HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
//  RCC_OscInitStruct.PLL.PLLM = 1;
//  RCC_OscInitStruct.PLL.PLLN = 40;
//  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
//  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief SPI1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_SPI1_Init(void)
//{
//
//  /* USER CODE BEGIN SPI1_Init 0 */
//
//  /* USER CODE END SPI1_Init 0 */
//
//  /* USER CODE BEGIN SPI1_Init 1 */
//
//  /* USER CODE END SPI1_Init 1 */
//  /* SPI1 parameter configuration*/
//  hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi1.Init.CRCPolynomial = 7;
//  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI1_Init 2 */
//
//  /* USER CODE END SPI1_Init 2 */
//
//}
//
///**
//  * @brief USART2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
//
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
//
//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 115200;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
//
//  /* USER CODE END USART2_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|GPIO_PIN_15, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin PA15 */
//  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : SMPS_PG_Pin */
//  GPIO_InitStruct.Pin = SMPS_PG_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LD4_Pin */
//  GPIO_InitStruct.Pin = LD4_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);
//
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
//
