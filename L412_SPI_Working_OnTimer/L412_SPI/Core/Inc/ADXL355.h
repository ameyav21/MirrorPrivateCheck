/*
 * ADXL355.h
 *
 *  Created on: Dec 5, 2021
 *      Author: avalsangkar
 */

#ifndef	INC_ADXL355_H_
#define INC_ADXL355_H_

#include <stdint.h>
#include <stdbool.h>
#include "CustomTypes.h"

#include "GeoLibrary.h"

#include "main.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
/*
 * DEFINE
 */

#define DEBUG_ADXL          (false && defined _DEBUG)

#if COMPILE_ANALYTICS && true
    // Preserve history for analysis.
    #define ADXL_SAMPLE_STORAGE_DEPTH   (400)
#else
    #define ADXL_SAMPLE_STORAGE_DEPTH   (1) // Store only latest sample.
#endif

//#define FAULT true
//#define NO_FAULT false

#define ANALOG_DEVICES_ID   0xAD
#define MEMS_ID             0x1D
#define PART_ID             0xED
#define REVISION_ID         0x01
#define RESET_CODE          0x52

#define STANDBY          (1 << 0)   // POWER_CTL BITS
#define MEASUREMENT_MODE (0 << 0)
#define TEMP_OFF         (1 << 1)
#define DRDY_OFF         (1 << 2)

#define DATA_RDY         (1 << 0)   // Status bits

#define HPF_CORNER   (0b000 << 4)   // No high-pass filter enabled

#define I2C_HS       (0b10000000)   // Default I2C, irrelevant.
#define G2_RANGE     (0b00000001)   // Default sensitivity: ±2.048
#define G4_RANGE     (0b00000010)   // ±4.096 g
#define G8_RANGE     (0b00000011)   // ±8.192 g

typedef struct {
    uint16_t ODR_LPF_setting;
    float DataRate;
    float CombinedDelay;
} DataRateConfiguration_t;

#define ADXL_TURNON_STANDBY2MEAS_MS (10)    // 2 g range "valid when the output is within 1 mg of the final value."
#define ADXL_TURNON_OFF2STANDBY_MS  (10)    // Power-off to standby

#define ADXL_TURN_ON_TIME_CLKS  (MCLK_FREQ * (ADXL_TURNON_OFF2STANDBY_MS) / 1000)
static const uint16_t ADXLturnOnTime2g_ms = ADXL_TURNON_STANDBY2MEAS_MS;

static const float ADXLsamplePeriod = 0.100f;   // Seconds.

static const DataRateConfiguration_t ADXLsampling [] =
{
    { .ODR_LPF_setting = 0b0000, .DataRate = 4000.0,   .CombinedDelay =   0.88 },
    { .ODR_LPF_setting = 0b0001, .DataRate = 2000.0,   .CombinedDelay =   1.51 },
    { .ODR_LPF_setting = 0b0010, .DataRate = 1000.0,   .CombinedDelay =   2.75 },
    { .ODR_LPF_setting = 0b0011, .DataRate =  500.0,   .CombinedDelay =   5.27 },
    { .ODR_LPF_setting = 0b0100, .DataRate =  250.0,   .CombinedDelay =  10.31 },
    { .ODR_LPF_setting = 0b0101, .DataRate =  125.0,   .CombinedDelay =  20.38 },
    { .ODR_LPF_setting = 0b0110, .DataRate =   62.5,   .CombinedDelay =  40.52 },
    { .ODR_LPF_setting = 0b0111, .DataRate =   31.25,  .CombinedDelay =  80.78 },
    { .ODR_LPF_setting = 0b1000, .DataRate =   15.625, .CombinedDelay = 161.31 },
    { .ODR_LPF_setting = 0b1001, .DataRate =    7.813, .CombinedDelay = 322.48 },
    { .ODR_LPF_setting = 0b1010, .DataRate =    3.906, .CombinedDelay = 644.39 },
};

// Add 6.02 dB to reported SNR for each quadrupling of sample size.
// Also consider addressable MEMS cycle time attributable to delay.
static const uint16_t OptimizedSPS = 0; // ADXLsampling[] index optimized for SNR.

typedef enum {
    eWriteByte = 0,
    eReadByte  = 1
} ADXL354direction;

typedef enum {
    eDEVID_AD     = 0x00,
    eDEVID_MST    = 0x01,
    ePARTID       = 0x02,
    eREVID        = 0x03,
    eStatus       = 0x04,
    eFIFO_ENTRIES = 0x05,
    eTEMP2        = 0x06,
    eTEMP1        = 0x07,
    eXDATA3       = 0x08,
    eXDATA2       = 0x09,
    eXDATA1       = 0x0A,
    eYDATA3       = 0x0B,
    eYDATA2       = 0x0C,
    eYDATA1       = 0x0D,
    eZDATA3       = 0x0E,
    eZDATA2       = 0x0F,
    eZDATA1       = 0x10,
    eFIFO_DATA    = 0x11,   // Last read-only register. Note gap.

    eOFFSET_X_H   = 0x1E,
    eOFFSET_X_L   = 0x1F,
    eOFFSET_Y_H   = 0x20,
    eOFFSET_Y_L   = 0x21,
    eOFFSET_Z_H   = 0x22,
    eOFFSET_Z_L   = 0x23,
    eACT_EN       = 0x24,
    eACT_THRESH_H = 0x25,
    eACT_THRESH_L = 0x26,
    eACT_COUNT    = 0x27,
    eFilter       = 0x28,
    eFIFO_SAMPLES = 0x29,
    eINT_MAP      = 0x2A,
    eSync         = 0x2B,
    eRange        = 0x2C,
    ePOWER_CTL    = 0x2D,
    eSELF_TEST    = 0x2E,
    eReset        = 0x2F,

    eSHADOW1      = 0x50,   //Shadow registers
    eSHADOW2      = 0x51,
    eSHADOW3      = 0x52,
    eSHADOW4      = 0x53,
    eSHADOW5      = 0x54,
} ADXL354registers;


typedef struct {
    union {
        struct {                // For value access.
            int32_t Reserved     :  4;
            int32_t Acceleration : 28;
        }; // Unnamed to abbreviate usage.
        struct {                // For register access.
            int32_t Data1        : 8;
            int32_t Data2        : 8;
            int32_t Data3        : 8;
            int32_t SignBits     : 8;
        };
        uint32_t Everything;    // For initialization and unit tests.
    };
} ADXL_axis_t;


typedef union {
    struct {
        // A complete x-axis, y-axis, and z-axis measurement was made and results can be read.
        uint8_t DataReady  : 1; // DATA_RDY
        uint8_t FIFOfull   : 1; // FIFO_FULL -- Watermark is reached.
        uint8_t FIFOoverrun: 1; // FIFO_OVR -- Oldest data is lost.
        uint8_t Activity   : 1; // Activity, as defined in the THRESH_ACT and COUNT_ACT registers, is detected.
        // NVM controller is busy with either refresh, programming, or built-in, self test (BIST).
        uint8_t NVMbusy    : 1; // NVM_BUSY
        uint8_t Reserved   : 3;
    };  // Unnamed to abbreviate usage.
    uint8_t AsByte;
} ADXL_status_t;


typedef union {
    struct {
        uint8_t ST1     : 1;
        uint8_t ST2     : 1;
        uint8_t Reserved: 6;
    };  // Unnamed to abbreviate usage.
    uint8_t AsByte;
} ADXL_selftest_t;


typedef union {
    struct {
        uint8_t Standby : 1;
        uint8_t TempOff : 1;
        uint8_t DrdyOff : 1;
        uint8_t Reserved: 5;
    };  // Unnamed to abbreviate usage.
    uint8_t AsByte;
} ADXL_powercontrol_t;


///////////////////////////////////////////////////////////

void SPI_read (uint8_t address, int bytes);
uint32_t ADXL355_SPI_Read(uint8_t ui8address);

void   WriteADXL355register(uint8_t RegAddr, uint8_t RegData);
uint8_t ReadADXL355register(uint8_t RegAddr);

bool ResetADXL355(void);
bool InitializeADXL355 (void);
bool ADXL355DataReady(void);
uint32_t ReadADXL355acceleration(const eAxis_t AxisNum);
const multisample_t* OverSampleADXL355(uint16_t ADXL355samplesInPeriod, uint16_t DataReadyWaitLimit_ms);

void adxl_init(void);
//int check_adxl(void);
int check_adxl(uint8_t RegAddr, uint8_t CheckValue);

void InitializeOversampleTotals(multisample_t* pSampleData, int16_t ArraySize); //Temporar comes from MEMsMATH


///////////////////////////////////



SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim16;

uint8_t data[9]; /*Buffer to hold SPI data. Axis acceleration is a 20 bit value, and it's stored
in consecutive registers, from the most significative to the least significative data
and in left-justified mode. It is thus needed to read 3 bytes for each axis (i.e. 9 consecutive bytes
for each axis). */
int32_t x,y,z; /*variables that hold the binary acceleration reads. */
float xg, yg, zg; /*variables that hold the data converted in g. */
uint8_t data_tx[2];
uint8_t temp;

static void MX_SPI1_Init(void);

static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_TIM16_Init(void);


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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin PA15 */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

}


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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  htim16.Init.Prescaler = 79;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}




//typedef struct {
//	/* I2C HANDLE */
//	SPI_HandleTypeDef *spiHandle;
//
//	/* Acceleration data (X,Y,Z) in m/s^2 */
//	float acc_mps2[3];
//
//	/* Temperature data in deg */
//	float temp_C;
//
//} ADXL355;
//
///*
// * Initialize
// */
//uint8_t ADXL355_Initialize( ADXL355 *dev, SPI_HandleTypeDef *spiHandle);
//
///*
// * Data Capture
// */
//HAL_StatusTypeDef ADXL355_ReadTemperature( ADXL355 *dev );
//HAL_StatusTypeDef ADL355_ReadAccelerations( ADXL355 *dev );

/*
 * Register operations
 */
//
//HAL_StatusTypeDef ADXL355_ReadRegister( ADXL355 *dev, uint8_t reg, uint8_t *data);
//HAL_StatusTypeDef ADXL355_ReadRegisters( ADXL355 *dev, uint8_t reg, uint8_t *data, uint8_t length);
//
//HAL_StatusTypeDef ADXL355_WriteRegister( ADXL355 *dev, uint8_t reg, uint8_t *data);


//void SPI_write (uint8_t address, uint8_t value);


uint32_t ReadADXL355acceleration(const eAxis_t AxisNum);

#endif /* SRC_ADXL355_H_ */
