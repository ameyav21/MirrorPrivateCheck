/*
 * ADXL355.c
 *
 *  Created on: Dec 5, 2021
 *      Author: avalsangkar
 */

#include "ADXL355.h"

// Private variables
static multisample_t ADXL355sampleData[3] = {{.ADCsum = 0,  // Three axes.
                                               .ADCsum_of_squares = 0,
                                               .Count = 0,
                                               .ReadOK = false}, /* etc. */ };

#if COMPILE_ANALYTICS // Math is computationally expensive.
    int32_t AxisBuffer[3][ADXL_SAMPLE_STORAGE_DEPTH];  // Store individual samples of x, y, and z.
    static const uint16_t AxisBufferDepth = sizeof(AxisBuffer[0])/sizeof(AxisBuffer[0][0]);    // Limit of x, y, and z buffers.
#endif


//extern ramTable_t Modbus_RAM ;


//uint32_t ADXL355_SPI_Read(uint8_t ui8address) {
//
//	HAL_StatusTypeDef status;
//	uint8_t recieveData;
//	uint8_t txData;
//
//	txData = (ui8address << 1) | 1 ;
//
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); //ON
//
//	status = HAL_SPI_Transmit (&hspi1, &txData, 1, HAL_MAX_DELAY);
//
//	status = HAL_SPI_Receive (&hspi1, &recieveData, 1, HAL_MAX_DELAY);
//
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); //OFF
//
//	if (status == HAL_OK)
//		return recieveData;
//	else{
//		printf("\r\nError Reading: Invalid HAL_STATUS\r\n");
//		char *mex="Error Read SPI... \r\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *) mex, 28, 10);
//	}
//	return 255;
//}
//
//void ADXL355_SPI_Write(uint8_t ui8address, uint8_t ui8Data) {
//
//	HAL_StatusTypeDef status;
//	uint8_t address;
//
//	address = ((ui8address << 1) & 0xFE);
//
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); //ON
//
//	status = HAL_SPI_Transmit (&hspi1, &address, 1, HAL_MAX_DELAY);
//	status = HAL_SPI_Transmit (&hspi1, &ui8Data, 1, HAL_MAX_DELAY);
//
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); //OFF
//
//	if (status != HAL_OK){
//		char *mex="Error Write SPI... \r\n";
//		HAL_UART_Transmit(&huart2, (uint8_t *) mex, 28, 10);
//		printf("\r\nError writing: Invalid HAL STATUS\r\n");
//	}
//}

void SPI_write (uint8_t address, uint8_t value) //function to write 1 byte in a register on the accelerometer  through SPI
{

    data_tx[0]= (address<<1) | 0x00; /* set write operation= to enter Write mode you have to set the 8th bit of the first byte sent to 0.*/
    data_tx[1] = value; /*byte to write in the register*/
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);  // pull the CS pin (PA12) low (selects the slave)
    HAL_SPI_Transmit (&hspi1, data_tx, 2, 100);  // write data to register specifying that it consists of 2 bytes (address+value)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  // pull the CS pin high
}

void SPI_read (uint8_t address, int bytes) //function to read multiple bytes from a register on the accelerometer through SPI
{
    address = (address<<1) | 0x01;  /* set read operation= to enter Read mode you have to set the 8th bit of the first byte sent to 1.*/
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);   // pull the CS pin low
    HAL_SPI_Transmit (&hspi1, &address, 1, 100);  // send address
    HAL_SPI_Receive (&hspi1, data, bytes, 100);  // receive the data
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // pull the CS pin high
}

void   WriteADXL355register(uint8_t RegAddr, uint8_t RegData)
{
	SPI_write (RegAddr, RegData);
	//ADXL355_SPI_Write(RegAddr, RegData);
    //ExchangeSPIword((((((uint16_t) RegAddr) << 1) | eWriteByte )  << 8) | RegData);
}


uint8_t ReadADXL355register(uint8_t RegAddr)
{
	SPI_read (RegAddr, 1);
	uint8_t returnspidata = (uint8_t) data[0];
	return returnspidata;
	//return ADXL355_SPI_Read(RegAddr);
    //return (uint8_t)(ExchangeSPIword(((((uint16_t) RegAddr) << 1) | eReadByte )  << 8));
}

void adxl_init(void){
    //Configuring the Range
    SPI_write(0x2C, 0x02);  /*b00000001=0x01 in 0x2C register (Interrupt Polarity, Range register)
    sets a 4g range. The ADXL also uses 20 bit RES and stores data in the left-justified mode.*/

    //Configuring the Power Control or POWER_CTL register:
    SPI_write(ePOWER_CTL, 0x06); /* enters measurement mode and disables temperature reading */
}

int check_adxl(uint8_t RegAddr, uint8_t CheckValue){
    SPI_read(RegAddr, 1);
    temp=(uint8_t)data[0];
    return (temp==CheckValue);
}

/*
 *  ADXL354/ADXL355 Data Sheet, Rev. B, D14205-6/20(B)
 *  In case of a software reset, an unlikely race condition may occur in
 *  products with REVID = 0x01 or earlier. If the race condition occurs,
 *  some factory settings in the NVM load incorrectly to shadow registers
 *  (the registers from which the internal logic configures the sensor
 *  and calculates the output after a power-on or a software reset).
 *  The incorrect loading of the NVM affects overall performance of the
 *  sensor, such as an incorrect 0 g bias and other performance issues.
 *  The incorrect loading of NVM does not occur from a power-on or
 *  after a power cycle. To guarantee reliable operation of the sensor
 *  after a software reset, the user can access the shadow registers after a
 *  power-on, read and store the values on the host microprocessor, and
 *  compare the values read from the same shadow registers after a
 *  software reset. This method guarantees proper operation in all devices
 *  and under all conditions. The recommended steps are as follows:
 *  1. Read the shadow registers, Register 0x50 to Register 0x54 (five 8-bit
 *     registers) after power-up, but before any software reset.
 *  2. Store these values in a host device (for example, a host microprocessor).
 *  3. After each software reset, read the same five registers. If the
 *     values differ, perform a software reset again until they match.
 */
bool ResetADXL355(void)
{
    uint8_t Shadow[eSHADOW5 - eSHADOW1 + 1];    // Intentionally uninitialized.
    uint16_t reg, try;                          // Intentionally uninitialized.
    bool ADXL355status;                         // Intentionally uninitialized.
    static const uint16_t retryLimit = 3;

    // Read before reset.
    for(reg = eSHADOW1; reg <= eSHADOW5; reg++)
    {
        Shadow[reg - eSHADOW1] = ReadADXL355register(reg);
    }

    for(try = 1; try <= retryLimit; try++)
    {
        // Write Code 0x52 to reset the device, similar to a power-on reset (POR).
        WriteADXL355register(eReset, RESET_CODE);
        HAL_Delay(1000);                      // 500 us delay. ADI minimum recommended delay after reset is 170uS
        for(reg = eSHADOW1; reg <= eSHADOW5; reg++)
        {
            if(Shadow[reg - eSHADOW1] != ReadADXL355register(reg))
            {
                //Modbus_RAM.ec2.reset = FAULT; //Where is ec2?
                ADXL355status = FAULT;  // Trapped twice in approximately 2700 measure cycles.
                char *mex="eSHADOW1Fault...";
				char line[2] = "\r\n";
				HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
				HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
                break;  // Terminate inner for loop.
            }
        }
        if(eSHADOW5 < reg)  // Counted all five registers as matches?
        {
            ADXL355status = NO_FAULT;
            break; // Terminate outer for loop. No need to retry.
        }
    }
    return ADXL355status;
}

bool InitializeADXL355(void)
{

    bool ADXL355status;  // Intentionally uninitialized.

#if DEBUG_ADXL  // Before
    volatile uint8_t RegNum;  // Use for arbitrary register reads and writes.
    volatile uint8_t ADXL355param[eReset + 1] = { 0, };
    for(RegNum = eDEVID_AD; RegNum <= eReset; RegNum++) {
        ADXL355param[RegNum] = ReadADXL355register(RegNum);
    }
#endif

    // Reset using guidance from Mahdi Sadeghi, MEMS Technology Group Product
    //  Marketing Manager, Analog Devices. In a September 23, 2020 video conference,
    //  he attributed spurious readings to a race condition, which would corrupt
    //  trim values that are set after the ICs are manufactured. He estimated that
    //  "shadow registers" could be misloaded one out of every 1000 times due to
    //  misalignment of internal clock and reset lines.
    //ADXL355status = ResetADXL355();
    ADXL355status = ResetADXL355();

    if (! (  (ANALOG_DEVICES_ID == ReadADXL355register(eDEVID_AD ))
         || (MEMS_ID           == ReadADXL355register(eDEVID_MST))
         || (PART_ID           == ReadADXL355register(ePARTID   )) ))
    {
        ADXL355status = FAULT;
        char *mex="STATUSFault...";
		char line[2] = "\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
		HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
        //Modbus_RAM.ec2.readADXLreg = FAULT;
    }

    // While configuring the ADXL355 in an application, all configuration registers
    //  must be programmed before enabling measurement mode in the POWER_CTL register.
    WriteADXL355register(ePOWER_CTL, STANDBY);
    if (STANDBY != ReadADXL355register(ePOWER_CTL)) {
        ADXL355status = FAULT;
        //Modbus_RAM.ec2.writeADXLreg = FAULT;
    }

    WriteADXL355register(eFilter, HPF_CORNER | ADXLsampling[OptimizedSPS].ODR_LPF_setting);
    WriteADXL355register(eRange, I2C_HS | G2_RANGE);    // Same as default.
    // ToDo: Explicitly set all other registers.
    //       Perhaps develop function for multi-byte writes and upload a table.
    WriteADXL355register(ePOWER_CTL, MEASUREMENT_MODE);

#if DEBUG_ADXL  // After
    for(RegNum = eDEVID_AD; RegNum <= eReset; RegNum++) {
        ADXL355param[RegNum] = ReadADXL355register(RegNum);
    }
#endif

    return ADXL355status;
}

uint32_t ReadADXL355acceleration(const eAxis_t AxisNum){
    uint8_t BaseRegister;   // Intentionally Uninitialized.
    ADXL_axis_t AxisData;   // 20-bit analog to digital conversions via 3 register reads.

#if false && DEBUG_ADXL
    AxisData.Everything = 0;
#endif

    BaseRegister = eXDATA3 + (3 * AxisNum);
    AxisData.Data3 = ReadADXL355register(BaseRegister + 0); // Bits[19:12]
    AxisData.Data2 = ReadADXL355register(BaseRegister + 1); // Bits[11:4]
    AxisData.Data1 = ReadADXL355register(BaseRegister + 2); // Bits[3:0], first four LSBs reserved!

    // Extend two's complement from 20 bit ADC to 32 bit register.
    if (AxisData.Data3 & (1 << 7)) AxisData.SignBits = 0xFF;
    else                           AxisData.SignBits = 0x00;

    return AxisData.Acceleration;
}


/*! \brief  Oversample MEMS IC to maximize precision.
 *  \param  Sample size and timeout duration (in milliseconds)
 *  \retval Constant array of three oversample structures, one of each axis
 *
 *  \note   Averaging reduces stochastic noise two fold for every quadrupling of population.
 *  \note   A 100 millisecond period will encompass exactly six periods of 60 Hz noise
 *           or exactly five of 50 Hz. The integration of an integer number of
 *           trigonometric periods, regardless of phase, is zero. Thus, we eliminate
 *           any noise from mains lines that couple with the Addressable MEMS string.
 *  \note   The buffer is employed only for review of real-time data.
 *           It can be as small as one 32-bit integer for release builds.
 */
//const multisample_t* OverSampleADXL355(uint16_t ADXL355samplesInPeriod, uint16_t DataReadyWaitLimit_ms)
//{
//    bool SensorResponse = NO_FAULT;
//    uint16_t SampleCount;       // Intentionally uninitialized.
//    uint16_t ReferenceTime;     // Intentionally uninitialized.
//    uint16_t ElectronicAxis;    // Physical axis of inclination, as wired. Intentionally uninitialized.
//    int32_t AxisData;           // Intentionally uninitialized.
//    static int32_t  AxisLast[3] = {0, 0, 0};    // Record readings for subsequent comparison.
//    int16_t AxisChangeCount[3] = {0, 0, 0};     // Count cycle-to-cycle variation.
//
//    // Initialize static structures.
//    InitializeOversampleTotals(ADXL355sampleData,
//                               sizeof(ADXL355sampleData)/sizeof(ADXL355sampleData[0]));
//
//    for(SampleCount = 1; SampleCount <= ADXL355samplesInPeriod; SampleCount++) {
//        ReferenceTime = GetStartTime();
//        //PET_THE_DOG;    // Soothe the pooch while waiting.
//        while (!ADXL355DataReady()) {
//            if (Elapsed_ms(&ReferenceTime) > DataReadyWaitLimit_ms) // More than double expected period?
//            {
//                //Modbus_RAM.ec2.readTimeout = FAULT;
//                SensorResponse = FAULT;
//                break; // Escape seizure.
//            }
//        }
//
//        if (FAULT != SensorResponse) {
//            for (ElectronicAxis = aAxis; ElectronicAxis <= cAxis; ElectronicAxis ++) {
//
//                AxisData = ReadADXL355acceleration((eAxis_t) ElectronicAxis);
//                ADXL355sampleData[ElectronicAxis].ADCsum += (int64_t) AxisData;
//                ADXL355sampleData[ElectronicAxis].Count = SampleCount;
//
//                ADXL355sampleData[ElectronicAxis].ADCsum_of_squares
//                                    += ((int64_t) AxisData * (int64_t) AxisData);
//
//#if COMPILE_ANALYTICS && true
//                if (AxisBufferDepth >= SampleCount) {
//                    AxisBuffer[ElectronicAxis][SampleCount - 1] = AxisData;
//                }
//#endif
//                // Count cycle-to-cycle variation to detect stuck axes.
//                // https://ez.analog.com/mems/f/q-a/91009/adxl355-adxl357-intermittent-output-on-1-or-all-channels
//                // https://ez.analog.com/mems/f/q-a/89182/adxl355-cant-read-xdata-register-while-y-and-z-looks-good/264168#264168
//                // https://ez.analog.com/mems/f/q-a/89324/adxl355-output-stuck-at-zero
//                if(AxisLast[ElectronicAxis] != AxisData) {
//                    AxisChangeCount[ElectronicAxis] ++;  // Increment if value differs.
//
//                    // Verify axis readings exhibit variation.
//                    //  In practice, almost every sample differs from the last. Use a one-half threshold.
//                    if(AxisChangeCount[ElectronicAxis] < (SampleCount >> 1)) {
//                        SensorResponse = FAULT; // Terminate next cycle if an axis is unchanging.
//                        //Modbus_RAM.ec2.duplicateValues = FAULT;
//                    }
//                    else {
//                        AxisLast[ElectronicAxis] = AxisData;
//                    }
//                }
//            }
//        }
//        else {
//            ADXL355sampleData[aAxis].ReadOK =
//            ADXL355sampleData[bAxis].ReadOK =
//            ADXL355sampleData[cAxis].ReadOK = false;
//            break; // Escape seizure.
//        }
//    }
//
//    return ADXL355sampleData;   // Pass pointer to file-scope variable.
//}

// DRDY is set when new acceleration data is available to the interface.
//  It clears on a read of the FIFO, on a read of XDATA, YDATA, or ZDATA.
#pragma FUNC_ALWAYS_INLINE(ADXL355DataReady)
bool ADXL355DataReady(void) {

	if (HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1)){
	char *mex="DATARDYACTIVE...";
	char line[2] = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
	HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
	}
	return (HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1));
    //return(0 != (P2IN & BIT7));    // DRDY
}


///// Temporary Comes from MEMsMATH/////////
void InitializeOversampleTotals(multisample_t* pSampleData, int16_t ArraySize) {
    int16_t i;

    // First clear everything with speedy low-level library.
    memset(pSampleData, 0, ArraySize * sizeof(multisample_t));

    // Next, set the flags.
    for (i= 0; i < ArraySize; i++) {
        pSampleData[i].ReadOK = true;   // Presume OK.
    }
}
//////////////////////////////////////////////

////////////////////////////////////////////////////////////
///*
// *  ADXL354/ADXL355 Data Sheet, Rev. B, D14205-6/20(B)
// *  In case of a software reset, an unlikely race condition may occur in
// *  products with REVID = 0x01 or earlier. If the race condition occurs,
// *  some factory settings in the NVM load incorrectly to shadow registers
// *  (the registers from which the internal logic configures the sensor
// *  and calculates the output after a power-on or a software reset).
// *  The incorrect loading of the NVM affects overall performance of the
// *  sensor, such as an incorrect 0 g bias and other performance issues.
// *  The incorrect loading of NVM does not occur from a power-on or
// *  after a power cycle. To guarantee reliable operation of the sensor
// *  after a software reset, the user can access the shadow registers after a
// *  power-on, read and store the values on the host microprocessor, and
// *  compare the values read from the same shadow registers after a
// *  software reset. This method guarantees proper operation in all devices
// *  and under all conditions. The recommended steps are as follows:
// *  1. Read the shadow registers, Register 0x50 to Register 0x54 (five 8-bit
// *     registers) after power-up, but before any software reset.
// *  2. Store these values in a host device (for example, a host microprocessor).
// *  3. After each software reset, read the same five registers. If the
// *     values differ, perform a software reset again until they match.
// */
//int ResetADXL355(void)
//{
//    uint8_t Shadow[eSHADOW5 - eSHADOW1 + 1];    // Intentionally uninitialized.
//    uint16_t reg, try;                          // Intentionally uninitialized.
//    int ADXL355status;                         // Intentionally uninitialized.
//    static const uint16_t retryLimit = 3;
//
//    //ADXL355_SPI_Write(0x2F, 0x52);
//    // Read before reset.
//    for(reg = eSHADOW1; reg <= eSHADOW5; reg++)
//    {
//        Shadow[reg - eSHADOW1] = ADXL355_SPI_Read(reg);
//    }
//
//    for(try = 1; try <= retryLimit; try++)
//    {
//        // Write Code 0x52 to reset the device, similar to a power-on reset (POR).
//    	ADXL355_SPI_Write(ADXL355_REG_RESET, ADXL355_RESET_CODE);
//    	HAL_Delay(5000);                      // 500 us delay. ADI minimum recommended delay after reset is 170uS
//        for(reg = eSHADOW1; reg <= eSHADOW5; reg++)
//        {
//            if(Shadow[reg - eSHADOW1] != ADXL355_SPI_Read(reg))
//            {
//                ADXL355status = 1;  // Trapped twice in approximately 2700 measure cycles.
//                char *mex="Shadow5Fault...............";
//                char line[2] = "\r\n";
//                HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
//                HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
//                break;  // Terminate inner for loop.
//            }
//        }
//        if(eSHADOW5 < reg)  // Counted all five registers as matches?
//        {
//            ADXL355status = 0;
//            break; // Terminate outer for loop. No need to retry.
//        }
//    }
//    return ADXL355status;
//}
//
//
//int InitializeADXL355(void){
//
//
//	int ADXL355status = 0;
//
//	ADXL355status = ResetADXL355();
//
//    if (ADXL355status==1) {
//  	  char *mex="ResetFault...............";
//  	      char line[2] = "\r\n";
//  	      HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
//  	      HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
//    }
//    else{
//  	      	  char *mex="ResetNoFault............";
//  	      	      char line[2] = "\r\n";
//  	      	      HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
//  	      	      HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
//    }
//
//	uint32_t volatile ui32test = ADXL355_SPI_Read(ADXL355_REG_DEVID_AD);                  /* Read the ID register */
//	uint32_t volatile ui32test2 = ADXL355_SPI_Read(ADXL355_REG_DEVID_MST);                  /* Read the ID register */
//	uint32_t volatile ui32test3 = ADXL355_SPI_Read(ADXL355_REG_PARTID);                  /* Read the ID register */
//	uint32_t volatile ui32test4 = ADXL355_SPI_Read(ADXL355_REG_REVID);                 /* Read the ID register */
////	if (    (ADXL355_DEVICE_ID != ADXL355_SPI_Read(ADXL355_REG_DEVID_AD))
////	         || (ADXL355_MEMS_ID           != ADXL355_SPI_Read(ADXL355_REG_DEVID_MST))
////	         || (ADXL355_PART_ID           != ADXL355_SPI_Read(ADXL355_REG_PARTID)) )
////	    {
////	        ADXL355status = 1;
////	        char *mex="Error Initialization... \r\n";
////	        HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
////	    }
////	else{
////        char *mex="Successful Initialization... \r\n";
////        HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
////	}
//
//	if ((ui32test != 0xAD) && (ui32test2 != 0x1D) && (ui32test3 != 0xED) && (ui32test4 != 0x01)) {
//		  char *mex="Error Initializing...";
//		  char line[2] = "\r\n";
//		  HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
//		  HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
//			   //printf("\n\rReset and initialized.\n\r");
//			   //ADXL355_SPI_Write(0x2F, 0x52); //reset
//		   }
//		   else{
//			   char *mex="Reset  .............";
//			   char line[2] = "\r\n";
//			   HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
//			   HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
//			   //printf("Error initializing\n\r");
//		   }
////    //Configuring the Range
////    SPI_write(0x2C, 0x02);  /*b00000001=0x01 in 0x2C register (Interrupt Polarity, Range register)
////    sets a 4g range. The ADXL also uses 20 bit RES and stores data in the left-justified mode.*/
////
////    //Configuring the Power Control or POWER_CTL register:
////    SPI_write(ADXL355_REG_POWER_CTL, 0x06); /* enters measurement mode and disables temperature reading */
//
//	ADXL355_SPI_Write(ADXL355_REG_POWER_CTL, STANDBY);
//	uint32_t volatile ui32testSTANDBY = ADXL355_SPI_Read(ADXL355_REG_POWER_CTL);
//	if (STANDBY != ui32testSTANDBY) {
//		ADXL355status = 1;
//	}
//
//	ADXL355_SPI_Write(ADXL355_REG_FILTER, HPF_CORNER | ADXLsampling[OptimizedSPS].ODR_LPF_setting);
//	ADXL355_SPI_Write(ADXL355_REG_RANGE, I2C_HS | G2_RANGE);    // Same as default.
//	// ToDo: Explicitly set all other registers.
//	//       Perhaps develop function for multi-byte writes and upload a table.
//	ADXL355_SPI_Write(ADXL355_REG_POWER_CTL, MEASUREMENT_MODE);
//    char *mex="done function......";
//    char line[2] = "\r\n";
//    HAL_UART_Transmit(&huart2, (uint8_t *) mex, 15, 10);
//    HAL_UART_Transmit(&huart2, (uint8_t *) line, 2, 10);
//    return ADXL355status;
//}
//
//
//uint32_t ReadADXL355acceleration(const eAxis_t AxisNum){
//    uint8_t BaseRegister;   // Intentionally Uninitialized.
//    ADXL_axis_t AxisData;   // 20-bit analog to digital conversions via 3 register reads.
//
////#if false && DEBUG_ADXL
////    AxisData.Everything = 0;
////#endif
//
//    BaseRegister = eXDATA3 + (3 * AxisNum);
//    AxisData.Data3 = ADXL355_SPI_Read(BaseRegister + 0); // Bits[19:12]
//    AxisData.Data2 = ADXL355_SPI_Read(BaseRegister + 1); // Bits[11:4]
//    AxisData.Data1 = ADXL355_SPI_Read(BaseRegister + 2); // Bits[3:0], first four LSBs reserved!
//
//    // Extend two's complement from 20 bit ADC to 32 bit register.
//    if (AxisData.Data3 & (1 << 7)) AxisData.SignBits = 0xFF;
//    else                           AxisData.SignBits = 0x00;
//
//    return AxisData.Acceleration;
//}
