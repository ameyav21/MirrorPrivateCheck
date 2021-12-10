/*
 * MEMSdevice.h
 *
 *  Created on: Dec 8, 2021
 *      Author: avalsangkar
 */

#ifndef INC_MEMSDEVICE_H_
#define INC_MEMSDEVICE_H_
#include "GeoLibrary.h"

typedef enum {
    SCA103T = 0,    // Two separate Murata (VTI) ICs
    ADXL355  = 1    // Single 3-axis Analog Devices IC
} eMEMSdeviceType;


// Total cycle time for angles and temperature using digital components.
#define MEASURE_WAIT_SCA103T    (321) // milliseconds
// ANALOG_POWER_DELAY + ADXLsamplePeriod + SI705X_RESET_POWERUP_TIME + k88msSampleSize
//  = 10 + 200 + 15 + 88 = 311.4 ms
//  285 milliseconds measured on scope with ~7 ms delays between Si7051 readings.
//   Pad because temperatures can take as much as 10.8 ms (See k44msSampleSize comments.)
//    285 + (8 * (10.8 - 7)) = 315.4
//  Added 16 milliseconds to account for potential TestADXL355ShadowRegisters iterations.
#define MEASURE_WAIT_ADXL355    (332) // milliseconds
static const uint16_t MeasureWaitADXL355 = MEASURE_WAIT_ADXL355;


eMEMSdeviceType AutoDetectMEMStype(void);
void IO_Reconfigure(eMEMSdeviceType DetectedMEMStype);
void InitializeInterfaces(eMEMSdeviceType DetectedMEMStype);
void EnergizeDevices(eMEMSdeviceType DetectedMEMStype);
void DeEnergizeDevices(eMEMSdeviceType DetectedMEMStype);


#endif /* INC_MEMSDEVICE_H_ */
