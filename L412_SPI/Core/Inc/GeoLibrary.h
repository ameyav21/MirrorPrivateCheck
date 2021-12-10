/*
 * GeoLibrary.h
 *
 *  Created on: Dec 8, 2021
 *      Author: avalsangkar
 */

#ifndef INC_GEOLIBRARY_H_
#define INC_GEOLIBRARY_H_
#include <stdint.h>

/*! \def FIXED_BROADCAST_DELAY
 * The delay multiplied by the number of devices, plus measurement cycle time,
 * is the minimum on time for the string.
 * The measurement cycle time divided by delay is number of drops drawing power
 * simultaneously.
 * For example, assuming a measurement cycle time of 321 ms, as last observed,
 * and a 50 ms delay, six or seven MEMS devices would be measuring at any
 * time, and a string of 16 MEMS devices (addressed 1 to 51) would require less
 * than 1.1 seconds for all to finish measuring.
 * Each drop consumes a maximum of 25 mA when measuring and 1.2 mA in standby.
 * The string current for seven active and 9 idle MEMS would be 186 mA.
 */
#define FIXED_BROADCAST_DELAY  (50) // milliseconds between consecutive server addresses.


static const uint16_t ErrorAcknowledgeTrigger = 0xA5;   // Arbitrary key.


// Transcribed verbatim from S-8020 Addressable VW project.
typedef union {
    uint16_t    AsWord;
    uint8_t     AsBytes[2];
    char        LetterPair[2]; // For " A" or "AA"
    struct {
        uint8_t Minor;
        uint8_t Major;
    };  // Anonymous structures are easier to reference.
} Version_t;



#define NO_FAULT 0
#define    FAULT 1


// Transcribed addressable error union,
//  universal to MEMS, thermistor, analog and vibrating wire.
typedef union {
    uint16_t CompleteCode;
    struct {
        // The first four error bits have been published in
        //  the 6150E manual, so should be preserved.
        uint16_t SensorResponse     : 1;
        uint16_t TempSensorFault    : 1;
        uint16_t TempSensorVerify   : 1;
        uint16_t WatchdogReset      : 1;
        // The remaining error bits have not yet been introduced.
        uint16_t MetaDataError      : 1;
        uint16_t CompassError       : 1;
        uint16_t OverrangeError     : 1;
        uint16_t BumpOrShakeError   : 1; // Shock or vibration detected.
        uint16_t ExcitationError    : 1;
        uint16_t Reserved           : 5;
        // Use Configuration bit to indicate any initialization faults.
        uint16_t Configuration      : 1;
        uint16_t UndefinedError     : 1;
    };
} ErrorCode_t;

typedef union {
    uint16_t asWord;
    struct {
        // The first four error bits have been published in
        //  the 6150E manual, so should be preserved.
        uint16_t readTimeout        : 1;
        uint16_t duplicateValues    : 1;
        uint16_t reset              : 1;
        uint16_t readADXLreg        : 1;
        // The remaining error bits have not yet been introduced.
        uint16_t writeADXLreg       : 1;
        uint16_t secondPass         : 1;
        uint16_t unused             : 10;
    };
} errorCode2_t;


#endif /* INC_GEOLIBRARY_H_ */
