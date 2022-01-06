/*
 * CustomTypes.h
 *
 *  Created on: Dec 8, 2021
 *      Author: avalsangkar
 */

#ifndef INC_CUSTOMTYPES_H_
#define INC_CUSTOMTYPES_H_
#include "GeoLibrary.h"
#include "MEMSdevice.h"
#include "math.h"

static const char IPI_15degreeModel[] = "6150E\0";
static const char IPI_30degreeModel[] = "6151E\0";

/*! \typedef eModel_t
 *  Alternate IPI models
 */
typedef enum {
    Model6150E, // Original uses 15-degree SCA103T-D04 MEMS IC.
    Model6151E, // Variant with 30-degree SCA103T-D05 MEMS IC.
} eModel_t;

#define COMPILE_ANALYTICS         (true && defined _DEBUG)// Compile time consuming math for analysis.
#define HOLD_ERRORS               (true && defined _DEBUG)// Preserve certain errors until acknowledged.
#define APPLY_CALIBRATION         (true)
#define POLYNOMIAL_CALIBRATION    (true && APPLY_CALIBRATION)

/*! \brief  Structure to return data from multi-sample routine.
 *          32 bits can hold 2^31 / 2^23 = 2^8 samples at max or min.
 *          2^63 >= ((2^23)^2 * DataCount) if DataCount <= 2^17. Sum of squares will fit.
 */
typedef struct
{
    bool            ReadOK;             // All ADC samples acquired legitimately?
    int_fast16_t    Count;              // Quantity of samples in fixed period
    int_fast64_t    ADCsum;             // Sum of samples taken during period
    int_fast64_t    ADCsum_of_squares;  // For development statistics.
} multisample_t;


/*! \typedef eAxis_t
 *  Represents the actual axis.
 */
typedef enum {
    aAxis = 0,
    bAxis = 1,
    cAxis = 2,
    xAxis = 0,  // Alternate designation to axis A.
    yAxis = 1,
    zAxis = 2,
} eAxis_t;

/*! \typedef eAxisTranspose_t
 *  Translate internal orientation to an alternate external orientation.
 *  Use when sensor mounting differs from legacy significance of outside features.
 *
 *  \note Six transpositions are possible for three axes (including null).
 *        One transposition is possible for two axes.
 */
typedef enum {
    NoChange,   // A' = A, B' = B, C' = C
    SwapAB,     // A' = B, B' = A, C' = C
    SwapAC,     // A' = C, B' = B, C' = A
    SwapBC,     // A' = A, B' = C, C' = B
    RollABC,    // A' = C, B' = A, C' = B
    RollACB     // A' = B, B' = C, C' = A
} eAxisTranspose_t;

/*! \typedef AxisInvert_t
 *  Bitfield used to record which output angles to negate.
 *
 *  \note   Transposition will be applied before inversion!
 */
typedef union {
    struct {    // Geokon legacy labels.
        uint8_t InvertAxisA: 1;
        uint8_t InvertAxisB: 1;
        uint8_t InvertAxisC: 1;
        uint8_t ReservedABC: 5;
    };  // Anonymous structures are easier to reference.
    struct {    // Traditional trigonometry labels.
        uint8_t InvertAxisX: 1;
        uint8_t InvertAxisY: 1;
        uint8_t InvertAxisZ: 1;
        uint8_t ReservedXYZ: 5;
    };
    uint8_t     AsByte;
} AxisInvert_t;

/*! \typedef AngleRemap_t
 *  Encodes all possible permutations of inversion and transposition in 16-bits.
 */
typedef union {
    struct {
        AxisInvert_t    Invert;
        uint8_t         Transpose;  // Don't use eAxisTranspose_t here. Enumerations are 16 bits long.
    };  // Anonymous structures are easier to reference.
    uint16_t    AsWord;
    uint8_t     AsBytes[2];
} AngleRemap_t;

static const AngleRemap_t GeoNetMEMSorientation = { // A.k.a. "MEMS in a box"
    .Invert.InvertAxisB = true,
    .Transpose = RollACB
};

static const AngleRemap_t IPIorientation = { // 6150-14 for Project 1907
    .Invert.InvertAxisB = true,
    .Transpose = SwapAC
};

/*! \var AxisTranslationTable
 * Axis translation lookup table
 * Logical axes obtained by indexing across by physical axis
 *  and by indexing down by translation type.
 */
static const eAxis_t AxisTranslationTable[6][3] = {
    {aAxis, bAxis, cAxis},  // NoChange: A' = A, B' = B, C' = C
    {bAxis, aAxis, cAxis},  // SwapAB:   A' = B, B' = A, C' = C
    {cAxis, bAxis, aAxis},  // SwapAC:   A' = C, B' = B, C' = A
    {aAxis, cAxis, bAxis},  // SwapBC:   A' = A, B' = C, C' = B
    {cAxis, aAxis, bAxis},  // RollABC:  A' = C, B' = A, C' = B
    {bAxis, cAxis, aAxis}   // RollACB:  A' = B, B' = C, C' = A
};


typedef struct {
    float Factor[3];    // Axis calibration factor, 3 axes
} CalCoefficient_t;


// Computed in ADXL355stuckAxes.m from five 6150-14 samples:
//  6150-14 error threshold: 1.008 +/- 0.05569 g
// Changed to 1.008 +/- 0.2g in version 2.7.
#define G_FORCE_THRESHOLD   (1.208f)  // 1.208 g

// Sample size, full scale and tolerable noise were used to determine limit.
//  The 0.70° (180° / 2^8) threshold corresponds to 48 dB SNR.
//   Compilation has become too convoluted to use FULL_SCALE_20BIT_SIGNED_ADC here.
#define TWO_PASS_THRESHOLD  (((uint32_t)1 << 20) >> 8)  // 2 * FULL_SCALE_20BIT_SIGNED_ADC / 2^8

// Final revision of S-6150-7, as made from June 2017 through early 2021.
// static const Version_t Final_S_6150_7_HardwareVersion = {.Major = 5, .Minor = 0};
// S-6150-13 Rev. A (card) or S-6150-14 Rev. 1 (insert) use new MEMS and temperature sensors.
//  Per Isaiah's 9/14/2020 email "Proposal: PCBA vs. Hardware Revision Scheme in Addressable Temperature Sensor."
static const Version_t Current_S_6150_13_14_HardwareVersion = {.Major = 3, .Minor = 0}; // Rev. C.

// \\Geodata\sys\Quality System Docs\Dept Folders & Assembly & Test Procedures\Engineering\TECHMEMO\TM_20-26.xlsx
static const uint8_t adxl355IPItype[] = { '1', '6', '0', '8', 0, 0 };   // 6180


// enumeration for various baud rates
typedef enum{
    b300=1,         //Begin enumeration at 1 to allow blank memory (0) to be recognized as blank
    b600,
    b1200,
    b2400,
    b4800,
    b9600,
    b14400,
    b19200,
    b28800,
    b38400,
    b57600,
    b115200,
    b230400
} standardBaudrates_t;

typedef struct{
    float coef[5];
} coef_t;

typedef union flash {
    struct
    {
        uint16_t address;
        uint8_t  sensorType[16];

        uint32_t serialNumber;
        Version_t softwareVersion;
        Version_t hardwareVersion;

        // Linear coefficients are IEEE 754 floating point triads.
        CalCoefficient_t calZeroth; // Degrees
        CalCoefficient_t calFirst;  // Degrees/degree (unitless)

        AngleRemap_t Orientation;   // Internal use only.

        // reserved locations
        uint16_t reserved[4];
        uint16_t measureWait;   // Maximum cycle time (ms) between trigger and data post.
        uint16_t baud;
        // -------------------- // April 2021 -- Appended remainder of flash segment.
        uint16_t broadcast_delay;   // Time in milliseconds.
        float gForceLimit;
        uint32_t twoPassLimit;
        ErrorCode_t persistMask;    // Keep reporting select errors until acknowledged.
        ErrorCode_t lastError;      // Cache unique errors for debugging.
        // Polynomial coefficients are IEEE 754 floating point triads.
        CalCoefficient_t calSecond; // Degrees^2/degree
        CalCoefficient_t calThird;  // Degrees^3/degree
        // Reserve higher order factors for future use.
        //  It's unlikely they'll be desired, but keep them contiguous.
        CalCoefficient_t calFourth; // Degrees^4/degree
        CalCoefficient_t calFifth;  // Degrees^5/degree
        // Polynomial factors may distort readings past calibration set points.
        //  Provide limit to prevent absurd output.
        uint16_t angleRangeLimit;   // Degrees (absolute value)
        // new, better coefficient table
        coef_t AA; // 20 bytes
        coef_t BA; // 20 bytes
        coef_t CA; // 20 bytes
        coef_t TA;
        coef_t TB;
        coef_t TC;
        uint16_t annex[256 - (32 + 32 + 20 + 20 + 20)]; // For future use
    };  // Anonymous structures are easier to reference.
    uint8_t  AsBytes[512];      // SEGMENT_SIZE_BYTES
    uint16_t AsWords[256];      // SEGMENT_SIZE_WORDS
} flashTable_t;

static const flashTable_t DefaultParameters =
{
     .address         =   1 ,
     .sensorType      = { '1', '6', '0', '5', 0, 'E', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  // 6150E
     .serialNumber    =   0 ,
     .softwareVersion = { .Major = 2, .Minor = 12 },
     .hardwareVersion = { .Major = 5, .Minor = 0},
     .calZeroth       = { 0.0f, 0.0f, 0.0f },   // Default: 0.0 degrees.
     .calFirst        = { 1.0f, 1.0f, 1.0f },   // Default: 1.0 degrees/degree.
     .Orientation     = {
         .Transpose = (uint8_t) NoChange,
         .Invert.InvertAxisA = false,
         .Invert.InvertAxisB = false,
         .Invert.InvertAxisC = false,
     },
     .reserved        = {0, },
     .measureWait     = MEASURE_WAIT_SCA103T,    // Default
     .baud            = b115200,     // Default 115.2k
     // -------------------- // April 2021 -- Appended remainder of flash segment.
     .broadcast_delay = FIXED_BROADCAST_DELAY,
     .gForceLimit     = G_FORCE_THRESHOLD,
     .twoPassLimit    = TWO_PASS_THRESHOLD,
     .persistMask     = // Don't clear automatically.
     {
#if HOLD_ERRORS // For evaluation.
      .WatchdogReset  = 1,
      .Configuration  = 1
#else   // !HOLD_ERRORS // For release.
      .CompleteCode   = 0   // Don't allow any errors to persist.
#endif  // HOLD_ERRORS
     },
     .lastError       = { .CompleteCode = 0 },
     // Populate with zeros for linear calibration.
     .calSecond       = { 0.0f, 0.0f, 0.0f },
     .calThird        = { 0.0f, 0.0f, 0.0f },
     .calFourth       = { 0.0f, 0.0f, 0.0f },
     .calFifth        = { 0.0f, 0.0f, 0.0f },
     .angleRangeLimit = 90, // Initial value must not impede calibration.
     .AA = { 1E-9, 2E-8, 3E-7, 4E-6, NAN},
     .BA = { 9.0f, 8.0f, 7.0f, 6.0f, NAN},
     .CA = { 9.0f, 8.0f, 7.0f, 6.0f, NAN},
     .TA = { 6E-4, 7E-3, 8E-2, 1.0f, NAN},
     .TB = { 4.0f, 3.0f, 2.0f, 1.0f, NAN},
     .TC = { 4.0f, 3.0f, 2.0f, 1.0f, NAN},
     .annex =
     {
      // Bytes 128 to 191
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      // Bytes 192 to 255
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,

      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,

      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
      0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
     }
};

typedef struct ram
{
    // Primary readings
    float aAxis;            // Degrees, 4-byte IEEE 754 floating point
    float bAxis;
    float cAxis;
    float temperature;      // Celsius

    // Raw sensor outputs
    float xAxis;            // Degrees, uncompensated for temperature
    float yAxis;
    float zAxis;
    uint16_t ThermistorADC; // Thermistor (PR103J2) ADC value.
    uint16_t heading;       // Degrees (X10)
    int16_t SNRdB[3];       // Signal-to-noise ratio, in decibels
    float gForce;
    int16_t sampleSize;
//    uint16_t reserved_RO;   // 2 bytes reserved.
    errorCode2_t ec2;
    ErrorCode_t errorCode;  // Error code from latest measurement cycle.
    uint16_t trigger;       // Non-zero value initiates measurement cycle.
    uint32_t password;      // FLASH_PASSWORD allows subsequent write to flash.
    uint16_t measureCycle;  // Measurement number since device energized.
    int16_t coreTemperature; // MSP430 internal sensor (±3°C)
    // Reserved locations
    uint16_t reserved_RW[3]; // 6 bytes reserved.
                             // 64 bytes / 32 registers
} ramTable_t;
#endif /* INC_CUSTOMTYPES_H_ */
