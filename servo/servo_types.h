#pragma once

const uint8_t DXL_ID = 2; // remove eventually, because we have to control several motors
const float DXL_PROTOCOL_VERSION = 2.0;
const float DXL_MIN_ANGLE = 0; 
const float DXL_MAX_ANGLE = 90;
const float DXL_MIN_VEL = 1; 
const float DXL_MAX_VEL = 229; // rpm
//const float DXL_GOAL_ANGLE = ;
const float DXL_PEAK_VELOCITY = 2.0;
const uint16_t DXL_M288_MAX_VELOCITY = 354; // To convert this number to RPM, multiply it by 0.229
const uint16_t DXL_M181_MAX_VELOCITY = 559; // To convert this number to RPM, multiply it by 0.229

const uint16_t DXL_M288_MIN_POS_LIMIT = 0;
const uint16_t DXL_M288_MAX_POS_LIMIT = 4095;

const uint16_t DXL_M181_MIN_POS_LIMIT = 0; // Not compensated, technically this is 180 degrees. But in our frame of reference, this is our 0.
const uint16_t DXL_M181_MAX_POS_LIMIT = 4095; // Not compensated, technically this is 270 degrees. But in our frame of reference, this is our 90.
#define HOST_COMMUNICATION_BAUDRATE 9600
#define DYNAMIXEL_COMMUNICATION_BAUDRATE 57600
#define NUMBER_OF_MOTORS 6

//8 bit bitfield, with an "all"
typedef struct
{
  union
  {
    struct
    {
      uint8_t voltageError : 1; // LSB
      uint8_t unused1 : 1;
      uint8_t overHeating : 1;
      uint8_t unused2 : 1;
      uint8_t electricalShock : 1;
      uint8_t overloadError : 1;
      uint32_t unused3 :  26; // MSB
    } bits;
    uint32_t all;
  };
} hardwareErrorStatus_t;

typedef enum
{
  normalDirection = 0x00,
  reverseDirection = 0x01
} motorDirection_e;

typedef enum
{
  DXL_M181,
  DXL_M288
} motorType_e;

typedef struct
{
  uint8_t motorName;
  motorType_e type;
  motorDirection_e direction;
} motorValues_t;

typedef struct
{
  float homeAngle;
  float straightAngle;
  float bentOutAngle;
} motorTranslationValues_t;

typedef struct __attribute__((packed))
{
  uint8_t motorName;
  float desiredPosition;
  float trajectoryVelocity;
} dataPacket_t;

static_assert(sizeof(dataPacket_t) == 9, "dataPacket_t is not 9 bytes!");