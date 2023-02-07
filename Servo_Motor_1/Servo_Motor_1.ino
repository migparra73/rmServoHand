/***The Following Library is for Romina Mir's Inspiring work on the tendon driven Servo Hand**/
/***This is For the Distal(The Furthest) Region of the Finger. Here we use XC-330 M181-T***/
/***Given Angle Limits -90(270) to 0  HI THIS A TEST***/
#include <DynamixelShield.h> // for DXL_DIR_PIN definition
#include <Dynamixel2Arduino.h> // For Dynamixel2Arduino Class definition.
#include <assert.h>
using namespace ControlTableItem;

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
      uint8_t unused3 :  2; // MSB
    } bits;
    uint8_t all;
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

Dynamixel2Arduino dxl(Serial, DXL_DIR_PIN); // When initializing the dxl class, you need to feed in the Serial port and direction pin you are going to use.

typedef struct
{
  uint8_t motorId;
  motorType_e type;
  motorDirection_e direction;
} motorValues_t;

motorValues_t motors[6] = { {1, DXL_M181, reverseDirection},
                            {2, DXL_M288, normalDirection},
                            {3, DXL_M288,  normalDirection},
                            {4, DXL_M181,  normalDirection},
                            {5, DXL_M288,  normalDirection},
                            {6, DXL_M181,  normalDirection}};


bool _setPosition(Dynamixel2Arduino *obj, uint8_t motorId, float desiredAngle)
{
  return obj->setGoalPosition(motorId, desiredAngle, UNIT_DEGREE);
}

#define END_OF_EEPROM_AREA_REGISTERS 64 // this tells us which register is the end of the eeprom area.
// This function will make sure that we can write safely to the control table, based on if the torque enable bit is true.
bool MotorControl_WriteControlTable(Dynamixel2Arduino *obj, uint8_t item_idx, uint8_t motorId, uint32_t data)
{
  bool torqueEnable = obj->getTorqueEnableStat(motorId);
  if (item_idx < END_OF_EEPROM_AREA_REGISTERS)
  {
    if(torqueEnable)
    {
      assert(0); // Die here.
      Serial1.println("Torque enabled, value not programmed.");
      return false;
    }
    else
    {
      obj->writeControlTableItem(item_idx,motorId, data);
      return true;
    }
  }
  else
  {
      // Value is greater, can always write.
      obj->writeControlTableItem(item_idx,motorId, data);
      return true;    
  }
}

bool MotorControl_SetPosition(Dynamixel2Arduino *obj, uint8_t motorId, float desiredAngle, hardwareErrorStatus_t *hardwareErrorStatus){
  hardwareErrorStatus->all = 0;
  if(!_setPosition(obj, motorId, desiredAngle))
  {
    // this should be a function call.
    delay(1000);
    hardwareErrorStatus->all = obj->readControlTableItem(HARDWARE_ERROR_STATUS, motorId); // read HW error status register
    // Motor didn't work, let's find out why.
    Serial1.write((char *)hardwareErrorStatus->all, sizeof(uint8_t));
    return false;
  }
  return true;
}


bool setupMotor(Dynamixel2Arduino *obj, uint8_t motorIdx, OperatingMode opMode, motorDirection_e direction)
{
  bool success = false;
  success = obj->torqueOff(motors[motorIdx].motorId);
  success &= obj->setOperatingMode(motors[motorIdx].motorId, opMode); // EEPROM AREA
  success &= MotorControl_WriteControlTable(obj, DRIVE_MODE, motors[motorIdx].motorId, (uint8_t) direction); // EEPROM AREA
  success &= MotorControl_WriteControlTable(obj, HOMING_OFFSET, motors[motorIdx].motorId, 0);
  if(DXL_M181 == motors[motorIdx].type)
  {
    MotorControl_WriteControlTable(&dxl, MAX_POSITION_LIMIT, motors[motorIdx].motorId, DXL_M181_MAX_POS_LIMIT);
    MotorControl_WriteControlTable(&dxl, MIN_POSITION_LIMIT, motors[motorIdx].motorId, DXL_M181_MIN_POS_LIMIT);
  }
  else if(DXL_M288 == motors[motorIdx].type)
  {
    MotorControl_WriteControlTable(&dxl, MAX_POSITION_LIMIT, motors[motorIdx].motorId, DXL_M288_MAX_POS_LIMIT);
    MotorControl_WriteControlTable(&dxl, MIN_POSITION_LIMIT, motors[motorIdx].motorId, DXL_M288_MIN_POS_LIMIT);    
  }
  success &= obj->torqueOn(motors[motorIdx].motorId);
  return success;
}

bool checkIfMotorAlive(Dynamixel2Arduino *obj, uint8_t motorId)
{
  // Check to see if the Dynamixel is alive
  if(obj->ping(motorId))
  {
    Serial1.println("Dynamixel detected");
  }
  else
  {
    Serial1.println("Not detected");
  }
}

bool MotorControl_Init(Dynamixel2Arduino *obj, uint8_t motorIdx)
{  
  checkIfMotorAlive(&dxl, motors[motorIdx].motorId);
  setupMotor(obj, motors[motorIdx].motorId, OP_POSITION, motors[motorIdx].direction); // Solo finger (Motor 1)
}

void setup() {
  const int numberOfMotors = (sizeof(motors) / sizeof(motorValues_t));
  Serial1.begin(HOST_COMMUNICATION_BAUDRATE); // Sets up the external serial port (USB dongle thing)
  dxl.begin(DYNAMIXEL_COMMUNICATION_BAUDRATE); //  Sets up communication with Dynamixel servos 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  


  for(int i = 0; i < numberOfMotors; i++)
  {
    MotorControl_Init(&dxl, i);
  }
 
  hardwareErrorStatus_t hwStatus;
  // MotorControl_SetPosition(&dxl, 5, 270.0, &hwStatus);
    
}



void loop() {

  float desiredAngle = 0.0; // apparently float and double have the same size on arduino, 4 bytes.
  uint32_t motorId = 2;
  hardwareErrorStatus_t hwStatus;
  int i;

  char arr2[5] = {'a'};

  if(Serial1.readBytes((char *) &desiredAngle, sizeof(desiredAngle)))
  {
    Serial1.println("Setting motor to desired position.");
    Serial1.write((char*)&desiredAngle, sizeof(desiredAngle));
    Serial1.println("");

    if(MotorControl_SetPosition(&dxl, motorId, desiredAngle, &hwStatus))
    {
      Serial1.println("Success!");
    }
    else
    {
      if(hwStatus.bits.electricalShock)
      {
        Serial1.println("Fail: Electrical shock detected");
      }
      else if(hwStatus.bits.overHeating)
      {
        Serial1.println("Fail: Overheating detected");        
      }
      else if(hwStatus.bits.overloadError)
      {
        Serial1.println("Fail: Overload error detected");        
      }
      else if(hwStatus.bits.voltageError)
      {
        Serial1.println("Fail: Voltage error detected");        
      }
      else
      {
        Serial1.println("Fail: Unknown error occured.");                
      }
      dxl.reboot(motorId);
    }
  }
}

/**
// the setup function runs once when you press reset or power the board
void setup2() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(921600);
}

// the loop function runs over and over again forever
void loop2() {
  float number = 4.0; // apparently float and double have the same size on arduino, 4 bytes.
  float array[16] = {0.0};
  int i;
  for (i=0; i<16; i++)
  {
    number += 10;
    array[i] = number;
  }
  Serial.write((uint8_t *)&array, sizeof(number)*16);
  char arr2[5] = {'a'};

  Serial.readBytes(arr2, 5);
  Serial.write(arr2, 5);
}
*/