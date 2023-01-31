/***The Following Library is for Romina Mir's Inspiring work on the tendon driven Servo Hand**/
/***This is For the Distal(The Furthest) Region of the Finger. Here we use XC-330 M181-T***/
/***Given Angle Limits -90(270) to 0  HI THIS A TEST***/
#include <DynamixelShield.h> // for DXL_DIR_PIN definition
#include <Dynamixel2Arduino.h> // For Dynamixel2Arduino Class definition.
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
const uint16_t DXL_M288_MAX_POS_LIMIT = 1023;

const uint16_t DXL_M181_MIN_POS_LIMIT = 2045; // Not compensated, technically this is 180 degrees. But in our frame of reference, this is our 0.
const uint16_t DXL_M181_MAX_POS_LIMIT = 3068; // Not compensated, technically this is 270 degrees. But in our frame of reference, this is our 90.
#define HOST_COMMUNICATION_BAUDRATE 921600
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
                            {2, DXL_M288, reverseDirection},
                            {3, DXL_M288,  normalDirection},
                            {4, DXL_M181,  normalDirection},
                            {5, DXL_M288,  normalDirection},
                            {6, DXL_M181,  normalDirection}};


bool _setPosition(Dynamixel2Arduino *obj, uint8_t motorId, float desiredAngle){
        obj->setGoalPosition(motorId, desiredAngle, UNIT_DEGREE);
        obj->writeControlTableItem(PROFILE_VELOCITY, motorId, 1000);
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

bool MotorControl_SetPosition(Dynamixel2Arduino *obj, uint8_t motorId, float desiredAngle){
  hardwareErrorStatus_t hardwareErrorStatus;
  hardwareErrorStatus.all = 0;
  if(!_setPosition(obj, motorId, desiredAngle))
  {
    Serial1.println("Unsuccessfully set position");
    // this should be a function call.
    hardwareErrorStatus.all = obj->readControlTableItem(HARDWARE_ERROR_STATUS, motorId); // read HW error status register
    // Motor didn't work, let's find out why.

    // Maybe the hardware error status register can tell us.
    if(hardwareErrorStatus.all)
    {
      // Yes, there was an error - print it out and reboot the motor.
      Serial1.print(hardwareErrorStatus.all, HEX);
      // Reboot if we detect an input voltage error.
      if(!obj->reboot(motorId))
      {
        // Uh oh! This is bad.
        Serial1.println("Reboot failed");
      }
      return; // This caused the problem. No need to check further.
    }

    // Check to see if the motor exists!
    if(!obj->ping(motorId))
    {
      // The motor isn't connected :) - just plug it back in.
      Serial1.println("Motor was disconnected");
      return;
    }
  }
}


bool setupMotor(Dynamixel2Arduino *obj, uint8_t motorId, OperatingMode opMode, motorDirection_e direction){
  bool success = false;
  success = obj->torqueOff(motorId);
  success &= obj->setOperatingMode(motorId, opMode); // EEPROM AREA
  success &= MotorControl_WriteControlTable(obj, DRIVE_MODE, motorId, (uint8_t) direction); // EEPROM AREA
  success &= obj->torqueOn(motorId);

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

  if(DXL_M181 == motors[motorIdx].type)
  {
    MotorControl_WriteControlTable(obj, PROFILE_VELOCITY, motors[motorIdx].motorId, DXL_M181_MAX_VELOCITY);
    MotorControl_WriteControlTable(&dxl, MAX_POSITION_LIMIT, motors[motorIdx].motorId, DXL_M181_MAX_POS_LIMIT);
    MotorControl_WriteControlTable(&dxl, MIN_POSITION_LIMIT, motors[motorIdx].motorId, DXL_M181_MIN_POS_LIMIT);
  }
  else if(DXL_M288 == motors[motorIdx].type)
  {
    MotorControl_WriteControlTable(obj, PROFILE_VELOCITY, motors[motorIdx].motorId, DXL_M288_MAX_VELOCITY);
    MotorControl_WriteControlTable(&dxl, MAX_POSITION_LIMIT, motors[motorIdx].motorId, DXL_M288_MAX_POS_LIMIT);
    MotorControl_WriteControlTable(&dxl, MIN_POSITION_LIMIT, motors[motorIdx].motorId, DXL_M288_MIN_POS_LIMIT);    
  }

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
}

void loop() {

  Serial1.println("Setting proximal to 0");
  MotorControl_SetPosition(&dxl, 2, 0.0);
  MotorControl_SetPosition(&dxl, 3, 0.0);
  MotorControl_SetPosition(&dxl, 5, 0.0); 
  delay(2000);

  Serial1.println("Setting distal to 0");
  MotorControl_SetPosition(&dxl, 1, 180); // Look on the distal motor, it is marked "180" on what we want to be the 0 position. todo - Make 180 degrees appear to be 0.
  MotorControl_SetPosition(&dxl, 4, 180);
  MotorControl_SetPosition(&dxl, 5, 180);
  delay(2000);

  Serial1.println("Setting distal to 90");
  MotorControl_SetPosition(&dxl, 1, 270); //    
  MotorControl_SetPosition(&dxl, 4, 270);
  MotorControl_SetPosition(&dxl, 5, 270);
  delay(2000);


  Serial1.println("Setting distal to 0");
  MotorControl_SetPosition(&dxl, 1, 180); // Look on the distal motor, it is marked "180" on what we want to be the 0 position. todo - Make 180 degrees appear to be 0.
  MotorControl_SetPosition(&dxl, 4, 180);
  MotorControl_SetPosition(&dxl, 5, 180);
  delay(2000);

  Serial1.println("Setting proximal to 0");
  MotorControl_SetPosition(&dxl, 2, 90.0);  
  MotorControl_SetPosition(&dxl, 3, 90.0);
  MotorControl_SetPosition(&dxl, 5, 90.0);
  delay(2000);
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