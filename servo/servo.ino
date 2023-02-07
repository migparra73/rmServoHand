// This is done to make sure we have the best code possible. Warnings => bugs later on.
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wpedantic"

#include <DynamixelShield.h> // for DXL_DIR_PIN definition
#include <Dynamixel2Arduino.h> // For Dynamixel2Arduino Class definition.
#include <assert.h>
#include "servo_api.h"
#include "servo_types.h"
using namespace ControlTableItem;

Dynamixel2Arduino dxl(Serial, DXL_DIR_PIN); // When initializing the dxl class, you need to feed in the Serial port and direction pin you are going to use.

motorValues_t motors[NUMBER_OF_MOTORS] = 
                          { {1, DXL_M181,  normalDirection},
                            {2, DXL_M288,  normalDirection},
                            {3, DXL_M288,  normalDirection},
                            {4, DXL_M181,  normalDirection},
                            {5, DXL_M288,  normalDirection},
                            {6, DXL_M181,  normalDirection}};


motorTranslationValues_t translations[NUMBER_OF_MOTORS] = {
  {180.0, 90.0, 0.0},
  {180.0, 270.0, 360.0},
  {360.0, 270.0, 180.0},
  {270.0, 180.0, 90.0},
  {180.0, 270.0, 360.0},
  {90.0, 180.0, 270.0}};


float translateAngle(uint8_t tableIdx, float desiredAngle)
{
  float realAngle = 0.0;

  // Example is motor #3
  // If home is 360 and out is 180, that means that we should map 0 -> 360 and 180 to 180.
  if(translations[tableIdx].homeAngle > translations[tableIdx].bentOutAngle)
  {
    realAngle = (-0.5*(desiredAngle)) + translations[tableIdx].homeAngle;
  }
  else
  {
    realAngle = (0.5*(desiredAngle)) + translations[tableIdx].homeAngle;
  }
  return realAngle;
}


bool _setPosition(Dynamixel2Arduino *obj, uint8_t motorName, float desiredAngle)
{
  return obj->setGoalPosition(motorName, desiredAngle, UNIT_DEGREE);
}

#define END_OF_EEPROM_AREA_REGISTERS 64 // this tells us which register is the end of the eeprom area.
// This function will make sure that we can write safely to the control table, based on if the torque enable bit is true.
bool MotorControl_WriteControlTable(Dynamixel2Arduino *obj, uint8_t item_idx, uint8_t motorName, uint32_t data)
{
  bool torqueEnable = obj->getTorqueEnableStat(motorName);
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
      obj->writeControlTableItem(item_idx,motorName, data);
      return true;
    }
  }
  else
  {
      // Value is greater, can always write.
      obj->writeControlTableItem(item_idx,motorName, data);
      return true;    
  }
}


// Top level handler for handling indexes instead of Ids
bool MotorControl_SetPositionWrapper(Dynamixel2Arduino *obj, uint8_t tableIdx, float desiredAngle, hardwareErrorStatus_t *hardwareErrorStatus)
{
  float actualAngle = 0.0;
  actualAngle = translateAngle(tableIdx, desiredAngle);
  return MotorControl_SetPosition(obj, motors[tableIdx].motorName, actualAngle, hardwareErrorStatus);
}

bool MotorControl_SetPosition(Dynamixel2Arduino *obj, uint8_t motorName, float desiredAngle, hardwareErrorStatus_t *hardwareErrorStatus){
  if(!_setPosition(obj, motorName, desiredAngle))
  {
    // this should be a function call.
    delay(1000);
    hardwareErrorStatus->all = obj->readControlTableItem(HARDWARE_ERROR_STATUS, motorName); // read HW error status register
    // Motor didn't work, let's find out why.
    Serial1.write((char *)hardwareErrorStatus->all, sizeof(uint8_t));
    return false;
  }
  return true;
}


bool setupMotor(Dynamixel2Arduino *obj, uint8_t motorName, OperatingMode opMode, motorDirection_e direction, motorType_e motorType)
{
  bool success = false;
  success = obj->torqueOff(motorName);
  success &= obj->setOperatingMode(motorName, opMode); // EEPROM AREA
  success &= MotorControl_WriteControlTable(obj, DRIVE_MODE, motorName, (uint8_t) direction); // EEPROM AREA
  success &= MotorControl_WriteControlTable(obj, HOMING_OFFSET, motorName, 0);
  if(DXL_M181 == motorType)
  {
    MotorControl_WriteControlTable(&dxl, MAX_POSITION_LIMIT, motorName, DXL_M181_MAX_POS_LIMIT);
    MotorControl_WriteControlTable(&dxl, MIN_POSITION_LIMIT, motorName, DXL_M181_MIN_POS_LIMIT);
  }
  else if(DXL_M288 == motorType)
  {
    MotorControl_WriteControlTable(&dxl, MAX_POSITION_LIMIT, motorName, DXL_M288_MAX_POS_LIMIT);
    MotorControl_WriteControlTable(&dxl, MIN_POSITION_LIMIT, motorName, DXL_M288_MIN_POS_LIMIT);    
  }
  success &= obj->torqueOn(motorName);
  return success;
}

bool checkIfMotorAlive(Dynamixel2Arduino *obj, uint8_t motorName)
{
  bool status = false;
  // Check to see if the Dynamixel is alive
  if(obj->ping(motorName))
  {
    status = true;
    Serial1.println("Dynamixel detected");
  }
  else
  {
    Serial1.println("Not detected");
  }
  return status;
}

// THIS FUNCTION USES INDEX, NOT THE MOTOR ID.
bool MotorControl_Init(Dynamixel2Arduino *obj, uint8_t tableIdx)
{  
  bool localStatus = false;
  localStatus = checkIfMotorAlive(&dxl, motors[tableIdx].motorName);
  localStatus = localStatus && (setupMotor(obj, motors[tableIdx].motorName, OP_POSITION, motors[tableIdx].direction, motors[tableIdx].type)); // Solo finger (Motor 1)
  return localStatus;
}

void setup() 
{
  const int numberOfMotors = NUMBER_OF_MOTORS;
  Serial1.begin(HOST_COMMUNICATION_BAUDRATE); // Sets up the external serial port (USB dongle thing)
  dxl.begin(DYNAMIXEL_COMMUNICATION_BAUDRATE); //  Sets up communication with Dynamixel servos 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for(int i = 0; i < numberOfMotors; i++)
  {
    MotorControl_Init(&dxl, i);
  }
 
  hardwareErrorStatus_t hwStatus;
  MotorControl_SetPosition(&dxl, 2, 270.0, &hwStatus);
}


bool packetRead(dataPacket_t *pCmds, size_t size)
{
  return Serial1.readBytes((char *) pCmds, size);
}

void loop(void) 
{
  dataPacket_t motorCommands[6]; 
  static_assert(sizeof(motorCommands) == 54, "motorCommands is not 54 bytes!");

  //hardwareErrorStatus_t hwStatus;
  if(packetRead(motorCommands, sizeof(motorCommands)))
  {
    // Do something.
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