// This is done to make sure we have the best code possible. Warnings => bugs later on.
#pragma GCC diagnostic error "-Wall"
#pragma GCC diagnostic error "-Wpedantic"

#include <Arduino.h>
#include <DynamixelShield.h> // for DXL_DIR_PIN definition
#include <Dynamixel2Arduino.h> // For Dynamixel2Arduino Class definition.
#include <assert.h>
#include "servo_api.h"
#include "servo_types.h"

Dynamixel2Arduino dxl(Serial, DXL_DIR_PIN); // When initializing the dxl class, you need to feed in the Serial port and direction pin you are going to use.

motorValues_t g_motors[NUMBER_OF_MOTORS] = 
                          { {1, DXL_M181,  reverseDirection},
                            {2, DXL_M288,  reverseDirection},
                            {3, DXL_M288,  normalDirection},
                            {4, DXL_M181,  normalDirection},
                            {5, DXL_M288,  normalDirection},
                            {6, DXL_M181,  normalDirection}};


motorTranslationValues_t g_translations[NUMBER_OF_MOTORS] = {
  {180.0, 90.0, 0.0},
  {180.0, 90.0, 0.0},
  {0.0,   90.0, 180.0},
  {90.0, 180.0, 270.0},
  {180.0, 270.0, 360.0},
  {90.0, 180.0, 270.0}};


DYNAMIXEL::XELInfoSyncWrite_t g_dynamixelInfo[NUMBER_OF_MOTORS];
uint32_t g_positionInformation[NUMBER_OF_MOTORS];

float translateAngle(uint8_t tableIdx, float desiredAngle)
{
  float realAngle = 0.0;

  // Example is motor #3
  // If home is 360 and out is 180, that means that we should map 0 -> 360 and 180 to 180.
  if(g_translations[tableIdx].homeAngle > g_translations[tableIdx].bentOutAngle)
  {
    realAngle = -((desiredAngle)) + g_translations[tableIdx].homeAngle;
  }
  else
  {
    realAngle = ((desiredAngle)) + g_translations[tableIdx].homeAngle;
  }
  return realAngle;
}


bool _setPosition(Dynamixel2Arduino *obj, uint8_t motorName, float desiredAngle)
{
  return obj->setGoalPosition(motorName, desiredAngle, UNIT_DEGREE);
}

// This function will make sure that we can write safely to the control table, based on if the torque enable bit is true.
bool MotorControl_WriteControlTable(Dynamixel2Arduino *obj, uint8_t item_idx, uint8_t motorName, uint32_t data)
{
  bool torqueEnable = obj->getTorqueEnableStat(motorName);
  if (item_idx < END_OF_EEPROM_AREA_REGISTERS)
  {
    if(torqueEnable)
    {
      assert(0); // Die here.
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

// Top level handler for handling indexes instead of Ids. Used for setting a single motor.
bool MotorControl_SetPositionWrapper(Dynamixel2Arduino *obj, uint8_t motorName, float desiredAngle, hardwareErrorStatus_t *hardwareErrorStatus)
{
  float actualAngle = 0.0;
  int tableIdx = motorName - 1;
  actualAngle = translateAngle(tableIdx, desiredAngle);
  return MotorControl_SetPosition(obj, motorName, actualAngle, hardwareErrorStatus);
}

// Does a sync write to set all motors to move at the exact same time.
bool MotorControl_SetPositionSyncWrite(Dynamixel2Arduino *obj, DYNAMIXEL::InfoSyncWriteInst_t *sWriteParams)
{
  return obj->syncWrite(sWriteParams);
}

bool MotorControl_SetPosition(Dynamixel2Arduino *obj, uint8_t motorName, float desiredAngle, hardwareErrorStatus_t *hardwareErrorStatus){
  if(!_setPosition(obj, motorName, desiredAngle))
  {
    // this should be a function call.
    delay(1000);
    hardwareErrorStatus->all = obj->readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, motorName); // read HW error status register
    // Motor didn't work, let's find out why.
    //Serial1.write((char *)hardwareErrorStatus->all, sizeof(uint8_t));
    return false;
  }
  return true;
}

bool setupMotor(Dynamixel2Arduino *obj, uint8_t motorName, OperatingMode opMode, motorDirection_e direction, motorType_e motorType)
{
  bool success = false;
  success = obj->torqueOff(motorName);
  success &= obj->setOperatingMode(motorName, opMode); // EEPROM AREA
  success &= MotorControl_WriteControlTable(obj, ControlTableItem::DRIVE_MODE, motorName, (uint8_t) direction); // EEPROM AREA
  if(DXL_M181 == motorType)
  {
    MotorControl_WriteControlTable(&dxl, ControlTableItem::MAX_POSITION_LIMIT, motorName, DXL_M181_MAX_POS_LIMIT);
    MotorControl_WriteControlTable(&dxl, ControlTableItem::MIN_POSITION_LIMIT, motorName, DXL_M181_MIN_POS_LIMIT);
  }
  else if(DXL_M288 == motorType)
  {
    MotorControl_WriteControlTable(&dxl, ControlTableItem::MAX_POSITION_LIMIT, motorName, DXL_M288_MAX_POS_LIMIT);
    MotorControl_WriteControlTable(&dxl, ControlTableItem::MIN_POSITION_LIMIT, motorName, DXL_M288_MIN_POS_LIMIT);    
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
    //Serial1.println("Dynamixel detected");
  }
  else
  {
    //Serial1.println("Not detected");
  }
  return status;
}

// THIS FUNCTION USES INDEX, NOT THE MOTOR ID.
bool MotorControl_Init(Dynamixel2Arduino *obj, uint8_t tableIdx)
{  
  bool localStatus = false;
  localStatus = checkIfMotorAlive(&dxl, g_motors[tableIdx].motorName);
  localStatus = localStatus && (setupMotor(obj, g_motors[tableIdx].motorName, OP_POSITION, g_motors[tableIdx].direction, g_motors[tableIdx].type)); // Solo finger (Motor 1)
  return localStatus;
}

void setNumberOfDevicesToWrite(uint8_t numberOfDevices, DYNAMIXEL::InfoSyncWriteInst_t *sWriteParams)
{
  sWriteParams->xel_count = numberOfDevices;
  sWriteParams->addr = ControlTableItem::GOAL_POSITION;
  sWriteParams->addr_length = (uint16_t) sizeof(uint32_t); // takes a 16 bit number, tells you how big each sWriteParams->pBuf is in bytes.
}

#define DXL_SPECIFIC_FLOAT_ANGLE_TO_INT_ANGLE_CONVERSION_FACTOR 11.375 // 11.375 integer degree units per floating point degree
// Does 2 things: 1 - converts the input angle to the real input angle found by testing to get the desired output (this changes per motor)
// and 2 - converts the floating point angle to its integer representation, betweeen 0 and 4095.
uint32_t angle2Int(uint8_t idx, float inputAngle)
{
  // The mapping from angle to integer is 0 - 4095 = 0.0 to 360.0.
  // First, we will need to convert the angle to its correct mapping, then convert to integer.
  float interimResult = NAN;
  uint32_t returnedData = (uint32_t) -1;
  interimResult = translateAngle(idx, inputAngle);
  if(NAN == interimResult || NAN == inputAngle)
  {
    assert(0); // die if the input is bad, or if the interim result is bad.
  }
  // Now that we have the desired angle, we can translate this to an integer between 0 and 4095.
  returnedData = (int) (DXL_SPECIFIC_FLOAT_ANGLE_TO_INT_ANGLE_CONVERSION_FACTOR * interimResult);
  if(returnedData > 4095)
  {
    assert(0); // We should never exceed 4095.
  }
  return returnedData;
}

// Uses some fancy pointer stuff to avoid a memcpy when setting goal position.
void setGoalPositionBulkWrite(uint8_t idx, uint32_t position, uint32_t *buf)
{
  if(buf == NULL)
  {
    assert(0);
  }
  *buf = position;
}

// Here we will setup the burst write packet to target our motors.
void setupDynamixelPositionPacket(uint8_t idx, dataPacket_t *pCmds, DYNAMIXEL::InfoSyncWriteInst_t *sWriteParams)
{
  uint32_t *pBuf;
  if(sWriteParams == NULL)
  {
    assert(0);
  }
  sWriteParams->p_xels = g_dynamixelInfo;
  if(sWriteParams->p_xels[idx].p_data == NULL)
  {
    assert(0);
  }
  
  *pBuf = (uint32_t *) sWriteParams->p_xels[idx].p_data;
  *pBuf = angle2Int(idx, pCmds[idx].desiredPosition);
}

// Manipulating globals, be careful.
void setupDynamixelTxBuffer(uint8_t i)
{
  memset((void *)&g_dynamixelInfo, 0x00, sizeof(DYNAMIXEL::XELInfoSyncWrite_t)*NUMBER_OF_MOTORS);
  memset((void *)&g_positionInformation, 0x00, sizeof(uint32_t)*NUMBER_OF_MOTORS);
  
  g_dynamixelInfo[i].id = g_motors[i].motorName;
  g_dynamixelInfo[i].p_data = (uint8_t *) g_positionInformation[i];
}

void setup(void) 
{
  Serial1.begin(HOST_COMMUNICATION_BAUDRATE); // Sets up the external serial port (USB dongle thing)
  dxl.begin(DYNAMIXEL_COMMUNICATION_BAUDRATE); //  Sets up communication with Dynamixel servos 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for(int i = 0; i < NUMBER_OF_MOTORS; i++)
  {
    setupDynamixelTxBuffer(i);
    MotorControl_Init(&dxl, i);
  }
}

bool packetRead(dataPacket_t *pCmds, size_t size)
{
  return Serial1.readBytes((uint8_t *) pCmds, size);
}

void loop(void)
{
  dataPacket_t motorCommands[6]; 
  static_assert(sizeof(motorCommands) == 54, "motorCommands is not 54 bytes!");
  hardwareErrorStatus_t hwStatus;
  DYNAMIXEL::InfoSyncWriteInst_t syncWriteParam;

  memset((void *) &syncWriteParam, 0x00, sizeof(DYNAMIXEL::InfoSyncWriteInst_t));
  memset((void *) &motorCommands, 0x00, sizeof(dataPacket_t)*6);

  if(packetRead(motorCommands, sizeof(motorCommands)))
  {
    for(int i = 0; i < NUMBER_OF_MOTORS; i++)
    {
      setupDynamixelPositionPacket(2, motorCommands, &syncWriteParam);
    }
    MotorControl_SetPositionSyncWrite(&dxl, &syncWriteParam);
  }
  else
  {
    Serial1.write((char *) motorCommands, sizeof(motorCommands));
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