#ifndef COMMAND_FUNCTIONS_H
#define COMMAND_FUNCTIONS_H

#include <Arduino.h>
#include "l298n_motor_control.h"
#include "encoder_setup.h"
#include "adaptive_low_pass_filter.h"
#include "simple_pid_control.h"
#include "eeprom_setup.h"
#include "global_eeprom_variables.h"

//------------ Communication Command IDs --------------//
const uint8_t START_BYTE = 0xAA;
const uint8_t WRITE_VEL = 0x01;
const uint8_t WRITE_PWM = 0x02;
const uint8_t READ_POS = 0x03;
const uint8_t READ_VEL = 0x04;
const uint8_t READ_UVEL = 0x05;
const uint8_t READ_TVEL = 0x06;
const uint8_t SET_PPR = 0x07;
const uint8_t GET_PPR = 0x08;
const uint8_t SET_KP = 0x09;
const uint8_t GET_KP = 0x0A;
const uint8_t SET_KI = 0x0B;
const uint8_t GET_KI = 0x0C;
const uint8_t SET_KD = 0x0D;
const uint8_t GET_KD = 0x0E;
const uint8_t SET_RDIR = 0x0F;
const uint8_t GET_RDIR = 0x10;
const uint8_t SET_CUT_FREQ = 0x11;
const uint8_t GET_CUT_FREQ = 0x12;
const uint8_t SET_MAX_VEL = 0x13;
const uint8_t GET_MAX_VEL = 0x14;
const uint8_t SET_PID_MODE = 0x15;
const uint8_t GET_PID_MODE = 0x16;
const uint8_t SET_CMD_TIMEOUT = 0x17;
const uint8_t GET_CMD_TIMEOUT = 0x18;
const uint8_t SET_I2C_ADDR = 0x19;
const uint8_t GET_I2C_ADDR = 0x1A;
const uint8_t RESET_PARAMS = 0x1B;
const uint8_t READ_MOTOR_DATA = 0x2A;
const uint8_t CLEAR_DATA_BUFFER = 0x2C;
//---------------------------------------------------//

void initLed0()
{
  pinMode(A0, OUTPUT);
}
void onLed0()
{
  digitalWrite(A0, HIGH);
}
void offLed0()
{
  digitalWrite(A0, LOW);
}

void initLed1()
{
  pinMode(A1, OUTPUT);
}
void onLed1()
{
  digitalWrite(A1, HIGH);
}
void offLed1()
{
  digitalWrite(A1, LOW);
}

//--------------- global functions ----------------//
float writeSpeed(float v0, float v1)
{
  float tVelA = constrain(v0, -1.00 * maxVelA, maxVelA);
  float tVelB = constrain(v1, -1.00 * maxVelB, maxVelB);

  targetA = rdirA * tVelA;
  targetB = rdirB * tVelB;

  cmdVelTimeout = millis();

  return 1.0;
}

float writePWM(int pwm0, int pwm1)
{
  if (!pidMode)
  {
    motorA.sendPWM((int)rdirA * pwm0);
    motorB.sendPWM((int)rdirB * pwm1);
    cmdVelTimeout = millis();
    return 1.0;
  }
  else
    return 0.0;
}

void readPos(float &pos0, float &pos1)
{  
  pos0 = rdirA * encA.getAngPos();
  pos1 = rdirB * encB.getAngPos();
}

void readFilteredVel(float &v0, float &v1)
{
  v0 = rdirA * filteredAngVelA;
  v1 = rdirB * filteredAngVelB;
}

void readUnfilteredVel(float &v0, float &v1)
{
  v0 = rdirA * encA.getAngVel();
  v1 = rdirB * encB.getAngVel();
}

void readTargetVel(float &v0, float &v1)
{
  v0 = rdirA * targetA;
  v1 = rdirB * targetB;
}

float setEncoderPPR(int motor_no, float ppr)
{
  if (motor_no == 0) {
    setPPR_A(ppr);
    encA_ppr = getPPR_A();
    return 1.0;
  }
  else if (motor_no == 1) {
    setPPR_B(ppr);
    encB_ppr = getPPR_B();
    return 1.0;
  }
  else {
    return 0.0;
  }
  
}
float getEncoderPPR(int motor_no)
{
  if (motor_no == 0) {
    return encA_ppr;
  }
  else if (motor_no == 1) {
    return encB_ppr;
  }
  else {
    return 0.0;
  }
}


float setMotorKp(int motor_no, double Kp)
{
  if (motor_no == 0) {
    setKP_A(Kp);
    kpA = getKP_A();
    return 1.0;
  }
  else if (motor_no == 1) {
    setKP_B(Kp);
    kpB = getKP_B();
    return 1.0;
  }
  else {
    return 0.0;
  }
}
float getMotorKp(int motor_no)
{
  if (motor_no == 0) {
    return kpA;
  }
  else if (motor_no == 1) {
    return kpB;
  }
  else {
    return 0.0;
  }
}


float setMotorKi(int motor_no, double Ki)
{
  if (motor_no == 0) {
    setKI_A(Ki);
    kiA = getKI_A();
    return 1.0;
  }
  else if (motor_no == 1) {
    setKI_B(Ki);
    kiB = getKI_B();
    return 1.0;
  }
  else {
    return 0.0;
  }
}
float getMotorKi(int motor_no)
{
  if (motor_no == 0) {
    return kiA;
  }
  else if (motor_no == 1) {
    return kiB;
  }
  else {
    return 0.0;
  }
}


float setMotorKd(int motor_no, double Kd)
{
  if (motor_no == 0) {
    setKD_A(Kd);
    kdA = getKD_A();
    return 1.0;
  }
  else if (motor_no == 1) {
    setKD_B(Kd);
    kdB = getKD_B();
    return 1.0;
  }
  else {
    return 0.0;
  }
}
float getMotorKd(int motor_no)
{
  if (motor_no == 0) {
    return kdA;
  }
  else if (motor_no == 1) {
    return kdB;
  }
  else {
    return 0.0;
  }
}


float setRdir(int motor_no, float dir)
{
  if (motor_no == 0) {
    setRDIR_A(dir);
    rdirA = getRDIR_A();
    return 1.0;
  }
  else if (motor_no == 1) {
    setRDIR_B(dir);
    rdirB = getRDIR_B();
    return 1.0;
  }
  else {
    return 0.0;
  }
}
float getRdir(int motor_no)
{
  if (motor_no == 0) {
    return rdirA;
  }
  else if (motor_no == 1) {
    return rdirB;
  }
  else {
    return 0.0;
  }
}



float setCutoffFreq(int motor_no, double f0)
{
  if (motor_no == 0) {
    setFilterCutOffFreq_A(f0);
    cutOffFreqA = getFilterCutOffFreq_A();
    return 1.0;
  }
  else if (motor_no == 1) {
    setFilterCutOffFreq_B(f0);
    cutOffFreqB = getFilterCutOffFreq_B();
    return 1.0;
  }
  else {
    return 0.0;
  }
}
float getCutoffFreq(int motor_no)
{
  if (motor_no == 0) {
    return cutOffFreqA;
  }
  else if (motor_no == 1) {
    return cutOffFreqB;
  }
  else {
    return 0.0;
  }
}



float setMaxVel(int motor_no, double max_vel)
{
  if (motor_no == 0) {
    setMAXVEL_A(max_vel);
    maxVelA = getMAXVEL_A();
    return 1.0;
  }
  else if (motor_no == 1) {
    setMAXVEL_B(max_vel);
    maxVelB = getMAXVEL_B();
    return 1.0;
  }
  else {
    return 0.0;
  }
}
float getMaxVel(int motor_no)
{
  if (motor_no == 0) {
    return maxVelA;
  }
  else if (motor_no == 1) {
    return maxVelB;
  }
  else {
    return 0.0;
  }
}



float setPidModeFunc(int motor_no, int mode)
{
  if (mode == 0)
  {
    pidMode = false;
    motorA.sendPWM(0);
    motorB.sendPWM(0);
    pidMotorA.begin();
    pidMotorB.begin();
  }
  else if (mode == 1)
  {
    pidMode = true;
    motorA.sendPWM(0);
    motorB.sendPWM(0);
    pidMotorA.begin();
    pidMotorB.begin();
  }

  return 1.0;
}
float getPidModeFunc(int motor_no)
{
  if (pidMode) return 1.0;
  else return 0.0;
}




float setCmdTimeout(int timeout_ms)
{
  cmdVelTimeoutSampleTime = (long)timeout_ms;
  return 1.0;
}
float getCmdTimeout()
{
  return (float)cmdVelTimeoutSampleTime;
}


float clearDataBuffer()
{
  encA.tickCount = 0;
  encB.tickCount = 0;
  filteredAngVelA = 0.0;
  filteredAngVelB = 0.0;
  targetA = 0.0;
  targetB = 0.0;
  return 1.0;
}


float triggerResetParams()
{
  setFIRST_TIME(0);
  updateGlobalParamsFromEERPOM();
  return 1.0;
}




#include "i2c_comm.h"
float setI2cAddress(int address)
{
  if((address <= 0) || (address > 255)){
    return 0.0;
  }
  else {
    setI2CADDRESS(address);
    i2cAddress = getI2CADDRESS();
    Wire.begin(i2cAddress);

    return 1.0;
  }
}
float getI2cAddress()
{
  return (float)i2cAddress;
}
//-------------------------------------------------------------------//


#endif