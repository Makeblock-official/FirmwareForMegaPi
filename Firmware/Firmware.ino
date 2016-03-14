/*************************************************************************
* File Name          : Firmware_for_MegaPi.ino
* Author             : myan
* Updated            : myan
* Version            : V0e.01.101
* Date               : 02/20/2016
* Description        : Firmware for Makeblock Electronic modules with Scratch.  
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 - 2016 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

//#define DEBUG_INFO

Servo servos[8];  
MeMegaPiDCMotor dc;
MeTemperature ts;
MeRGBLed led;
MeUltrasonicSensor *us = NULL;
Me7SegmentDisplay seg;
MePort generalDevice;
MeLEDMatrix ledMx;
MeInfraredReceiver *ir = NULL;
MeGyro gyro;
MeCompass Compass;
MeJoystick joystick;
MeStepper steppers[2];
MeBuzzer buzzer;
MeHumiture humiture;
MeFlameSensor FlameSensor;
MeGasSensor GasSensor;
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);
MeEncoderOnBoard Encoder_4(SLOT4);
MeSerial mySerial(PORT_9);

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.GetPortB()) == 0)
  {
    Encoder_1.PulsePosMinus();
  }
  else
  {
    Encoder_1.PulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.GetPortB()) == 0)
  {
    Encoder_2.PulsePosMinus();
  }
  else
  {
    Encoder_2.PulsePosPlus();
  }
}

void isr_process_encoder3(void)
{
  if(digitalRead(Encoder_3.GetPortB()) == 0)
  {
    Encoder_3.PulsePosMinus();
  }
  else
  {
    Encoder_3.PulsePosPlus();
  }
}

void isr_process_encoder4(void)
{
  if(digitalRead(Encoder_4.GetPortB()) == 0)
  {
    Encoder_4.PulsePosMinus();
  }
  else
  {
    Encoder_4.PulsePosPlus();
  }
}

typedef struct MeModule
{
  int16_t device;
  int16_t port;
  int16_t slot;
  int16_t pin;
  int16_t index;
  float values[3];
} MeModule;

union
{
  byte byteVal[4];
  float floatVal;
  long longVal;
}val;

union
{
  byte byteVal[8];
  double doubleVal;
}valDouble;

union
{
  byte byteVal[2];
  short shortVal;
}valShort;
MeModule modules[12];
#if defined(__AVR_ATmega32U4__) 
  int analogs[12]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};
#endif
#if defined(__AVR_ATmega328P__) or defined(__AVR_ATmega168__)
  int analogs[8]={A0,A1,A2,A3,A4,A5,A6,A7};
#endif
#if defined(__AVR_ATmega1280__)|| defined(__AVR_ATmega2560__)
  int analogs[16]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
#endif

int16_t len = 52;
int16_t servo_pins[8]={0,0,0,0,0,0,0,0};
int16_t moveSpeed = 255;

#define MOVE_STOP       0x00
#define MOVE_FORWARD    0x01
#define MOVE_BACKWARD   0x02

int16_t move_status = MOVE_STOP;

#define BLUETOOTH_MODE                       0x00
#define AUTOMATIC_OBSTACLE_AVOIDANCE_MODE    0x01
#define BALANCED_MODE                        0x02
#define IR_REMOTE_MODE                       0x03

//#define POWER_PORT                           A4
//#define BUZZER_PORT                          45
//#define RGBLED_PORT                          44

#define DATA_SERIAL                            0
#define DATA_SERIAL1                           1
#define DATA_SERIAL2                           2
#define DATA_SERIAL3                           3

uint8_t megapi_mode = BLUETOOTH_MODE;
uint8_t index = 0;
uint8_t dataLen;
uint8_t modulesLen=0;
uint8_t irRead = 0;
uint8_t prevc=0;
uint8_t BluetoothSource = DATA_SERIAL;
char serialRead;
char buffer[52];
char bufferBt1[52];
char bufferBt2[52];
double lastTime = 0.0;
double currentTime = 0.0;
float angleServo = 90.0;

boolean isStart = false;
boolean isAvailable = false;

String mVersion = "0e.01.101";

#define VERSION 0
#define ULTRASONIC_SENSOR 1
#define TEMPERATURE_SENSOR 2
#define LIGHT_SENSOR 3
#define POTENTIONMETER 4
#define JOYSTICK 5
#define GYRO 6
#define SOUND_SENSOR 7
#define RGBLED 8
#define SEVSEG 9
#define MOTOR 10
#define SERVO 11
#define ENCODER 12
#define IR 13
#define PIRMOTION 15
#define INFRARED 16
#define LINEFOLLOWER 17
#define SHUTTER 20
#define LIMITSWITCH 21
#define BUTTON 22
#define HUMITURE 23
#define FLAMESENSOR 24
#define GASSENSOR 25
#define COMPASS 26
#define TEMPERATURE_SENSOR_1 27
#define DIGITAL 30
#define ANALOG 31
#define PWM 32
#define SERVO_PIN 33
#define TONE 34
#define PULSEIN 35
#define ULTRASONIC_ARDUINO 36
#define STEPPER 40
#define LEDMATRIX 41
#define TIMER 50
#define JOYSTICK_MOVE 52
#define COMMON_COMMONCMD 60
  //Secondary command
  #define SET_STARTER_MODE     0x10
  #define SET_AURIGA_MODE      0x11
  #define SET_MEGAPI_MODE      0x12
  #define GET_BATTERY_POWER    0x70
#define ENCODER_BOARD 61
  //Read type
  #define ENCODER_BOARD_POS    0x01
  #define ENCODER_BOARD_SPEED  0x02

#define GET 1
#define RUN 2
#define RESET 4
#define START 5
long measurement_speed_time = 0;
long lasttime_angle = 0;
long lasttime_speed = 0;
long last_Pulse_pos_encoder1 = 0;
long last_Pulse_pos_encoder2 = 0;
long last_Pulse_pos_encoder3 = 0;
long last_Pulse_pos_encoder4 = 0;
//////////////////////////////////////////////////////////////////////////////////////
float RELAX_ANGLE = 0;     //��
typedef struct
{
  double P, I, D;
  double Setpoint, Output, Integral,differential, last_error;
} PID;

PID  PID_angle, PID_speed, PID_turn;
PID  PID_speed_left, PID_speed_right;

void WriteBalancedDataToEEPROM(void)
{
  EEPROM.write(BALANCED_CAR_PARTITION_CHECK, EEPROM_IF_HAVEPID_CHECK1);
  EEPROM.write(BALANCED_CAR_PARTITION_CHECK + 1, EEPROM_IF_HAVEPID_CHECK2);
  EEPROM.write(BALANCED_CAR_START_ADDR, EEPROM_CHECK_START);

  EEPROM.put(BALANCED_CAR_NATURAL_BALANCE, RELAX_ANGLE);
  EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR, PID_angle.P);
  EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR+4, PID_angle.I);
  EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR+8, PID_angle.D);

  EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR, PID_speed.P);
  EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR+4, PID_speed.I);
  EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR+8, PID_speed.D);

  EEPROM.put(BALANCED_CAR_DIR_PID_ADDR, PID_turn.P);
  EEPROM.write(BALANCED_CAR_END_ADDR, EEPROM_CHECK_END);

  EEPROM.write(AURIGA_MODE_START_ADDR, EEPROM_CHECK_START);
  EEPROM.write(BALANCED_CAR_SPEED_PID_ADDR, megapi_mode);
  EEPROM.write(AURIGA_MODE_END_ADDR, EEPROM_CHECK_END);
}

void WriteAurigaModeToEEPROM(void)
{
  EEPROM.write(MEGAPI_MODE_PARTITION_CHECK, EEPROM_IF_HAVEPID_CHECK1);
  EEPROM.write(MEGAPI_MODE_PARTITION_CHECK + 1, EEPROM_IF_HAVEPID_CHECK2);
  EEPROM.write(MEGAPI_MODE_START_ADDR, EEPROM_CHECK_START);
  EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
  EEPROM.write(MEGAPI_MODE_END_ADDR, EEPROM_CHECK_END);
}

int readEEPROM(void)
{
  if((EEPROM.read(BALANCED_CAR_PARTITION_CHECK) == EEPROM_IF_HAVEPID_CHECK1) && (EEPROM.read(BALANCED_CAR_PARTITION_CHECK + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    if((EEPROM.read(BALANCED_CAR_START_ADDR)  == EEPROM_CHECK_START) && (EEPROM.read(BALANCED_CAR_END_ADDR)  == EEPROM_CHECK_END))
    {
      EEPROM.get(BALANCED_CAR_NATURAL_BALANCE, RELAX_ANGLE);
      EEPROM.get(BALANCED_CAR_ANGLE_PID_ADDR, PID_angle.P);
      EEPROM.get(BALANCED_CAR_ANGLE_PID_ADDR+4, PID_angle.I);
      EEPROM.get(BALANCED_CAR_ANGLE_PID_ADDR+8, PID_angle.D);

      EEPROM.get(BALANCED_CAR_SPEED_PID_ADDR, PID_speed.P);
      EEPROM.get(BALANCED_CAR_SPEED_PID_ADDR+4, PID_speed.I);
      EEPROM.get(BALANCED_CAR_SPEED_PID_ADDR+8, PID_speed.D);

      EEPROM.get(BALANCED_CAR_DIR_PID_ADDR, PID_turn.P);
#ifdef DEBUG_INFO
      Serial.println( "Read data from EEPROM:");
      Serial.print(RELAX_ANGLE);
      Serial.print( "  ");
      Serial.print(PID_angle.P);
      Serial.print( "  ");
      Serial.print(PID_angle.I);
      Serial.print( "  ");
      Serial.print(PID_angle.D);
      Serial.print( "  ");
      Serial.print(PID_speed.P);
      Serial.print( "  ");
      Serial.print(PID_speed.I);
      Serial.print( "  ");
      Serial.print(PID_speed.D);
      Serial.print( "  ");
      Serial.println(PID_turn.P);
#endif
    }
    else
    {
      Serial.println( "Data area damage on balanced car pid!" );
    }
  }
  else
  {
#ifdef DEBUG_INFO
    Serial.println( "First written Balanced data!" );
#endif
    WriteBalancedDataToEEPROM();
  }

  if((EEPROM.read(AURIGA_MODE_PARTITION_CHECK) == EEPROM_IF_HAVEPID_CHECK1) && (EEPROM.read(AURIGA_MODE_PARTITION_CHECK + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    if((EEPROM.read(AURIGA_MODE_START_ADDR)  == EEPROM_CHECK_START) && (EEPROM.read(AURIGA_MODE_END_ADDR)  == EEPROM_CHECK_END))
    {
      EEPROM.get(AURIGA_MODE_CONFIGURE, megapi_mode);
#ifdef DEBUG_INFO
      Serial.print( "Read auriga_mode from EEPROM:");
      Serial.println(auriga_mode);
#endif
    }
    else
    {
      Serial.println( "Data area damage on auriga mode!" );
    }
  }
  else
  {
#ifdef DEBUG_INFO
    Serial.println( "First written auriga mode!" );
#endif
    WriteAurigaModeToEEPROM();
  }
}

void Forward(void)
{
//  Encoder_1.setMotorPwm(-moveSpeed);
//  Encoder_2.setMotorPwm(moveSpeed);
  PID_speed_left.Setpoint = -moveSpeed;
  PID_speed_right.Setpoint = moveSpeed;
  move_status = MOVE_FORWARD;
  PWM_Calcu();
}

void Backward(void)
{
//  Encoder_1.setMotorPwm(moveSpeed);
//  Encoder_2.setMotorPwm(-moveSpeed);
  PID_speed_left.Setpoint = moveSpeed;
  PID_speed_right.Setpoint = -moveSpeed;
  move_status = MOVE_BACKWARD;
  PWM_Calcu();
}

void Stop(void)
{
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
  move_status = MOVE_STOP;
}

unsigned char readBuffer(int index)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    return buffer[index];
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    return bufferBt1[index];
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    return bufferBt2[index];
  }
}

void writeBuffer(int index,unsigned char c)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    buffer[index]=c;
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    bufferBt1[index]=c;
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    bufferBt2[index]=c;
  }
}

void writeHead(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
}

void writeEnd(void)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    Serial.println();
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    Serial2.println();
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    Serial3.println();
  }
}

void writeSerial(unsigned char c)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    Serial.write(c);;
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    Serial2.write(c);;
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    Serial3.write(c);;
  }
}

void readSerial(void)
{
  isAvailable = false;
  BluetoothSource = DATA_SERIAL;
  if(Serial.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL;
    serialRead = Serial.read();
  }
  else if(Serial2.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL2;
    serialRead = Serial2.read();
  }
  else if(Serial3.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL3;
    serialRead = Serial3.read();
  }
}
/*
ff 55 len idx action device port  slot  data a
0  1  2   3   4      5      6     7     8
*/
void parseData(void)
{
  isStart = false;
  int idx = readBuffer(3);
  int action = readBuffer(4);
  int device = readBuffer(5);
  switch(action)
  {
    case GET:
      {
        writeHead();
        writeSerial(idx);
        readSensor(device);
        writeEnd();
      }
      break;
    case RUN:
      {
        runModule(device);
        callOK();
      }
      break;
    case RESET:
      {
        //reset
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        Encoder_1.SetPulsePos(0);
        Encoder_2.SetPulsePos(0);
        PID_speed_left.Setpoint = 0;
        PID_speed_right.Setpoint = 0;
        dc.reset(M1);
        dc.run(0);
        dc.reset(M2);
        dc.run(0);
        dc.reset(PORT_1);
        dc.run(0);
        dc.reset(PORT_2);
        dc.run(0);
        callOK();
      }
      break;
     case START:
      {
        //start
        callOK();
      }
      break;
  }
}

void callOK(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}

void sendByte(char c)
{
  writeSerial(1);
  writeSerial(c);
}

void sendString(String s)
{
  int l = s.length();
  writeSerial(4);
  writeSerial(l);
  for(int i=0;i<l;i++)
  {
    writeSerial(s.charAt(i));
  }
}

void sendFloat(float value)
{ 
  writeSerial(2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

void sendLong(long value)
{ 
  writeSerial(6);
  val.longVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

void sendShort(short value)
{
  writeSerial(3);
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
}

void sendDouble(double value)
{
  writeSerial(5);
  valDouble.doubleVal = value;
  writeSerial(valDouble.byteVal[0]);
  writeSerial(valDouble.byteVal[1]);
  writeSerial(valDouble.byteVal[2]);
  writeSerial(valDouble.byteVal[3]);
}

short readShort(int idx)
{
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx+1);
  return valShort.shortVal; 
}

float readFloat(int idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.floatVal;
}

long readLong(int idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.longVal;
}

char _receiveStr[20] = {};
uint8_t _receiveUint8[16] = {};

char* readString(int idx,int len)
{
  for(int i=0;i<len;i++)
  {
    _receiveStr[i]=readBuffer(idx+i);
  }
  _receiveStr[len] = '\0';
  return _receiveStr;
}

uint8_t* readUint8(int idx,int len)
{
  for(int i=0;i<len;i++)
  {
    if(i > 15)
    {
      break;
    }
    _receiveUint8[i] = readBuffer(idx+i);
  }
  return _receiveUint8;
}

void runModule(int device)
{
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
  int port = readBuffer(6);
  int pin = port;
  switch(device)
  {
    case MOTOR:
      {
        int speed = readShort(7);
        dc.reset(port);
        dc.run(speed);
      }
      break;
    case ENCODER_BOARD:
      if(port == 0)
      {
        uint8_t slot = readBuffer(7);
        int16_t speed_value = readShort(8);
        if(slot == SLOT_1)
        {
          Encoder_1.setMotorPwm(speed_value);
        }
        else if(slot == SLOT_2)
        {
          Encoder_2.setMotorPwm(speed_value);
        }
        else if(slot == SLOT_3)
        {
          Encoder_3.setMotorPwm(speed_value);
        }
        else if(slot == SLOT_4)
        {
          Encoder_4.setMotorPwm(speed_value);
        }
      }
      break;
    case JOYSTICK:
      {
        int leftSpeed = readShort(6);
        dc.reset(M1);
        dc.run(leftSpeed);
        int rightSpeed = readShort(8);
        dc.reset(M2);
        dc.run(rightSpeed);
      }
      break;
    case STEPPER:
      {
        int maxSpeed = readShort(7);
        int distance = readShort(9);
        if(port == PORT_1)
        {
          steppers[0] = MeStepper(PORT_1);
          steppers[0].moveTo(distance);
          steppers[0].setMaxSpeed(maxSpeed);
          steppers[0].setSpeed(maxSpeed);
        }
        else if(port == PORT_2)
        {
          steppers[1] = MeStepper(PORT_2);
          steppers[1].moveTo(distance);
          steppers[1].setMaxSpeed(maxSpeed);
          steppers[1].setSpeed(maxSpeed);
        }
      } 
      break;
    case RGBLED:
      {
        int slot = readBuffer(7);
        int idx = readBuffer(8);
        int r = readBuffer(9);
        int g = readBuffer(10);
        int b = readBuffer(11);
        if(port != 0)
        {
          led.reset(port,slot);
        }
        if(idx>0)
        {
          led.setColorAt(idx-1,r,g,b); 
        }
        else
        {
          led.setColor(r,g,b); 
        }
        led.show();
      }
      break;
    case COMMON_COMMONCMD:
      {
        int8_t subcmd = port;
        int8_t cmd_data = readBuffer(7);
        if(SET_MEGAPI_MODE == subcmd)
        {
          Stop();
          if((cmd_data == BALANCED_MODE) || 
             (cmd_data == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE) || 
             (cmd_data == BLUETOOTH_MODE) ||
             (cmd_data == IR_REMOTE_MODE))
          {
            megapi_mode = cmd_data;
            EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
          }
          else
          {
            megapi_mode = BLUETOOTH_MODE;
            EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
          }
        }
      }
      break;
    case SERVO:
      {
        int slot = readBuffer(7);
        pin = slot==1?mePort[port].s1:mePort[port].s2;
        int v = readBuffer(8);
        Servo sv = servos[searchServoPin(pin)];
        if(v >= 0 && v <= 180)
        {
          if(port > 0)
          {
            sv.attach(pin);
          }
          else
          {
            sv.attach(pin);
          }
          sv.write(v);
        }
      }
      break;
    case SEVSEG:
      {
        if(seg.getPort() != port)
        {
          seg.reset(port);
        }
        float v = readFloat(7);
        seg.display(v);
      }
      break;
    case LEDMATRIX:
      {
        if(ledMx.getPort()!=port)
        {
          ledMx.reset(port);
        }
        int action = readBuffer(7);
        if(action==1)
        {
          int px = buffer[8];
          int py = buffer[9];
          int len = readBuffer(10);
          char *s = readString(11,len);
          ledMx.drawStr(px,py,s);
        }
        else if(action==2)
        {
          int px = readBuffer(8);
          int py = readBuffer(9);
          uint8_t *ss = readUint8(10,16);
          ledMx.drawBitmap(px,py,16,ss);
        }
        else if(action==3)
        {
          int point = readBuffer(8);
          int hours = readBuffer(9);
          int minutes = readBuffer(10);
          ledMx.showClock(hours,minutes,point);
        }
      }
      break;
    case LIGHT_SENSOR:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
        }
        int v = readBuffer(7);
        generalDevice.dWrite1(v);
      }
      break;
    case SHUTTER:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
        }
        int v = readBuffer(7);
        if(v < 2)
        {
          generalDevice.dWrite1(v);
        }
        else
        {
          generalDevice.dWrite2(v-2);
        }
      }
      break;
    case DIGITAL:
      {
        pinMode(pin,OUTPUT);
        int v = readBuffer(7);
        digitalWrite(pin,v);
     }
     break;
    case PWM:
      {
        pinMode(pin,OUTPUT);
        int v = readBuffer(7);
        analogWrite(pin,v);
      }
      break;
    case TONE:
      {
        pinMode(pin,OUTPUT);
        int hz = readShort(7);
        int ms = readShort(9);
        if(ms > 0)
        {
          buzzer.tone(pin, hz, ms); 
        }
        else
        {
          buzzer.noTone(pin); 
        }
      }
      break;
    case SERVO_PIN:
      {
        int v = readBuffer(7);
        if(v >= 0 && v <= 180)
        {
          Servo sv = servos[searchServoPin(pin)];
          sv.attach(pin);
          sv.write(v);
        }
      }
      break;
    case TIMER:
      {
        lastTime = millis()/1000.0; 
      }
      break;
    case JOYSTICK_MOVE:
      {
        if(port == 0)
        {
          //if needed balance mode, add here
        }
      }
      break;
  }
}

int searchServoPin(int pin)
{
  for(int i=0;i<8;i++)
  {
    if(servo_pins[i] == pin)
    {
      return i;
    }
    if(servo_pins[i] == 0)
    {
      servo_pins[i] = pin;
      return i;
    }
  }
  return 0;
}
void readSensor(int device)
{
  /**************************************************
      ff 55 len idx action device port slot data a
      0  1  2   3   4      5      6    7    8
  ***************************************************/
  float value=0.0;
  int8_t port,slot,pin;
  port = readBuffer(6);
  pin = port;
  switch(device)
  {
    case ULTRASONIC_SENSOR:
      {
        if(us == NULL)
        {
          us = new MeUltrasonicSensor(port);
        }
        else if(us->getPort() != port)
        {
          delete us;
          us = new MeUltrasonicSensor(port);
        }
        value = us->distanceCm();
        sendFloat(value);
      }
      break;
    case  TEMPERATURE_SENSOR:
      {
        slot = readBuffer(7);
        if(ts.getPort() != port || ts.getSlot() != slot)
        {
          ts.reset(port,slot);
        }
        value = ts.temperature();
        sendFloat(value);
      }
      break;
    case  LIGHT_SENSOR:
    case  SOUND_SENSOR:
    case  TEMPERATURE_SENSOR_1:
    case  POTENTIONMETER:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(),INPUT);
        }
        value = generalDevice.aRead2();
        sendFloat(value);
      }
      break;
    case  JOYSTICK:
      {
        slot = readBuffer(7);
        if(joystick.getPort() != port)
        {
          joystick.reset(port);
        }
        value = joystick.read(slot);
        sendFloat(value);
      }
      break;
    case  INFRARED:
      {
        if(ir == NULL)
        {
          ir = new MeInfraredReceiver(port);
          ir->begin();
        }
        else if(ir->getPort() != port)
        {
          delete ir;
          ir = new MeInfraredReceiver(port);
          ir->begin();
        }
        irRead = ir->getCode();
        if((irRead < 255) && (irRead > 0))
        {
          sendFloat((float)irRead);
        }
        else
        {
          sendFloat(0);
        }
      }
      break;
    case  PIRMOTION:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(),INPUT);
        }
        value = generalDevice.dRead2();
        sendFloat(value);
      }
      break;
    case  LINEFOLLOWER:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin1(),INPUT);
          pinMode(generalDevice.pin2(),INPUT);
        }
        value = generalDevice.dRead1()*2+generalDevice.dRead2();
        sendFloat(value);
      }
      break;
    case LIMITSWITCH:
      {
        slot = readBuffer(7);
        if(generalDevice.getPort() != port || generalDevice.getSlot() != slot)
        {
          generalDevice.reset(port,slot);
        }
        if(slot == 1)
        {
          pinMode(generalDevice.pin1(),INPUT_PULLUP);
          value = generalDevice.dRead1();
        }
        else
        {
          pinMode(generalDevice.pin2(),INPUT_PULLUP);
          value = generalDevice.dRead2();
        }
        sendFloat(value);  
      }
      break;
    case COMPASS:
      {
        if(Compass.getPort() != port)
        {
          Compass.reset(port);
          Compass.setpin(Compass.pin1(),Compass.pin2());
        }
        double CompassAngle;
        CompassAngle = Compass.getAngle();
        sendDouble(CompassAngle);
      }
      break;
    case HUMITURE:
      {
        uint8_t index = readBuffer(7);
        if(humiture.getPort() != port)
        {
          humiture.reset(port);
        }
        uint8_t HumitureData;
        humiture.update();
        HumitureData = humiture.getValue(index);
        sendByte(HumitureData);
      }
      break;
    case FLAMESENSOR:
      {
        if(FlameSensor.getPort() != port)
        {
          FlameSensor.reset(port);
          FlameSensor.setpin(FlameSensor.pin2(),FlameSensor.pin1());
        }
        int16_t FlameData; 
        FlameData = FlameSensor.readAnalog();
        sendShort(FlameData);
      }
      break;
    case GASSENSOR:
      {
        if(GasSensor.getPort() != port)
        {
          GasSensor.reset(port);
          GasSensor.setpin(GasSensor.pin2(),GasSensor.pin1());
        }
        int16_t GasData; 
        GasData = GasSensor.readAnalog();
        sendShort(GasData);
      }
      break;
    case  GYRO:
      {
        int axis = readBuffer(7);
        if(port == 0)
        {
          gyro.update();
          value = gyro.getAngle(axis);
          sendFloat(value);
        }
      }
      break;
    case  VERSION:
      {
        sendString(mVersion);
      }
      break;
    case  DIGITAL:
      {
        pinMode(pin,INPUT);
        sendFloat(digitalRead(pin));
      }
      break;
    case  ANALOG:
      {
        pin = analogs[pin];
        pinMode(pin,INPUT);
        sendFloat(analogRead(pin));
      }
      break;
    case  PULSEIN:
      {
        int pw = readShort(7);
        pinMode(pin, INPUT);
        sendShort(pulseIn(pin,HIGH,pw));
      }
      break;
    case ULTRASONIC_ARDUINO:
      {
        int trig = readBuffer(6);
        int echo = readBuffer(7);
        pinMode(trig,OUTPUT);
        digitalWrite(trig,LOW);
        delayMicroseconds(2);
        digitalWrite(trig,HIGH);
        delayMicroseconds(10);
        digitalWrite(trig,LOW);
        pinMode(echo, INPUT);
        sendFloat(pulseIn(echo,HIGH,30000)/58.0);
      }
      break;
    case TIMER:
      {
        sendFloat((float)currentTime);
      }
      break;
    case ENCODER_BOARD:
      {
        if(port == 0)
        {
          slot = readBuffer(7);
          uint8_t read_type = readBuffer(8);
          if(slot == SLOT_1)
          {
            if(read_type == ENCODER_BOARD_POS)
            {
              sendLong(Encoder_1.GetPulsePos());
            }
            else if(read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(Encoder_1.GetCurrentSpeed());
            }
          }
          else if(slot == SLOT_2)
          {
            if(read_type == ENCODER_BOARD_POS)
            {
              sendLong(Encoder_2.GetPulsePos());
            }
            else if(read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(Encoder_2.GetCurrentSpeed());
            }
          }
          else if(slot == SLOT_3)
          {
            if(read_type == ENCODER_BOARD_POS)
            {
              sendLong(Encoder_3.GetPulsePos());
            }
            else if(read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(Encoder_3.GetCurrentSpeed());
            }
          }
          else if(slot == SLOT_4)
          {
            if(read_type == ENCODER_BOARD_POS)
            {
              sendLong(Encoder_4.GetPulsePos());
            }
            else if(read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(Encoder_4.GetCurrentSpeed());
            }
          }
        }
      }
      break;
    case COMMON_COMMONCMD:
      {
        int8_t subcmd = port;
        if(GET_BATTERY_POWER == subcmd)
        {
          //do nothing
        }
      }
      break;    
  }//switch
}

char buf[64];
char bufindex;
void PWM_Calcu()
{
  double speed1;
  double speed2;
  if((millis() - lasttime_speed) > 20)
  {
    speed1 = Encoder_1.GetCurrentSpeed();
    speed2 = Encoder_2.GetCurrentSpeed();

#ifdef DEBUG_INFO
    Serial.print("S1: ");
    Serial.print(speed1);
    Serial.print(" S2: ");
    Serial.print(speed2);
    Serial.print("left: ");
    Serial.print(PID_speed_left.Setpoint);
    Serial.print(" right: ");
    Serial.println(PID_speed_right.Setpoint);
#endif

    if(abs(abs(PID_speed_left.Setpoint) - abs(PID_speed_right.Setpoint)) >= 0)
    {
      Encoder_1.setMotorPwm(PID_speed_left.Setpoint);
      Encoder_2.setMotorPwm(PID_speed_right.Setpoint);
      return;
    }

    if((abs(PID_speed_left.Setpoint) == 0) && (abs(PID_speed_right.Setpoint) == 0))
    {
      return;
    }

    if(abs(speed1) - abs(speed2) >= 0)
    {
      if(PID_speed_left.Setpoint > 0)
      {
        Encoder_1.setMotorPwm(PID_speed_left.Setpoint - (abs(speed1) - abs(speed2)));
        Encoder_2.setMotorPwm(PID_speed_right.Setpoint);
      }
      else
      {
        Encoder_1.setMotorPwm(PID_speed_left.Setpoint + (abs(speed1) - abs(speed2)));
        Encoder_2.setMotorPwm(PID_speed_right.Setpoint);
      }
    }
    else
    {
      if(PID_speed_right.Setpoint > 0)
      {
        Encoder_1.setMotorPwm(PID_speed_left.Setpoint);
        Encoder_2.setMotorPwm(PID_speed_right.Setpoint - (abs(speed2) - abs(speed1)));
      }
      else
      {
        Encoder_1.setMotorPwm(PID_speed_left.Setpoint);
        Encoder_2.setMotorPwm(PID_speed_right.Setpoint + (abs(speed2) - abs(speed1)));
      }
    }
    lasttime_speed = millis(); 
  }
}

void setup()
{
  delay(5);
  attachInterrupt(Encoder_1.GetIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.GetIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(Encoder_3.GetIntNum(), isr_process_encoder3, RISING);
  attachInterrupt(Encoder_4.GetIntNum(), isr_process_encoder4, RISING);
  PID_speed_left.Setpoint = 0;
  PID_speed_right.Setpoint = 0;
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  delay(100);
  Serial.print("Version: ");
  Serial.println(mVersion);
  measurement_speed_time = lasttime_speed = lasttime_angle = millis();
}

void loop()
{
  currentTime = millis()/1000.0-lastTime;
  steppers[0].runSpeedToPosition();
  steppers[1].runSpeedToPosition();
  readSerial();
  if(isAvailable)
  {
    unsigned char c = serialRead & 0xff;
    if((c == 0x55) && (isStart == false))
    {
      if(prevc == 0xff)
      {
        index=1;
        isStart = true;
      }
    }
    else
    {
      prevc = c;
      if(isStart)
      {
        if(index == 2)
        {
          dataLen = c; 
        }
        else if(index > 2)
        {
          dataLen--;
        }
        writeBuffer(index,c);
      }
    }
    index++;
    if(index > 51)
    {
      index=0; 
      isStart=false;
    }
    if(isStart && (dataLen == 0) && (index > 3))
    { 
      isStart = false;
      parseData(); 
      index=0;
    }
  }
  Encoder_1.Update_speed();
  Encoder_2.Update_speed();
  Encoder_3.Update_speed();
  Encoder_4.Update_speed();
//  if(megapi_mode == BLUETOOTH_MODE)
//  {
//    PWM_Calcu(); 
//  }
//  Encoder_1.setMotorPwm(255);
//  Encoder_2.setMotorPwm(255);
//  Encoder_3.setMotorPwm(255);
//  Encoder_4.setMotorPwm(255);
}
