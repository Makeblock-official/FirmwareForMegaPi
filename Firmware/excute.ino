void runModule(int device)
{
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
  int port = readBuffer(6);
  int pin = port;
  int speed;
  int position;
  int distance;
  int type;
  int slot;
  int extId;
  switch(device)
  {
    case MOTOR:
      {
        speed = readShort(7);
        dc.reset(port);
        dc.run(speed);
      }
      break;
    case ENCODER_BOARD:
      type = readBuffer(7);
      switch(type){
        case ENCODER_BOARD_RUN:
        {
          if(port == 0)
          {
            slot = readBuffer(7);
            int speed_value = readShort(8);
            encoders[slot-1].runSpeed(speed_value);
          }
        }
        break;
        case ENCODER_BOARD_MOVE:
        {
          if(port == 0)
          {
            slot = readBuffer(7);
            int speed_value = readShort(8);
            int distance_value = readShort(10);
            encoders[slot-1].setSpeed(speed_value);
            extId = readBuffer(3);
            encoders[slot-1].move(distance_value,onEncoderMovingFinish,extId);
          }
        }
        break;
        case ENCODER_BOARD_MOVE_TO:
        {
          if(port == 0)
          {
            slot = readBuffer(7);
            int speed_value = readShort(8);
            int position_value = readShort(10);
            encoders[slot-1].setSpeed(speed_value);
            extId = readBuffer(3);
            encoders[slot-1].moveTo(position_value,onEncoderMovingFinish,extId);
          }
        }
        break;
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
    case STEPPER_BOARD:
      {
        type = readBuffer(7);
        switch(type){
          case STEPPER_BOARD_RUN:
            speed = readShort(8);
            steppers[port-1].setSpeed(speed);
          break;
          case STEPPER_BOARD_MOVE:
            speed = readShort(8);
            distance = readShort(10);
            extId = readBuffer(3);
            steppers[port-1].setSpeed(speed);
            steppers[port-1].move(distance,onStepperMovingFinish,extId);
          break;
          case STEPPER_BOARD_MOVE_TO:
            speed = readShort(8);
            position = readShort(10);
            extId = readBuffer(3);
            steppers[port-1].setSpeed(speed);
            steppers[port-1].moveTo(position,onStepperMovingFinish,extId);
          break;
        }
      } 
      break;
    case RGBLED:
      {
        slot = readBuffer(7);
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
        slot = readBuffer(7);
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
              sendLong(encoders[0].GetPulsePos());
            }
            else if(read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(encoders[0].GetCurrentSpeed());
            }
          }
          else if(slot == SLOT_2)
          {
            if(read_type == ENCODER_BOARD_POS)
            {
              sendLong(encoders[1].GetPulsePos());
            }
            else if(read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(encoders[1].GetCurrentSpeed());
            }
          }
          else if(slot == SLOT_3)
          {
            if(read_type == ENCODER_BOARD_POS)
            {
              sendLong(encoders[2].GetPulsePos());
            }
            else if(read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(encoders[2].GetCurrentSpeed());
            }
          }
          else if(slot == SLOT_4)
          {
            if(read_type == ENCODER_BOARD_POS)
            {
              sendLong(encoders[3].GetPulsePos());
            }
            else if(read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(encoders[3].GetCurrentSpeed());
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
void resetAll()
{
  encoders[0].setMotorPwm(0);
  encoders[1].setMotorPwm(0);
  encoders[2].setMotorPwm(0);
  encoders[3].setMotorPwm(0);
  encoders[0].SetPulsePos(0);
  encoders[1].SetPulsePos(0);
  encoders[2].SetPulsePos(0);
  encoders[3].SetPulsePos(0);
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
