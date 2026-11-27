#include <Arduino.h>
#include <defines.h>
#include <config.h>

#ifdef HOVER_SERIAL
  #include <Hoverserial.h>
  Hoverserial hoverserial(Serial2);
#endif

#ifdef REMOTE
  #include <RCSwitch.h>
  RCSwitch mySwitch = RCSwitch();
#endif

#ifdef DEBUG_SERIAL
  #include <RTTStream.h>
  RTTStream rtt;
  #include <DebugCommander.h>
  DebugCommander commander(rtt);
  int debug = 0;
#endif

#ifdef MPU6050
  #include <Adafruit_MPU6050.h>
  Adafruit_MPU6050 mpu;
  #define IMU MPU6050
#endif

#ifdef ICM20948
  #include <Adafruit_ICM20948.h>
  #include <Adafruit_ICM20X.h>
  Adafruit_ICM20948 mpu;
  #define IMU ICM20948
#endif

#ifdef IMU
  Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;
  int imu_found = 0;

  #ifdef AHRS
    #include "Adafruit_AHRS_Madgwick.h"
   Adafruit_Madgwick filter;
  #endif

#endif //IMU

#if defined(IMU) && defined(DEBUG_SERIAL)
  #include <Wire.h>
  // Read the ID registers directly, before (and independently of) the library's
  // own chip check, to identify the IMU and tell a bus problem from a wrong ID:
  //   0x75: WHO_AM_I of the MPU6050 family (MPU6050 0x68, ICM-20689 0x98, ...)
  //   0x00: WHO_AM_I of the ICM-20x48 family (ICM-20618, ICM-20948)
  static int readImuReg(uint8_t addr, uint8_t reg){
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return -1;            // no ACK
    if (Wire.requestFrom(addr, (uint8_t)1) != 1) return -1;
    return Wire.read();
  }

  static void printImuId(int id){
    if (id < 0) { DEBUG_SERIAL.print(F("--")); return; }
    DEBUG_SERIAL.print(F("0x"));
    DEBUG_SERIAL.print(id, HEX);
  }

  void probeImu(){
    Wire.begin();
    for (uint8_t addr = 0x68; addr <= 0x69; addr++){         // AD0 low / high
      DEBUG_SERIAL.print(F("IMU @0x"));
      DEBUG_SERIAL.print(addr, HEX);
      int id75 = readImuReg(addr, 0x75);
      int id00 = readImuReg(addr, 0x00);
      if (id75 < 0 && id00 < 0){
        DEBUG_SERIAL.println(F(": no ACK"));
        continue;
      }
      DEBUG_SERIAL.print(F(": reg 0x75="));
      printImuId(id75);
      DEBUG_SERIAL.print(F(" reg 0x00="));
      printImuId(id00);
      DEBUG_SERIAL.println();
    }
  }

  void onCmdWhoAmI(char* cmd){ probeImu(); }
#endif


// Callbacks for custom commander commands
#ifdef DEBUG_SERIAL
  #ifdef HOVER_SERIAL
  void onCmdSerialIn(char* cmd){ debug ^= PRINT_SERIAL_IN; }
  void onCmdSerialOut(char* cmd){ debug ^= PRINT_SERIAL_OUT; }
  #endif
  #ifdef REMOTE
  void onCmdRemote(char* cmd){ debug ^= PRINT_REMOTE; }
  #endif
  #ifdef IMU  
    void onCmdAccel(char* cmd){ debug ^= PRINT_ACCEL;}
    void onCmdGyro(char* cmd){ debug ^= PRINT_GYRO;}
    void onCmdTemp(char* cmd){ debug ^= PRINT_TEMP; }
    #ifdef AHRS
      void onCmdQuat(char* cmd){ debug ^= PRINT_QUAT; }
      void onCmdEuler(char* cmd){ debug ^= PRINT_EULER; }
    #endif    
  #endif //IMU
#endif

// ########################## SETUP ##########################
void setup(){
  // Define Leds as output
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_UP_PIN, OUTPUT);
  pinMode(LED_DOWN_PIN, OUTPUT);

  // Turn all Leds ON for half a second and then OFF
  digitalWrite(LED_RED_PIN,HIGH);
  digitalWrite(LED_GREEN_PIN,HIGH);
  digitalWrite(LED_YELLOW_PIN,HIGH);
  digitalWrite(LED_UP_PIN,HIGH);
  digitalWrite(LED_DOWN_PIN,HIGH);
  delay(500);
  digitalWrite(LED_RED_PIN,LOW);
  digitalWrite(LED_GREEN_PIN,LOW);
  digitalWrite(LED_YELLOW_PIN,LOW);
  digitalWrite(LED_UP_PIN,LOW);
  digitalWrite(LED_DOWN_PIN,LOW);
 
  // Define Sensors as input
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);

  #ifdef HOVER_SERIAL
    Serial2.begin(HOVER_SERIAL_BAUD);
  #endif

  #ifdef REMOTE
    mySwitch.enableReceive(digitalPinToInterrupt(RECEIVER_PIN));  // Receiver on interrupt
  #endif
  
  #if defined(IMU) && defined(AHRS)
    filter.begin(1000.0f / TIME_IMU);   // [Hz] filter is updated every TIME_IMU ms
  #endif

  #ifdef IMU
    #ifdef DEBUG_SERIAL
      probeImu();                                    // prints the IMU IDs even if begin() rejects the chip
      commander.add('w',onCmdWhoAmI,"IMU WHO_AM_I");
    #endif
    #ifdef MPU6050
    if (!mpu.begin()){
    #endif
    #ifdef ICM20948
    if (!mpu.begin_I2C()){
    #endif
      digitalWrite(LED_RED_PIN,HIGH);
      #ifdef DEBUG_SERIAL
      DEBUG_SERIAL.println(F("Failed to find IMU chip"));
      #endif
    }else{
      digitalWrite(LED_GREEN_PIN,HIGH);
      #ifdef DEBUG_SERIAL
      DEBUG_SERIAL.println(F("IMU found!"));
      #endif
      imu_found = 1;

      mpu_temp = mpu.getTemperatureSensor();
      #ifdef DEBUG_SERIAL
        mpu_temp->printSensorDetails();
      #endif

      mpu_accel = mpu.getAccelerometerSensor();
      #ifdef DEBUG_SERIAL
        mpu_accel->printSensorDetails();
      #endif

      mpu_gyro = mpu.getGyroSensor();
      #ifdef DEBUG_SERIAL
        mpu_gyro->printSensorDetails();
      #endif
    }
  #endif //IMU 


  // Add custom commander commands
  #ifdef DEBUG_SERIAL
    #ifdef HOVER_SERIAL
      commander.add('i',onCmdSerialIn,"Print Serial In");
      commander.add('o',onCmdSerialOut,"Print Serial Out");
    #endif
    #ifdef REMOTE
      commander.add('r',onCmdRemote,"Print Remote");
    #endif

    #ifdef IMU
      commander.add('a',onCmdAccel,"Print Accelerometer");
      commander.add('g',onCmdGyro,"Print Gyroscope");
      commander.add('t',onCmdTemp,"Print Temperature");
      #ifdef AHRS
        commander.add('q',onCmdQuat,"Print Quaternions");
        commander.add('e',onCmdEuler,"Print Euler Angles");
      #endif
    #endif
    DEBUG_SERIAL.println(F("Hoverboard Serial v1.0"));
  #endif

}


// ########################## LEDS ##########################
// With a mainboard link, the mainboard owns the LEDs (cmdLed). Without one
// (as hoverboard-sideboard-hack-STM): yellow blinks = no valid feedback,
// red blinks = IMU not found, green = IMU OK. Called every TIME_SEND ms.
static void writeLeds(uint16_t leds){
  digitalWrite(LED_RED_PIN,    leds & LED_RED_SET    ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN,  leds & LED_GREEN_SET  ? HIGH : LOW);
  digitalWrite(LED_YELLOW_PIN, leds & LED_YELLOW_SET ? HIGH : LOW);
  digitalWrite(LED_UP_PIN,     leds & LED_UP_SET     ? HIGH : LOW);
  digitalWrite(LED_DOWN_PIN,   leds & LED_DOWN_SET   ? HIGH : LOW);
}

void handleLeds(){
  static bool blink = false;
  blink = !blink;
  uint16_t leds = 0;
  #ifdef HOVER_SERIAL
    if (hoverserial.feedbackOk()){
      writeLeds(hoverserial.getCmdLed());
      return;
    }
    if (blink) leds |= LED_YELLOW_SET;
  #endif
  #ifdef IMU
    if (imu_found)  leds |= LED_GREEN_SET;
    else if (blink) leds |= LED_RED_SET;
  #endif
  writeLeds(leds);
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
#ifdef IMU
  unsigned long iTimeImu = 0;
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
#endif

void loop(){

  #ifdef HOVER_SERIAL
    // Check for new received data
    hoverserial.receive();
  #endif

  #ifdef DEBUG_SERIAL
    commander.run(); // reads Serial instance form constructor
  #endif
  
  #ifdef IMU
    // Fixed-rate IMU update: the AHRS filter assumes 1000/TIME_IMU Hz
    if (imu_found && (millis() - iTimeImu >= TIME_IMU)){
      iTimeImu += TIME_IMU;
      if (millis() - iTimeImu >= TIME_IMU) iTimeImu = millis();   // fell behind (e.g. at boot): resync, don't burst
      mpu.getEvent(&accel, &gyro, &temp);   // one I2C burst instead of three

      #ifdef AHRS
        filter.updateIMU(gyro.gyro.x * 57.29578f, 
                         gyro.gyro.y * 57.29578f,
                         gyro.gyro.z * 57.29578f,
                         accel.acceleration.x, 
                         accel.acceleration.y,
                         accel.acceleration.z);
      #endif
    }
  #endif //IMU

  // Delay
  unsigned long timeNow = millis();
  if (timeNow - iTimeSend < TIME_SEND) return;   // wrap-safe
  iTimeSend = timeNow;
  
  #ifdef IMU
    if (imu_found){
      #ifdef DEBUG_SERIAL
        if (debug & PRINT_TEMP){
          DEBUG_SERIAL.print(F("Temperature "));
          DEBUG_SERIAL.print(temp.temperature);
          DEBUG_SERIAL.println(F(" deg C"));
        }
        
        if (debug & PRINT_ACCEL){
          /* Display the results (acceleration is measured in m/s^2) */
          DEBUG_SERIAL.print(F("Accel X: "));
          DEBUG_SERIAL.print(accel.acceleration.x);
          DEBUG_SERIAL.print(F(" Y: "));
          DEBUG_SERIAL.print(accel.acceleration.y);
          DEBUG_SERIAL.print(F(" Z: "));
          DEBUG_SERIAL.print(accel.acceleration.z);
          DEBUG_SERIAL.println(F(" m/s^2 "));
        }

        if (debug & PRINT_GYRO){
          /* Display the results (rotation is measured in rad/s) */
          DEBUG_SERIAL.print(F("Gyro X: "));
          DEBUG_SERIAL.print(gyro.gyro.x);
          DEBUG_SERIAL.print(F(" Y: "));
          DEBUG_SERIAL.print(gyro.gyro.y);
          DEBUG_SERIAL.print(F(" Z: "));
          DEBUG_SERIAL.print(gyro.gyro.z);
          DEBUG_SERIAL.println(F(" radians/s "));
        }
        
        #ifdef AHRS
          if (debug & PRINT_QUAT){
            float qw, qx, qy, qz;
            filter.getQuaternion(&qw, &qx, &qy, &qz);
            DEBUG_SERIAL.print(F("qW: "));
            DEBUG_SERIAL.print(qw);
            DEBUG_SERIAL.print(F(" qX: "));
            DEBUG_SERIAL.print(qx);
            DEBUG_SERIAL.print(F(" qY: "));
            DEBUG_SERIAL.print(qy);
            DEBUG_SERIAL.print(F(" qZ: "));
            DEBUG_SERIAL.println(qz);
          }

          if (debug & PRINT_EULER){
            DEBUG_SERIAL.print(F("Roll: "));
            DEBUG_SERIAL.print(filter.getRoll());
            DEBUG_SERIAL.print(F(" Pitch: "));
            DEBUG_SERIAL.print(filter.getPitch());
            DEBUG_SERIAL.print(F(" Yaw: "));
            DEBUG_SERIAL.println(filter.getYaw());
          }
        #endif //AHRS
      #endif
    }
  #endif //IMU

  #ifdef REMOTE
    if (mySwitch.available()) {
      #ifdef DEBUG_SERIAL
        if (debug & PRINT_REMOTE){
          DEBUG_SERIAL.print(F("Received "));
          DEBUG_SERIAL.print(mySwitch.getReceivedValue());
          DEBUG_SERIAL.print(F(" / "));
          DEBUG_SERIAL.print(mySwitch.getReceivedBitlength());
          DEBUG_SERIAL.print(F("bit "));
          DEBUG_SERIAL.print(F("Protocol: "));
          DEBUG_SERIAL.println(mySwitch.getReceivedProtocol());
        }
      #endif

      int button_pressed = 0;
      switch(mySwitch.getReceivedValue()){
        case REMOTE_BUTTON1:
          button_pressed = 1;
          #ifdef HOVER_SERIAL
            hoverserial.setCtrlTyp(FOC_CTRL);
          #endif
          break;
        case REMOTE_BUTTON2:
          button_pressed = 2;
          #ifdef HOVER_SERIAL
            hoverserial.setCtrlTyp(SIN_CTRL);
          #endif
          break;
        case REMOTE_BUTTON3:
          button_pressed = 3;
          #ifdef HOVER_SERIAL
            hoverserial.setCtrlTyp(COM_CTRL);
          #endif
          break;
        case REMOTE_BUTTON4:
          button_pressed = 4;
          break;
      }

      #ifdef DEBUG_SERIAL 
        if (button_pressed > 0){
          DEBUG_SERIAL.print(F("Button "));
          DEBUG_SERIAL.print(button_pressed); 
          DEBUG_SERIAL.println(F(" pressed"));
        }
      #endif

      mySwitch.resetAvailable();
    }
  #endif

  #ifdef HOVER_SERIAL
    #ifdef IMU
      hoverserial.setImuStatus(imu_found);
      #ifdef AHRS
        if (imu_found){
          hoverserial.setPitch((int16_t)(filter.getPitch() * 100.0f),             // [deg*100]
                               (int16_t)CLAMP(gyro.gyro.y * 57.29578f * 16.4f, -32768.0f, 32767.0f)); // raw LSB at 2000 deg/s
        }
      #endif
    #endif
    #ifdef TEST
      hoverserial.test();
    #else
      hoverserial.setCmd1(0);
      hoverserial.setCmd2(0);
    #endif
    hoverserial.send();
  #endif

  handleLeds();
}
