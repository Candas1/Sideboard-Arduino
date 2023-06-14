#include <Arduino.h>
#include <Defines.h>
#include <Config.h>
#include <SimpleFOC.h>

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
  Commander commander = Commander(rtt);
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

  BLDCDriver6PWM motor = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15);

  #ifdef HOVER_SERIAL
    Serial2.begin(HOVER_SERIAL_BAUD);
  #endif

  #ifdef REMOTE
    mySwitch.enableReceive(digitalPinToInterrupt(RECEIVER_PIN));  // Receiver on interrupt
  #endif
  
  #ifdef IMU
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


// ########################## LOOP ##########################

unsigned long iTimeSend = 0;

void loop(){

  #ifdef HOVER_SERIAL
    // Check for new received data
    hoverserial.receive();
  #endif

  #ifdef DEBUG_SERIAL
    commander.run(); // reads Serial instance form constructor
  #endif
  
  #ifdef IMU
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    if (imu_found){ 
      mpu_temp->getEvent(&temp);
      mpu_accel->getEvent(&accel);
      mpu_gyro->getEvent(&gyro);

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
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  
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
          hoverserial.setCtrlTyp(FOC_CTRL);
          break;
        case REMOTE_BUTTON2:
          button_pressed = 2;
          hoverserial.setCtrlTyp(SIN_CTRL);
          break;
        case REMOTE_BUTTON3:
          button_pressed = 3;
          hoverserial.setCtrlTyp(COM_CTRL);
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
    #ifdef TEST
      hoverserial.test();
    #else
      hoverserial.setCmd1(0);
      hoverserial.setCmd2(0);
    #endif
    hoverserial.send();
  #endif
}