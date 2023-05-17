#include <Arduino.h>
#include <defines.h>
#include <config.h>

#ifdef DEBUG_SERIAL
#include <RTTStream.h>
RTTStream rtt;
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
#endif //IMU

#ifdef HOVER_SERIAL
  // Global variables
  uint8_t idx = 0;                        // Index for new data pointer
  uint16_t bufStartFrame;                 // Buffer Start Frame
  byte *p;                                // Pointer declaration for the new received data
  byte incomingByte;
  byte incomingBytePrev;

typedef struct{
  uint16_t  start;
  int16_t   pitch;      // Angle
  int16_t   dPitch;     // Angle derivative
  int16_t   cmd1;       // RC Channel 1
  int16_t   cmd2;       // RC Channel 2
  uint16_t  sensors;    // RC Switches and Optical sideboard sensors
  uint16_t  checksum;
} SerialCommand;
SerialCommand Command;


typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;
#endif

// ########################## SETUP ##########################
void setup(){
  
  // Define Leds as output
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);

  // Turn all Leds ON for half a second and then OFF
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED3,HIGH);
  digitalWrite(LED4,HIGH);
  digitalWrite(LED5,HIGH);
  delay(500);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);
  digitalWrite(LED4,LOW);
  digitalWrite(LED5,LOW);
 
  // Define Sensors as input
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);

  #ifdef HOVER_SERIAL
  Serial2.begin(HOVER_SERIAL_BAUD);
  #endif

  #ifdef DEBUG_SERIAL
    Serial.println("Hoverboard Serial v1.0");
  #endif
  
  #ifdef IMU
    #ifdef MPU6050
    if (!mpu.begin()){
    #endif
    #ifdef ICM20948
    if (!mpu.begin_I2C()){
    #endif
      digitalWrite(LED1,HIGH);
      #ifdef DEBUG_SERIAL
      Serial.println(F("Failed to find IMU chip"));
      #endif
    }else{
      digitalWrite(LED2,HIGH);
      #ifdef DEBUG_SERIAL
      Serial.println(F("IMU found!"));
      #endif

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
        Serial.println(F("Setup end!"));
      #endif
    }
  #endif //IMU 
}


#ifdef HOVER_SERIAL  
/*
 * Send command
 */
void Send(int16_t uSteer, int16_t uSpeed)
{
  uint16_t cmdSwitch;
  cmdSwitch   = (uint16_t)(
                 1 | // Aux input OFF
                 2 << 1 | // 0:Commutation 1:Sinusoidal 2:FOC
                 3 << 3 | // 0:OPEN 1:VOLTAGE 2:SPEED 3:TORQUE
                 0 << 5); // Field Weakening OFF
                                     
  // Create command
  Command.start   = (uint16_t)START_FRAME;
  Command.pitch   = 0;
  Command.dPitch  = 0;
  Command.cmd1    = (int16_t)uSteer;
  Command.cmd2    = (int16_t)uSpeed;
  Command.sensors = (uint16_t)( (cmdSwitch << 8)  | (digitalRead(SENSOR1) | (digitalRead(SENSOR2) << 1) | (0 << 2)) );
  Command.checksum = (uint16_t)(Command.start ^ Command.pitch ^ Command.dPitch ^ Command.cmd1 ^ Command.cmd2 ^ Command.sensors);
  
  // Write to Serial
  Serial2.write((uint8_t *) &Command, sizeof(Command)); 
}

/*
 * Receive feedback
 */
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (Serial2.available()) {
      incomingByte 	  = Serial2.read();                                   // Read the incoming byte
      bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
  }
  else {
      return;
  }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
      Serial.print(incomingByte);
      return;
  #endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
      p       = (byte *)&NewFeedback;
      *p++    = incomingBytePrev;
      *p++    = incomingByte;
      idx     = 2;	
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
      *p++    = incomingByte; 
      idx++;
  }	
  
  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
      uint16_t checksum;
      checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

      // Check validity of the new data
      if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
          // Copy the new data
          memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

          #ifdef DEBUG_SERIAL
            // Print data to built-in Serial
            Serial.println();
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
          #endif

          // handle Leds
          digitalWrite(LED1,Feedback.cmdLed & LED1_SET?HIGH:LOW);
          digitalWrite(LED2,Feedback.cmdLed & LED2_SET?HIGH:LOW);
          digitalWrite(LED3,Feedback.cmdLed & LED3_SET?HIGH:LOW);
          digitalWrite(LED4,Feedback.cmdLed & LED4_SET?HIGH:LOW);
          digitalWrite(LED5,Feedback.cmdLed & LED5_SET?HIGH:LOW);
      } else {
        #ifdef DEBUG_SERIAL
          Serial.println("Non-valid data skipped");
        #endif
      }
      idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}
#endif

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;

#ifdef TEST
int iTest = 0;
int iStep = SPEED_STEP;
#endif

void loop(){
  #ifdef IMU
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    
    // Delay
    unsigned long timeNow = millis();
    if (iTimeSend > timeNow) return;
    iTimeSend = timeNow + TIME_SEND;
 
    mpu_temp->getEvent(&temp);
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);

    #ifdef DEBUG_SERIAL
      Serial.print("Temperature ");
      Serial.print(temp.temperature);
      Serial.println(" deg C");

      /* Display the results (acceleration is measured in m/s^2) */
      Serial.print("Accel X: ");
      Serial.print(accel.acceleration.x);
      Serial.print(" \tY: ");
      Serial.print(accel.acceleration.y);
      Serial.print(" \tZ: ");
      Serial.print(accel.acceleration.z);
      Serial.println(" m/s^2 ");

      /* Display the results (rotation is measured in rad/s) */
      Serial.print("Gyro X: ");
      Serial.print(gyro.gyro.x);
      Serial.print(" \tY: ");
      Serial.print(gyro.gyro.y);
      Serial.print(" \tZ: ");
      Serial.print(gyro.gyro.z);
      Serial.println(" radians/s ");
      Serial.println();
    #endif
  #endif //IMU

  #ifdef HOVER_SERIAL
    // Check for new received data
    Receive();
 
    #ifdef TEST
      Send(0, iTest);
      // Calculate test command signal
      iTest += iStep;

      // invert step if reaching limit
      if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
        iStep = -iStep;
    #else
      Send(0,0);
    #endif
  #endif
}