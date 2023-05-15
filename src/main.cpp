#include <Arduino.h>
#include <defines.h>
#include <config.h>

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

void setup(){
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);

  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) delay(10); // will pause Zero, Leonardo, etc until Serial console opens
  #endif //DEBUG
  
  #ifdef IMU
    #ifdef MPU6050
    if (!mpu.begin()){
    #endif
    #ifdef ICM20948
    if (!mpu.begin_I2C()){
    #endif
      digitalWrite(LED1,HIGH);
      #ifdef DEBUG
      Serial.println("Failed to find IMU chip");
      #endif
    }else{
      digitalWrite(LED2,HIGH);
      #ifdef DEBUG
      Serial.println("IMU found!");
      #endif
    }

    mpu_temp = mpu.getTemperatureSensor();
    #ifdef DEBUG
      mpu_temp->printSensorDetails();
    #endif

    mpu_accel = mpu.getAccelerometerSensor();
    #ifdef DEBUG
      mpu_accel->printSensorDetails();
    #endif

    mpu_gyro = mpu.getGyroSensor();
    #ifdef DEBUG
      mpu_gyro->printSensorDetails();
      Serial.println("Setup end!");
    #endif
  #endif //IMU 
}


void loop(){
  #ifdef IMU
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    mpu_temp->getEvent(&temp);
    mpu_accel->getEvent(&accel);
    mpu_gyro->getEvent(&gyro);

    #ifdef DEBUG
      Serial.print("\t\tTemperature ");
      Serial.print(temp.temperature);
      Serial.println(" deg C");

      /* Display the results (acceleration is measured in m/s^2) */
      Serial.print("\t\tAccel X: ");
      Serial.print(accel.acceleration.x);
      Serial.print(" \tY: ");
      Serial.print(accel.acceleration.y);
      Serial.print(" \tZ: ");
      Serial.print(accel.acceleration.z);
      Serial.println(" m/s^2 ");

      /* Display the results (rotation is measured in rad/s) */
      Serial.print("\t\tGyro X: ");
      Serial.print(gyro.gyro.x);
      Serial.print(" \tY: ");
      Serial.print(gyro.gyro.y);
      Serial.print(" \tZ: ");
      Serial.print(gyro.gyro.z);
      Serial.println(" radians/s ");
      Serial.println();
    #endif
  #endif //IMU

  delay(100);
}