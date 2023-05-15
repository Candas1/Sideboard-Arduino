// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

//#define DEBUG
#define MPU6050
//#define ICM20948


#if defined(MPU6050) + defined(ICM20948) > 1
  #error Only one IMU is allowed
#endif

#endif //  CONFIG_H
