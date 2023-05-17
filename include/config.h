// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H


// SEGGER RTT Debug
#define DEBUG_SERIAL // Uncomment to enable Debug
#ifdef DEBUG_SERIAL
  #undef Serial
  #define Serial rtt
#endif

// Hover serial protocol
#define HOVER_SERIAL                  // Send commands to the mainboard and receive feedback
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval

// RF Receiver
#define RECEIVER

// Test
#define TEST                          // Will send test commands to the mainboard
#define SPEED_MAX_TEST      50          // [-] Maximum speed for testing
#define SPEED_STEP          2           // [-] Speed step

// Control
#define COM_CTRL        0               // [-] Commutation Control Type
#define SIN_CTRL        1               // [-] Sinusoidal Control Type
#define FOC_CTRL        2               // [-] Field Oriented Control (FOC) Type

#define OPEN_MODE       0               // [-] OPEN mode
#define VLT_MODE        1               // [-] VOLTAGE mode
#define SPD_MODE        2               // [-] SPEED mode
#define TRQ_MODE        3               // [-] TORQUE mode

//#define MPU6050
//#define ICM20948


#if (defined(MPU6050) + defined(ICM20948)) > 1
  #error Only one IMU is allowed
#endif

#endif //  CONFIG_H
