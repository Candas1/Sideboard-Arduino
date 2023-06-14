// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H


// SEGGER RTT Debug
#define DEBUG_SERIAL rtt // Uncomment to enable DEBUG
#ifdef DEBUG_SERIAL
  #undef BUFFER_SIZE_UP
  #define BUFFER_SIZE_UP (512)  // Size of the buffer for terminal output of target, up to host (Default: 1k)
#endif

// Hover serial protocol
#define HOVER_SERIAL                    // Send commands to the mainboard and receive feedback
#ifdef HOVER_SERIAL
  #define HOVER_SERIAL_BAUD   115200    // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#endif

// Test
#define TEST                            // Will send test commands to the mainboard, HOVER_SERIAL should also be enabled
#ifdef TEST
  #define SPEED_MAX_TEST      50        // [-] Maximum speed for testing
  #define SPEED_STEP          2         // [-] Speed step
#endif

#define TIME_SEND           100         // [ms] Sending time interval

// 433Mhz RF Remote
//#define REMOTE                          // Uncomment to enable 433Mhz Receiver. 
#ifdef REMOTE 
  #define REMOTE_BUTTON1 6637793        // Switch to FOC control type
  #define REMOTE_BUTTON2 6637794        // Switch to SIN control type
  #define REMOTE_BUTTON3 6637796        // Switch to COM control type
  #define REMOTE_BUTTON4 6637800        // Does nothing for now
#endif 

//#define MPU6050
//#define ICM20948
//#define AHRS

#if (defined(MPU6050) + defined(ICM20948)) > 1
  #error Only one IMU is allowed
#endif

#endif //  CONFIG_H
