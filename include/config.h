// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H


// Features are enabled with build_flags in platformio.ini ([features] section):
//   DEBUG_RTT         SEGGER RTT debug
//   HOVER_SERIAL      Send commands to the mainboard and receive feedback
//   TEST              Send test commands to the mainboard (needs HOVER_SERIAL)
//   REMOTE            433Mhz receiver
//   MPU6050 / ICM20948, AHRS

// Debug output stream. Defined here, not as a build flag, because libraries
// (e.g. Adafruit BusIO) use their own DEBUG_SERIAL macro.
#ifdef DEBUG_RTT
  #define DEBUG_SERIAL rtt
#endif

// Hover serial protocol
#ifdef HOVER_SERIAL
  #define HOVER_SERIAL_BAUD   115200    // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#endif

// Test
#ifdef TEST
  #define SPEED_MAX_TEST      50        // [-] Maximum speed for testing
  #define SPEED_STEP          2         // [-] Speed step
#endif

#define TIME_SEND           100         // [ms] Sending time interval
#define TIME_IMU            10          // [ms] IMU read / AHRS update interval (fixed rate for the filter)

// 433Mhz RF Remote
#ifdef REMOTE 
  #define REMOTE_BUTTON1 6637793        // Switch to FOC control type
  #define REMOTE_BUTTON2 6637794        // Switch to SIN control type
  #define REMOTE_BUTTON3 6637796        // Switch to COM control type
  #define REMOTE_BUTTON4 6637800        // Does nothing for now
#endif 

#if (defined(MPU6050) + defined(ICM20948)) > 1
  #error Only one IMU is allowed
#endif

#endif //  CONFIG_H
