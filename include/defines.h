// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H

#define LED1    PA0 // RED
#define LED2    PB9 // GREEN
#define LED3    PB8 // YELLOW
#define LED4    PB5 // BLUE1
#define LED5    PB4 // BLUE2

#define SENSOR1 PA4
#define SENSOR2 PC14

#define MPU_SCL PB6
#define MPU_SDA PB7

#define LED1_SET                    (0x01)
#define LED2_SET                    (0x02)
#define LED3_SET                    (0x04)
#define LED4_SET                    (0x08)
#define LED5_SET                    (0x10)

#endif //  DEFINES_H
