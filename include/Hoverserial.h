#ifndef Hoverserial_h
#define Hoverserial_h

#include <Arduino.h>
#include <Stream.h>

typedef enum{
    FOC_CTRL,
    SIN_CTRL,
    COM_CTRL
}ctrlTyp_t;

typedef enum{
    VLT_MODE,
    SPD_MODE,
    TRQ_MODE
}ctrlMod_t;

typedef struct{
  uint16_t  start;
  int16_t   pitch;      // Angle
  int16_t   dPitch;     // Angle derivative
  int16_t   cmd1;       // RC Channel 1
  int16_t   cmd2;       // RC Channel 2
  uint16_t  sensors;    // RC Switches and Optical sideboard sensors
  uint16_t  checksum;
} SerialCommand;

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

class Hoverserial
{
  public:
    Hoverserial(Stream &serial);
    void setCmd1(int16_t cmd1);
    void setCmd2(int16_t cmd2);
    void setCtrlMod(ctrlMod_t ctrlMod);
    void setCtrlTyp(ctrlTyp_t ctrlTyp);
    void setFieldWeak(uint8_t fieldWeak);
    void setAux(uint8_t aux);
    void setLock(uint8_t lock);
    void send(void);
    void receive(void);
  private:
    int16_t _cmd1;
    int16_t _cmd2;
    ctrlTyp_t _ctrlTyp;
    ctrlMod_t _ctrlMod;
    uint8_t _fieldWeak;
    uint8_t _aux;
    uint8_t _lock;
    Stream &_serial;
    SerialFeedback _Feedback;
    SerialFeedback _NewFeedback;
    uint8_t _idx;                            // Index for new data pointer
    uint16_t _bufStartFrame;                 // Buffer Start Frame
    byte *_p;                                // Pointer declaration for the new received data
    byte _incomingByte;
    byte _incomingBytePrev;
};

#endif