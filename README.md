# Sideboard-Arduino

### Why yet another repository ?
There are custom firmwares available for Hoverboards [sideboards](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Sideboards).<br>
The challenge is that there is a separate firmware for different chip brand/families using different drivers (SPL for GD32F130C8/C6 or STM32 Hal for STM32F103C8/C6 and GD32F103C8/C6).<br> This duplicates the effort for any enhancement.<br>
This firmware is a tentative to unify the code, using the Arduino platform on Platformio thanks to projects like [STM32duino](https://github.com/stm32duino) and [CommunityGD32Cores](https://github.com/CommunityGD32Cores/ArduinoCore-GD32).<br>

### Benefits
* Being able to run the same firmware on most of the sideboards, and potentially on Arduino boards/ESP8288/ESP32 with small changes
* Arduino code would make this firmware more inclusive for contributions, and would leverage a bigger community
* Many available librairies that can help extend the support of new IMUs,sensors,encoders,displays,....
* Possibility to later extend this firmware with SimpleFOC motor control for [splitboards](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Firmware-Compatibility#splitboards) ( Can be GD32F130C8/C6, STM32F103C8/C6, GD32F103C8/C6, GD32F130K6, GD32E230K6 )

### How to use
I am only using Platformio, I cannot support other IDEs.<br>
* Install [Vscode](https://code.visualstudio.com/download)
* Install [Platformio](https://platformio.org/platformio-ide) from the website or from VSCODE's extensions
* Download the project or use git to clone this repository in VSCODE
* Select the environment that matches with your chip in platformio's bottom bar (For GD32F108C8 and STM32FEBK chips, STM32F103 should work) <br>
* [Unlock](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/How-to-Unlock-MCU-Flash) the chip
* Press the 'PlatformIO:Upload' button (bottom left in vscode) to build and upload the firmwware

### Debug
Enable debug by uncommenting DEBUG_SERIAL in the config.h file.<br>
[SEGGER RTT](https://www.segger.com/products/debug-probes/j-link/technology/about-real-time-transfer/) let's you debug without the need for an USART to USB bridge. It uses the STLINK and the programming pins.<br>
Only downside is that you need to make sure you stop the rtt connection before flashing.<br>
[This](https://www.youtube.com/watch?v=_vNCye_IlYU) video shows how to use it for monitoring in Platformio.<br>
The debug menu options depend on what features are enabled in config.h.<br>

```
=== HELP ===
h: Print Help
s: Print Serial
r: Print Remote
a: Print Accelerometer
g: Print Gyroscope
q: Print Quaternion
e: Print Euler angles
t: Print Temperature
============
```

### Remote
Some of the sideboards have a 433Mhz receiver and a remote.<br>
You can enable this feature by uncommenting REMOTE in the config.h file.<br>
To identify the code of each remote button, enable [DEBUG](https://github.com/Candas1/Sideboard-Arduino#debug) and use the r command to see the remote output.<br>
Update the button codes in the config.h file.<br>

```
#define REMOTE_BUTTON1 6637793        // Switch to FOC control type
#define REMOTE_BUTTON2 6637794        // Switch to SIN control type
#define REMOTE_BUTTON3 6637796        // Switch to COM control type
#define REMOTE_BUTTON4 6637800        // Does nothing for now
```

### IMUs
I aim with this firmware to support more IMUs by only retrieving raw data and doing the angle calculation on the MCU, so no DMP features will be supported. This should be enough for balancing use cases.<br>
I also would like to implement a test mode to identify the IMU like [this](https://github.com/Levi--G/IMU-WhoAmIVerifier) project does as the writing on the chip is not always accurate.

### Status:
This is a very early development so it requires more testing and documentation.<br>
|                          |STM32F103C8|GD32F130C6| Comment                  |  
|--------------------------|-----------|----------|--------------------------| 
| Leds                     |✔️        |✔️       |                          |
| Sensors                  |✔️        |✔️       |                          |
| Segger RTT debug         |✔️        |✔️       | Use simpleFOC commander custom commands ?                         |
| Usart command send       |✔️        |✔️       |                          |
| Usart feedback receive   |✔️        |Stops working after some time ([issue](https://github.com/CommunityGD32Cores/ArduinoCore-GD32/issues/76)),fix to be tested       |                          |
| 433mhz remote            |✔️        |✔️       |                          |
| MPU6050 raw data         |✔️        | I2c fix to be tested ([issue](https://github.com/CommunityGD32Cores/ArduinoCore-GD32/issues/99))         |                          |
| MPU6050A raw data        |           |          | Different WHOAMI        |
| MPU6050C raw data        |✔️        |          |                          |
| MPU6052C raw data        |❌        |❌       | Requires [I2C single byte](https://github.com/EFeru/hoverboard-sideboard-hack-STM/pull/10) |
| ICM-20X raw data         |         |          | Try [this](https://github.com/adafruit/Adafruit_ICM20X)                         |
| Pitch/Roll angle calculation    |           |          | Use [this](https://github.com/adafruit/Adafruit_AHRS)?                         |
| SimpleFOC trapezoidal    |        |          | Needs a new GD32 driver (work in progress [here](https://github.com/Candas1/Arduino-FOC/tree/master/src))<br> Should already work on STM32 splitboards                         |
| SimpleFOC Sinusoidal     |        |          | Needs [sensor smoothing](https://community.simplefoc.com/t/smoothingsensor-experimental-sensor-angle-extrapoltion/3105), already in dev branch of SimpleFOC |
| SimpleFOC FOC            |        |          | Needs a new GD32 driver for current sensing       |
