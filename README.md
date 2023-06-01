# Sideboard-Arduino

### Why yet another repository ?
There are custom firmwares available for Hoverboards [sideboards](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Sideboards).<br>
The challenge is that there is a separate firmware for different chip brand/families using different drivers (SPL for GD32F130C8/C6 or STM32 Hal for STM32F103C8/C6 and GD32F103C8/C6).<br> This duplicates the effort for any enhancement.<br>
This firmware is a tentative to unify the code, using the Arduino platform on Platformio thanks to projects like [STM32duino](https://github.com/stm32duino) and [CommunityGD32Cores](https://github.com/CommunityGD32Cores).<br>

### Benefits
* Being able to run the same firmware on most of the sideboards, and potentially on Arduino boards/ESP8288/ESP32 with small changes
* Arduino code would make this firmware more inclusive for contributions, and would leverage a bigger community
* Many available librairies that can help extend the support of new IMUs,sensors,encoders,displays,....
* Possibility to later extend this firmware with SimpleFOC motor control for [splitboards](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Firmware-Compatibility#splitboards) ( Can be GD32F130C8/C6, STM32F103C8/C6, GD32F103C8/C6, GD32F130K6, GD32E230K6 )

### IMUs
I aim with this firmware to support more IMUs by only retrieving raw data and doing the angle calculation on the MCU, so no DMP features will be supported.<br>
This should be enough for balancing use cases.

### How to use
I am only using Platformio, I cannot support other IDEs.<br>
* Install [Vscode](https://code.visualstudio.com/download)
* Install [Platformio](https://platformio.org/platformio-ide) for the website or from VSCODE's extensions
* Download the project or use git to clone to repository in VSCODE
* Select the environment that matches with your chip in platformio's bottom bar (For GD32F108C8 and STM32FEBK chips, STM32F103 should work) <br>

### Status:
This is a very early development so it requires more testing and documentation.<br>
|                          |STM32F103C8|GD32F130C6| Comment                  |  
|--------------------------|-----------|----------|--------------------------| 
| Leds                     |✔️        |✔️       |                          |
| Sensors                  |✔️        |✔️       |                          |
| Segger RTT debug         |✔️        |✔️       |                          |
| Usart command send       |✔️        |✔️       |                          |
| Usart feedback receive   |✔️        |Stops after a few seconds ([issue](https://github.com/CommunityGD32Cores/ArduinoCore-GD32/issues/76))       |                          |
| 433mhz remote            |✔️        |✔️       |                          |
| MPU6050 raw data         |✔️        |          |                          |
| MPU6050A raw data        |           |          |                          |
| MPU6050C raw data        |✔️        |          |                          |
| MPU6052C raw data        |❌        |❌       | Requires I2C single byte |
| Pitch/Roll angle calculation    |           |          | Use [this](https://github.com/arduino-libraries/MadgwickAHRS)?                         |
| SimpleFOC trapezoidal    |        |          |                          |
| SimpleFOC FOC            |        |          |                          |

