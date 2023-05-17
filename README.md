# Sideboard-Arduino

There are custom firmwares available for Hoverboards [sideboards](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Sideboards).<br>
The challenge is that there is a separate firmware for different chip brand/families using different drivers (SPL for GD32F130C8/C6 or STM32 Hal for STM32F103C8/C6 and GD32F103C8/C6).<br> This duplicates the effort for any enhancement.<br>

This firmware is a tentative to unify the code, using the Arduino platform on Platformio thanks to projects like [STM32duino](https://github.com/stm32duino) and [CommunityGD32Cores](https://github.com/CommunityGD32Cores).<br>

The benefits would be:
* Being able to run the same firmware on sideboards, Arduino, ESP8288/ESP32
* Arduino code would make this firmware more inclusive for contributions
* Many available librairies that can help extend the support of new IMUs or sensors (encoders,displays,....)
* Bigger community  
* Possibility to later extend this firmware with SimpleFOC motor control on [splitboards](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Firmware-Compatibility#splitboards) ( Can be GD32F130C8/C6, STM32F103C8/C6, GD32F103C8/C6, GD32F130K6, GD32E230K6 )

For now it's only a Proof of concept.



