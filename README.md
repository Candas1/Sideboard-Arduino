# Sideboard-Arduino

There are custom firmwares available for Hoverboards [sideboards](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Sideboards).<br>
The challenge is that there is a separate firmware for different chip brand/families using different drivers (SPL for GD32F130 or STM32 Hal for STM32F103).<br> This duplicates the effort for any enhancement.<br>

This firmware is work in progress and is a tentative to unify the code, using the Arduino platform on Platformio thanks to projects like [STM32duino](https://github.com/stm32duino) and [CommunityGD32Cores](https://github.com/CommunityGD32Cores).<br>

The benefits would be:
* Being able to run the same firmware on sideboards, Arduino, ESP8288/ESP32
* Arduino code would make this firmware more inclusive for contributions
* Many available librairies that can help extend the support of new IMUs or sensors (encoders,displays,....)
* Bigger community  
* Possibility to even extend this firmware with SimpleFOC motor control on [splitboards](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki/Firmware-Compatibility#splitboards)
