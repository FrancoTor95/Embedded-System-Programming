# Exercises for the ESP course
Exercises for **E**mbedded **S**ystem **P**rogramming. Everithing is diveded into sections each of which related to specific topics.
Here students can find a list of useful materials, concepts and datasheets related to the ESP course.
Every lesson will be updated during the course.

## References and usefull notes

### Software e Development Environment
* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html#tools-software)

### Hardware -suggested-
* [STM32 Nucleo-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)
* [Base components](https://www.amazon.it/dp/B01MQIO78W)
* [Sensors](https://www.amazon.it/dp/B01N79PG4G)
* [Detailed list of materials used during the course](https://github.com/FrancoTor95/Embedded-Test#detailed-list-of-the-materials-used)

### References
* [Nucleo 64 - User Manual](https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo-64-boards-mb1136-stmicroelectronics.pdf)
* [STM32F446xx - Reference Manual](https://www.st.com/resource/en/reference_manual/dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
* [STM32F446xC/E - Datasheet](https://www.st.com/resource/en/datasheet/stm32f446mc.pdf)
* [STM32 Cortex M4 Programming Manual](https://www.st.com/resource/en/programming_manual/dm00046982-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf)
* [STM32 Description of STM32F4 HAL and LL drivers](https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32f4-hal-and-ll-drivers-stmicroelectronics.pdf)
* [Developing applications on STM32Cube with RTOS - User Manual](https://www.st.com/resource/en/user_manual/dm00105262-developing-applications-on-stm32cube-with-rtos-stmicroelectronics.pdf)
* [STM32CubeMX for STM32 user manual](https://www.st.com/content/ccc/resource/technical/document/user_manual/10/c5/1a/43/3a/70/43/7d/DM00104712.pdf/files/DM00104712.pdf/jcr:content/translations/en.DM00104712.pdf)


### Books
* [**Modern C** | Jens Gustedt](https://www.manning.com/books/modern-c)
* [**Mastering the STM32 Microcontroller** | Carmine Noviello](https://leanpub.com/mastering-stm32)
* [**The Definitive Guide to ARM Cortex-M3 and Cortex-M4 Processors** | Joseph Yiu](https://www.amazon.it/Definitive-Guide-Cortex®-M3-Cortex®-M4-Processors/dp/0124080820)
* [**STM32 Arm Programming for Embedded Systems** |  M.A. Mazidi, S. Chen, E. Ghaemi](https://www.amazon.it/STM32-Arm-Programming-Embedded-Systems/dp/0997925949/)

### Header pinout
* [CN5 Header Pinout](https://github.com/FrancoTor95/Embedded-Test/blob/main/HeadersPinout.md#cn5-header-pinout)
* [CN6 Header Pinout](https://github.com/FrancoTor95/Embedded-Test/blob/main/HeadersPinout.md#cn6-header-pinout)
* [CN7 Header Pinout](https://github.com/FrancoTor95/Embedded-Test/blob/main/HeadersPinout.md#cn7-header-pinout)
* [CN10 Header Pinout](https://github.com/FrancoTor95/Embedded-Test/blob/main/HeadersPinout.md#cn10-header-pinout)


## Topics
* [C basic programming](https://github.com/FrancoTor95/Embedded-System-Programming/tree/main/01%20-%20C%20Programming#introduction-to-the-c-programming-language)
* [Microcontroller programming with STM32](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/02%20-%20STM32%20Programmazione%20Base)
    * [Architecture and registers](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/02%20-%20STM32%20Programmazione%20Base#arm-architecture-registers)
    * [Bitwise operations](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/03%20-%20Digital%20IO%20e%20GPIO#bitwise-operations)
* [GPIO](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/03%20-%20Digital%20IO%20e%20GPIO#gpio-and-related-registers)
    * [GPIO basic programming in STM32Cube](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/03%20-%20Digital%20IO%20e%20GPIO#lets-get-to-work)
* [Interrupt](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/04%20-%20GPIOs%20and%20Interrupts#gpio-ed-interrupt)
    * [Interrupts in STM32Cube](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/04%20-%20GPIOs%20and%20Interrupts#implementation-and-other-details)
* [ADC](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/05%20-%20Analog%20IO#analog-io)
    * [ADC in STM32Cube](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/05%20-%20Analog%20IO#adcdac-and-stm32)
        * [Polling Mode](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/05%20-%20Analog%20IO#adc-use-case)
        * [Interrupt Mode](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/05%20-%20Analog%20IO#interrupt-mode)
* [Timers](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/08%20-%20Timers%20e%20PWM#timers-e-pwm)
    * [Timers in STM32Cube](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/08%20-%20Timers%20e%20PWM#using-timers-in-stm32cube)
* [PWM](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/08%20-%20Timers%20e%20PWM#pulse-width-modulation-pwm)
    * [Gnerating PWM signals in STM32Cube](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/08%20-%20Timers%20e%20PWM#pwm-generation-in-stm32cube)
* [Seriale communication Part 1](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/06%20-%20Serial%20Communication#serial-communication-uartusart)
    * [UART/USART](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/06%20-%20Serial%20Communication#uart-e-usart)
        * [UART in STM32Cube](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/06%20-%20Serial%20Communication#uartusart-in-stm32-microcontrollers)
* [Seriale communication Part 2](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/07%20-%20Serial%20Communication_%20Part%202#serial-communication-part-2-ic-and-spi)
    * [I<sup>2</sup>C](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/07%20-%20Serial%20Communication_%20Part%202#i2c)
        * [I<sup>2</sup>C in STM32Cube](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/07%20-%20Serial%20Communication_%20Part%202#i2c-in-stm32-microcontrollers)
    * [SPI](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/07%20-%20Serial%20Communication_%20Part%202#spi)
        * [SPI in STM32Cube](https://github.com/FrancoTor95/Embedded-System-Programming/tree/master/07%20-%20Serial%20Communication_%20Part%202#spi-in-stm32-microcontrollers)
* [Embedded Systems and RTOS]
    * [CMSIS-RTOS]
    * [RTOS in STM32Cube // FreeRTOS]

The following references are useful for mastering the basic knowledge of the development platform [mbed](https://os.mbed.com).
* [Mbed online compiler](https://ide.mbed.com/compiler)
* [Mbed Studio](https://os.mbed.com/studio)
* [Mbed Platform]
    * [GPIO programming with mbed]
    * [Interrupts with mbed]
    * [mbed with ADC and DAC]
    * [Timers with mbed]
    * [Generating PWM signals with mbed]
    * [UART communication with mbed]
    * [I<sup>2</sup>C communication with mbed]
    * [SPI communication with mbed]
     * [RTOS with mbed // RTX]
***

## Detailed list of the materials used

- STM32 Nucleo-F446RE
- Mini-USB <-> USB cable
- Breadboard
- Cables
- Resistors
- Capacitors
- Buttons
- LED
- Motion Sensor (Type: HC-SR501)
- Rotary Potentiometer
- Analog temperature sensor (Type: LMT84, LM35, NTC Thermistor 10K)
- Buzzer
- Shift Register (Type: 74HC595N)
- I2C IMU (Type: MPU-6050 in GY-521 module)

##### Material used exclusively in the second module of the course
- DC motor with control driver
- Servomotor
- Digital rotary encoder
- Ultrasonic sensor (Type: HC-SR04)
-------------------
