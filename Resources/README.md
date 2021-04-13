# Zephyr RTOS Rescources 

Autopilot Development project for Autonomous Aerial Vehicles.

# Getting Started

* Autopilot is being built upon Zephyr RTOS, overview of the Zephyr is provided over here at [Introduction to Zephyr RTOS](https://docs.zephyrproject.org/latest/introduction/index.html).
* [Setting up Zephyr RTOS on your system](https://docs.zephyrproject.org/latest/getting_started/index.html)
    * Points to remember while setting up the Zephyr RTOS.
        * When cloning the [manifest repository](https://gerrit.googlesource.com/git-repo/+/master/docs/manifest-format.md) (which is the zephyr repository) please use the repository hosted on Dr. Mangal Kothari's Github and not the one provided by the Zephyr Project. 
        * To clone the Autopilot development manifest repository use the following command "west init -m https://github.com/mangaljain82/zephyr" instead of the "west init ~/zephyrproject".
        * Follow rest of the instructions mentioned in the toolchain setup of Zephyr RTOS.
* The complete [Documentation of the Zephyr RTOS project](https://docs.zephyrproject.org/latest/) can be found at the linked page.

# Main Board running the Autopilot Software Stack

* STM32F407G Discovery Board

# Sensors being Used in Autopilot stack

* FrSKY SBUS receiver
* ICM-42605 - Invensense 6-Axis MEMS Motion Sensor, 3-Axis Gyroscope and 3-Axis Accelerometer.
* LIS2MDLTR - STMicroelectronics 3-Axis Magnetometer.
* MS560702BA0350 - TE Connectivity Barometric Sensor.
* ENS210-LQFM - ScioSense Relative Humidity and Temperature Sensor. 


# Lectures, Blogs and Videos to understand concepts

## Basics of Embedded Systems
* Fastbit Academy Mastering RTOS Lecture Series - [Udemy](https://www.udemy.com/course/mastering-rtos-hands-on-with-freertos-arduino-and-stm32fx/) (Will help in understanding basics of RTOS).
* Fastbit Academy Master Microcontroller and Embedded Driver Development(MCU1) - [Udemy](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/)
* Fastbit Academy Master Microcontroller :Timers, PWM, CAN,RTC,Low Power(MCU2) - [Udemy](https://www.udemy.com/course/microcontroller-programming-stm32-timers-pwm-can-bus-protocol/)

## Zephyr's Important parts of Documentation

* Zephyr complete API [overview](https://docs.zephyrproject.org/latest/reference/overview.html).
* Zephyr Device Driver [Documentation](https://docs.zephyrproject.org/latest/reference/drivers/index.html?highlight=device%20driver).
* Zephyr Device Tree [Documentation](https://docs.zephyrproject.org/latest/reference/devicetree/index.html).
 

 ## Zephyr RTOS development Blogs and Videos to refer.

 * Zephyr PWM Implementation - [Blog](https://medium.com/home-wireless/using-a-pwm-device-in-zephyr-7100d089f15c).
 * Zephyr Device Driver Implementation - [Youtube Video](https://youtu.be/oAKmip6Aoj0)
 * Zephyr Device Driver Model - [Youtube Video](https://youtu.be/RYbKALYRYCM)
 * Zephyr RTOS Device Tree Implementation - [Youtube video](https://youtu.be/eOZ0_pNU5vg)


