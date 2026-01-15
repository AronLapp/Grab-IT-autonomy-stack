# Grab-IT-autonomy-stack

Making a Joy-IT Grab-it robot arm compatible with algorithm-driven control

# Purpose

This Project aims to upgrade a [Joy-IT Grab-IT](https://joy-it.net/de/products/Robot02) model robotic arm into an autonomous-ready device. In order to do so, the robot will first be supplemented with [OSRAM AS5600 Magnetic Encoders](https://ams-osram.com/products/sensor-solutions/position-sensors/ams-as5600-position-sensor) for every servo motor. The encoders' signals will be multiplexed using an [Adafruit TCA9548A I2C Multiplexer](https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/overview). An [Arduino Nano](https://store.arduino.cc/products/arduino-nano) MCU will then evaluate the encoders' positions periodically and forward them to the motion control unit consisting of an [Arduino Uno R3](https://store.arduino.cc/products/arduino-uno-rev3) and a [Joy-IT Motorino](https://joy-it.net/de/products/ARD-Moto3) shield.

Once the hardware is up and running, a RaspberryPi3 computing unit running ubuntu 22.04 Server and ROS2 Humble will be added to implement Moveit Control and autonomous behavior control.

## Parts

- [Joy-IT Grab-IT](https://joy-it.net/de/products/Robot02)
- [Joy-IT Motorino](https://joy-it.net/de/products/ARD-Moto3)
- [Arduino Uno R3](https://store.arduino.cc/products/arduino-uno-rev3)
- [Arduino Nano](https://store.arduino.cc/products/arduino-nano)
- [OSRAM AS5600 Magnetic Encoder](https://ams-osram.com/products/sensor-solutions/position-sensors/ams-as5600-position-sensor)
- [Adafruit TCA9548A I2C Multiplexer](https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/overview)
