# Weather Station project for Embedded Systems course

## Project info

Created by Wojciech Jagielnicki, Marek Antoszewski & Mariusz Ladra

Created for STM32F446RE (Nucleo prototype board) using STM32CubeIDE

**Additional equipment:**

* BME280 temperature, pressure & humidity sensor
* Standard 16x2 LCD Display

## Description

This project is a prototype of a basic indoor-use weather station system using the BME280 sensor with STM32 as the controller. 
Data from the BME sensor is collected periodically and displayed on the LCD.

## Implementation details

* This project is using the Hardware Abstraction Layer (HAL) for basic low-level functions like driving the pins, I2C read/write, delay etc.
* BME280 sensor is driven using I2C protocol, in accordance with the official Bosch library for STM32
* LCD display is driven using a library by Mohamed Yaqoob
