# FreeRTOS_BME280

## Description
This project demonstrates the use of FreeRTOS queues to read temperature data from a BME280 sensor via I2C on an STM32F446ZET6. One task reads and compensates temperature data, sending it to a queue.

## FreeRTOS Features
- Queue management (`xQueueCreate`, `xQueueSendToBack`, `xQueueReceive`)
- Task management (multiple tasks with different priorities)
- I2C communication with HAL

## Hardware
- **Microcontroller**: STM32F446ZET6
- **Sensor**: BME280 (I2C, address 0x76, connected to I2C1 pins)

## Setup
1. Open the project in STM32CubeIDE.
2. Ensure FreeRTOS and HAL libraries are included.
3. Connect the BME280 sensor to I2C1 (PB8-SCL, PB9-SDA).

## Test Results
- Temperature data was successfully read from the BME280 sensor and transmitted via the queue.
