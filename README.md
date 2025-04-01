# STM32WB_CAN_Sense

This project includes custom drivers for the BME280 sensor as well as the driver for the MCP2515 CAN chip. The driver for the CAN chip was reduced to the essentials for sending the CAN messages and for testing them in loopback mode during development. The environmental measurements ​​(temperature, humidity, and air pressure) from the BME280 sensor are read at one-second intervals using a timer and placed on the bus as separate CAN messages. The data flow is controlled by two flags (Timer and Tx-Buffer-Ready) to keep the entire process as simple as possible. As soon as the timer triggers, a timer flag is set there. The readiness of the transmit buffer in the MCP2515 is handled in a similar way. Both flags are constantly queried in the main loop, and if the conditions are met, new values ​​are read and sent. Moreover a simple error handling mechanism has also been implemented. In the event of an error, the corresponding error is logged using a simple logger (is visible in the debug output), and a blue LED on the board is also activated. The pinout for the STM32WB55 microcontroller is as shown in the image below.

![STM32WB55_Pinout_Overview](/docs/STM32WB55_CAN_Sense_Pinout_View.png)

## License

This project is licensed under the MIT License. You can find the full license text in the [LICENSE](https://github.com/AndreasCnaus/PiCAN_Sense/blob/main/docs/LICENSE) file.

### Third-Party Software

This project uses STM32 libraries, which are distributed under the [BSD-3-Clause License](https://opensource.org/licenses/BSD-3-Clause). Please ensure compliance with their license terms if you use or distribute this software.

The licenses of any additional third-party components are provided in their respective documentation.

