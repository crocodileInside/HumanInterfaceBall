# HumanInterfaceBall
A ball-shaped, BLE enabled human interface device using a **Bosch BNO055** IMU and a **Bosch BME280** environment sensor for mouse input control.  


## Hardware  
Using a RedBearLabs BLE Nano v2 (**nRF52832**) as MCU and for BLE communication.

## Software  
Written in Arduino, see Arduino folder


### Installation instructions  
In order to setup the required toolchain, follow these instructions:

1. Download and install [Arduino IDE](https://www.arduino.cc/en/Main/Software) >=1.6.12 (tested with 1.6.12)  
2. Install `arduino-nRF5` as described here: https://github.com/sandeepmistry/arduino-nRF5
3. Copy the contents of the `Arduino/libraries/` folder to your Arduino library folder (most likely at `Documents/Arduino/libraries`)
4. The default pin function mapping of our board in Arduino doesn't work with our PCB, so you need to replace your `variant.h` with the one provided in `Arduino/variant.h`  
The original variant.h is found at `C:\Users\<Username>\AppData\Local\Arduino15\packages\sandeepmistry\hardware\nRF5\0.4.0\variants\RedBear_BLENano2`
5. Plug in the DAPLink
6. Install the [mbed Serial Driver](https://os.mbed.com/handbook/Windows-serial-configuration)
7. Open the Arduino IDE (or restart, if it was open)
8. Choose *Tools -> Board -> RedBear BLE Nano 2*
9. Choose *Tools -> Softdevice -> S132*
10. Select the correct serial port for the DAPLink at *Tools -> Port* (sometimes displayed as *BBC:microBit*)
11. Choose *Tools -> Programmer -> CMSIS-DAP*
12. **factory new boards only**: Select *Tools -> nRF5 Flash SoftDevice* and wait for completion
13. Compile and flash the software using Arduino (or hit CTRL+U)
14. Finished!




*Created 01/2018 by C. Locher, M. Schuster, S. Oechslein, N. Rieth, T. Windberg*

