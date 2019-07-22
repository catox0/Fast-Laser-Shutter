# Fast-Laser-Shutter
Arduino-controlled laser-shutter (USB and push-button control) with controlled acceleration and breaking.

The magentic coil of a hard-drive is perfectly suited to create a fast laser shutter with down to single-ms opening/closing time. [cf: L. P. Maguire, S. Szilagyi, and R. E. Scholten, Review of Scientific Instruments 75:9, 3077 (2004).] We use such shutters in the lab for more than a decade. Here is a program to control such a shutter with an Arduino and a LN298 or TB6612FNG module (around 1$ at Aliexpress). 

For fast opening/closing times, use the biggest power supply available (the L298 is rated up to 46V, 2A, 3A peak). The shutters can make a lot of noise, hence this program will decellerate the shutter for a soft landing at the stop position. The Program also reduces the holding current to avoid overheating. When the shutter settings are optimized, the shutter will open / close with maximal speed, minimal noise, and minimal power consumption. The shutter will accelerate for an adjustable time (variable names accelOpen, accelClose), then break for an adjustable time (variable names breakOpen, breakClose), and then hold the final position with reduced power (variable names openPower, closePower, value range 0-255). Each shutter / power supply combination must be optimized separately. Below is a guideline to finding the correct values.

##Setting up and functions
I used simple jumper wires to connect Arduino digital pins to the LM298 module pins (ENA, In1, In2; see source code) and connected the Arduino and the LM298 input power to the same power supply. (You need separate power for the Arduino if you go beyond 20V.) I only connected 1 shutter in this example, but the L298 can drive two. I also connected the Arduino to a pushbutton for manual operation and an LED to indicate an 'open' shutter position. \
The push-button toggles the shutter position. Other functions can only be accessed via Serial/USB communication with the Arduino. Serial functions open, close, or toggle the shutter. A program can be defined for single or periodic shutter opening. The shutter settings for accelleration and breaking duration and for the holding power can be programmed. A save routine writes the program and shutter settings to EEPROM. Upon reset, these settings are read from EEPROM.

##How to find the correct shutter parameters for fast and silent shutter operation:
Connect the shutter microcontroller by USB to your computer Open a serial terminal (e.g., in the Arduino IDE) to communicate with the microcontroller at 19200 baud, with newline terminator.\
For the following, I assume that the 'open' shutter position requires a holding force against a spring and the 'close' position is held by the spring.\
(1) create a program to continuously open/close the shutter and run the program ('prog p 2000', 'prog o 1000' for opening / closing at 1/2 Hz, 'p' to run).\ 
(2) Set accelOpen to zero ('set ao 0'). Now adjust the openPower ('set op *int', *int=0..255) until you find the smallest value that fully opens the shutter. Increase that value by 10%.\
(3) Set breakOpen to zero and increase accelOpen until the shutter hits the open limit hard (with a loud noise). Now set breakOpen to 1/2 accelOpen and increase breakOpen gradually, until the shutter comes to a stop at the open position without bouncing. To achieve maximum speed, gradually increase the accelOpen value and re-optimize the breakOpen value until you achieve the largest values that open the shutter without bouncing.\
(4) Set accelClose and breakClose to the accelOpen values. The shutter will bounce off the end stop. Reduce the accelClose value until the initial bounce is no longer audible. Reduce the breakClose value until the shutter closes without any bouncing. 

##Example: hard-drive shutter with 12V, 2.5A power supply, L298N driver.

High-power settings (largest motion)\
_accelClose = 9000\
_breakClose = 5500\
_closePower = 15\
_accelOpen  = 9200\
_breakOpen = 5300\
_openPower = 60\

Low-power settings\
_accelClose = 8100\
_breakClose = 5900\
_closePower = 0\
_accelOpen  = 9300\
_breakOpen = 5100\
_openPower = 40
