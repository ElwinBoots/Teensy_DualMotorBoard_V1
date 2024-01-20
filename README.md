This project aims to deliver high performance (fast loops > 40 kHz, fast motion, and small following errors) brushless motor control.

## Videos in action 
See: https://www.youtube.com/@elwinboots2614

## Hardware 
- 1 Simple custom PCB for making connections
- 1 Teensy 4.1
- 2 DRV8301 motor drive boards, see Aliexpress
- 2 BLDC motors (sensorless or ABI encoders)
- 4 Pololu ACS711EX current sensor boards (drive boards have low side sensors. Prefer to use these instead to have in line sensing)

## Firmware
See firmware folder. Almost all code can be found in Firmware.ino. 

### Software to build firmware
"Old" Arduino IDE
- Arduino IDE 1.8.19 https://www.arduino.cc/en/software (Legacy IDE)
- Teensyduino https://www.pjrc.com/teensy/td_download.html (Follow the "Arduino 1.8.x Software Development" steps)
Or,
"New" Arduino IDE:
- Arduino IDE https://www.arduino.cc/en/software (current version: 2.2.1)
- Teensyduino https://www.pjrc.com/teensy/td_download.html (Follow the "Arduino 2.0.x Software Development" steps
 
If you want to test with 1 or 0 DRV8301 boards connected, search for the line: SPI_init( SSPIN ); and/or SPI_init( SSPIN2 ); and comment them out. With these lines commented the firmware and software can run without boards. With these lines the board waits for the DRV8301 boards, before continuing to boot.

Almost all parameters are defined in defines.h. All of these are accessible (read and write) from the host pc using the software provided. 

To add a variable and make it accessible from the PC:
1. Add parameter in defines.h 
2. run "gentracefile (Run this to update trace.h).bat". 
3. Compile and download Firmware.ino in Arduino IDE

Note: I use the 816 Mhz (overclock) option in the CPU speed menu.

## Software
- Python 3.x
- Spyder
- Code in Software folder

### To install: 
1. Download latest version of Python: https://www.python.org/downloads/
2. Install Python
3. In a command window, run `pip install spyder`
4. Run Spyder (C:\Users\XXX\AppData\Local\Programs\Python\PythonXXX\Scripts\spyder.exe)

### To use: 
1. In Spyder, run first cell (ctrl + enter) in test_object.py (for any missing modules, run `pip install <modulename>` in a cmd window.
2. When a Teensy is found with the correct firmware, you should get: "Connected to Teensy on COMX with serial number: XXXXXXXX."
3. Check that the LED on the Teensy is off. If it is still on, make check connection and power to DRV8301 boards, or comment out SPI_init( SSPIN ); and/or SPI_init( SSPIN2 ) to test without DRV8301 boards.
4. Now you can type `motor` in the IPython console, and it should show all parameters:\
![afbeelding](https://github.com/ElwinBoots/Teensy_DualMotorBoard_V1/assets/79989749/8c7ca115-ee76-41d0-a0b3-b3fa819649d2)


5. To see for example only the configuration of motor 1, you can type `motor.conf1`:\
![afbeelding](https://github.com/ElwinBoots/Teensy_DualMotorBoard_V1/assets/79989749/6f874f37-aab8-481b-9e51-7816cb238084)

6. To retreive a single variable, just type the variable name and click enter. Example: `motor.state1.encoderPos1`. Note that you can use `tab` to autocomplete the name of a variable.
7. To set a variable, simply write a value to it. Example: `motor.conf1.commutationoffset = 0`, or use `m.setpar('motor.conf1.commutationoffset',0)`\
![afbeelding](https://github.com/ElwinBoots/Teensy_DualMotorBoard_V1/assets/79989749/20b2c352-d93a-4b81-975d-39fbd66a95b2)\
The setting of variables is fast. Almost 20000 values can be written in 1 second when writing one by one (even more when using arrays):\
![afbeelding](https://github.com/ElwinBoots/Teensy_DualMotorBoard_V1/assets/79989749/df5f36a1-0cf0-4db3-b5f5-8d81579bf42e)

### Tracing
To trace variables/signals: 
1. Use m.setTrace() to select signals. Example: `m.setTrace([ 'motor.state1.emech' , 'motor.state1.rmech' , 'motor.state1.ymech' ])`
2. Use `df = m.trace(2)` to trace for 2 seconds.
3. To plot, use `df.plot()`

Note1: By default tracing occurs at the full sample rate. Up to 50 signals at 60 kHz is possible. Downsampling is also possible to reduce load.\
Note2: All communication is in binary. This means that no precision is lost in the tracing.

### Tracing (background)
For non-blocking tracing of variables/signals:
1. Use m.setTrace() to select signals. Example: `m.setTrace([ 'motor.state1.emech' , 'motor.state1.rmech' , 'motor.state1.ymech' ])`
2. Use `m.tracebg( )` to start a background trace.
3. Use `df = m.stoptracegetdata()` to stop the trace and get the data
4. To plot, use `df.plot()`

Note: during the tracing you can still set variables. You can however not retrieve other variables.
