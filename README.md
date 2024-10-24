# ServoSDK-Arduino
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Welcome to ServoSDK-Arduino, an open-source software development kit designed specifically for the Energize Lab Servo. This SDK provides multi-platform servo usage examples and motion control demos based on Arduino, aimed at helping users easily and quickly master the use of various models, including the EM-2030. It is strongly recommended to refer to the corresponding memory table parameter for understanding and verification.

## Supported
- Protocols:
  - Energize Lab Servo Communication Protocol  
- Memory Tables: 
  - Primary Memory Table 
- Servos:  
  - EM Series, EH-3030 
- Programming Languages:
  - Arduino: Includes source code, with compatible examples and demos across multiple platforms.

## Repository
- File Description: 
  - src: Source code that includes instruction generation and parsing for the entire memory table of servo.
  - Example: Demonstrate the use of Ping instruction, read data instruction, write data instruction, sync write instruction, parameter reset instruction, factory reset instruction, and reboot instruction.
  - Demo: Demonstrate the motion of the servo in different control modes.

## Servo Preparation
- To run the ServoSDK-Arduino Example and Demo, you need to change the following parameters of the servo:
  - ID = 1 (for testing Sync Write instruction, use 1 and 2 respectively)
  - Baud Rate = 1 Mbps (the corresponding value in the memory table is 7, which usually doesn't need to be changed)
- To change the servo parameters, please use ServoStudio or [ServoCommand](https://energizelab.cn/serial-tool/):
  - Refer to the corresponding user manual for details.

## Communication Circuits
- Servo and MCU Communication
  - To enable communication between the Energize Lab Servo and the MCU, a UART to TTL module is needed; if the MCU operates at 5V, a level shifting module is also necessary.

**Please visit [ServoSDK](https://github.com/EnergizeLab/ServoSDK) for more detailed information.**
