# SBUS
A simple library to parse S.BUS protocol with serial port.  

## Dependency
[serial](https://github.com/wjwwood/serial)  

## Install
Use .sln file for Visual Studio (2019). You need define a environment variable `SERIAL_ROOT` pointing to the install directory of serial library.  

## Use
1. Create a SBUS object.  
2. Specify the serial port by `connect` method.  
3. Set callback function to receive S.BUS data.  
4. Start reading.  