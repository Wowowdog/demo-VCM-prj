# Voice Coil Motor 
## introduction
This is a demo script for voice coild motor control on STM32/ARM M4 cortex. 
## feature
* muti-channel ADC reading
* two directional movement ctrl  
* IIR or FIR filter (optional)
* PID based controller
* IIC b/w DW9800
* UART data transmittion 
* USB receiving callback
## IIR filter design
![alt text](https://github.com/Wowowdog/demo-VCM-prj/blob/master/png/fil1.png?raw=true)

## controller design
![alt text](https://github.com/Wowowdog/demo-VCM-prj/blob/master/png/ctl1.png?raw=true)

## image snippet
![alt text](https://github.com/Wowowdog/demo-VCM-prj/blob/master/png/mag1.png?raw=true)

## associated doc.
[filter design note](https://drive.google.com/file/d/1aqMnyfdr6wfS0KhNzTkJJ_yOEIsFtxLj/view?usp=share_link)

[ontroller design note](https://drive.google.com/file/d/1aofWPQ_WVctiZDtwdYiYLdtpWiVnDgS7/view?usp=share_link)
