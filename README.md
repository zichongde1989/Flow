## PX4Flow Firmware (Support Maxbotix XL-MaxSonar-EZ Series ultrasonic module MB1240)

```
#请注意，该版本光流固件仅支持maxbotix公司的XL-MaxSonar®- EZ™ Series超声波模块MB1240，如果您使用的超声波模块是其他版本，请下载官方的光流固件。
#Please note, This version of the optical flow firmware only support maxbotix company XL-MaxSonar-EZ series ultrasonic named MB1240. If you use the ultrasonic module is the other version, please download the primitive firmware. 
This version firmware uses the PIN3-AN of XL-MaxSonar-EZ Series ultrasonic's which output analopg voltage to get the range data,instead of using serial! 

```

PX4 FLOW is a smart camera processing optical flow directly on the camera module. It is optimized for processing and outputs images only for development purposes. Its main output is a UART or I2C stream of flow measurements at ~400 Hz.

Project:
http://px4.io/modules/px4flow

Dev guide / toolchain installation:
http://px4.io/dev/px4flow

For help, run:

```
make help

```


To build, run:
```
  make archives - this needs to be done only once
  make

```

To flash via the PX4 bootloader (first run this command, then connect the board):
```
  make upload-usb
```

By default the px4flow-v1_default is uploaded to upload a different version

```
  make <target> upload-usb
```
Where <target> is one of the px4flow tatgets listed by ```make help```


