## PX4Flow Firmware (Support Maxbotix XL-MaxSonar-EZ Series ultrasonic module MB1240)

```
请注意，该版本光流固件仅支持maxbotix公司的XL-MaxSonar?- EZ? Series超声波模块MB1240，如果您使用的超声波模块是其他版本，请下载官方的光流固件。
Please note, This version of the optical flow firmware only support maxbotix company XL-MaxSonar-EZ series ultrasonic named MB1240. If you use the ultrasonic module is the other version, please download the primitive firmware. 
This version firmware uses the PIN3-AN of XL-MaxSonar-EZ Series ultrasonic's which output analopg voltage to get the range data,instead of using serial! 
```

##修改说明
修改代码主要是在文件 sonar.c文件里面。
修改原因：我们错误的购买了超声波模块MB1240，该类超声波模块输出RS232电平的串口数据，但是我们购买的光流模块恰好不支持这种电平。所幸的是，超声波除了从串口输出数据外，还从引脚2以脉宽的方式输测量信息，以及从引脚3以电压的方式输出测量信息。因此，需要修改光流，使其能够利用引脚3输出的电压信息获取超声波的测量值。

即：
修改的目标：添加光流模块对MB1240类型的超声波支持；
修改依托的原理：MB1240类超声波模块的引脚3输出与距离成正比的电压信息，可以利用光流模块内部的ADC采集并转换为距离信息；
修改原则：不改变原有的程序逻辑结构，通过宏SONAR_USING_MB1240 选择功能；


##使用方法
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
如果执行make提示找不到编译器，可以在该网址下载编译器： https://launchpad.net/gcc-arm-embedded/+download

To flash via the PX4 bootloader (first run this command, then connect the board):
```
  make upload-usb
```

By default the px4flow-v1_default is uploaded to upload a different version

```
  make <target> upload-usb
```
Where <target> is one of the px4flow tatgets listed by ```make help```

