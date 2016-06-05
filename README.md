## PX4Flow Firmware (Support Maxbotix XL-MaxSonar-EZ Series ultrasonic module MB1240)

```
��ע�⣬�ð汾�����̼���֧��maxbotix��˾��XL-MaxSonar?- EZ? Series������ģ��MB1240�������ʹ�õĳ�����ģ���������汾�������عٷ��Ĺ����̼���
Please note, This version of the optical flow firmware only support maxbotix company XL-MaxSonar-EZ series ultrasonic named MB1240. If you use the ultrasonic module is the other version, please download the primitive firmware. 
This version firmware uses the PIN3-AN of XL-MaxSonar-EZ Series ultrasonic's which output analopg voltage to get the range data,instead of using serial! 
```

##�޸�˵��
�޸Ĵ�����Ҫ�����ļ� sonar.c�ļ����档
�޸�ԭ�����Ǵ���Ĺ����˳�����ģ��MB1240�����೬����ģ�����RS232��ƽ�Ĵ������ݣ��������ǹ���Ĺ���ģ��ǡ�ò�֧�����ֵ�ƽ�����ҵ��ǣ����������˴Ӵ�����������⣬��������2������ķ�ʽ�������Ϣ���Լ�������3�Ե�ѹ�ķ�ʽ���������Ϣ����ˣ���Ҫ�޸Ĺ�����ʹ���ܹ���������3����ĵ�ѹ��Ϣ��ȡ�������Ĳ���ֵ��

����
�޸ĵ�Ŀ�꣺��ӹ���ģ���MB1240���͵ĳ�����֧�֣�
�޸����е�ԭ��MB1240�೬����ģ�������3������������ȵĵ�ѹ��Ϣ���������ù���ģ���ڲ���ADC�ɼ���ת��Ϊ������Ϣ��
�޸�ԭ�򣺲��ı�ԭ�еĳ����߼��ṹ��ͨ����SONAR_USING_MB1240 ѡ���ܣ�


##ʹ�÷���
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
���ִ��make��ʾ�Ҳ����������������ڸ���ַ���ر������� https://launchpad.net/gcc-arm-embedded/+download

To flash via the PX4 bootloader (first run this command, then connect the board):
```
  make upload-usb
```

By default the px4flow-v1_default is uploaded to upload a different version

```
  make <target> upload-usb
```
Where <target> is one of the px4flow tatgets listed by ```make help```

