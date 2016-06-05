## PX4Flow Firmware (Support Maxbotix XL-MaxSonar-EZ Series ultrasonic module MB1240)

```
Çë×¢Òâ£¬¸Ã°æ±¾¹âÁ÷¹Ì¼ş½öÖ§³Ömaxbotix¹«Ë¾µÄXL-MaxSonar?- EZ? Series³¬Éù²¨Ä£¿éMB1240£¬Èç¹ûÄúÊ¹ÓÃµÄ³¬Éù²¨Ä£¿éÊÇÆäËû°æ±¾£¬ÇëÏÂÔØ¹Ù·½µÄ¹âÁ÷¹Ì¼ş¡£
Please note, This version of the optical flow firmware only support maxbotix company XL-MaxSonar-EZ series ultrasonic named MB1240. If you use the ultrasonic module is the other version, please download the primitive firmware. 
This version firmware uses the PIN3-AN of XL-MaxSonar-EZ Series ultrasonic's which output analopg voltage to get the range data,instead of using serial! 

```
##ĞŞ¸ÄËµÃ÷
ĞŞ¸Ä´úÂëÖ÷ÒªÊÇÔÚÎÄ¼ş sonar.cÎÄ¼şÀïÃæ¡£
ĞŞ¸ÄÔ­Òò£ºÎÒÃÇ´íÎóµÄ¹ºÂòÁË³¬Éù²¨Ä£¿éMB1240£¬¸ÃÀà³¬Éù²¨Ä£¿éÊä³öRS232µçÆ½µÄ´®¿ÚÊı¾İ£¬µ«ÊÇÎÒÃÇ¹ºÂòµÄ¹âÁ÷Ä£¿éÇ¡ºÃ²»Ö§³ÖÕâÖÖµçÆ½¡£ËùĞÒµÄÊÇ£¬³¬Éù²¨³ıÁË´Ó´®¿ÚÊä³öÊı¾İÍâ£¬»¹´ÓÒı½Å2ÒÔÂö¿íµÄ·½Ê½Êä²âÁ¿ĞÅÏ¢£¬ÒÔ¼°´ÓÒı½Å3ÒÔµçÑ¹µÄ·½Ê½Êä³ö²âÁ¿ĞÅÏ¢¡£Òò´Ë£¬ĞèÒªĞŞ¸Ä¹âÁ÷£¬Ê¹ÆäÄÜ¹»ÀûÓÃÒı½Å3Êä³öµÄµçÑ¹ĞÅÏ¢»ñÈ¡³¬Éù²¨µÄ²âÁ¿Öµ¡£
¼´£º
ĞŞ¸ÄµÄÄ¿±ê£ºÌí¼Ó¹âÁ÷Ä£¿é¶ÔMB1240ÀàĞÍµÄ³¬Éù²¨Ö§³Ö£»
ĞŞ¸ÄÒÀÍĞµÄÔ­Àí£ºMB1240Àà³¬Éù²¨Ä£¿éµÄÒı½Å3Êä³öÓë¾àÀë³ÉÕı±ÈµÄµçÑ¹ĞÅÏ¢£¬¿ÉÒÔÀûÓÃ¹âÁ÷Ä£¿éÄÚ²¿µÄADC²É¼¯²¢×ª»»Îª¾àÀëĞÅÏ¢¡£

ĞŞ¸ÄÔ­Ôò£º²»¸Ä±äÔ­ÓĞµÄ³ÌĞòÂß¼­½á¹¹£¬Í¨¹ıºêSONAR_USING_MB1240 Ñ¡Ôñ¹¦ÄÜ
=======
#è¯·æ³¨æ„ï¼Œè¯¥ç‰ˆæœ¬å…‰æµå›ºä»¶ä»…æ”¯æŒmaxbotixå…¬å¸çš„XL-MaxSonarÂ®- EZâ„¢ Seriesè¶…å£°æ³¢æ¨¡å—MB1240ï¼Œå¦‚æœæ‚¨ä½¿ç”¨çš„è¶…å£°æ³¢æ¨¡å—æ˜¯å…¶ä»–ç‰ˆæœ¬ï¼Œè¯·ä¸‹è½½å®˜æ–¹çš„å…‰æµå›ºä»¶ã€‚
#Please note, This version of the optical flow firmware only support maxbotix company XL-MaxSonar-EZ series ultrasonic named MB1240. If you use the ultrasonic module is the other version, please download the primitive firmware. 
This version firmware uses the PIN3-AN of XL-MaxSonar-EZ Series ultrasonic's which output analopg voltage to get the range data,instead of using serial! 

```
>>>>>>> ef7c686ddab143c0b6afadd9576895b92a3d4073

##Ê¹ÓÃ·½·¨
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
Èç¹ûÖ´ĞĞmakeÌáÊ¾ÕÒ²»µ½±àÒëÆ÷£¬¿ÉÒÔÔÚ¸ÃÍøÖ·ÏÂÔØ±àÒëÆ÷£º https://launchpad.net/gcc-arm-embedded/+download

To flash via the PX4 bootloader (first run this command, then connect the board):
```
  make upload-usb
```

By default the px4flow-v1_default is uploaded to upload a different version

```
  make <target> upload-usb
```
Where <target> is one of the px4flow tatgets listed by ```make help```

