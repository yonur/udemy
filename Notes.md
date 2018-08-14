STM32F446RE evalbord UART Usb dönüştürücü ile ilgili fiziksel konfigürasyonlar:
1- CN7 konektöründeki 5 ve 7.pinler kısa devre edilmelidir. Bu pinler soldaki konnektörün sol sütunundaki 3.üncü ve 4.üncü
pinler.
2- PC11'e USART3'ün TX pini bağlanacak(konektörde sağ sütun ilk sıra), 
3- PC10'e USART3'ün RX pini bağlanacak(konektörde sol sütun ilk sıra)
4- PC19 GND pini (konektörde sol sütun ve 10.uncu sıra) ve USB-UART dönüştürücünün toprağı ile bağlanmalı.


ADXL345: Faydalı Linkler:
http://www.analog.com/en/analog-dialogue/articles/detecting-falls-3-axis-digital-accelerometer.html
https://github.com/arduino/Arduino/releases/tag/1.8.5

