STM32F446RE evalbord UART Usb dönüştürücü ile ilgili fiziksel konfigürasyonlar:
1- CN7 konektöründeki 5 ve 7.pinler kısa devre edilmelidir. Bu pinler soldaki konnektörün sol sütunundaki 3.üncü ve 4.üncü
pinler.
2- PC11'e USART3'ün TX pini bağlanacak(konektörde sağ sütun ilk sıra), 
3- PC10'e USART3'ün RX pini bağlanacak(konektörde sol sütun ilk sıra)
4- PC19 GND pini (konektörde sol sütun ve 10.uncu sıra) ve USB-UART dönüştürücünün toprağı ile bağlanmalı.


ADXL345: Faydalı Linkler:
http://www.analog.com/en/analog-dialogue/articles/detecting-falls-3-axis-digital-accelerometer.html
https://github.com/arduino/Arduino/releases/tag/1.8.5
https://learn.sparkfun.com/tutorials/adxl345-hookup-guide?_ga=2.85584985.1167364778.1534249884-451285960.1522738016

Double Tap'ın geçersi olduğu durumlar vardır bunlar:
- İlk tap(single tap)'ın latency time'ı dolmadan treshold'u geçen bir spike olursa bu doube tap olarak değerlendirlmez
- Eğer ikinci second tap için olan time window'un başlangıcında thresholdu aşan spike saptanırsa bu double tap olarakdeğerlendirilmez
- Eğer ikinci spike DUR ile belirtilen time limiti aşarsa double tap olarak değerlendirilmez


