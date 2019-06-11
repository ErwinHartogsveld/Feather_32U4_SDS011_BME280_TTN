# Feather_32U4_SDS011_BME280_TTN
Feather 32u4 + SDS011 Finedust sensor + BME280 + LoRa + TTN

This is a step-by-step configuration to connect your SDS011 to TTN.

This is a project for meassuring fine dust with:
  - Micro controller Feather 32u4 product ID3078 (https://www.adafruit.com/product/3078)
  - Fine dust sensor Novafitness SDS011 (PM10 & PM2.5) (http://inovafitness.com/en/a/chanpinzhongxin/95.html)
  - BME280 RH/Temp/Pressure (https://www.ebay.com/itm/401000227934)

  
  Principle:
  Sample frequency is 1 second. The average of each 160 sec of PM10 and PM2.5 data and the actual valueus of RH, temp and Pressure will be   queued and send to TTN. Also the ID of the SDS011 sensor will be transmitted to the TTN.
  
  See the setup document to configure your system. This document includes the wireing, uploading yor sketch and the configuration of the     TTN.
  
  
