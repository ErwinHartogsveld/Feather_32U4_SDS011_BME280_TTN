#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, network session key from TTN (use the same format as this example)
static const PROGMEM u1_t NWKSKEY[16] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
// LoRaWAN AppSKey, application session key From TTN (use the same format as this example)
static const u1_t PROGMEM APPSKEY[16] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
// LoRaWAN end-device address (DevAddr) from TTN (use the same format as this example)
static const u4_t DEVADDR = 0x220114f6;

//=======================================================
//For the BME

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme; // I2C
bool status_BME = 0;
float BME_Temp = 0.0;
float BME_Pres = 0.0;
float BME_Humi = 0.0;
bool Debug_On = 0;//EH Set this '0' to disable debugging via comport to Arduino-IDE

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
// const unsigned TX_INTERVAL = 60;
const unsigned TX_INTERVAL = 170;//was 30

int NumLoop = 0;
int Meas_BME = TX_INTERVAL - 5;//Meassure Temp/RH/Pressure 5 sec before sending

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {7, 6, LMIC_UNUSED_PIN},
};

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
unsigned int Pm25 = 0;
unsigned int Pm10 = 0;
unsigned int Sds_ID = 0;
int  Status = -1;
int LastError = 0;
float fpm10 = 0.0, fpm25 = 0.0;

unsigned int Pm10_Total = 0;//EH
unsigned int Pm10_Avg;//EH
unsigned int Pm25_Total = 0;//EH
unsigned int Pm25_Avg;//EH
unsigned int Counter_Pm = 0;//EH
float fpm10_Avg = 0.0, fpm25_Avg = 0.0;//EH

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
void onEvent (ev_t ev) {

  Serial.println(F("onEvent"));

  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }

  Serial.println(F("onEvent"));
  NumLoop = 0;//Zero Counter here
  Pm10_Total = 0;//EH
  Pm25_Total = 0;//EH
  Counter_Pm = 0;//EH
  
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
void ProcessSerialData()
{
  uint8_t mData = 0;
  uint8_t i = 0;
  uint8_t mPkt[10] = {0};
  uint8_t mCheck = 0;

  Status = -1;  // Default: no data availabe ...
  fpm10 = -99.0;
  fpm25 = -99.0;

  while (Serial1.available() > 0)
  {
    // from www.inovafitness.com
    // packet format: AA C0 PM25_Low PM25_High PM10_Low PM10_High 0 0 CRC AB

    Status = 0; // There is data!

    mData = Serial1.read();
    delay(10);//wait until packet is received

    if (mData == 0xAA) //head1 ok AA = 170 
    {
      mPkt[0] =  mData;
      mData = Serial1.read();
      if (mData == 0xc0) //head2 ok
      {
        mPkt[1] =  mData;
        mCheck = 0;
        for (i = 0; i < 6; i++) //data recv and crc calc//Read first (x) 6 Data bytes from sds
        {
          mPkt[i + 2] = Serial1.read();
          delay(2);
          mCheck += mPkt[i + 2];
        }
        mPkt[8] = Serial1.read();
        delay(1);
        mPkt[9] = Serial1.read();
        if (mCheck == mPkt[8]) //crc ok
        {
          Serial1.flush();
          //Serial.write(mPkt,10);

          Pm25 = (uint16_t)mPkt[2] | (uint16_t)(mPkt[3] << 8);
          Pm10 = (uint16_t)mPkt[4] | (uint16_t)(mPkt[5] << 8);
          Sds_ID = (uint16_t)mPkt[6] | (uint16_t)(mPkt[7] << 8);
               
          fpm25 = Pm25 / 10.0;
          fpm10 = Pm10 / 10.0;
          
          Pm10_Total = Pm10 + Pm10_Total;//EH
          Pm25_Total = Pm25 + Pm25_Total;//EH
          Counter_Pm = Counter_Pm + 1;//EH
          Pm10_Avg = Pm10_Total / Counter_Pm;//EH
          Pm25_Avg = Pm25_Total / Counter_Pm;//EH
          fpm25_Avg = Pm25_Avg / 10.0;//EH
          fpm10_Avg = Pm10_Avg / 10.0;//EH
                   
          if (Debug_On) {
          Serial.println(F(""));///////////////////////////////////////
          Serial.println(F(""));///////////////////////////////////////
          Serial.println(F("-------------------------------"));///////////////////////////////////////
          Serial.print("Sds_ID; ");///////////////////////////////////////
          Serial.println(Sds_ID);///////////////////////////////////////
          Serial.print("Counter_Pm; ");///////////////////////////////////////
          Serial.println(Counter_Pm);///////////////////////////////////////
          Serial.println(F("-------------------------------"));///////////////////////////////////////
          Serial.print("PM10 Total : ");///////////////////////////////////////
          Serial.println(Pm10_Total);///////////////////////////////////////
          Serial.print("Pm10_AVG: ");///////////////////////////////////////
          Serial.println(Pm10_Avg);///////////////////////////////////////
          Serial.print("Pm10_Sensor: ");///////////////////////////////////////
          Serial.println(Pm10);///////////////////////////////////////
          Serial.println(F("-------------------------------"));///////////////////////////////////////
          Serial.print("PM2.5 Total : ");///////////////////////////////////////
          Serial.println(Pm25_Total);///////////////////////////////////////
          Serial.print("Pm2.5_AVG: ");///////////////////////////////////////
          Serial.println(Pm25_Avg);///////////////////////////////////////
          Serial.print("Pm2.5_Sensor: ");///////////////////////////////////////
          Serial.println(Pm25);///////////////////////////////////////
          Serial.print("Temp_Sensor: ");///////////////////////////////////////
          Serial.println(BME_Temp);///////////////////////////////////////
          Serial.print("Pres_Sensor: ");///////////////////////////////////////
          Serial.println(BME_Pres);///////////////////////////////////////
          Serial.print("Humi_Sensor: ");///////////////////////////////////////
          Serial.println(BME_Humi);///////////////////////////////////////
          Serial.println(F("-------------------------------"));///////////////////////////////////////
          }

          if (Pm25 > 999)
            Pm25 = 999;
          if (Pm10 > 999)
            Pm10 = 999;
           if (Pm25_Avg > 999)//EH
            Pm25_Avg = 999;//EH
           if (Pm10_Avg > 999)//EH
            Pm10_Avg = 999; //EH  
          
          //get one good packet
          return;
        } // CRC ??
        else {
          //Serial.println("CRC != OK,  ");
          Status = 3;
        }

      } // Head2 ??
      else {
        //Serial.println("mData != 0xc0 ");
        Status = 2;
      }

    } // Head1 ??
    else {
      //Serial.println("mData != 0xAA ");
      Status = 1;
    }

  }
}
void do_send(osjob_t* j) {
  //  Serial.println(F("ENTER do_send"));

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
#define VBATPIN A9

    float temp = -99.;
    float rh = -99.;
    float p = -99.;
    float pm10 = -99.;
    float pm25 = -99.;
    float pm10_Avg = -99.;//EH
    float pm25_Avg = -99.;//EH
    float lat = -99.;
    float lon = -99.;
    float Spare1 = -99.;
    float Spare2 = -99.;
    
    pm10 = fpm10;
    pm25 = fpm25;
    pm10_Avg = fpm10_Avg;//EH
    pm25_Avg = fpm25_Avg;//EH
    temp = BME_Temp;
    rh = BME_Humi;
    p = BME_Pres;
    lat=52.2345678;//EH = fixed in code!!
    lon=5.9345678;//EH = fixed in code!!
    
    int16_t Sds_ID_int = (Sds_ID);
    int16_t temp_int = (temp) * 100;
    int16_t rh_int = (rh) * 100 ;
    int16_t p_int = round(p) + 100;
    int16_t pm10_Avg_int = pm10_Avg * 10 + 1000;//EH
    int16_t pm25_Avg_int = pm25_Avg * 10 + 1000;//EH
    //int16_t pm10_int = pm10 * 10 + 1000;
    //int16_t pm25_int = pm25 * 10 + 1000;
    //int32_t lat_int = lat * 10000 + 2000000;
    //int32_t lon_int = lon * 10000 + 2000000;
    
    uint8_t buffer[12];
    buffer[0]  = Sds_ID_int >> 8;
    buffer[1]  = Sds_ID_int;
    buffer[2]  = temp_int >>8; 
    buffer[3]  = temp_int;
    buffer[4]  = rh_int >> 8;
    buffer[5]  = rh_int; ;
    buffer[6]  = p_int >> 8;
    buffer[7]  = p_int;
    buffer[8]  = pm10_Avg_int >> 8;
    buffer[9]  = pm10_Avg_int;
    buffer[10]  = pm25_Avg_int >> 8;
    buffer[11]  = pm25_Avg_int;
    //buffer[12]  = pm10_int >> 8;
    //buffer[13]  = pm10_int;
    //buffer[14]  = pm25_int >> 8;
    //buffer[15] = pm25_int;
    //buffer[16] = lat_int >> 24;
    //buffer[17] = lat_int >> 16;
    //buffer[18] = lat_int >> 8;
    //buffer[19] = lat_int;
    //buffer[20] = lon_int >> 24;
    //buffer[21] = lon_int >> 16;
    //buffer[22] = lon_int >> 8;
    //buffer[23] = lon_int;
    
   
    //    Serial.print("Tx: " ); Serial.println(measuredvbat);
    LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
    Serial.println(F("Packet queued"));
    Serial.print("Sds_ID: ");
    Serial.println(Sds_ID);
    Serial.print("Temp: ");
    Serial.println(BME_Temp);
    Serial.print("RH: ");
    Serial.println(BME_Humi);
    Serial.print("Pressure: ");
    Serial.println(BME_Pres);
    Serial.print("PM10_Avg: ");
    Serial.println(pm10_Avg);
    Serial.print("PM2.5 Avg: ");
    Serial.println(pm25_Avg);
    //Serial.print("PM10: ");
    //Serial.println(pm10);
    //Serial.print("PM2.5: ");
    //Serial.println(pm25);
    //Serial.print("lat: ");
    //Serial.println(lat);
    //Serial.print("lon: ");
    //Serial.println(lon);
    
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(9600);
  //Serial.println("Starting SETUP");

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  //  Serial.println(F("FREQS"));

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif


  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  //  Serial.println(F("LMIC_setDrTxpow"));

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF12, 14);

  // Start job
  //  Serial.println(F("do_send 0"));

  do_send(&sendjob);

  Serial.println(F("do_send 1"));

  Serial1.begin(9600);

  Serial.begin(9600);
  Pm25 = 0;
  Pm10 = 0;

  //===================================
  // BME init
  Serial.println(F("BME280 test"));
  status_BME = bme.begin(0x076);
  if (!status_BME) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(1000);
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(1000);
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(1000);
  }


}



//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
void loop() {

  //Serial.print.clear
  Serial.print(NumLoop);
  Serial.print(" ");
  NumLoop++;

  if (NumLoop % Meas_BME == 0){ //if (NumLoop % 165 == 0) {//After X counts read BME Sensor (Default = 100
    if (status_BME) {
      BME_Temp = bme.readTemperature();
      BME_Pres = bme.readPressure() / 100.0F;
      BME_Humi = bme.readHumidity();
    }
  }
  delay(1000);//This is a one sec loop 1000 = 1000 msec

  ProcessSerialData();

  os_runloop_once();
}
