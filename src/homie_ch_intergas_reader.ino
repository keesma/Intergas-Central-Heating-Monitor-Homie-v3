/*
 * cvintergas - read status from Intergas Central Heating and publish it through MQTT
 *
 * Based on info Leo van der Kallen: http://www.circuitsonline.net/forum/view/80667/last/intergas
 *
 * MQTT topics
 *
 * - settings
 *   -devices/<deviceid>/
 *      - scan-period      - how often is status read from Intergas (in seconds)
 *      - temp-scan-period - how often are tempearatures read (in seconds)
 *      - send-raw-data    - determines whether the raw data is send
 *      - mode             - mode of the monitor (0=normal, 1=receive data through mqtt, 2=standalone)
 * - sensor data
 *   - devices/ichm/...
 *     - temperature/t1, t2, t3, t4, t5, t1-ext, t2-ext, unit, flow, flow-ext, return-ext, setpoint, T1-address, T2-address
 *     - pressure/pressure
 *     - fan/pwm, setpoint, speed, unit
 *     - heating/io-current, opentherm, pump-running, status, status-code, fault-code
 *
 * todo
 * - implement other messages: VER, CRC, REV, EN
 * - set scan period long when CH is idle, and short when active
 *
 * Revision history
 * 0.05   201610xx  First public release
 * 0.1x   20161105  Added two new status codes
 *                  Added parameterto enable/disable logging of raw data
 *                  Increased default scan period from 5.000 to 10.000 ms]
 * 0.2x   20161217  Converted to Homie framework
 * 0.3x   20161230  Converted to Homie v2.0
 * 0.4x   20170103  Added 2 external temperature sensors (flow & return)
 *                  Added simulation by other central heating monitor (conditional)
 * 0.5x   20170505  Changed formatting of status codes. Fixed logging. Changed period temperature sensors reading, formatting codes
 * 2.0.0  20171201  Upgraded to Homie 2.0.0 beta 1
 * 2.0.7  20180508  Upgraded to latest Homie, corrected some lint errors
 * 2.0.9  20181230  Added more central heating status fields that can be sent through mqtt
 * 2.1.0  20190104  Introduced mode: normal, standalone, receive raw data (via mqtt)
 * 2.1.1  20190106  Corrected issue when receiving data
 * 2.1.2  20190302  Corrected pin numbers TX/RX
 * 2.2.0  20190303  Changed handling of mode (not a custom config parameter anymore)
 * 2.2.1  20190303  Added length check to received mqtt data (96 bytes)
 * 3.0.0  20200217  Adapted for Homie 3.0
 * 3.0.1  20200218  Added delays to ensure more messages are send
 * 3.0.2  20200222  Improved handling of fault codes
 * 3.0.6  20200229  Added data types, units and names to advertised data.
 * 3.0.7  20200301  Changed °C character to \u00B0
 */

#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>

#define FW_NAME       "homie-ch"
#define FW_VERSION    "3.0.9"

#define DEBUG  0

#if DEBUG==1
//#define debugln(s)    Serial.println(s)
//#define debug(s)      Serial.print(s)
#else
#define debugln(s)
#define debug(s)
#endif

HomieNode temperatureNode("temperature", "Temperature node", "temperature");
HomieNode fanNode("fan", "Fan node", "fan");
HomieNode pressureNode("pressure", "Pressure node", "pressure");
HomieNode centralHeatingNode("heating", "Heating node", "heating");
//HomieSetting<long> modeSetting("mode", "0=normal, 1=receive raw data, 2=standalone");  // id, description
//HomieSetting<const char *> rawMQTTTopic("MQTT raw data topic","MQTT topic with the raw Intergas data");
HomieSetting<const char *> flowTAddress("Flow sensor address","Flow temperature sensor address");
HomieSetting<const char *> returnTAddress("Return sensor address","Return temperature sensor address");

#define ModeNormal       0
#define ModeRecvRawData  1
#define ModeStandalone   2
long    monitorMode;

bool messageReceived;
bool waitingForAnswer;
bool toggleBuffer;

#define InputBufferLen   64
#define StatusMsgLen     32

byte inputBuffer[InputBufferLen];

byte inputBufferSim1[] = { 0x65, 0x0c, 0x50, 0x0c, 0x46, 0x0c, 0x77, 0x0a, 0x97, 0xf3,
                          0x96, 0xf3, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x41, 0x40, 0x00, 0xff,
                          0x00, 0xff };

byte inputBufferSim2[] = { 0x65, 0x0c, 0x50, 0x0c, 0x46, 0x0c, 0x77, 0x0a, 0x97, 0xf3,
                           0x96, 0xf3, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x41, 0x40, 0x00, 0xff,
                           0x00, 0xff };

int writeIndex;
#define PinLED            D6

#define ONE_WIRE_BUS      D5  // DS18B20 pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
DeviceAddress t1Address, t2Address;
bool t1Present = false, t2Present = false;
char t1AddressString[24], t2AddressString[24];
char t1Name[15];
char t2Name[15];

#define PinRX   D2
#define PinTX   D1
SoftwareSerial intergas(PinRX, PinTX, false ); // RX, TX, not inverted

#define TimeoutIntergas  (3*1000) // in milliseconds

long  receiveTimer;
long  scanPeriod;
long  tScanPeriod;
bool  sendRawData;

double  ch_pressure;
unsigned int ch_status_code, ch_fault_code;
bool         ch_has_pressure_sensor, ch_opentherm;
char         ch_input_buffer[3*StatusMsgLen+5];      // ASCII bytes with spaces
char         ch_output_buffer[3*StatusMsgLen+5];      // ASCII bytes with spaces

byte toHex(char c) {
   byte  result = 0;
   if (c >= '0' && c <= '9') {
      result = c - '0';
   } else if (tolower(c) >= 'a' && tolower(c) <= 'f') {
      result = tolower(c) - 'a' + 0x0a;
   } else {
      // Error
      result = 0;
      Homie.getLogger() << "ERROR: converting character to hex " << c << endl;
   }
   return result;
}

byte toHex(char c1, char c2) {
   byte result = (toHex(c1) << 4) + toHex(c2);
   return result;
}

bool bufferInputHandler(HomieRange range, String value) {
       //Receive data from another device
    bool result;

    result = false;
    Homie.getLogger() << "Received data from Intergas through MQTT" << endl;
    Homie.getLogger() << "Received " << value.length() << " bytes" << endl;

    if (monitorMode == ModeRecvRawData) {
        if (value.length() == 96) {
            ledOn();
            // Messagee has correct length
            for (int i = 0; i < 32; i++) {
                inputBuffer[i] = toHex(value.charAt(3*i),value.charAt(3*i+ 1));
                Homie.getLogger() << c2h((inputBuffer[i] & 0xf0) >> 4) << c2h(inputBuffer[i] & 0x0f) << " ";
            }
            Homie.getLogger() << endl;
            waitingForAnswer = false;
            messageReceived = true;
            result = true;
            ledOff();
        }
   } else {
       result = true;
   }
   return result;
}

char *ftoa(char *a, double f, int precision) {
   long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

   char *ret = a;
   long heiltal = (long)f;
   itoa(heiltal, a, 10);
   while (*a != '\0') a++;
   *a++ = '.';
   long deci = abs((long)((f - heiltal) * p[precision]));
   itoa(deci, a, 10);

   return ret;
}

void printFreeHeap() {
   Serial.printf("Free heap: %u\r\n", ESP.getFreeHeap());
}

void ledOn() {
    digitalWrite(PinLED, HIGH);    // turn the LED on by making the voltage HIGH
}

void ledOff() {
    digitalWrite(PinLED, LOW);    // turn the LED off by making the voltage LOW
}

long lastMsg = 0;
long lastTMsg = 0;

bool centralHeatingModeHandler(const HomieRange& range, const String& value) {
    long mode;

    Homie.getLogger() << "Received monitor mode " << value << endl;
    mode = value.toInt();
    if (mode >= 0 && mode <= 2) {
        monitorMode = mode;
        return true;
    } else {
        return false;
    }
}

bool centralHeatingTPeriodHandler(const HomieRange& range, const String& value) {
  long period;
  period = value.toInt();
  if (period > 1) {
     tScanPeriod = period*1000;  // in milliseconds
     centralHeatingNode.setProperty("temp-scan-period").send(String(period));
     return true;
  } else {
     return false;
  }
}

bool centralHeatingPeriodHandler(const HomieRange& range, const String& value) {
  long period;
  period = value.toInt();
  if (period > 1) {
     scanPeriod = period*1000;  // in milliseconds
     centralHeatingNode.setProperty("scan-period").send(String(period));
     return true;
  } else {
     return false;
  }
}

bool centralHeatingRawDataHandler(const HomieRange& range, const String& value) {
  if (value == "true") {
    centralHeatingNode.setProperty("send-raw-data").setRetained(true).send("true");
    sendRawData = true;
  } else if (value == "false") {
    centralHeatingNode.setProperty("send-raw-data").send("false");
    sendRawData = false;
  } else {
    return false;
  }
  return true;
}

void intergasFlush() {
     // Flush already received characters.
   while (intergas.available() > 0) {
      intergas.read();
   }
}

void requestChStatus() {
   if (monitorMode == ModeNormal) {
     // in other modes we do not have a connection to the Intergas
       ledOn();
       Homie.getLogger() << "Request status from Intergas - normal mode" << endl;;
       waitingForAnswer = true;
       intergasFlush();
       intergas.write('S');
       intergas.write('?');
       intergas.write('\r');
       receiveTimer = millis();
       ledOff();
   } else if (monitorMode == ModeStandalone) {
       ledOn();
       Homie.getLogger() << "Request status from Intergas - simulated mode" << endl;
       int i;

       toggleBuffer = !toggleBuffer;
       if (toggleBuffer)
            for (i = 0; i < 32; i++) inputBuffer[i] = inputBufferSim1[i];
        else
            for (i = 0; i < 32; i++) inputBuffer[i] = inputBufferSim2[i];
        waitingForAnswer = false;
        messageReceived = true;
        ledOff();
   }
}

char c2h(char c){
  return "0123456789ABCDEF"[0x0F & (unsigned char)c];
}

void readChStatus(){
      // read status message from central heating
   byte  ch;
   long  t;

   if (waitingForAnswer) {
      t = millis();
      if ((t - receiveTimer) <= TimeoutIntergas) {
//         Serial.print(F("timer: ")); Serial.println(t);
         if (intergas.available()) {
            ch = intergas.read();
            if (writeIndex == 0) {
               Serial.println(F("Received from Intergas:"));
            }
            inputBuffer[writeIndex++] = ch;
            Serial.print(F(" ")); Serial.print(c2h( ((ch&0xf0) >> 4))); Serial.print(c2h(ch&0x0f));
            if (writeIndex == StatusMsgLen) {
              // Message does not have markers. When 32 bytes are received stop reading
              Serial.println();
              writeIndex = 0;
              waitingForAnswer = false;
              messageReceived = true;
            }
         }
      } else {
         writeIndex = 0;
         waitingForAnswer = false;
         Serial.println(F("Timeout Intergas"));
      }
   }
}

double getDouble(byte msb, byte lsb) {
      // Calculate float value from two bytes
   double  result;
//   double  res1;

   if (msb > 127) {
       result = -(((msb ^ 255) + 1) * 256 - lsb) / 100;
   } else {
       result = ((float)(msb * 256 + lsb)) / 100;
   }
   return result;
}

void processChStatus() {
      // Retrieve the variables from the message
   if (messageReceived) {
      char   scratch[100];

      messageReceived = false;
      temperatureNode.setProperty("t1").send(ftoa(scratch, getDouble(inputBuffer[1],  inputBuffer[0]), 2));
      temperatureNode.setProperty("t2").send(ftoa(scratch, getDouble(inputBuffer[5],  inputBuffer[4]), 2));
      temperatureNode.setProperty("t3").send(ftoa(scratch, getDouble(inputBuffer[7],  inputBuffer[6]), 2));
      temperatureNode.setProperty("t4").send(ftoa(scratch, getDouble(inputBuffer[9],  inputBuffer[8]), 2));
      temperatureNode.setProperty("t5").send(ftoa(scratch, getDouble(inputBuffer[11], inputBuffer[10]), 2));
      temperatureNode.setProperty("flow").send(ftoa(scratch, getDouble(inputBuffer[3], inputBuffer[2]), 2));
      temperatureNode.setProperty("setpoint").send(ftoa(scratch, getDouble(inputBuffer[15], inputBuffer[14]), 2));
      delay(20);  // to ensure all messages are send now
      fanNode.setProperty("speed").send(ftoa(scratch, getDouble(inputBuffer[19],  inputBuffer[18])*100, 0));
      fanNode.setProperty("setpoint").send(ftoa(scratch, getDouble(inputBuffer[17],  inputBuffer[16])*100, 0));
      fanNode.setProperty("pwm").send(ftoa(scratch, getDouble(inputBuffer[21],  inputBuffer[20])*10, 0));

      ch_pressure = getDouble(inputBuffer[13],  inputBuffer[12]);
      ch_has_pressure_sensor    = (inputBuffer[28] & 0x20) == 0x20;
      ch_opentherm  = (inputBuffer[26] & 0x80) == 0x80;
      if (!ch_has_pressure_sensor) {
         ch_pressure   = -35;
      }
      delay(20);  // to ensure all messages are send now
      centralHeatingNode.setProperty("io-current").send(ftoa(scratch, getDouble(inputBuffer[23],  inputBuffer[22]), 0));
      pressureNode.setProperty("pressure").send(ftoa(scratch, ch_pressure, 0));
      sprintf(scratch, "%u",inputBuffer[28]*256+inputBuffer[26]);
      centralHeatingNode.setProperty("status").send(scratch);

// you can decide to send these fields individually. Bytes 26 & 28 containing the status individually.

      centralHeatingNode.setProperty("gp-switch").send((inputBuffer[26] & 0x01) == 0x01 ? "true" : "false");
      centralHeatingNode.setProperty("tap-switch").send((inputBuffer[26] & 0x02) == 0x02 ? "true" : "false");
      centralHeatingNode.setProperty("roomtherm").send((inputBuffer[26] & 0x04) == 0x04 ? "true" : "false");
      centralHeatingNode.setProperty("pump-running").send(((inputBuffer[26] & 0x08) == 0x08 ? "true" : "false"));
      centralHeatingNode.setProperty("dwk").send(((inputBuffer[26] & 0x10) == 0x10 ? "true" : "false"));
      centralHeatingNode.setProperty("alarm-status").send(((inputBuffer[26] & 0x20) == 0x20 ? "true" : "false"));
      centralHeatingNode.setProperty("cascade-relay").send(((inputBuffer[26] & 0x40) == 0x40 ? "true" : "false"));
      centralHeatingNode.setProperty("opentherm").send((inputBuffer[26] & 0x80) == 0x80 ? "true" : "false");
//
delay(20);  // to ensure all messages are send now
//
      centralHeatingNode.setProperty("gas-valve").send((inputBuffer[28] & 0x01) == 0x01 ? "true" : "false");
      centralHeatingNode.setProperty("spark").send((inputBuffer[28] & 0x02) == 0x02 ? "true" : "false");
      centralHeatingNode.setProperty("ionisation-signal").send((inputBuffer[28] & 0x04) == 0x04 ? "true" : "false");
      centralHeatingNode.setProperty("ot-disabled").send(((inputBuffer[28] & 0x08) == 0x08 ? "true" : "false"));
      centralHeatingNode.setProperty("has-low-water-pressure").send(((inputBuffer[28] & 0x10) == 0x10 ? "true" : "false"));
      centralHeatingNode.setProperty("has-pressure-sensor").send(((inputBuffer[28] & 0x20) == 0x20 ? "true" : "false"));
      centralHeatingNode.setProperty("burner-block").send(((inputBuffer[28] & 0x40) == 0x40 ? "true" : "false"));
      centralHeatingNode.setProperty("gradient-flag").send(((inputBuffer[28] & 0x80) == 0x80 ? "true" : "false"));

      ch_status_code = inputBuffer[24];
      if (inputBuffer[27] == 0x80) {
         ch_fault_code  = (unsigned int)inputBuffer[29];
//         ch_status_code = 256 + ch_fault_code;
      } else {
         ch_fault_code = 0;
      }
      sprintf(scratch, "%u", (unsigned int)inputBuffer[29]);
      centralHeatingNode.setProperty("last-fault-code").send(scratch);
      sprintf(scratch, "%u", ch_fault_code);
      centralHeatingNode.setProperty("fault-code").send(scratch);
      sprintf(scratch, "%u", ch_status_code);
      centralHeatingNode.setProperty("status-code").send(scratch);
      if (sendRawData) {
        for (int i = 0; i < StatusMsgLen; i++) {
           ch_output_buffer[3*i] = c2h((inputBuffer[i] & 0xf0) >> 4);
           ch_output_buffer[3*i + 1] = c2h(inputBuffer[i] & 0x0f);
           ch_output_buffer[3*i + 2] = ' ';
        }
//
delay(20);  // to ensure all messages are send now
//
        centralHeatingNode.setProperty("output-buffer").send(ch_output_buffer);
      }
	}
}

void requestTemperatures() {
    float  temperature;

    ledOn();
    DS18B20.requestTemperatures();
    if (t1Present) {
        temperature = DS18B20.getTempCByIndex(0);
        temperatureNode.setProperty(t1Name).send(String(temperature));
        Homie.getLogger() << "T1: " << temperature << endl;
    }
    if (t2Present) {
        temperature = DS18B20.getTempCByIndex(1);
        temperatureNode.setProperty(t2Name).send(String(temperature));
        Homie.getLogger() << "T2: " << temperature << endl;
    }
    ledOff();
}

void setupHandler() {
   const char *t;

   DS18B20.begin();
   Serial.print("Found "); Serial.print(DS18B20.getDeviceCount()); Serial.println(" sensors");
   Serial.print("Sensor flow config: ["); Serial.print(flowTAddress.get()); Serial.println("]");
   Serial.print("Sensor return config: ["); Serial.print(returnTAddress.get()); Serial.println("]");
     if (DS18B20.getAddress(t1Address, 0)) {
        t1Present = true;
        Homie.getLogger() << "Temperature sensor 1: ";
        addressToString(t1Address, t1AddressString);
        Homie.getLogger() << t1AddressString << endl;
        t = flowTAddress.get();
        if (t != 0) {
          if (!strncmp(t,t1AddressString,23)) {
             strcpy(t1Name, "flow-ext");
          } else if (!strncmp(t,t2AddressString,23)) {
             strcpy(t1Name, "return-ext");
          } else {
             Homie.getLogger() << "Warning: unknown sensor address" << endl;
          }
        } else {
          Homie.getLogger() << "T1 sensor addresss config is empty" << endl;
        }
     }
     if (DS18B20.getAddress(t2Address, 1)) {
        t2Present = true;
        Homie.getLogger() << "Temperature sensor 2: ";
        addressToString(t2Address, t2AddressString);
        Homie.getLogger() << t2AddressString << endl;
        if (!strncmp(returnTAddress.get(),t1AddressString,23)) {
           strcpy(t2Name, "flow-ext");
        } else if (!strncmp(returnTAddress.get(),t2AddressString,23)) {
           strcpy(t2Name, "return-ext");
        } else {
          Homie.getLogger() << "Warning: unknown sensor address" << endl;
        }
     }
}

void addressToString(DeviceAddress deviceAddress, char *s) {
  for (uint8_t i = 0; i < 8; i++) {
    // zero pad the address if necessary
    s[2*i] = c2h(deviceAddress[i] & 0xf00 >> 4);
    s[2*i + 1] = c2h(deviceAddress[i] & 0x0f);
  }
  s[17] = '\0';
}

void setup() {
    Serial.begin(115200);
//    delay(10000);
    Serial.println();
    Serial.println();
    Homie_setFirmware(FW_NAME, FW_VERSION);

    pinMode(PinLED, OUTPUT);
    ledOn();
    messageReceived = false;
    waitingForAnswer = false;
    toggleBuffer = false;

    scanPeriod =  10000;  // milliseconds
    tScanPeriod = 60000;  // milliseconds
    sendRawData = false;
    monitorMode = ModeNormal;

    lastMsg = 0;
    lastTMsg = 0;

    Serial.println(F("\r\nIntergas Central Heating Monitor " FW_VERSION));
    Serial.println(F(__TIMESTAMP__));
    Serial.println();
    Serial.printf("RX pin %d, TX pin %d, One Wire bus %d, LED pin %d\r\n", PinRX, PinTX, ONE_WIRE_BUS, PinLED);
    Serial.flush();
    intergas.begin(9600);
    printFreeHeap();

   //Homie.disableResetTrigger();  To check if it restarts better
    Homie.disableLedFeedback();
    Homie.setSetupFunction(setupHandler);
    Homie.setLoopFunction(loopHandler);

    strcpy(t1Name, "t1-ext");
    strcpy(t2Name, "t2-ext");
    strcpy(t1AddressString, "undefined");
    strcpy(t2AddressString, "undefined");
    flowTAddress.setDefaultValue("");
    returnTAddress.setDefaultValue("");

    //   temperatureNode.setProperty("unit").send("\u00B0");
    //   temperatureNode.setUnit("\u00B0").setDatatype("float");
       temperatureNode.setProperty("T1-address").send(t1AddressString);
       temperatureNode.setProperty("T2-address").send(t2AddressString);
    //   fanNode.setProperty("unit").send("rpm");
       centralHeatingNode.advertise("send-raw-data").settable(centralHeatingRawDataHandler).setDatatype("boolean").setName("Send raw data from Central heating");
       centralHeatingNode.advertise("scan-period").settable(centralHeatingPeriodHandler).setDatatype("integer").setName("Central heating scan interval").setUnit("s");
       centralHeatingNode.advertise("temp-scan-period").settable(centralHeatingTPeriodHandler).setDatatype("integer").setName("Temperature scan interval").setUnit("s");
       centralHeatingNode.advertise("mode").settable(centralHeatingModeHandler).setDatatype("integer").setName("Central heating monitor operating mode");
       centralHeatingNode.advertise("input-buffer").settable(bufferInputHandler).setDatatype("string").setName("Input buffer");

       // Central heating
       centralHeatingNode.advertise("io-current").setDatatype("float").setUnit("mA?").setName("Ionisation current");
       centralHeatingNode.advertise("status").setDatatype("integer").setName("Status word");

       centralHeatingNode.advertise("gp-switch").setDatatype("boolean").setName("Gp switch");
       centralHeatingNode.advertise("tap-switch").setDatatype("boolean").setName("Tap switch");
       centralHeatingNode.advertise("roomtherm").setDatatype("boolean").setName("Roomtherm");
       centralHeatingNode.advertise("pump-running").setDatatype("boolean").setName("Pump running");
       centralHeatingNode.advertise("dwk").setDatatype("boolean").setName("Send raw data from Central heating");
       centralHeatingNode.advertise("alarm-status").setDatatype("boolean").setName("Alarm status");
       centralHeatingNode.advertise("cascade-relay").setDatatype("boolean").setName("Cascade relay");
       centralHeatingNode.advertise("opentherm").setDatatype("boolean").setName("Opentherm");
       centralHeatingNode.advertise("gas-valve").setDatatype("boolean").setName("Gas valve open");
       centralHeatingNode.advertise("spark").setDatatype("boolean").setName("Spark");
       centralHeatingNode.advertise("ionisation-signal").setDatatype("boolean").setName("Ionisation signal");
       centralHeatingNode.advertise("ot-disabled").setDatatype("boolean").setName("Opentherm disable");
       centralHeatingNode.advertise("has-low-water-pressure").setDatatype("boolean").setName("Has low water pressure");
       centralHeatingNode.advertise("has-pressure-sensor").setDatatype("boolean").setName("Has pressure sensor");
       centralHeatingNode.advertise("burner-block").setDatatype("boolean").setName("Burner block");
       centralHeatingNode.advertise("gradient-flag").setDatatype("boolean").setName("Gradient flag");

       centralHeatingNode.advertise("last-fault-code").setDatatype("integer").setName("Last fault code");
       centralHeatingNode.advertise("fault-code").setDatatype("integer").setName("Fault code");
       centralHeatingNode.advertise("status-code").setDatatype("integer").setName("Status code");

       // Temperatures
       temperatureNode.advertise("t1").setDatatype("float").setUnit("C").setName("Temperature t1");
       temperatureNode.advertise("t2").setDatatype("float").setUnit("C").setName("Temperature t2");
       temperatureNode.advertise("t3").setDatatype("float").setUnit("C").setName("Temperature hot water (t3)");
       temperatureNode.advertise("t4").setDatatype("float").setUnit("C").setName("Temperature boiler (t4)");
       temperatureNode.advertise("t5").setDatatype("float").setUnit("C").setName("Temperature outside (t5)");
       temperatureNode.advertise("flow").setDatatype("float").setUnit("C").setName("Temperature flow");
       temperatureNode.advertise("setpoint").setDatatype("float").setUnit("C").setName("Temperature setpoint");
       temperatureNode.advertise("T1-address").setDatatype("string").setName("Temperature sensor address T1");
       temperatureNode.advertise("T2-address").setDatatype("string").setName("Temperature sensor address T2");

       // Fan
       fanNode.advertise("speed").setDatatype("float").setUnit("rpm").setName("Fan speed");
       fanNode.advertise("setpoint").setDatatype("float").setUnit("rpm").setName("Fan setpoint");
       fanNode.advertise("pwm").setDatatype("float").setUnit("%").setName("Fan pmw");

       // Pressure
       pressureNode.advertise("pressure").setDatatype("integer").setUnit("Pa").setName("Pressure");

    Homie.setup();
    ledOff();
    delay(5000);
}

void loopHandler() {
  long timeInMillis = millis();
  if (timeInMillis - lastTMsg > tScanPeriod) {
     lastTMsg = timeInMillis;
     requestTemperatures();
  }
  timeInMillis = millis();
  if (timeInMillis - lastMsg > scanPeriod) {
     lastMsg = timeInMillis;
     requestChStatus();
  }
  readChStatus();
  processChStatus();
}

void loop() {
  Homie.loop();
}
