/*
 Basic MQTT example 
 
  - connects to an MQTT server
  - publishes "hello world" to the topic "outTopic"
  - subscribes to the topic "inTopic"
  */

  #include <SPI.h>
  #include <PubSubClient.h>
  //#include "PubSubClient/PubSubClient.h"
  #include "CC3000.h"
  #include "PubSubClient.h"
  #include <OneWire.h>

  OneWire  ds(8);  // on pin 10 (a 4.7K resistor is necessary)



// Update these with values suitable for your network.
char mac[]    = {  0x8c, 0x3a, 0xe3, 0x43, 0x9f, 0xdf };
char server[] = "m2m.eclipse.org"; //Denna används inte, utan det är en fullösning i WiFiClient.cpp connect()
char ip[]     = { 172, 16, 0, 100 };



void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}



WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);


void setup(){Serial.begin(115200);}

void loop()
{
  delay(500);
  //client.loop();
  char cmd;

  Serial.println();
  Serial.println(F("+-------------------------------------------+"));
  Serial.println(F("|      TriForce         |"));
  Serial.println(F("+-------------------------------------------+"));
  Serial.println();
  Serial.println(F("  1 - Anslut till internet"));
  Serial.println(F("  2 - Skicka data till server"));
  Serial.println(F("  3 - Loopa data till server"));
  Serial.println(F("  4 - Disconnect från internet"));
  
  Serial.println();
  for (;;) {
    while (!Serial.available()) {
      
      }
    cmd = Serial.read();
    if (cmd!='\n' && cmd!='\r') {
      break;
      }
    }

  switch(cmd) {
    case '1':
      ConnectWifi();
      break;
    case '2':
      SendData();
      break;
    case '3':
      SendDataLoop();
      break;
    case '4':
      Disconnect();
      break;
    default:
      Serial.print(F("**Unknown command \""));
      Serial.print(cmd);
      Serial.println(F("\"**"));
      break;
    }

  }

void ConnectWifi(void){

  char ssid[] = "and";
  char pass[] = "hejsanhejsan";
  WiFi.begin(ssid, pass);
}

void SendData(void){

  float celsius = temp();

  //Serial.print("Temperaturen är: ");
  //Serial.println(celsius);  

  char temp[10];
  int nycels = int(celsius);
  String stringTemp(nycels);
  //Serial.print("avrundad temp är:");Serial.println(nycels);
  stringTemp.toCharArray(temp, stringTemp.length()+1);

  if (client.connect("ArduinoClient")) {

    Serial.println("Det gick att connecta to client...");
    client.publish("triforce", (uint8_t*)temp, stringTemp.length()+1, 1);
  } else if(client.loop()){
    Serial.println("Det gick att loopa to client...");
    client.publish("triforce", (uint8_t*)temp, stringTemp.length()+1, 1);
  }
}

void SendDataLoop(void){

  Serial.println("Send anything.");
  while (!Serial.available()) { 
    float celsius = temp();
    char temp[10];
    int nycels = int(celsius);
    String stringTemp(nycels);
    //Serial.print("avrundad temp är:");Serial.println(nycels);

    //Test
  
  char tmp[10];
  dtostrf(celsius,1,2,tmp);
  
    //test
    stringTemp.toCharArray(temp, stringTemp.length()+1);

    if (client.connect("ArduinoClient")) {

      Serial.println("Det gick att connecta to client...");
      client.publish("triforce", (uint8_t*)tmp, 5, 1);
    } else if(client.loop()){
      Serial.println("Det gick att loopa to client...");
      client.publish("triforce", (uint8_t*)tmp, 5, 1);
    }
    delay(5000); 
  }
}

void Disconnect(void){
  // client.disconnect();
  WiFi.disconnect();
}

float temp()
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    // return;
  }
  
  // Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    // Serial.write(' ');
    // Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      // return;
  }
  // Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      // Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      // Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      // Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      // return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  // Serial.print("  Data = ");
  // Serial.print(present, HEX);
  // Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    // Serial.print(data[i], HEX);
    // Serial.print(" ");
  }
  // Serial.print(" CRC=");
  // Serial.print(OneWire::crc8(data, 8), HEX);
  // Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0 - 2;
  // fahrenheit = celsius * 1.8 + 32.0;
  // Serial.print("  Temperature = ");
  // Serial.print(celsius);
  // Serial.print(" Celsius, ");
  // Serial.print(fahrenheit);
  // Serial.println(" Fahrenheit");

  return celsius;
} 
