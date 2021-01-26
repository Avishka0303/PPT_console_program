#include<SPI.h>   //for spi communications
#include<RF24.h>
#include<RF24Network.h>

#define CE_PIN 7
#define CSN_PIN 8
#define READY_PIN 6

RF24 radio(CE_PIN,CSN_PIN);
RF24Network network(radio);

const uint16_t main_node=00;
const uint16_t fore_arm=01;
const uint16_t upper_arm=02;

void setup() {
  pinMode(READY_PIN,OUTPUT);
  SPI.begin();
  radio.begin();
  network.begin(90,main_node);
  radio.setDataRate(RF24_2MBPS);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(READY_PIN,LOW);
  network.update();
  while(network.available()){
    RF24NetworkHeader header;
    int len=0;
    char msg[]="";
    len=radio.getDynamicPayloadSize();
    network.read(header,&msg,len);  
    digitalWrite(READY_PIN,HIGH);
    Serial.println(msg);
  }
}
