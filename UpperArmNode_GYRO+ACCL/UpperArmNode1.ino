#include<SPI.h>   //for spi communications
#include<RF24.h>
#include<RF24Network.h>
#include <Wire.h>

#define CE_PIN 8  //define the CE pins
#define CSN_PIN 9 //define the CSN pins

int readyToRun = 4; 
int xAngleLED=3;
int yAngleLED=5;
int readyLED=6;

const uint16_t main_node=00;
const uint16_t fore_arm=01;
const uint16_t upper_arm=02;
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, timex, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
float desired_angle = 0; //angle which we want to balance and stedy

const unsigned long interval =5;  
unsigned long last_sent;
int rstatus=0;

const uint8_t MPU_addr=0x68;
size_t no_of_reg_acc=6;
size_t no_of_reg_gyr=4;

RF24 radio(CE_PIN,CSN_PIN);
RF24Network network(radio);     //initialize the network instance

void setup() {

  pinMode(readyToRun,OUTPUT);
  pinMode(xAngleLED,OUTPUT);
  pinMode(yAngleLED,OUTPUT);
  pinMode(readyLED,OUTPUT);
  
  //-------------------------initializations for the network connections
  Serial.begin(115200);
  SPI.begin();
  radio.begin();
  network.begin(90,upper_arm);
  radio.setDataRate(RF24_2MBPS);

  //------------------------ initialize the mpu 6050 --------------------
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68); //give the slave address for MPU 6050
  Wire.write(0x6B);           
  Wire.write(0);
  Wire.endTransmission(true);
  timex = millis(); //Start counting time in milliseconds

  //------------------------ready to calibrate indicator------------------
  for(int i=0;i<30;i++){
    digitalWrite(readyToRun,HIGH);
    delay(50);
    digitalWrite(readyToRun,LOW);
    delay(50);
  }
  
}

void loop() {

  digitalWrite(readyToRun,LOW);
  digitalWrite(readyLED,HIGH);
  
  timePrev = timex;  // the previous time is stored before the actual time read
  timex = millis();  // actual time read
  elapsedTime = (timex - timePrev) / 1000; 
  
  /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   */
  
   Wire.beginTransmission(0x68);    //give the slave address
   Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
   Wire.endTransmission(false);
   //Wire.requestFrom(MPU_addr,no_of_reg_acc,true);
   Wire.requestFrom(0x68,6,true); 
    
   Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
   Acc_rawY=Wire.read()<<8|Wire.read();
   Acc_rawZ=Wire.read()<<8|Wire.read();
   
   /*---X---*/
   Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
   /*---Y---*/
   Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
    * the Z axis (YAW).*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
//   Wire.requestFrom(MPU_addr,no_of_reg_gyr,true); //Just 4 registers
   Wire.requestFrom(0x68,4,true);
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

   float totalAngleInterpolation1=Total_angle[0]/80*90;
   
   float totalAngleInterpolation2=Total_angle[1]/80*90;

   int valx=map((int)totalAngleInterpolation1,-90,90,0,255);
   int valy=map((int)totalAngleInterpolation2,-90,90,0,255);

   analogWrite(xAngleLED,valx);
   analogWrite(yAngleLED,valy);  
   
   String angle="002,"+String(totalAngleInterpolation1)+","+String(totalAngleInterpolation2);
   
   //Serial.println(angle);
   char bufferstr[16];
   angle.toCharArray(bufferstr,16);
   Serial.println(bufferstr);
   
   //----------------------------update the network-----------------------------------------------------
  network.update();
  unsigned long now = millis();
  
  if(now - last_sent >=interval){
    last_sent=now;
    RF24NetworkHeader header(main_node);
    bool ok=network.write(header,&bufferstr,strlen(bufferstr));
    if(ok==1)digitalWrite(readyLED,LOW);
  }  
}
