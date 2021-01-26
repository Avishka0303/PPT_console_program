/*
  AUTHOR - AVISHKA DHANANJAYA ATHAPATTU
  DATE   - 2019/12/03
  DESC   - PERSONAL PHYSICAL TRAINER VERSION 1 (COMPLETED) PROGRAM FOR LOWER ARM
*/

//----------------------------------------------------------------------------------- INITIALIZE THE LIBRARIES -------------------------------------------------------------------------------------
#include<SPI.h>   //for spi communications
#include<RF24.h>
#include<RF24Network.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

//---------------------------------------------------------------------------------- CUSTOMIZE THE CE AND CSN PINS ----------------------------------------------------------------------------------
#define CE_PIN 7  
#define CSN_PIN 8 
#define INTERRUPT_PIN 3
#define READY 5
#define CALIBRATE 6

const int main_node=00;
const int fore_arm=01;
const int upper_arm=02;

int Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, timex, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
float desired_angle = 0; //angle which we want to balance and stedy

const unsigned long interval =5;  
unsigned long last_sent;

int azimuth_angle,pre_angle=0,yaw_angle;
bool isCalibrated=false;
int buttonState=0;

//------------------------------------------------------------------- INSTANCE FOR COMPASS MODULE ------------------------------------------------------------------------------------------
QMC5883LCompass compass;

//------------------------------------------------------------------ INITIALIZE THE NETWORK INSTANCES ---------------------------------------------------------------------------------------
RF24 radio(CE_PIN,CSN_PIN);
RF24Network network(radio);    

// -------------------------------------------------------------------------- SETUP THE INITIAL SETTINGS ----------------------------------------------------------------------------------------
void setup() {

  pinMode(READY,OUTPUT);
  pinMode(CALIBRATE,OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  
//------------------------------------------------------------------ INITIALIZE THE NETWORK SETUP   --------------------------------------------------------------------------------------------
  
  SPI.begin();
  radio.begin();
  network.begin(90,fore_arm);
  radio.setDataRate(RF24_2MBPS);

  Serial.begin(115200);

//------------------------------------------------------------------ INITIALIZE THE MPU 6050 MODULE -------------------------------------------------------------------------------------------
  Wire.begin(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);           
  Wire.write(0);
  Wire.endTransmission(true);
  timex = millis();

//---------------------------------------------------------------- INITIALIZE THE HMC5883l MODULE ---------------------------------------------------------------------------------------------
  compass.init();

//------------------------------------------------------------------ INITIALIZE CALIBRATIONS ------------------------------------------------------------------------------------------------
  
  while(!isCalibrated){
     compass.read();
     pre_angle = compass.getAzimuth();
     Serial.println(pre_angle);
     delay(100);
     buttonState = digitalRead(INTERRUPT_PIN);
     if(buttonState == HIGH){
        Serial.println(pre_angle);
        delay(1000);
        isCalibrated = true;
     }
  }
}

//--------------------------------------------------------------------SETUP FINISHED --------------------------------------------------------------------------------------------------------
void loop() {
  
  digitalWrite(READY,64);
  
  timePrev = timex;  // the previous time is stored before the actual time read
  timex = millis();  // actual time read
  elapsedTime = (timex - timePrev) / 1000; //ELAPSED TIME FROM LAST LOOP

//------------------------------------------------------------------------------------------ READ ACCELERATION DATA ----------------------------------------------------------------------------
   Wire.beginTransmission(0x68);    //give the slave address
   Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
   Wire.endTransmission(false);
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
    * of the MPU6050. In this case we request just 4 values*/

//----------------------------------------------------------------------------------------- READ GYROSCOPE DATA ---------------------------------------------------------------------------------
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
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

//--------------------------------------------------------------------------- READ THE COMPASS READING ---------------------------------------------------------------------------------------
   compass.read();
   azimuth_angle = compass.getAzimuth();
   if(pre_angle>=0 && pre_angle<=90){
      if(azimuth_angle<=359 && azimuth_angle>=270){
          azimuth_angle =-1* ( (360-azimuth_angle)+pre_angle );  
      }else{
          azimuth_angle = azimuth_angle - pre_angle;
      }
   }else if(pre_angle>=270 && pre_angle<=359){
       if(azimuth_angle<=90 && azimuth_angle>=0){
          azimuth_angle = (360-pre_angle)+azimuth_angle;  
      }else{
          azimuth_angle = azimuth_angle - pre_angle;
      }
   }else{
      azimuth_angle = azimuth_angle - pre_angle;
   } 
   analogWrite(READY,64); 
   
   String angle="001,"+String(totalAngleInterpolation1)+","+String(totalAngleInterpolation2)+","+String(azimuth_angle);
   char bufferstr[21];
   angle.toCharArray(bufferstr,21);
   Serial.println(bufferstr);
   char last[21];
   strncpy(last,bufferstr,21);
   
//--------------------------------------------------------------------------- UPDATE THE NETWORK --------------------------------------------------------------------------------------------
  network.update();
  unsigned long now = millis();
  
  if(now - last_sent >=interval){
    last_sent=now;
    RF24NetworkHeader header(main_node);
    bool ok=network.write(header,&last,strlen(last));
    if(ok==1)digitalWrite(READY,LOW);
  }  
}
