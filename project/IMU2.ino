#include <Wire.h>
#include "Kalman.h"
//#include <SPI.h>
#include <nRF24L01.h>
//#include <RF24.h>
#include <I2Cdev.h>
//RF24 radio(7, 8); // CE, CSN
//const byte address[6] = "00001";
///////////

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication


///////////

Kalman kalmanX;
Kalman kalmanY;
//uint8_t IMUAddress = 0x68;
/* IMU Data */
int16_t accX;
int16_t accY;
int16_t accZ;
//int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
double accXangle; // Angle calculate using the accelerometer
double accYangle;
double gyroXangle = 180; // Angle calculate using the gyro
double gyroYangle = 180;
double compAngleX = 180; // Calculate the angle using a Kalman filter
double compAngleY = 180;
double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;
///////////


uint8_t i2cData[14]; // Buffer for I2C data
 
int pitch,roll,yawr,yawl,yaw,throttle,thmap,ylmap,yrmap,newAngleX,newAngleY,medX,medY,arming;
uint32_t timer;
String pixhawk;
byte armstate,pressed,transperm,calflag,calbutton;
////
int yawllb = 315; //low ,active and upper boundaries
int yawlub = 500;
int yawlact = 420;
////
int yawrlb = 280; 
int yawrub = 470;
int yawract = 420;
String inByte = " ";
///
void setup() {
  Serial.println("FF");
  pinMode( A0, INPUT ); //throttle sensor
  pinMode( A1, INPUT ); //left sensor
  pinMode( A2, INPUT ); //right sensor
  
  pinMode(4,INPUT_PULLUP); //arming button
  pinMode(3,INPUT_PULLUP); // calibration button

  pinMode(5,OUTPUT); // arming led


  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  
  Serial.begin(9600/*115200*/);
  Serial.println("test");
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  
  //i2cWrite(0x6B,0x00); // Disable sleep mode
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  Serial.println("de");
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  /*
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  //display.display();
  //delay(2000); // Pause for 2 seconds
 // display.clear();
  display.clearDisplay();
  */
 // blinkled(5,10);
  kalmanX.setAngle(180); // Set starting angle
  kalmanY.setAngle(180);
  timer = micros();
  digitalWrite(5,LOW);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();
  arming = 1000;
  calflag = 0;
  calbutton = 0;
  newAngleX = 0;
  newAngleY = 0;
  
}

void loop() {
  /* Update all the values */

  /*if(Serial.available()){ // only send data back if data has been sent
    inByte = Serial.readString(); // read the incoming data
    Serial.println(inByte); // send the data back in a new line so that it is not all one long line
  }*/

    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
//    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
    
  /* Calculate the angls based on the different sensors and algorithm */
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;    
  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  
  kalAngleX = newAngleX + kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);// - newAngleX; // Calculate the angle using a Kalman filter
  kalAngleY = newAngleY + kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);// - newAngleY;
  timer = micros();
  ////360 FLIP TEST REQUIRED
  pitch = map(int(kalAngleX),110,250,1200,2000)-100;//int to double stable check
  roll = map(int(kalAngleY),110,250,1200,2000)-100;
  pitch = pitch;
  roll = roll;
  
  if(pitch > 1999){
     pitch = 1999;
  }
  if(roll > 1999){
     roll = 1999;
  }
  if(pitch < 999){
     pitch = 1000;
  }
  if(roll < 999){
     roll = 1000;
  }
  
  yaw = 1500;
  yawr = analogRead(A1); //left and right sensors data
  yawl = analogRead(A2);
     
  if(yawr > yawract){//previous yawl value 780
      if (yawr > yawract){//previous yawl value 770
        
        yaw = map(yawr,yawrlb,yawrub,0,120);
       yaw = 1500 - yaw;
       if(yaw < 1300){
        yaw = 1300;
       }
      }
      else yaw = 1500;
  }
  else if(yawl >yawlact){//previous yawr 910
      yaw = map(yawl,yawllb,yawlub,1500,1780);
      yaw = yaw - 100;
      if(yaw > 1799){
        yaw = 1799;
      }
      if(yaw < 1500){
        yaw = 1500;
      }
  }
  else yaw = 1500;
  //////1023 check!!!!!!!!!!
  if(yaw >= 2000){
    yaw = 1999;
  }

  if(yaw <= 1000){
    yaw = 1999;
  }
  throttle= analogRead(A0);
  //throttle = 1023;
  /*if (throttle == 1023 || throttle == 0){
    while(1 > 0 ){
      blinkled(5,3);
      delay(100);
    }
  }*/
  thmap = map(throttle,616,820,1000,1999);
  thmap = thmap - thmap%10;
  thmap = thmap - 100;
  if (thmap > 1700  ){
    thmap = 1700;
  }
  if(thmap < 1000){
    thmap = 1000;
  }
    
  calbutton = digitalRead(3);
  if( calbutton == 0 && calflag == 0){
    delay(1);
    calbutton = digitalRead(3);
    if(calbutton == 0){
      calflag = 1;
      Serial.println("CALIBRATION");
      calibration();
      delay(100);
      transperm = 1;
    }
  }
  
  
  armstate = digitalRead(4);
  if (armstate == 0 && pressed == 0){
    delay(10);
    armstate = digitalRead(4);
    if (armstate == 0){
      pressed = 1;
    }
  }
  else if (armstate == 1 && pressed == 1){
    pressed = 0;
    if(arming == 2000){
      arming = 1000;
      digitalWrite(5,LOW);
      //Serial.println("Disarmed");
    }
    else if(arming == 1000){
      if(thmap < 1900){
        arming = 2000;
        digitalWrite(5,HIGH);
      }
      else {
       /* digitalWrite(4,HIGH);
        delay(200);
        digitalWrite(4,LOW);
        delay(200);
        digitalWrite(4,HIGH);
        delay(200);
        digitalWrite(4,LOW);
        delay(200);
        digitalWrite(4,HIGH);
        delay(200);
        digitalWrite(4,LOW);*/
        blinkled(5,3);
      }
      //Serial.println("Armed");
    }  
  }
  /*
  if (arming == 2000 && throttle < 1100 && yaw == 1500){

  //autostabilize,transfer control  to pixhawl
  }
  */
  /*display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(27,30);
  display.print(throttle);*/
  char transdata[20];
  String gimbal;
  pixhawk = makedata(thmap, yaw, pitch, roll, arming);
  pixhawk +=gimbal;
  
  
  /*for(int i = 0; i < 20;i++){
    transdata[i] = pixhawk[i]; 
  }
  Serial.println(pixhawk);
  
  if( transperm == 1){ // begin transmission after calibration
    radio.write(&transdata, sizeof(transdata));
  }  //if nrf24 isnt connected programm will stop after this check 

  */

}
/*
void i2cWrite(uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}
uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];  
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++)
    data[i] = Wire.read();
  return data;
} */
String makedata(int throttle,int yaw,int pitch,int roll,int arming){
  String ans = "";
  if(throttle < 1000)
    ans = ans + "0" + throttle;
  else
    ans += throttle; 
  if(yaw < 1000)
    ans = ans + "0" + yaw;
  else 
    ans += yaw;
    
  if (pitch < 1000)
    ans = ans + "0" + pitch;
  else 
    ans += pitch;
  if (roll < 1000)
    ans = ans + "0" + roll;
  else
    ans += roll;
  ans += arming;
  
  return ans;
}
void calibration(){
 for(int i = 0; i< 10;i++){
    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
 //   tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
    
    accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
    accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;    
    
    double gyroXrate = (double)gyroX/131.0;
    double gyroYrate = -((double)gyroY/131.0);
    //gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter  
    //gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
 
    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
    timer = micros();
    medX += kalAngleX;
    medY += kalAngleY;
    delay(80);
  }
  newAngleY =180 - medY /10;
  newAngleX= 180 - medX / 10;
  //Serial.println(newAngleX);
  //Serial.println(newAngleY);
  //kalmanX.setAngle(newAngleX); // Set starting angle
  //kalmanY.setAngle(newAngleY);
  
    
}


void blinkled(int port,int number){
  for(int i = 0; i < number; i++){
    digitalWrite(port,HIGH);
    delay(100);
    digitalWrite(port,LOW);
    delay(100);
  }

}



uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}


uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
