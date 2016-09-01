//CODE BY VINÍCIUS JEAN FERREIRA
//EASILY AND CLEAN SYSTEM TO OBTAIN ANGLES OF MPU9255
//NOTE THAT THE ANGLES ARE FUSED TO BEST RESULTS USING A FUSION ALGORITHM BASED ON STATISTICAL METHODS (VARIANCE, OPTIMAL ESTIMATE)
//NOTE THAT THE SYSTEM NEEDS A STARTING TIME WHICH WILL VARY IF YOU CHANGE THE NUMBER OF MEASUREMENTES, THIS WILL PREJUDICE THE PRECISION OF THE MEASURE
//USE FOR YOU PROJECTS AND CONTACT ME FOR ANY DOUBT: Shout.and.Hawks@hotmail.com

#include<Wire.h>
#include "Statistic.h"

//I2C ADRESS
const int MPU=0x68;
 
//Variaveis globais
int acelX, acelY, acelZ, gyroX, gyroY, gyroZ, magX, magY; //variáveis que recebem o valor I2C em LSB
float acelXr, acelYr, acelZr, gyroXr, gyroYr, gyroZr; //variáveis que recebem o valor convertido pra unidades físicas
float AngGyroX, AngGyroY, AngGyroZ, AngAcelX, AngAcelY, AngAcelZ, AngMagZ; //Variáveis que recebem os valores angulares
float timeiAcel[3], timefAcel[3];
float timeiGyro[3], timefGyro[3];
int i = 0;
float offsetGyroX, offsetGyroY, offsetGyroZ;
float offsetAcelX, offsetAcelY, offsetAcelZ;
float AngXfused, AngYfused, AngZfused;
float AngNorthi;


Statistic AngAcel[3];
Statistic AngGyro[3];
Statistic AngMag[1];
 

void setup()
{ 
  
  Serial.begin(9600); 
  Wire.begin();                 
  Wire.beginTransmission(MPU);  
  Wire.write(0x6B);             
   
  //INITIALIZING MPU 9255
  Wire.write(0); 
  Wire.endTransmission(true);

  //CONFIGURE MAGNETOMETER TO CONTINUES MEASURES
  Wire.beginTransmission(MPU);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x01);
  Wire.endTransmission();


  //CALCULA OFFSETS
  Serial.println("CALCULATING OFFSETS..."); 
  
  for(i = 0; i < 200; i++)
  {
    getGyroData();
    getAcelData();
    offsetGyroX += gyroX;
    offsetGyroY += gyroY;
    offsetGyroZ += gyroZ;
    offsetAcelX += acelX;
    offsetAcelY += acelY;
    offsetAcelZ += acelZ; 
  }
  offsetGyroX /= 200;
  offsetGyroY /= 200;
  offsetGyroZ /= 200;
  offsetAcelX /= 200;
  offsetAcelY /= 200;
  offsetAcelZ /= 200;
  offsetAcelZ -= 16384;

  //MEASURES THE INITIAL ANGLE TO NORTH AND FORCE HIM TO ZERO
  getMagData();
  AngNorthi = atan2(magX,magY)*(180/3.1415);  
}

void getMagData()
{
    //GETS FIELD INTENSITY FROM COMPASS
  int xl, xh, yl,yh;
  int i = 0;
  int numLeituras = 30;
  float mediaMagX = 0;
  float mediaMagY = 0;
  float mediaMagZ = 0;
  for(i=0;i<numLeituras;i++)
  {
    Wire.beginTransmission(0X0C);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(0x0C, 4); 
    xl = Wire.read();
    xh = Wire.read();
    yl = Wire.read();
    yh = Wire.read();

    mediaMagX += (xh<<8|xl) - 30;
    mediaMagY += (yh<<8|yl) - 45;

    Wire.beginTransmission(0x0C);
    Wire.write(0x0A);
    Wire.write(0x01);
    Wire.endTransmission();
  }
  magX = mediaMagX/numLeituras;
  magY = mediaMagY/numLeituras;
  
}

void getGyroData()
{
   //GETS ANGULAR SPEED VALUES
  int i = 0;
  int numLeituras = 30;
  float mediaGyroX = 0;
  float mediaGyroY = 0;
  float mediaGyroZ = 0;
  for(i=0;i<numLeituras;i++)
  {
    Wire.beginTransmission(MPU);      //transmite
    Wire.write(0x43);                 // Endereço registrador do Gyro
    Wire.endTransmission(false);     //Finaliza transmissão
    Wire.requestFrom(MPU,6,true);   //requisita 6 bytes  
    mediaGyroX += Wire.read()<<8|Wire.read();  //GYRO EIXO X  
    mediaGyroY += Wire.read()<<8|Wire.read();  //GYRO EIXO Y 
    mediaGyroZ += Wire.read()<<8|Wire.read();  //GYRO EIXO Z 
  }
  gyroX = mediaGyroX/numLeituras;
  gyroY = mediaGyroY/numLeituras;
  gyroZ = mediaGyroZ/numLeituras;
}

void getAcelData()
{
  //GETS ACCEL VALUES
  int i = 0;
  int numLeituras = 30;
  float mediaAcelX = 0;
  float mediaAcelY = 0;
  float mediaAcelZ = 0;
  for(i=0;i<numLeituras;i++)
  {
    Wire.beginTransmission(MPU);      //transmite
    Wire.write(0x3B);                 // Endereço 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);     //Finaliza transmissão
    Wire.requestFrom(MPU,6,true);   //requisita bytes  
    mediaAcelX += Wire.read()<<8|Wire.read();  //ACELEROMETRO EIXO X   
    mediaAcelY += Wire.read()<<8|Wire.read();  //ACELEROMETRO EIXO Y  
    mediaAcelZ += Wire.read()<<8|Wire.read();  //ACELEROMETRO EIXO Z
     
  } 
  acelX = mediaAcelX/numLeituras;
  acelY = mediaAcelY/numLeituras;
  acelZ = mediaAcelZ/numLeituras;
}

//SENSOR FUSION ALGORITHM
float fusionAlgorithm(float Za, float Zb, float varA, float varB)
{
  if(Za > 0 && Zb < 0)
  {
    return Za;
  }
  else if(Za < 0 && Zb > 0)
  {
    return Za;
  }
  float result;
  result = (varB/(varA + varB))*Za + (varA/(varA + varB))*Zb;
  return result;
}
 
//Loops until socialism works
void loop()
{
  getAcelData();
  getGyroData();
  getMagData();

  //CALCULATE THE ACCEL MAGNITUDE
  timefAcel[0] = micros();
  acelX = acelX - offsetAcelX;
  acelXr = ((((float)acelX)/(16276.2744833)));  //x y z 16276.2744833    16376.972     16243.9889
  timeiAcel[0] = micros();

  timefAcel[1] = micros();
  acelY = acelY - offsetAcelY;
  acelYr = ((((float)acelY)/(16376.972)));
  timeiAcel[1] = micros();

  timefAcel[2] = micros();
  acelZ = acelZ - offsetAcelZ;
  acelZr = ((((float)acelZ)/(16243.9889)));
  timeiAcel[2] = micros();

  //CALCULATE THE ANGULAR SPEED MAGNITUDE
  timefGyro[0] = micros();
  AngGyroZ = AngGyroZ - gyroZr*((timefGyro[0] - timeiGyro[0])/1000000);
  gyroZ = gyroZ - offsetGyroZ;
  gyroZr = ((((float)gyroZ)/(131)));
  timeiGyro[0] = micros();
    if(AngGyroZ > 180 - AngNorthi)
  {
    AngGyroZ = -AngNorthi - 180;
  }
  else if(AngGyroZ < -AngNorthi - 180)
  {
    AngGyroZ = 180 - AngNorthi;
  } 

  timefGyro[1] = micros();
  AngGyroY = AngGyroY - gyroYr*((timefGyro[1] - timeiGyro[1])/1000000);
  gyroY = gyroY - offsetGyroY;
  gyroYr = ((((float)gyroY)/(131)));
  timeiGyro[1] = micros();
  if(AngGyroY < -180)
  {
    AngGyroY = 180;
  }
  else if(AngGyroY > 180)
  {
    AngGyroY = -180;
  }
  

  timefGyro[2] = micros();
  AngGyroX = AngGyroX  + gyroXr*((timefGyro[2] - timeiGyro[2])/1000000);
  gyroX = gyroX - offsetGyroX;
  gyroXr = ((((float)gyroX)/(131)));
  timeiGyro[2] = micros();
  if(AngGyroX < -90)
  {
    AngGyroX = 270;
  }
  else if(AngGyroX > 270)
  {
    AngGyroX = -90;
  }

  //CALCULATE ANGLE X AND Y WITH ACCELEROMETER
  AngAcelX = -atan2(acelZr,acelYr)*(180/3.1415) + 90;
  AngAcelY = atan2(acelXr,acelZr)*(180/3.1415);

  //CALCULATE ANGLE Z WITH MAGNETOMETER
  AngMagZ = atan2(magX,magY)*(180/3.1415) - AngNorthi;

 
  //CALCULATE THE SQUARE OF THE STD DEVIATION FROM MEASURES
  //SENSOR A
  AngAcel[0].add(AngAcelX);
  AngAcel[1].add(AngAcelY);

  //SENSOR B
  AngGyro[0].add(AngGyroX);
  AngGyro[1].add(AngGyroY);
  AngGyro[2].add(AngGyroZ);

  //SENSOR C
  AngMag[0].add(AngMagZ);
  
  //ALGORITMO DE FUSÃO DE SENSORES
  AngXfused = fusionAlgorithm(AngAcelX, AngGyroX, AngAcel[0].variance(), AngGyro[0].variance());
  AngYfused = fusionAlgorithm(AngAcelY, AngGyroY, AngAcel[1].variance(), AngGyro[1].variance());
  AngZfused = fusionAlgorithm(AngMagZ, AngGyroZ, AngMag[0].variance(), AngGyro[2].variance());
 

  Serial.println(" "); 
//    Serial.print("  X:");Serial.print(AngAcelX,1);
//    Serial.print("  Y:");Serial.print(AngAcelY,1);
//  Serial.print("  X:");Serial.print(AngGyroX,1);
//  Serial.print("  Y:");Serial.print(AngGyroY,1);
//  Serial.print("  Z:");Serial.print(AngGyroZ,1);
//Serial.print("  Z:");Serial.print(AngMagZ,1);

    Serial.print("  X:");Serial.print(AngXfused,1);
    Serial.print("  Y:");Serial.print(AngYfused,1);
    Serial.print("  Z:");Serial.print(AngZfused,1);

  delay(1);
}
