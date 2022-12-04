#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <QMC5883LCompass.h>
#include <QList.h>

//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
QMC5883LCompass compass;

QList<float> Ylist;
QList<float> Zlist;

String temp;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
float z,sumZ,sumY,cpZ;
//double previous;
int Size=5;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
//double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

struct Position{
  float X;
  float Y;
  float Z;
  } PrevPos, DPos;

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

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
//  compAngleX = roll;
//  compAngleY = pitch;

  timer = micros();
  
    compass.init();
  // Continuos Mode, output rate 200hz, range 8G, oversample ratio 512
  compass.setCalibration(-1443, 817, -458, 1692, 0, 2687); //calibration from example
  compass.setSmoothing(10, true);
}

void loop() {
    // Read compass values
  compass.read();

  // Return Azimuth reading
  z = atan2( compass.getY(), compass.getX() ) * 180.0 / PI;
  z = z<0?360+z:z;
  
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  gyroZ=gyroZ/113;
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = (gyroX+300) / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
//    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
//    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt)-2; // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt)-1; // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

//  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
//  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print(", ");
  Serial.print(accY); Serial.print(", ");
  Serial.print(accZ); 

  Serial.print((gyroX+300)/113); Serial.print(", ");
  Serial.print(gyroY/113); Serial.print(", ");
  Serial.print(gyroZ/113); 

  
#endif

//  Serial.print(roll); Serial.print("\t");
//  Serial.print(gyroXangle); Serial.print("\t");
//  Serial.print(compAngleX); Serial.print("\t");
//  Serial.print(kalAngleX); Serial.print(",");
//
//  Serial.print("\t");
//
//  Serial.print(pitch); Serial.print("\t");
//  Serial.print(gyroYangle); Serial.print("\t");
//  Serial.print(compAngleY); Serial.print("\t");
//  Serial.print(kalAngleY); Serial.print("\t");
//  Serial.print("A: ");
//  Serial.print(z);

#if 0 // Set to 1 to print the temperature
//  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif

//  Serial.print("\r\n");

if (PrevPos.X==0 and PrevPos.Y==0 and PrevPos.Z==0){
  PrevPos.X=kalAngleX;
  PrevPos.Y=kalAngleY;
  PrevPos.Z=z;
  }
else{
  DPos.X=kalAngleX-PrevPos.X;
  DPos.Y=kalAngleY-PrevPos.Y;
  cpZ=z-PrevPos.Z;
//  DPos.Z=abs(gyroZ)>20?((z-PrevPos.Z)*0.75+gyroZ*0.25)/10:((z-PrevPos.Z)*0.5+gyroZ*0.3)/8;
  if(abs(gyroZ)<20){
    DPos.Z=gyroZ*0.25/10;
    }
  else{
//    if(abs(DPos.Y)<0.35){
//      DPos.Z=(cpZ*0.75+gyroZ*0.25)/10;
//      }
//    else{
//      DPos.Z=(cpZ*0.55+gyroZ*0.45)/10;
//      }
     DPos.Z=(cpZ*0.75+gyroZ*0.25)/10;
    }
  PrevPos.X=kalAngleX;
  PrevPos.Y=kalAngleY;
  PrevPos.Z=z;
  if (abs(DPos.Z)>300){
    DPos.Z=DPos.Z<0?360+DPos.Z:360-DPos.Z;
    }
  if(abs(DPos.Y)>300){
    DPos.Y=DPos.Y<0?360+DPos.Y:360-DPos.Y;
    }
//  Serial.print(DPos.Y);Serial.print(",");Serial.println(DPos.Z);
  }
  
if(Ylist.size()<Size){
  Ylist.push_back(DPos.Y);
  Zlist.push_back(DPos.Z);
  sumY+=DPos.Y;
  sumZ+=DPos.Z;
  }
  
else{
//  Serial.print((sumY-Ylist.front()+DPos.Y)/6);Serial.print(",");
//  Serial.println((sumZ-Zlist.front()+DPos.Z)/6);
temp=String(DPos.X)+","+String((sumY-Ylist.front()+DPos.Y)/Size)+","+String((sumZ-Zlist.front()+DPos.Z)/Size)+"\n";
char* output=temp.c_str();
Serial.write(output);
  sumY=sumY-Ylist.front()+DPos.Y;
  sumZ=sumZ-Zlist.front()+DPos.Z;
  Ylist.pop_front();
  Ylist.push_back(DPos.Y);
  Zlist.pop_front();
  Zlist.push_back(DPos.Z);
  }
    
//  Serial.print(DPos.X);Serial.print(",");
//  Serial.print(DPos.Y);Serial.print(",");
//  Serial.println(DPos.Z);

//  if (millis()-previous>1000){
//    Serial.println(count);
//    previous=millis();
//    count=0;
//    }
//  else{
//    count++;
//    }
  delay(10);
}
