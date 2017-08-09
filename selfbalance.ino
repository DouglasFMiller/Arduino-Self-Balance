class PID{
public:
  
  double error;
  double sample;
  double lastSample;
  double kP, kI, kD;      
  double P, I, D;
  double pid;
  
  double setPoint;
  long lastProcess;
  
  PID(double _kP, double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }

 void addP(double _kP){

  kP = kP+_kP;
  
  }

  void addI(double _kI){

  kI = kI+_kI;
  
  }

  void addD(double _kD){

  kD = kD+_kD;
  
  }

  
  
  void addNewSample(double _sample){
    sample = _sample;
  }
  
  void setSetPoint(double _setPoint){
    setPoint = _setPoint;
  }

  double getsetpoint(){
    return setPoint;
    }
  
  double process(double _sample){
  
        sample = _sample;
    
    // Implementação PID
    error = setPoint - sample;
    float deltaTime = (millis() - lastProcess) / 1000.0;
    lastProcess = millis();
    
    //P
    P = error * kP;
    
    //I
    I = I + (error * kI) * deltaTime;
    
    //D
    D = (lastSample - sample) * kD / deltaTime;
    lastSample = sample;
    
    // Soma tudo
    pid = P + I + D;
    
    return pid;
  }
};

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include "SoftwareSerial.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* ---------------------- IMU Data Start ---------------------- */

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

/* ---------------------- IMU Data Ends ---------------------- */


int vel;
PID meupid(25.5,77,0.58);

  int INA =11;           //Ligado ao INA do modulo ponte H
  int INB = 10;           //Ligado ao INB do modulo ponte H
  int velocidadeA = 6;
  int INC = 9;           //Ligado ao INA do modulo ponte H
  int IND = 8;           //Ligado ao INB do modulo ponte H
  int velocidadeB = 5;
  
    double valorA;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

// Rutine for repeat part of the code every X miliseconds
#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))

/*--------------------------------------INICIO JOYSTICK BLUETOOTH--------------------------------------*/



#define    STX          0x02
#define    ETX          0x03
#define    ledPin       13
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)

SoftwareSerial mySerial(2,3);                           // BlueTooth module: pin#2=TX pin#3=RX
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
String displayStatus = "xxxx";                          // message to Android device

#define VERSION     "\n\nAndroTest V2.0 - @kas2014\ndemo for V5.x App"
/*--------------------------------------FIM JOYSTICK BLUETOOTH--------------------------------------*/

double directionA = 0;
double directionB = 0;
void setup() {
/*--------------------------------------INICIO JOYSTICK BLUETOOTH--------------------------------------*/
 mySerial.begin(9600);                                // 57600 = max value for softserial
 while(mySerial.available())  mySerial.read();         // empty RX buffer
 /*--------------------------------------FIM JOYSTICK BLUETOOTH--------------------------------------*/
  
  pinMode(INA,OUTPUT);           //Define a porta 2 como saída
  pinMode(INB,OUTPUT);           //Define a porta 4 como saída
  pinMode(velocidadeA,OUTPUT);
  pinMode(INC,OUTPUT);           //Define a porta 2 como saída
  pinMode(IND,OUTPUT);           //Define a porta 4 como saída
  pinMode(velocidadeB,OUTPUT);
  
  meupid.setSetPoint(0);
  
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157 // if arduino version > 1.57
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else // if arduino version < 1.57
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

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
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {

  runEvery(10){
  
  controlaMotor(-meupid.process(calcularAngulo()),directionA,directionB);
    
  }

  /*---------------------------------------------------------------------------------------------------------------------------------------*/

 if(mySerial.available())  {                           // data received from smartphone
   delay(2);
   cmd[0] =  mySerial.read();  
   if(cmd[0] == STX)  {
     int i=1;      
     while(mySerial.available())  {
       delay(1);
       cmd[i] = mySerial.read();
       if(cmd[i]>127 || i>7)                 break;     // Communication error
       if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
       i++;
     }
     if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
     else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
   }
 }
 sendBlueToothData();



  /*---------------------------------------------------------------------------------------------------------------------------------------*/
}




/*======================================================================================================INICIO METODOS BLUETOOTH==================================================================================*/


void sendBlueToothData()  {
 static long previousMillis = 0;                            
 long currentMillis = millis();
 if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
   previousMillis = currentMillis;

// Data frame transmitted back from Arduino to Android device:
// < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
// < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

   mySerial.print((char)STX);                                             // Start of Transmission
   mySerial.print(getButtonStatusString());  mySerial.print((char)0x1);   // buttons status feedback
   mySerial.print(GetdataInt1());            mySerial.print((char)0x4);   // datafield #1
   mySerial.print(GetdataFloat2());          mySerial.print((char)0x5);   // datafield #2
   mySerial.print(displayStatus);                                         // datafield #3
   mySerial.print((char)ETX);                                             // End of Transmission
 }  
}

String getButtonStatusString()  {
 String bStatus = "";
 for(int i=0; i<6; i++)  {
   if(buttonStatus & (B100000 >>i))      bStatus += "1";
   else                                  bStatus += "0";
 }
 return bStatus;
}

int GetdataInt1()  {              // Data dummy values sent to Android device for demo purpose
 static int i= -30;              // Replace with your own code
 i ++;
 if(i >0)    i = -30;
 return i;  
}

float GetdataFloat2()  {           // Data dummy values sent to Android device for demo purpose
 static float i=50;               // Replace with your own code
 i-=.5;
 if(i <-50)    i = 50;
 return i;  
}

void getJoystickState(byte data[8])    {
 int joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
 int joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
 joyX = joyX - 200;                                                  // Offset to avoid
 joyY = joyY - 200;                                                  // transmitting negative numbers

 if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return;      // commmunication error
 
// Your code here ...
    meupid.setSetPoint((joyY*0.015));

    if (joyX<0) directionA = joyX*0.2;
    if (joyX>0) directionB = joyX*0.2;
    if (joyX==0){ 
      directionA = 0;
      directionB = 0;
      }
    
    
   /*Serial.print("Joystick position:  ");
   Serial.print(joyX);  
   Serial.print(", ");  
   Serial.println(joyY);
   */
}

void getButtonState(int bStatus)  {
 switch (bStatus) {
// -----------------  BUTTON #1  -----------------------
   case 'A':
     buttonStatus |= B000001;        // ON
     Serial.println("\n** Button_1: ON **");
     // your code...      
     displayStatus = "LED <ON>";
     Serial.println(displayStatus);
     digitalWrite(ledPin, HIGH);
     break;
   case 'B':
     buttonStatus &= B111110;        // OFF
     Serial.println("\n** Button_1: OFF **");
     // your code...      
     displayStatus = "LED <OFF>";
     Serial.println(displayStatus);
     digitalWrite(ledPin, LOW);
     break;

// -----------------  BUTTON #2  -----------------------
   case 'C':
     buttonStatus |= B000010;        // ON
     Serial.println("\n** Button_2: ON **");
     // your code...      
     displayStatus = "Button2 <ON>";
     Serial.println(displayStatus);
     break;
   case 'D':
     buttonStatus &= B111101;        // OFF
     Serial.println("\n** Button_2: OFF **");
     // your code...      
     displayStatus = "Button2 <OFF>";
     Serial.println(displayStatus);
     break;

// -----------------  BUTTON #3  -----------------------
   case 'E':
     buttonStatus |= B000100;        // ON
     Serial.println("\n** Button_3: ON **");
     // your code...      
     displayStatus = "Motor #1 enabled"; // Demo text message
     Serial.println(displayStatus);
     break;
   case 'F':
     buttonStatus &= B111011;      // OFF
     Serial.println("\n** Button_3: OFF **");
     // your code...      
     displayStatus = "Motor #1 stopped";
     Serial.println(displayStatus);
     break;

// -----------------  BUTTON #4  -----------------------
   case 'G':
     buttonStatus |= B001000;       // ON
     Serial.println("\n** Button_4: ON **");
     // your code...      
     displayStatus = "Datafield update <FAST>";
     Serial.println(displayStatus);
     sendInterval = FAST;
     break;
   case 'H':
     buttonStatus &= B110111;    // OFF
     Serial.println("\n** Button_4: OFF **");
     // your code...      
     displayStatus = "Datafield update <SLOW>";
     Serial.println(displayStatus);
     sendInterval = SLOW;
    break;

// -----------------  BUTTON #5  -----------------------
   case 'I':           // configured as momentary button
//      buttonStatus |= B010000;        // ON
     Serial.println("\n** Button_5: ++ pushed ++ **");
     // your code...      
     displayStatus = "Button5: <pushed>";
     break;
//   case 'J':
//     buttonStatus &= B101111;        // OFF
//     // your code...      
//     break;

// -----------------  BUTTON #6  -----------------------
   case 'K':
     buttonStatus |= B100000;        // ON
     Serial.println("\n** Button_6: ON **");
     // your code...      
      displayStatus = "Button6 <ON>"; // Demo text message
    break;
   case 'L':
     buttonStatus &= B011111;        // OFF
     Serial.println("\n** Button_6: OFF **");
     // your code...      
     displayStatus = "Button6 <OFF>";
     break;
 }
// ---------------------------------------------------------------
}

/*========================================================================================================FIM METODOS BLUETOOTH===================================================================================*/








void controlaMotor(double _entradaA, double dirA, double dirB){
 

  
  if(_entradaA<0){
  valorA = _entradaA * -1;

  if(valorA > 255){valorA = 255;}
  
      
    digitalWrite(INA,LOW);      
    digitalWrite(INB,HIGH);
    digitalWrite(INC,LOW);      
    digitalWrite(IND,HIGH);
    analogWrite(velocidadeA,valorA+dirA);
    analogWrite(velocidadeB,valorA+dirB);
  
  }
  else{
    
    
    valorA = _entradaA;

  
  if(valorA > 255){valorA = 255;}
    
    
    digitalWrite(INA,HIGH);      
    digitalWrite(INB,LOW);
    digitalWrite(INC,HIGH);      
    digitalWrite(IND,LOW);
    analogWrite(velocidadeA,valorA+dirA);
    analogWrite(velocidadeB,valorA+dirB);

    
  
  
  }
  
    
 
  
  }

double calcularAngulo(){

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

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
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
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Mostrar dados na serial */
 /* Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

  Serial.print("\r\n");
  delay(2);
  */

  return kalAngleX;
}



