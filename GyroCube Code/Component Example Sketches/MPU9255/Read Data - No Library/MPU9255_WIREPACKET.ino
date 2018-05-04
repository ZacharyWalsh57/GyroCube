#include <Wire.h> 

//Variables for the raw values
int16_t ACC_RAW_X, ACC_RAW_Y, ACC_RAW_Z ,GYRO_RAW_X, GYRO_RAW_Y, GYRO_RAW_Z;
float ACCEL_ANGLE[2]; //Acceleration
float GYRO_ANGLE[2]; //Gyro Angle
float TOTAL_ANGLE[2]; //Angle summations
float TO_DEGREE = 180/3.141592654;

//Timer:
float TIME, ELAPSED_TIME, LAST_TIME;

void setup() 
{
    Wire.begin();
    Wire.beginTransmission(0x73); //Open Comms
    Wire.write(0x6B); //Wake up the MPU
    Wire.write(0); //Clean data
    Wire.endTransmission();
    Serial.begin(38400);
    Serial.println(F("Connected 9255"));
    delay(1500);
    Serial.println(F("GRABBING DATA"));
} 

void loop() {
    //Timer:
    LAST_TIME = TIME;  // the previous time is stored before the actual time read
    TIME = millis();  // actual time read
    ELAPSED_TIME = (TIME - LAST_TIME) / 1000; 

    //Get Accel Data
    Wire.beginTransmission(0x73); //Open up again
    Wire.write(0x3B); //Lookup the X ACCEL register bits
    Wire.endTransmission(false); //Keep comms open
    Wire.requestFrom(0x73,6,true); //Request the MPU to send 6 ACCEL registers
    ACC_RAW_X = Wire.read()<<8|Wire.read();
    ACC_RAW_Y = Wire.read()<<8|Wire.read();  //Request and read the registers here and save
    ACC_RAW_Z = Wire.read()<<8|Wire.read();
    
    //Saving the accel angle/making it a real thing.
    ACCEL_ANGLE[0] = atan((ACC_RAW_Y/16384.0)/sqrt(pow((ACC_RAW_X/16384.0),2) + pow((ACC_RAW_Z/16384.0),2)))*TO_DEGREE;
    ACCEL_ANGLE[1] = atan(-1*(ACC_RAW_X/16384.0)/sqrt(pow((ACC_RAW_Y/16384.0),2) + pow((ACC_RAW_Z/16384.0),2)))*TO_DEGREE;

    //GYRO Data begins here
    Wire.beginTransmission(0x73);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x73,4,true);
    GYRO_RAW_X=Wire.read()<<8|Wire.read(); //Shift + Sum
    GYRO_RAW_Y=Wire.read()<<8|Wire.read();

    //Save them as actual numbers
    GYRO_ANGLE[0] = GYRO_RAW_X/131.0;
    GYRO_ANGLE[1] = GYRO_RAW_Y/131.0;

    //Make the total angles here factoring in Acceleration:
    TOTAL_ANGLE[0] = 0.98 *(TOTAL_ANGLE[0] + GYRO_ANGLE[0]*ELAPSED_TIME) + 0.02*ACCEL_ANGLE[0];
    TOTAL_ANGLE[1] = 0.98 *(TOTAL_ANGLE[1] + GYRO_ANGLE[1]*ELAPSED_TIME) + 0.02*ACCEL_ANGLE[1];

    Serial.print("ANGLE VALUE:  ");
    Serial.println(TOTAL_ANGLE[1]);


}