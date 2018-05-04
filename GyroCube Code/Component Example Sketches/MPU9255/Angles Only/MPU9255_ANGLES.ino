//Less clunky version of the example code that only takes angles.
//Will also be making another seperate program which only
#include "quaternionFilters.h"
#include "MPU9250.h"

#define INT_PIN 2

MPU9250 MPU_GYRO;

void setup() {
    Serial.begin(38400);
    Wire.begin();
    pinMode(INT_PIN, INPUT);
    digitalWrite(INT_PIN, LOW);
    MPU_GYRO.calibrateMPU9250(MPU_GYRO.gyroBias, MPU_GYRO.accelBias);


    MPU_GYRO.initMPU9250();
    Serial.println("MPU9250 initialized for active data mode....");

    //--------------------------------------------------------------------------------------

    MPU_GYRO.initAK8963(MPU_GYRO.factoryMagCalibration);
    Serial.println("AK8963 initialized for active data mode....");

    MPU_GYRO.getAres();
    MPU_GYRO.getGres();
    MPU_GYRO.getMres();
}

void loop() {

    if (MPU_GYRO.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        
        MPU_GYRO.readAccelData(MPU_GYRO.accelCount);  // Read the x/y/z adc values
        MPU_GYRO.ax = (float)MPU_GYRO.accelCount[0] * MPU_GYRO.aRes;
        MPU_GYRO.ay = (float)MPU_GYRO.accelCount[1] * MPU_GYRO.aRes;
        MPU_GYRO.az = (float)MPU_GYRO.accelCount[2] * MPU_GYRO.aRes;
        
        MPU_GYRO.readGyroData(MPU_GYRO.gyroCount);  // Read the x/y/z adc values
        MPU_GYRO.gx = (float)MPU_GYRO.gyroCount[0] * MPU_GYRO.gRes;
        MPU_GYRO.gy = (float)MPU_GYRO.gyroCount[1] * MPU_GYRO.gRes;
        MPU_GYRO.gz = (float)MPU_GYRO.gyroCount[2] * MPU_GYRO.gRes;
        
        MPU_GYRO.readMagData(MPU_GYRO.magCount);  // Read the x/y/z adc values
        MPU_GYRO.mx = (float)MPU_GYRO.magCount[0] * MPU_GYRO.mRes
               * MPU_GYRO.factoryMagCalibration[0] - MPU_GYRO.magBias[0];
        MPU_GYRO.my = (float)MPU_GYRO.magCount[1] * MPU_GYRO.mRes
               * MPU_GYRO.factoryMagCalibration[1] - MPU_GYRO.magBias[1];
        MPU_GYRO.mz = (float)MPU_GYRO.magCount[2] * MPU_GYRO.mRes
               * MPU_GYRO.factoryMagCalibration[2] - MPU_GYRO.magBias[2];
               
        MPU_GYRO.updateTime();
        
        MahonyQuaternionUpdate(MPU_GYRO.ax, MPU_GYRO.ay, MPU_GYRO.az, MPU_GYRO.gx * DEG_TO_RAD,
                         MPU_GYRO.gy * DEG_TO_RAD, MPU_GYRO.gz * DEG_TO_RAD, MPU_GYRO.my,
                         MPU_GYRO.mx, MPU_GYRO.mz, MPU_GYRO.deltat);
        
        MPU_GYRO.delt_t = millis() - MPU_GYRO.count;

        MPU_GYRO.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
        MPU_GYRO.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
        MPU_GYRO.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
        MPU_GYRO.pitch *= RAD_TO_DEG;
        MPU_GYRO.yaw   *= RAD_TO_DEG;
        MPU_GYRO.yaw  -= 8.5;
        MPU_GYRO.roll *= RAD_TO_DEG;

        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(MPU_GYRO.yaw, 2);
        Serial.print(", ");
        Serial.print(MPU_GYRO.pitch, 2);
        Serial.print(", ");
        Serial.println(MPU_GYRO.roll, 2);
    }
}