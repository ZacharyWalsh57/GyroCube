#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

int LED_LEFT_RED = 11; //LEFT RED
int LED_LEFT_GRN = 10; //LEFT GREEN
int LED_RIGHT_RED = 9; //RIGHT RED
int LED_RIGHT_GRN = 6; //RIGHT GREEN

//Make MPU Object and the needed variables
MPU6050 MPU_GYRO;
bool DMP_READY = false;  //Set to TRUE if the init went off right
uint8_t MPU_INT_STATUS;  //Holds our acrual interupt signal and status byte
uint8_t READY;           //Return status value for each packet/operation complete
uint8_t PACKET_SIZE;     //The EXPECTED DMP packet size (42 bytes according to datasheet)
uint8_t FIFO_COUNT;      //The old FIFO counter for bytes within the FIFO buffer itself
uint8_t FIFO_BUFFER[64]; //The actual FIFO Buffer which we use to move data from I2C to wire

Quaternion QUAT;         // [W , X, Y, Z] - Quaternion list/container
VectorFloat GRAVITY;     // [X, Y, Z] - Gravity force on each reading
float EULER_LIST[3];     // [PSI, THETA, PHI] - Euler Angle list/container
float YAW_PITCH_ROLL[3]; // [YAW, PITCH, ROLL] - YPR list/container
float ROLL_CORRECTION;
float ROLL_CORRECTED;
bool GET_CORRECTION = true;

volatile bool MPU_INTERUPT = false;
void DMP_DATA_READY()
{
    MPU_INTERUPT = true;
}

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("CONNECTING TO MPU");
    MPU_GYRO.initialize();
    delay(2000);

    Serial.println(MPU_GYRO.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    pinMode(LED_LEFT_RED, OUTPUT);  //LEFT RED PIN OUT
    pinMode(LED_LEFT_GRN, OUTPUT);  //LEFT GREEN PIN OUT
    pinMode(LED_RIGHT_RED, OUTPUT); //RIGHT RED PIN OUT
    pinMode(LED_RIGHT_GRN, OUTPUT); //RIGHT GREEN PIN OUT

    Serial.println("INIT DMP NOW");
    READY = MPU_GYRO.dmpInitialize();
    MPU_GYRO.setXGyroOffset(114);
    MPU_GYRO.setYGyroOffset(-101);
    MPU_GYRO.setZGyroOffset(-3);
    MPU_GYRO.setZAccelOffset(1928);

    if (READY == 0)
    {
        Serial.println("DMP READY!");
        analogWrite(LED_LEFT_RED, 255);
        analogWrite(LED_LEFT_GRN, 0);
        analogWrite(LED_RIGHT_RED, 255);
        analogWrite(LED_RIGHT_GRN, 0);

        MPU_GYRO.setDMPEnabled(true);
        attachInterrupt(0, DMP_DATA_READY, RISING);
        MPU_INT_STATUS = MPU_GYRO.getIntStatus();
        DMP_READY = true;
        PACKET_SIZE = MPU_GYRO.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.println("DMP NOT WORKING.  KILLING");
        return;
    }
}

void loop()
{
    if (!DMP_READY)
        return;

    MPU_INTERUPT = false;
    MPU_INT_STATUS = MPU_GYRO.getIntStatus();
    FIFO_COUNT = MPU_GYRO.getFIFOCount();

    if ((MPU_INT_STATUS & 0x10) || FIFO_COUNT == 1024)
    {
        MPU_GYRO.resetFIFO();
        Serial.println("FIFO FUCKED GOOD. Resetting");
    }
    else if (MPU_INT_STATUS & 0x02)
    {
        while (FIFO_COUNT < PACKET_SIZE)
            FIFO_COUNT = MPU_GYRO.getFIFOCount();
        MPU_GYRO.getFIFOBytes(FIFO_BUFFER, PACKET_SIZE);
        FIFO_COUNT -= PACKET_SIZE;

        MPU_GYRO.dmpGetQuaternion(&QUAT, FIFO_BUFFER);
        MPU_GYRO.dmpGetGravity(&GRAVITY, &QUAT);
        MPU_GYRO.dmpGetYawPitchRoll(YAW_PITCH_ROLL, &QUAT, &GRAVITY);

        float ROLL_RAW = (YAW_PITCH_ROLL[2] * 180 / M_PI);

        //Since we have some kinda weird roll error, just set the correction
        //to whatever the angle is when the cube is "flat"
        if ((ROLL_RAW > 1) && (ROLL_RAW < 2.2) && GET_CORRECTION)
        {
            ROLL_CORRECTION = abs(ROLL_RAW);
            GET_CORRECTION = false;
        }
        else
        {
            if (ROLL_RAW > 0)
            {
                ROLL_CORRECTED = ROLL_RAW - ROLL_CORRECTION;
                Serial.print("ROLL IS:\t");
                Serial.print(ROLL_CORRECTED);
                Serial.print("\tRAW ROLL IS:\t");
                Serial.print(ROLL_RAW);
                Serial.print("\tCORRECTION IS:\t");
                Serial.println(ROLL_CORRECTION);
                GET_CORRECTION = true;
            }
            if (ROLL_RAW < 0)
            {
                ROLL_CORRECTED = ROLL_RAW + ROLL_CORRECTION;
                Serial.print("ROLL IS:\t");
                Serial.print(ROLL_CORRECTED);
                Serial.print("\tRAW ROLL IS:\t");
                Serial.print(ROLL_RAW);
                Serial.print("\tCORRECTION IS:\t");
                Serial.println(ROLL_CORRECTION);
                GET_CORRECTION = true;
            }
        }
    }
}
