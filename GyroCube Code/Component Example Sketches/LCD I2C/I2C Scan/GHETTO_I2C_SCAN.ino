// i2c_scanner
 //
 // This program (or code that looks like it)
 // can be found in many places.
 // For example on the Arduino.cc forum.
 // The original author is not know.
 //
 // This sketch tests the standard 7-bit addresses
 // from 0 to 127. Devices with higher bit address
 // might not be seen properly.
 //
 // Adapted to be as simple as possible by Arduino.cc user Krodal

#include <Wire.h>

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    Serial.println("\nI2C Scanner");
}

void loop()
{
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for(address = 0; address <= 127; address++ )
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        }
        else if (error==4)
        {
            Serial.print("Unknow error at address 0x");
            if (address<16)
                Serial.print("0");
            Serial.println(address,HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
    delay(30000);
 }