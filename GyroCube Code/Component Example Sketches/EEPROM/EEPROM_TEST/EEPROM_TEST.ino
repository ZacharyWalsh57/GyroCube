#include <EEPROM.h>
const int START_ADDRESS = 800;
long  sum;                    //the sum of read values

void setup(){
      Serial.begin(9600);
      sum = 0;
      byte  values[10] = {1, 3, 5, 12, 43, 23, 5, 124, 99, 55};  
      for (int i=0; i<10; i++){
            EEPROM.write(i+START_ADDRESS, values[i]);
      }
}

void loop(){
      delay(3000);
      for (int i=0; i<10; i++){
            byte readByte = EEPROM.read(i+START_ADDRESS);
            sum += readByte;
      }
      Serial.println(sum);
      
}