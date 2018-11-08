#include <Arduino.h>

void setup();
void loop();
#line 1 "src/sketch.ino"
char dataString[5] = {0};
int a = 0;

void setup(){
  Serial.begin(9600);
  Serial.available();
}

void loop(){
  if(Serial.available() > 0){
    Serial.readBytes(dataString, 5);
    char command[] = {dataString[0], dataString[1], dataString[2]};
    int num1 = dataString[3] - '0';
    int num2 = dataString[4] - '0';
    Serial.println("ack");
    String str(command);
    
    if(str == "add"){
      Serial.println(num1 + num2);
    }
    Serial.flush();
    } 
  //a++; //
  //sprintf(dataString, "%02X", a);
  //Serial.println(dataString);
 
  Serial.println("acky");
  delay(1000); 
}
