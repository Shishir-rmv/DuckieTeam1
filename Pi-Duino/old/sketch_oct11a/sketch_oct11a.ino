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
    Serial.println(strlen(dataString));
    int num1 = dataString[3] - '0';
    int num2 = dataString[4] - '0';
    Serial.println(num1);
    Serial.println("ack");
    String str(command);
    Serial.println(strlen(command));
    Serial.println(str);
    if(str == "add"){
      Serial.println(num1 + num2);
    }
    Serial.flush();
    } 
  //a++; //
  //sprintf(dataString, "%02X", a);
  //Serial.println(dataString);
 
  delay(1000); 
}
