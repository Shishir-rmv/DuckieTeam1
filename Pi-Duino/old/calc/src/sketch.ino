int r = 1;
void setup(){
  Serial.begin(9600);  
}

void loop(){
  if(Serial.available()){    //From RPi to Arduino
    String input = Serial.readString();
    
    String opStr = input.substring(0,3);
    int op1 = input.substring(3,4).toInt();
    int op2 = input.substring(4,5).toInt();
    
    int res = 0;
    if(opStr.equals("add"))
    {  
      res = op1 + op2;
      Serial.println(res);
    }
    else if(opStr.equals("mul"))
    {
      res = op1 * op2;
      Serial.println(res);
    }
    else{
      Serial.println("err"); 
    }
  }  
  
  
}

