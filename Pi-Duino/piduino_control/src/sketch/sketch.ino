#include <EnableInterrupt.h>
#include <Encoder.h>
#include "DualMC33926MotorShield.h"
#include "types.h"

#define L_ENC_A 4 //change to actual used pins. A is front wheel. B is back
#define L_ENC_B 5 //check that pins mathc up with right sensors and encoder init
#define R_ENC_A 6
#define R_ENC_B 7
#define PING_PIN 8

#define ENC_PORT PIND


#define WHEEL_BASE 137 //current approximation in mm, chassis constant
#define WHEEL_CIRCUMFERENCE 219.9115 //circumference = 2*pi*r, r = 35mm, pi = 3.14
#define PPR 8 //pulses per revolution is = to # of lines per revoluion aka number of white segments which is half of the total segments, this is assumig 32 segments

// set all global position variables to 0
DualMC33926MotorShield md;
double theta = 0;
double x = 0;
double y = 0;
double l_s; //left distance changed
double r_s; //right distance changed
double delta_x;
double heading;
double ping_duration;
char dataString[5] = {0};

// M1 is Right, M2 is left
// Interrupt 3,dig 6 for right, interrupt 2,dig 5 for left
Encoder myEnc_L(2,5);
Encoder myEnc_R(3,6);

str_code hashit (String inString) {
   if (inString == "mtr") return motor;
   if (inString == "irr") return irSensor;
   if (inString == "png") return png;
   if (inString == "stp") return stopp;
   if (inString == "upd") return update;
   if (inString == "none") return none;
}

void setup() {
  // put your setup code here, to run once:

  pinMode(L_ENC_A, INPUT);
  digitalWrite(L_ENC_A, HIGH);
  pinMode(L_ENC_B, INPUT);
  digitalWrite(L_ENC_B, HIGH);

  enableInterrupt(L_ENC_A, encoder, RISING);
  enableInterrupt(L_ENC_B, encoder, RISING);

  pinMode(R_ENC_A, INPUT);
  digitalWrite(R_ENC_A, HIGH);
  pinMode(R_ENC_B, INPUT);
  digitalWrite(R_ENC_B, HIGH);

  enableInterrupt(R_ENC_A, encoder, RISING);
  enableInterrupt(R_ENC_B, encoder, RISING);

  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);

  md.init();
  md.setM1Speed(0);
  md.setM2Speed(0);
  Serial.begin(9600);
  //Serial.println("Start");

}

void loop() {
  // put your main code here, to run repeatedly:
//stopIfFault();
String opStr;
static unsigned int arg1 = 0;
static unsigned int arg2 = 0;
static unsigned int arg3 = 0;
  if(Serial.available()){
    String input = Serial.readString();
    // Serial.print("input: "+input);

    opStr = input.substring(0,3);
    // Serial.println(opStr);

    arg1 = input.substring(3,7).toFloat();
    arg2 = input.substring(7,11).toFloat();
    arg3 = input.substring(11,15).toFloat();
  }

  else{
    opStr = "none";
  }

  switch(hashit(opStr)){
    case none :
      break;

    case motor :
      // Serial.println("motor");
      // Serial.println(arg1);
      // Serial.println(arg2);
      md.setM1Speed((int)arg1);
      md.setM2Speed((int)arg2);
      break;

    case png :
      // Serial.print("ping duration: ");
      ping();
      break;

    case stopp :
      Stop();
      break;

    case irSensor :
      encoder();
      break;

    case update :
    theta = arg1;
    x = arg2;
    y = arg3;

    default:
      break;
  }

}


void encoder() {
  long r_enc = myEnc_R.read();
  long l_enc = myEnc_L.read();
  r_s = r_enc*WHEEL_CIRCUMFERENCE/PPR;
  l_s = l_enc*WHEEL_CIRCUMFERENCE/PPR;

  //update the change in avg position and current heading
  delta_x = (l_s + r_s)/2;
  heading = atan2((r_s-l_s)/2, WHEEL_BASE/2);

  //update overall global positioning
  theta += heading;
  x += delta_x*cos(theta);
  y += delta_x*sin(theta);

  String ret = "";
  ret = String(l_s) + " " + String(r_s) + " " + String(heading) + " " + String(theta) + " " + String(x) + " " + String(y);
  Serial.println(ret);
}

void ping() {
  ping_duration = pulseIn(PING_PIN, HIGH);
  Serial.println(ping_duration);
}

void Stop() {
  md.setM1Speed(0);
  md.setM2Speed(0);
}

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
      md.setM1Speed(0);
      md.setM2Speed(0);
    while(1);
  }
}



// int8_t read_encoder(int8_t new_val)
// {
//   //defines array that describes moving states of the QE for CW & CCW motion
//   static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
//   static uint8_t enc_val = 0;

//   enc_val <<= 2; //preserve old value
//   enc_val |= new_val; //add new value to last 2 LSB

//   return (enc_states[(enc_val & 0x0f)]); //get state and remove old values
// }
