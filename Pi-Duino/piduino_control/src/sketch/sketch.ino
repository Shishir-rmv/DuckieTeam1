#include <EnableInterrupt.h>
//#include <Encoder.h>
#include "DualMC33926MotorShield.h"
#include "types.h"

#define L_ENC_A 12 //change to actual used pins. A is front wheel. B is back
#define L_ENC_B 13 //check that pins mathc up with right sensors and encoder init
#define R_ENC_A 5
#define R_ENC_B 6
#define PING_PIN 8

#define ENC_PORT PIND
#define ENC_PORT2 PINB

#define WHEEL_BASE 137 //current approximation in mm, chassis constant
#define WHEEL_CIRCUMFERENCE 219.9115 //circumference = 2*pi*r, r = 35mm, pi = 3.14
#define PPR 32 //pulses per revolution is = to # of lines per revoluion aka number of white segments which is half of the total segments, this is assumig 32 segments

#define k 5
#define b 3
// set all global position variables to 0
DualMC33926MotorShield md;
double theta = 0;
double x = 0;
double y = 0;
double l_s; //left distance changed
double r_s; //right distance changed
short l_count;
short r_count;
double motorL = 0;
double motorR = 0;
double error = 0;
double error_dot = 0;
double delta_x;
double heading;
double ping_duration;
char dataString[5] = {0};

// M1 is Right, M2 is left
// Interrupt 3,dig 6 for right, interrupt 2,dig 5 for left
//Encoder myEnc_L(L_ENC_A,L_ENC_B);
//Encoder myEnc_R(R_ENC_A,R_ENC_B);

str_code hashit (String inString) {
   if (inString == "mtr") return motor;
   if (inString == "irr") return irSensor;
   if (inString == "png") return png;
   if (inString == "stp") return stopp;
   if (inString == "upd") return update;
   if (inString == "odo") return odometry;
   if (inString == "viz") return visual;
   if (inString == "cal") return calibrat;
   if (inString == "none") return none;
}

void setup() {
  // put your setup code here, to run once:

  pinMode(L_ENC_A, INPUT);
  digitalWrite(L_ENC_A, LOW);

  enableInterrupt(L_ENC_A, encoder, CHANGE);

  pinMode(L_ENC_B, INPUT);
  //digitalWrite(L_ENC_B, LOW);
  
  enableInterrupt(L_ENC_B, encoder, CHANGE);

  pinMode(R_ENC_A, INPUT);
  //digitalWrite(R_ENC_A, HIGH);
  
  enableInterrupt(R_ENC_A, encoder, CHANGE);

  pinMode(R_ENC_B, INPUT);
  //digitalWrite(R_ENC_B, HIGH);
  
  enableInterrupt(R_ENC_B, encoder, CHANGE);

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
  Serial.begin(2000000);
  // Serial.println("Start");
}

void loop() {
  // put your main code here, to run repeatedly:
stopIfFault();
static int prevError = 0;
static int output;
static String opStr;
static unsigned int arg1 = 0;
static unsigned int arg2 = 0;

  if(Serial.available()){
    String input = Serial.readString();
    // Serial.print("input: "+input);

    opStr = input.substring(0,3);
    // Serial.println(opStr);

    arg1 = input.substring(3,7).toInt();
    arg2 = input.substring(7,11).toInt();
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
      motorL = arg1;
      motorR = arg2;

      md.setM2Speed(motorL);
      md.setM1Speed(motorR);
      break;

    case png :
      // Serial.print("ping duration: ");
      ping();
      break;

    case stopp :
      Stop();
      break;

    case irSensor :
      getEnc();
      break;

    default:
      break;
  }

//  error = r_count - l_count;
//  error_dot = error - prevError;
//  output = -k*error - b*error_dot;
//  motorR += output;
//  motorL -= output;
  
//  md.setM1Speed(motorL);
//  md.setM2Speed(motorR);
//
//  prevError = error;
}


 



void encoder() {
  long l_enc = read_encoderL((ENC_PORT2 & 0b110000) >> 4); 
  long r_enc = read_encoderR((ENC_PORT & 0b1100000) >> 5);
  // Serial.println(ENC_PORT2, BIN);
  // Serial.println(ENC_PORT, BIN);
  // Serial.println((ENC_PORT2 & 0b110000) >> 4, BIN);
  // Serial.println((ENC_PORT & 0b1100000) >> 5, BIN);

  //r_s = r_enc*WHEEL_CIRCUMFERENCE/PPR;
  //l_s = l_enc*WHEEL_CIRCUMFERENCE/PPR;

  l_count += l_enc;
  r_count += r_enc;
  // //update the change in avg position and current heading
  // delta_x = (l_s + r_s)/2;
  // heading = atan2((r_s-l_s)/2, WHEEL_BASE/2);

  //  //update overall global positioning
  // theta += heading;
  // x += delta_x;//*cos(theta);
  // y += delta_x;//*sin(theta);

  // String ret = " ";
  // ret = "l_enc: " + String(l_enc) + " r_enc: " + String(r_enc);
  // Serial.println(ret);
  // ret = "l_count: " + String(l_count) + " r_count: " + String(r_count);
  // Serial.println(ret);
  // ret = "l_S: " + String(l_s) + " r_s: " + String(r_s);
  // Serial.println(ret);
  // ret = "delta x: " + String(delta_x) + " heading: " + String(heading);
  // Serial.println(ret);
  // ret = "x: " + String(x) + " y: " + String(y) + " theta: " + String(theta);
  // Serial.println(ret);
  // Serial.println();
}
void getEnc() {
  Serial.println(String(l_count)+","+String(r_count));
  l_count = 0;
  r_count = 0;
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


int8_t read_encoderL(int8_t new_val)
{
  //defines array that describes moving states of the QE for CW & CCW motion
  static int8_t enc_statesL[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_valL = 0;
  static uint8_t stateL = 0;

  enc_valL <<= 2; //preserve old value
  enc_valL |= new_val; //add new value to last 2 LSB 
  stateL = enc_valL & 0b1111;

  return (enc_statesL[stateL]); //get state and remove old values
}

int8_t read_encoderR(int8_t new_val)
{
  //defines array that describes moving states of the QE for CW & CCW motion
  static int8_t enc_statesR[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_valR = 0;
  static uint8_t stateR = 0;

  enc_valR <<= 2; //preserve old value
  enc_valR |= new_val; //add new value to last 2 LSB 
  stateR = enc_valR & 0b1111;
 // Serial.println(stateR, BIN);

  return (enc_statesR[stateR]); //get state and remove old values
}
