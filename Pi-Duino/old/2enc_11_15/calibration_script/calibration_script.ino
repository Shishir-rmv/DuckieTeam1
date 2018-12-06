#include <EnableInterrupt.h>
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


DualMC33926MotorShield md;
double theta = 0;
double x = 0;
double y = 0;
double l_s; //left distance changed
double r_s; //right distance changed
double delta_x;
double heading;
int l_enc_count;
int r_enc_count;
int old_l_enc_count = 0;
int old_r_enc_count = 0;
double ping_duration;
char dataString[5] = {0};
double duration_L;
double duration_R;
double dist_L=0;
double dist_R=0;
double prev_dist_L=0;
double prev_dist_R=0;
double prevmillis_L = micros();
double prevmillis_R = micros();
double distance_R = 0;
double distance_L=0;
int update_rate = 1;//set from 1 to PPR or maybe more

double error = 0;
double error_dot = 0;
double del_v = 0;

double pwm_L = 200;
double pwm_R = 200;
double prev_error = 0;
double distance = 0;
double C = 1;
static double rpm_L = (pwm_L-96.23)/2.114;
static double rpm_R = (pwm_R-100.9)/2.02;
double rpm_target_L = rpm_L;
double rpm_target_R = rpm_R;

str_code hashit (String inString) {
   if (inString == "mtr") return motor;
   if (inString == "irr") return irSensor;
   if (inString == "png") return png;
   if (inString == "stp") return stopp;
   if (inString == "upd") return update;
   if (inString == "none") return none;
}

void setup() {
  pinMode(L_ENC_A, INPUT);
  digitalWrite(L_ENC_A, HIGH);
  pinMode(L_ENC_B, INPUT);
  digitalWrite(L_ENC_B, HIGH);

  enableInterrupt(L_ENC_A, encoder, CHANGE);
  enableInterrupt(L_ENC_B, encoder, CHANGE);

  pinMode(R_ENC_A, INPUT);
  digitalWrite(R_ENC_A, HIGH);
  pinMode(R_ENC_B, INPUT);
  digitalWrite(R_ENC_B, HIGH);

  enableInterrupt(R_ENC_A, encoder, CHANGE);
  enableInterrupt(R_ENC_B, encoder, CHANGE);

  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);

  md.init();
  //md.setM1Speed(pwm_R);
  //md.setM2Speed(pwm_L);
  Serial.begin(115200);
  // Serial.println("Start");
  delay(000);
}

void loop() {
  // put your main code here, to run repeatedly:
//stopIfFault();
static String opStr;
static unsigned int arg1 = 0;
static unsigned int arg2 = 0;
static char input[15];
static char opStrA[4];
static char arg1A[5];
static char arg2A[5];
  if (Serial.available()) {

    Serial.readBytesUntil('\n', input, 12);
    opStrA[0] = input[0];
    opStrA[1] = input[1];
    opStrA[2] = input[2];
    opStrA[3] = 0;

    arg1A[0] = input[3];
    arg1A[1] = input[4];
    arg1A[2] = input[5];
    arg1A[3] = input[6];
    arg1A[4] = 0;

    arg2A[0] = input[7];
    arg2A[1] = input[8];
    arg2A[2] = input[9];
    arg2A[3] = input[10];
    arg2A[4] = 0;

    String opStr = String(opStrA);
    int arg1 = atoi(arg1A);
    int arg2 = atoi(arg2A);
  }else{
    
   md.setM1Speed(pwm_R);
   md.setM2Speed(pwm_L);
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

    default:
      break;
  }
}


void encoder() {
  int l_enc = read_encoderL((ENC_PORT2 & 0b110000) >> 4);
  //Serial.println(ENC_PORT,BIN); 
  int r_enc = read_encoderR((ENC_PORT & 0b1100000) >> 5);

  r_s = r_enc*WHEEL_CIRCUMFERENCE/PPR;
  l_s = l_enc*WHEEL_CIRCUMFERENCE/PPR;
  l_enc_count += l_enc;
  r_enc_count += r_enc;
//  if ((pwm_R*r_enc)<0 && old_r_enc_count != 0){
//    r_enc_count -= 2*r_enc;
//   r_s -= 2*r_enc*WHEEL_CIRCUMFERENCE/PPR;
//  }else{
//   r_enc_count += r_enc; 
//  }
//  if ((pwm_L*l_enc)<0 && old_l_enc_count != 0){
//    l_enc_count -= 2*l_enc;
//    l_s -= 2*l_enc*WHEEL_CIRCUMFERENCE/PPR;
//  }else{
//  l_enc_count += l_enc;
//  }

  
  // //update the change in avg position and current heading

  distance_L = l_enc_count*WHEEL_CIRCUMFERENCE/PPR; //CHECK THESE
//(********)CHECK THESE THEY ARE TOTAL DISPLACEMENTS //(*******)
//(********) BEING USED IN ERROR CALCULATIONS //(*******)
  distance_R = r_enc_count*WHEEL_CIRCUMFERENCE/PPR;
  
  if (l_enc_count!= old_l_enc_count){
    old_l_enc_count = l_enc_count;
    if (l_enc_count%update_rate==0){
      duration_L = (micros() - prevmillis_L); // Time difference between revolution in microsecond
      rpm_L = update_rate*(60000000/duration_L)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      prevmillis_L = micros();
      Serial.print("L "); 
      Serial.print(l_enc_count);
      Serial.print("   ");  
      Serial.print(rpm_L);
      Serial.print("   ");
      }
  }
  if (r_enc_count!= old_r_enc_count){
    old_r_enc_count = r_enc_count;
    if(r_enc_count%update_rate==0){
      duration_R = (micros() - prevmillis_R); // Time difference between revolution in microsecond
      rpm_R = update_rate*(60000000/duration_R)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      old_r_enc_count = r_enc_count;
      prevmillis_R = micros();
      Serial  .print("  R ");
      Serial.print(r_enc_count);
      Serial.print("   ");  
      Serial.print(rpm_R);
      Serial.print("   ");;
    }
  }

}

void getEnc() {
  Serial.write(lowByte(l_enc_count));
  Serial.write(lowByte(r_enc_count));
  Serial.flush();
  l_enc_count = 0;
  r_enc_count = 0;
}

void ping() {
  ping_duration = pulseIn(PING_PIN, HIGH);
  Serial.println(ping_duration);      //THIS MUST BE FIXED LATER
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

  enc_valL <<= 2; //preserve old value
  enc_valL |= new_val; //add new value to last 2 LSB 
  return (enc_statesL[enc_valL & 0b1111]); //get state and remove old values
}

int8_t read_encoderR(int8_t new_val)
{
  //defines array that describes moving states of the QE for CW & CCW motion
  static int8_t enc_statesR[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_valR = 0;

  enc_valR <<= 2; //preserve old value
  enc_valR |= new_val; //add new value to last 2 LSB 
  return (enc_statesR[enc_valR & 0b1111]); //get state and remove old values
}
