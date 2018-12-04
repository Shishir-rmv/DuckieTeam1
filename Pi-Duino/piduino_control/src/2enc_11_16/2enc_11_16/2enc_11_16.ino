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
double prev_dist_L=0;
double prev_dist_R=0;
double prevmillis_L = micros();
double prevmillis_R = micros();
double distance_R = 0;
double distance_L=0;
int update_rate = 1;//set from 1 to PPR or maybe more
double distance_total=0;
double distance_total_R = 0;
double distance_total_L=0;
int third=0;

double l_enc_count_total = 0;
double r_enc_count_total = 0;
int turn = 0;

double error = 0;
double error_dot = 0;
double del_v = 0;

double pwm_L = 180;
double pwm_R = 180;
double prev_error = 0;
double distance = 0;
double C = 1;
static double rpm_L = ((pwm_L/1.3)-96.23)/2.114;
static double rpm_R = (pwm_R-100.9)/2.02;
double rpm_target_L = rpm_L;
double rpm_target_R = rpm_R;
int second = 0;

str_code hashit (String inString) {
   if (inString == "mtr") return motor;
   if (inString == "irr") return irSensor;
   if (inString == "png") return png;
   if (inString == "srt") return start;
   if (instring == "vrf") return vOffset;
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
//Serial.println(distance_total);
  if(Serial.available()){
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
//  }else if(distance_total>1100 && distance_total<1345){//&& micros()>1000000
//    //Serial.println(micros());
//    //
//     l_enc_count=0;
//     r_enc_count=0;
    
  }else if(distance_total>1835){//1870 if(turn != 1)
    //Serial.println(distance);
    Stop();
//  }else if(distance_total>1345 && second != 1){
//    l_enc_count=0;
//    r_enc_count=0;
//    prev_error = 0;
//    second = 1;
//    Serial.println(distance);
    
  }
//  else if(distance>100){
//    //Serial.println(micros());
//    //Stop();
//  //  C = 0.7;
//    distance_L = 0;
//    distance_R = 0;
//
//    
//}
else{
   md.setM2Speed(pwm_L);    
   md.setM1Speed(pwm_R);

   opStr = "none";
   }

  switch(hashit(opStr)){
    case none :
      break;

    case motor :
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
  l_enc_count_total += l_enc;
  r_enc_count_total += r_enc;
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

  distance_L = l_enc_count*WHEEL_CIRCUMFERENCE/PPR;
  distance_R = r_enc_count*WHEEL_CIRCUMFERENCE/PPR;
  
  if (l_enc_count!= old_l_enc_count){
    old_l_enc_count = l_enc_count;
    if (l_enc_count%update_rate==0){
      duration_L = (micros() - prevmillis_L); // Time difference between revolution in microsecond
      rpm_L = update_rate*(60000000/duration_L)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      prevmillis_L = micros();
//      Serial.print("L "); 
//      Serial.print(l_enc_count);
//      Serial.print("   ");  
//      Serial.print(rpm_L);
//      Serial.print("   ");
      }
  }
  if (r_enc_count!= old_r_enc_count){
    old_r_enc_count = r_enc_count;
    if(r_enc_count%update_rate==0){
      duration_R = (micros() - prevmillis_R); // Time difference between revolution in microsecond
      rpm_R = update_rate*(60000000/duration_R)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      old_r_enc_count = r_enc_count;
      prevmillis_R = micros();
//      Serial  .print("  R ");
//      Serial.print(r_enc_count);
//      Serial.print("   ");  
//      Serial.print(rpm_R);
//      Serial.print("   ");
    }
  }

//  Serial.print(dist_R-prev_dist_R);
//  Serial.print(" ");
//  if ((dist_L-prev_dist_L)>30 ||(dist_R-prev_dist_R)>30){
//    error = (dist_L-prev_dist_L) - (dist_R-prev_dist_R);
//    error_dot = error - prev_error;
//    del_v = -(0.1*error) - (0.01*error_dot);
//    del_v = (del_v*60)/(70*3.14);
//    rpm_target_L = rpm_target_L + del_v;
//    rpm_target_R = rpm_target_R - del_v;
//    pwm_L = (2.114*rpm_target_L + 96.23);
//    pwm_R = (2.02*rpm_target_R + 100.9);
//    prev_error = error;
//    prev_dist_L=dist_L;
//    Serial.print(prev_dist_L);
//    prev_dist_R=dist_R; 
//    Serial.print(" ");
//    Serial.println(prev_dist_R);
//  }
  delta_x = (l_s + r_s)/2;
  heading = atan2((l_s-r_s)/2, WHEEL_BASE/2);
  
  theta += heading;
  Serial.println(theta);
  x += delta_x*cos(theta);
  y += delta_x*sin(theta);
  distance = (distance_L + distance_R)/2;
  distance_total_L = l_enc_count_total*WHEEL_CIRCUMFERENCE/PPR;
  distance_total_R = r_enc_count_total*WHEEL_CIRCUMFERENCE/PPR;
  
  distance_total = ((distance_total_L + distance_total_R))/2; 
  
  if (distance_total<950){
    Serial.println("going straight 1");
    error = (distance_L) - (distance_R);//0.975*
    error_dot = error - prev_error;
    del_v = -(.8*error) - (5.9*error_dot);
    del_v = (del_v*60)/(70*3.14);
    rpm_target_L = rpm_target_L + del_v;
    rpm_target_R = rpm_target_R - del_v;
    pwm_L = 1.3*(2.114*rpm_target_L + 96.23);
    pwm_R = (2.02*rpm_target_R + 100.9);
    prev_error = error;
    
  }else if (theta>=1.72 || third == 1){ 
    if( theta>=1.72 && second != 1){
      Serial.println("finished turn");
      l_enc_count = 0;
      r_enc_count = 0;
      prev_error = 0;
      distance_L = 0;
      distance_R = 0;
      rpm_target_L = ((180/1.3)-96.23)/2.114;
      rpm_target_R = (180-100.9)/2.02;
      second = 1;
      turn = 1;
      third =1;
     }
    Serial.println("going straight");
    error = (distance_L) - (distance_R);//0.975*
    error_dot = error - prev_error;
    del_v = -(0.8*error) - (5.9*error_dot);
    del_v = (del_v*60)/(70*3.14);
    rpm_target_L = rpm_target_L + del_v;
    rpm_target_R = rpm_target_R - del_v;
    pwm_L = 1.3*(2.114*rpm_target_L + 96.23);
    pwm_R = (2.02*rpm_target_R + 100.9);
    prev_error = error;
  }else if(turn != 1){
    Serial.println("in turn");
    C = 0.3;
    error = (C*distance_L) - (distance_R);
    error_dot = error - prev_error;
    del_v = -(0.003*error) - (0.05*error_dot);
    del_v = (del_v*60)/(70*3.14);
    rpm_target_L = rpm_target_L + del_v; // i switched this
    rpm_target_R = rpm_target_R - del_v;
    prev_error = error;
    pwm_L = 1.3*(2.114*rpm_target_L + 96.23);
    pwm_R = (2.02*rpm_target_R + 100.9);
  }
  
   String ret = "";
   ret =  String(error);
   //Serial.println(ret);
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
