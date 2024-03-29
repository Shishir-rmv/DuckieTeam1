//improvements needed to code:
//start out at equal rpms instead of equal PWMs
//PWMs might not matter if the initial target rpms are not equal
//rpm_traget have no relation to the rpms being read by encoders
//for suitable gains, need equations to convert the delv to mm/sec
//which is currently in mm/(duration)

#include <EnableInterrupt.h>
#include "DualMC33926MotorShield.h"
#include "types.h"
#include <string.h>
#include<math.h>

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
double ping_duration;

// positional variables
double theta = 0, x = 0, y = 0, C = 1;
double delta_x, heading, l_s, r_s; //left & right distance changed

// encoder counts
int l_enc_count, r_enc_count, old_l_enc_count = 0, old_r_enc_count = 0;

// total values can be removed for normal operation
// used only for demo to segregate distances at different stretches
double l_enc_count_total = 0, r_enc_count_total = 0,prev_l_enc_count=0,prev_r_enc_count=0;

// distances
double distance = 0, distance_R = 0, distance_L = 0, distance_total=0, distance_total_R = 0, distance_total_L=0;

// durations?
double duration_L, duration_R, prevmillis_L = micros(), prevmillis_R = micros();

// errors
double prev_error = 0, error = 0, error_dot = 0, del_v = 0, act = 0, ref=0;
int fourth;

// rpm's
double rpm_target_L = 0, rpm_target_R = 0, rpm_R = 0, rpm_L = 0,rpm_R_ref = 0, rpm_L_ref = 0, pwm_L = 0, pwm_R = 0;

int v_err = 0;

// BELOW VARIABLES NEED TO BE REMOVED FOR NORMAL OPERATION
// USED ONLY FOR THE DEMO USING ENCODER ODOMETRY
int update_rate = 4;//set from 1 to PPR or maybe more
int third=0;
int second = 0;

str_code hashit (String inString) {
   if (inString == "mtr") return motor;
   if (inString == "irr") return irSensor;
   if (inString == "png") return png;
   if (inString == "srt") return start;
   if (inString == "ver") return vOffset; 
   if (inString == "stp") return stopp;
   if (inString == "upd") return update;
   if (inString == "none") return none;
   if (inString == "st1") return state1;
   if (inString == "rtn") return rtn;
   if (inString == "ltn") return ltn;
}

void setup() {
  //setting interrupts on these pins
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
  
  md.init();
  //md.setM1Speed(pwm_R);
  //md.setM2Speed(pwm_L);
  Serial.begin(9600);
  // Serial.println("Start");
  delay(000);
}

void loop() {
//stopIfFault();
  static String opStr;
  static unsigned int arg1 = 0;
  static unsigned int arg2 = 0;
  static char input[15];
  static char opStrA[4];
  static char arg1A[5];
  static char arg2A[5];
  static double prevmillis_L = micros();
  static double prevmillis_R = micros();
//  Serial.println("HI MOM!");/

  if(Serial.available()){
    Serial.readBytesUntil('\n', input, 12);
    opStrA[0] = input[0];
    opStrA[1] = input[1];
    opStrA[2] = input[2];
    // WHY WERE THESE HERE?
    opStrA[3] ='\0';
    // Serial.println("opStrA");
    // Serial.println(opStrA);

    // if there's a first argument
    if (strlen(input) >= 7){
      arg1A[0] = input[3];
      arg1A[1] = input[4];
      arg1A[2] = input[5];
      arg1A[3] = input[6];
      // WHY WERE THESE HERE?
      arg1A[4] = '\0';
      // Serial.println("arg1A");
      // Serial.println(arg1A);
    }

    // if there's a second argument
    if (strlen(input) >= 11){
      arg2A[0] = input[7];
      arg2A[1] = input[8];
      arg2A[2] = input[9];
      arg2A[3] = input[10];
      // WHY WERE THESE HERE?
      arg2A[4] = '\0';

    }
      // Serial.println("arg2A");
      // Serial.println(arg2A);
    
  
    String opStr = String(opStrA);
    int arg1 = atoi(arg1A);
    int arg2 = atoi(arg2A);
    
////  }else if(distance_total>1100 && distance_total<1345){//&& micros()>1000000
////    //Serial.println(micros());
////    //
////     l_enc_count=0;
////     r_enc_count=0;
//    
//  }else if(distance_total>2225){//1870 if(turn != 1)
//    //Serial.println(distance);
//    Stop(); 
//  
  if opStr ==
  Serial.print(opStr);
  switch(hashit(opStr)){
    case none :
      Serial.println("none");
      break;

    case motor :
      Serial.println("motor");
      // Serial.println(arg1);
      // Serial.println(arg2);
      md.setM1Speed((int)arg1);
      md.setM2Speed((int)arg2);
      break;

    case png :
      Serial.print("ping duration: ");
      ping();
      break;
    
    case start :
      rpm_L_ref=arg2;//(arg1*60)/(70*3.14);
      rpm_R_ref=rpm_L_ref;
      pwm_L = (2.2*rpm_L_ref + 85);
      pwm_R = (2.1*rpm_R_ref + 81);
      break;
    
    case vOffset :
      v_err = arg2;
      break;

    case ltn :
      C = arg1;
      rpm_R_ref=arg2;//(arg1*60)/(70*3.14);
      rpm_L_ref=C*rpm_L_ref;
      pwm_L = (2.2*rpm_L_ref + 85);
      pwm_R = (2.1*rpm_R_ref + 81);
      break;
            
    case rtn :
      C = arg1;
      rpm_L_ref=arg2;//(arg1*60)/(70*3.14);
      rpm_R_ref=C*rpm_L_ref;
      pwm_L = (2.2*rpm_L_ref + 85);
      pwm_R = (2.1*rpm_R_ref + 81);
      break;

    case state1 :
        static double rpm_L_ref = 0;
        static double rpm_R_ref = 0;
        ref = 0;
        act = y;
        straight();
        del_v = -(0.3*error) - (0.01*error_dot);
        del_v = (del_v*60)/(70*3.14);
        rpm_target_L = rpm_L_ref + del_v;
        rpm_target_R = rpm_R_ref - del_v;
        pwm_L = (2.2*rpm_target_L + 85);
        pwm_R = (2.1*rpm_target_R + 81);
        Serial.print("State1");
      if (x <500){
        opStr = "st1";
        
      }else{
        opStr = "stp";
      }
      break;
       
      
    default:
      opStr="";
      break;
      
    case stopp :
      Stop();
      break;

    case irSensor :
      encoder();
      break;
  }
  }
  if (v_err != prev_error && fourth!=1){
    error_dot = v_err - prev_error;
    del_v = -(0.3*v_err) - (0.01*error_dot);
    del_v = (del_v*60)/(70*3.14);
    rpm_target_L = rpm_L_ref + del_v;
    rpm_target_R = rpm_R_ref - del_v;
//    if(millis()%1000 == 0 ){
//        Serial.write('a');
//        Serial.write(lowByte((int)rpm_target_L));
//        Serial.write('b');
//        Serial.write(lowByte((int)rpm_target_R));
//    }
    pwm_L = (2.2*rpm_target_L + 85);
    pwm_R = (2.1*rpm_target_R + 81);
    prev_error = v_err;
  }
   


 duration_L = micros()-prevmillis_L;
 if (duration_L > 2000000){
      l_s = (l_enc_count-prev_l_enc_count)*WHEEL_CIRCUMFERENCE*2000000/(PPR*duration_L);
      r_s = (r_enc_count-prev_r_enc_count)*WHEEL_CIRCUMFERENCE*2000000/(PPR*duration_L); 
      delta_x = (l_s + r_s)/2;
      heading = atan2((l_s-r_s)/2, WHEEL_BASE/2);
      theta += heading;
      x += delta_x*cos(theta);
      y += delta_x*sin(theta);
      
      String ret = "x "+String(x)+" y "+String(y)+" "+String(l_s)+" "+String(r_s);
      Serial.println(ret);
      prev_l_enc_count = l_enc_count;
      prev_r_enc_count = r_enc_count;
      prevmillis_L = micros();
  }    
//   
//  if (x>0 && y>0 && x<1000 && y<1000){
//  ref = 0;
//  act = y;
//  straight();
//  del_v = -(0.3*error) - (0.01*error_dot);
//  del_v = (del_v*60)/(70*3.14);
//  rpm_target_L = rpm_L_ref + del_v;
//  rpm_target_R = rpm_R_ref - del_v;
//  pwm_L = (2.2*rpm_target_L + 85);
//  pwm_R = (2.1*rpm_target_R + 81);
//  }else if(x>1000 && y>1000 && x<1200 && y<200){
//   ref = 100;
//   act = pow((x-1000),2) + pow(y,2);
//   turn();
//   del_v = -(0.3*v_err) - (0.01*error_dot);
//   del_v = (del_v*60)/(70*3.14);
//   rpm_target_L = rpm_L_ref + del_v;
//   rpm_target_R = rpm_R_ref - del_v;
//   pwm_L = (2.2*rpm_target_L + 85);
//   pwm_R = (2.1*rpm_target_R + 81);
//  }else{
//    Stop()
//  }
//         
  md.setM2Speed(pwm_L);    
  md.setM1Speed(pwm_R);
  //opStr = "none";
  //arg1 = 0;
  //arg2 = 0;
}

void straight(){
prev_error = error;
error = act - ref;
}
void turn(){
prev_error = error;
error = act -ref;
}


void encoder() {
  /* 
  
      
  */
     
  int l_enc = read_encoderL((ENC_PORT2 & 0b110000) >> 4);
  int r_enc = read_encoderR((ENC_PORT & 0b1100000) >> 5);

  //distance and distance_total have just been put in place for
  //having different total distance for different state and turns
  l_enc_count += l_enc;
  r_enc_count += r_enc;
  //Serial.println(l_enc_count);
  //distance_L = l_enc_count*WHEEL_CIRCUMFERENCE/PPR;
  //distance_R = r_enc_count*WHEEL_CIRCUMFERENCE/PPR;  
  //distance = (distance_L + distance_R)/2;    
   
  //l_enc_count_total += l_enc;
  //r_enc_count_total += r_enc;
  //distance_total_L = l_enc_count_total*WHEEL_CIRCUMFERENCE/PPR;
  //distance_total_R = r_enc_count_total*WHEEL_CIRCUMFERENCE/PPR;
  //distance_total = ((distance_total_L + distance_total_R))/2;
   
  
  //r_s = r_enc*WHEEL_CIRCUMFERENCE/PPR;
  //l_s = l_enc*WHEEL_CIRCUMFERENCE/PPR; 
  //delta_x = (l_s + r_s)/2;
  //heading = atan2((l_s-r_s)/2, WHEEL_BASE/2);
  
  //theta += heading;   
  //x += delta_x*cos(theta);
  //y += delta_x*sin(theta);
   
 // if (l_enc_count!= old_l_enc_count){
 //   old_l_enc_count = l_enc_count;
 //   if (l_enc_count%update_rate==0){
 //   duration_L = (micros() - prevmillis_L); // Time difference between revolution in microsecond
 //   if(abs(l_enc_count > 2)){
 //   rpm_L = update_rate*(60000000/duration_L)/PPR; // rpm = (1/ time millis)*1000*1000*60;
 //     }
 //     prevmillis_L = micros();
 //     }
 // }
 // if (r_enc_count!= old_r_enc_count){
  //  old_r_enc_count = r_enc_count;
    //if(r_enc_count%update_rate==0 ){
    //  duration_R = (micros() - prevmillis_R); // Time difference between revolution in microsecond
     // if(abs(r_enc_count > 2)){
     // rpm_R = update_rate*(60000000/duration_R)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      //}
      //prevmillis_R = micros();
    //}
  }


void ping(){
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);
  ping_duration = pulseIn(PING_PIN, HIGH);
  Serial.println(ping_duration);
}


//check the stop if fault in the library
void stopIfFault(){
  if (md.getFault()){
    Serial.println("fault");
    Stop();
    while(1);
  }
}

void Stop() {
  pwm_R = 0;
  pwm_L = 0;
  rpm_target_R = 0;
  rpm_target_L = 0;
  v_err=0;
  md.setM2Speed(pwm_L);    
  md.setM1Speed(pwm_R);
  fourth = 1;
}

int8_t read_encoderL(int8_t new_val){
  //defines array that describes moving states of the QE for CW & CCW motion
  static int8_t enc_statesL[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_valL = 0;

  enc_valL <<= 2; //preserve old value
  enc_valL |= new_val; //add new value to last 2 LSB 
  return (enc_statesL[enc_valL & 0b1111]); //get state and remove old values
}

int8_t read_encoderR(int8_t new_val){
  //defines array that describes moving states of the QE for CW & CCW motion
  static int8_t enc_statesR[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_valR = 0;

  enc_valR <<= 2; //preserve old value
  enc_valR |= new_val; //add new value to last 2 LSB 
  return (enc_statesR[enc_valR & 0b1111]); //get state and remove old values
}
