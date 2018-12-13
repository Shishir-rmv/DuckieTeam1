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


#define L_ENC_A 12 //change to actual used pins. A is front wheel. B is back
#define L_ENC_B 13 //check that pins mathc up with right sensors and encoder init
#define R_ENC_A 5
#define R_ENC_B 6
#define PING_PIN 11

#define ENC_PORT PIND
#define ENC_PORT2 PINB

#define WHEEL_BASE 137 //current approximation in mm, chassis constant
#define WHEEL_CIRCUMFERENCE 219.9115 //circumference = 2*pi*r, r = 35mm, pi = 3.14
#define PPR 32 //pulses per revolution is = to # of lines per revoluion aka number of white segments which is half of the total segments, this is assumig 32 segments

DualMC33926MotorShield md;
long ping_duration;
double ping_slowdown = 1;

// positional variables
double theta = 0, x = 0, y = 0, C = 1;
double delta_x, heading, l_s, r_s; //left & right distance changed

// encoder counts
int l_enc_count, r_enc_count, prev_l_enc_count = 0, prev_r_enc_count = 0, blocking = 0, v_err = 0;

// durations?
double duration, prevmillis, turn_micros;

// errors
double prev_error = 0, error = 0, error_dot = 0, del_v = 0,act,ref;
// rpm's
double rpm_target_L = 0, rpm_target_R = 0,rpm_R_ref = 0, rpm_L_ref = 0, pwm_L = 0, pwm_R = 0, local_L_ref, local_R_ref;

String opStrB;

str_code hashit (String inString) {
   if (inString == "mtr") return motor;
   if (inString == "srt") return start;
   if (inString == "ver") return vOffset; 
   if (inString == "stp") return stopp;
   if (inString == "upd") return update;
   if (inString == "none") return none;
   if (inString == "st1") return state1;
   if (inString == "rtn") return rtn;
   if (inString == "ltn") return ltn;
   if (inString == "xyt") return xyt;
}

void setup() {
  //setting interrupts on these pins
  pinMode(L_ENC_A, INPUT);
  digitalWrite(L_ENC_A, HIGH);
  pinMode(L_ENC_B, INPUT);
  digitalWrite(L_ENC_B, HIGH);
   
 //enableInterrupt(L_ENC_A, encoder, CHANGE);
 //enableInterrupt(L_ENC_B, encoder, CHANGE);

  pinMode(R_ENC_A, INPUT);
  digitalWrite(R_ENC_A, HIGH);
  pinMode(R_ENC_B, INPUT);
  digitalWrite(R_ENC_B, HIGH);

  //enableInterrupt(R_ENC_A, encoder, CHANGE);
  //enableInterrupt(R_ENC_B, encoder, CHANGE);
  
  md.init();
  //md.setM1Speed(pwm_R);
  //md.setM2Speed(pwm_L);
  Serial.begin(9600);
  // Serial.println("Start");
  delay(000);
}

void loop() {
  static long last_ping = 0, curr_ping = 0;
//stopIfFault();
  static String opStr;
  static float arg1 = 0;
  static int arg2 = 0;
  static char input[15];
  static char opStrA[4];
  static char arg1A[5];
  static char arg2A[5];

  // investigate
  static double prevmillis = micros();

  if(Serial.available()){
    Serial.readBytesUntil('\n', input, 12);
    opStrA[0] = input[0];
    opStrA[1] = input[1];
    opStrA[2] = input[2];
    opStrA[3] ='\0';


    if (strlen(input) >= 7){
      arg1A[0] = input[3];
      arg1A[1] = input[4];
      arg1A[2] = input[5];
      arg1A[3] = input[6];
      arg1A[4] = '\0';
    }

    // if there's a second argument
    if (strlen(input) >= 11){
      arg2A[0] = input[7];
      arg2A[1] = input[8];
      arg2A[2] = input[9];
      arg2A[3] = input[10];
      arg2A[4] = '\0';
    }

    opStr = String(opStrA);
    arg1 = atof(arg1A);
    arg2 = atoi(arg2A);
  

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
    
    case start :
      blocking = 0;
      rpm_R_ref=arg2;//(arg1*60)/(70*3.14);
      rpm_L_ref=rpm_R_ref;//Vref = 45 C = 0.2  small right turn 0.45 big turn
      pwm_L = (2.2*rpm_L_ref + 85);
      pwm_R = (2.1*rpm_R_ref + 81);
      break;
    
    case ltn :
      turn_micros = micros();
      opStrB = "ltn";
      break; 
            
    case rtn :
      turn_micros = micros();
      opStrB = "rtn";
      break;

    case vOffset :
      prev_error = v_err;
      v_err = arg2;
      break;
    
    case stopp :
      Stop();
      break;
      
    case state1 :
      rpm_R_ref=arg2;//(arg1*60)/(70*3.14);
      rpm_L_ref=rpm_R_ref;//Vref = 45 C = 0.2  small right turn 0.45 big turn
      opStrB = "st1";
      break;

    case xyt :
      Serial.write('x');
      Serial.write(lowByte(int(x)));
      Serial.write('y');
      Serial.write(lowByte(int(y)));
      Serial.write('t'); 
      Serial.write(lowByte(int(theta)));
      break;
      
    default:
      break;
  }
  }

    switch(hashit(opStrB)){    
      case rtn :
        blocking = 1;
        C = arg1;
        local_L_ref=arg2;//C=0.2 V45 C2 22.5
        local_R_ref=C*rpm_L_ref;
        pwm_L = (2.2*local_L_ref + 85);
        pwm_R = (2.1*local_R_ref + 81);
        if (micros()-turn_micros > 4000000){
          // Serial.print(C);
          Serial.write('D');
          opStrB="";
          blocking = 0;
        }
        break;

      case ltn :
        blocking = 1;
        C = arg1;
        local_R_ref=arg2;//C=0.2 V45 C2 22.5
        local_L_ref=C*rpm_R_ref;
        pwm_L = (2.2*local_L_ref + 85);
        pwm_R = (2.1*local_R_ref + 81);
        if (micros()-turn_micros > 5500000){
          Serial.write('D');
          opStrB="";
          blocking = 0;
        }
        break;

      case state1 :
        ref = 0;
        act = y;
        straight();
        pd();
        break;

      case state2 :
        ref = 100;
        act = pow((x-1000),2) + pow(y,2);
        turn();
        break;
      
      default :
        opStr = "";
        break; 
  }
  if (v_err != prev_error && blocking != 1){
    error_dot = v_err - prev_error;
    del_v = -(0.3*v_err) - (0.01*error_dot);
    del_v = (del_v*60)/(70*3.14);
    rpm_target_L = rpm_L_ref + del_v;
    rpm_target_R = rpm_R_ref - del_v;
//    if(millis()%1000 == 0){
//        Serial.write('a');
//        Serial.write(lowByte((int)rpm_target_L));
//        Serial.write('b');
//        Serial.write(lowByte((int)rpm_target_R));
//    }
    pwm_L = (2.2*rpm_target_L + 85);
    pwm_R = (2.1*rpm_target_R + 81);
  }

 duration = micros()-prevmillis;
 
 if (duration > 2000000){
      Serial.write("HEARTBEAT");
      l_s = (l_enc_count-prev_l_enc_count)*WHEEL_CIRCUMFERENCE*2000000/(PPR*duration);
      r_s = (r_enc_count-prev_r_enc_count)*WHEEL_CIRCUMFERENCE*2000000/(PPR*duration); 
      delta_x = (l_s + r_s)/2;
      heading = atan2((l_s-r_s)/2, WHEEL_BASE/2);
      theta += heading;
      x += delta_x*cos(theta);
      y += delta_x*sin(theta);
      //String ret = "x "+String(x)+" y "+String(y)+" theta "+String(theta);
      //Serial.println(ret);
      prev_l_enc_count = l_enc_count;
      prev_r_enc_count = r_enc_count;
      prevmillis = micros();
  }    

  curr_ping = micros();

  if( (curr_ping-last_ping) > 1000000){ //test every 2 s
    ping();
    last_ping = curr_ping;   
  }

  md.setM2Speed(pwm_L*ping_slowdown);    
  md.setM1Speed(pwm_R*ping_slowdown);
}


void encoder() {
  int l_enc = read_encoderL((ENC_PORT2 & 0b110000) >> 4);
  int r_enc = read_encoderR((ENC_PORT & 0b1100000) >> 5);
  l_enc_count += l_enc;
  r_enc_count += r_enc;
}

void pd(){
  del_v = -(0.3*error) - (0.01*error_dot);
  del_v = (del_v*60)/(70*3.14);
  rpm_target_L = rpm_L_ref + del_v;
  rpm_target_R = rpm_R_ref - del_v;
  pwm_L = (2.2*rpm_target_L + 85);
  pwm_R = (2.1*rpm_target_R + 81);
}

void straight(){
  prev_error = error;
  error = act - ref;
  error_dot = error-prev_error;
}
void turn(){
  prev_error = error;
  error = act -ref;
  error_dot = error-prev_error;
}
void ping() {
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);
  ping_duration = pulseIn(PING_PIN, HIGH, 3000);
  
 
  if( (ping_duration >= 100) && (ping_duration <= 800) ){
    ping_slowdown = 0;
  }
  else if( (ping_duration > 800) && (ping_duration <= 1600) ){
    ping_slowdown = (double)ping_duration/1600.0;
  }
  else{
    ping_slowdown = 1;
  }
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
  Serial.write("STOPPING");
  blocking = 1;
  pwm_R = 0;
  pwm_L = 0;
  rpm_target_R = 0;
  rpm_target_L = 0;
  md.setM2Speed(pwm_L);    
  md.setM1Speed(pwm_R);
  opStrB = "";
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
