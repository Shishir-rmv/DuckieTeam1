//improvements needed to code:
//start out at equal rpms instead of equal PWMs
//PWMs might not matter if the initial target rpms are not equal
//rpm_traget have no relation to the rpms being read by encoders
//for suitable gains, need equations to convert the delv to mm/sec
//which is currently in mm/(duration)

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
double ping_duration;
double theta = 0;
double x = 0;
double y = 0;
double C = 1;
double l_s; //left distance changed
double r_s; //right distance changed
double delta_x;
double heading;

int l_enc_count;
int r_enc_count;
int old_l_enc_count = 0;
int old_r_enc_count = 0;

// total values can be removed for normal operation
// used only for demo to segregate distances at different stretches
double l_enc_count_total = 0;
double r_enc_count_total = 0;

double distance = 0;
double distance_R = 0;
double distance_L = 0;
double distance_total=0;
double distance_total_R = 0;
double distance_total_L=0;

double duration_L;
double duration_R;
double prevmillis_L = micros();
double prevmillis_R = micros();

double prev_error = 0;
double error = 0;
double error_dot = 0;
double del_v = 0;

double rpm_target_L = 0;
double rpm_target_R = 0;
double rpm_R = 0;
double rpm_L = 0;
double pwm_L = 0;
double pwm_R = 0;

int v_err = 0;

// BELOW VARIABLES NEED TO BE REMOVED FOR NORMAL OPERATION
// USED ONLY FOR THE DEMO USING ENCODER ODOMETRY
int update_rate = 4;//set from 1 to PPR or maybe more
int third=0;
int second = 0;
int turn = 0;

str_code hashit (String inString) {
   if (inString == "mtr") return motor;
   if (inString == "irr") return irSensor;
   if (inString == "png") return png;
   if (inString == "srt") return start;
   if (inString == "ver") return vOffset; 
   if (inString == "stp") return stopp;
   if (inString == "upd") return update;
   if (inString == "none") return none;
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

  //setting up pins for ping 
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
      //rpm_target_L=(arg2*60)/(70*3.14);
      //rpm_target_R=rpm_target_L;
      pwm_L = arg2;//(2.114*rpm_target_L + 96.23);
      pwm_R = arg2;//(2.02*rpm_target_R + 100.9);
      break;
    
    case vOffset :
      v_err = arg2;
      Serial.println(v_err);
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
  md.setM2Speed(pwm_L);    
  md.setM1Speed(pwm_R);
  opStr = "none";
}


void encoder() {
  int l_enc = read_encoderL((ENC_PORT2 & 0b110000) >> 4);
  int r_enc = read_encoderR((ENC_PORT & 0b1100000) >> 5);

  //distance and distance_total have just been put in place for
  //having different total distance for different state and turns
  l_enc_count += l_enc;
  r_enc_count += r_enc;
  distance_L = l_enc_count*WHEEL_CIRCUMFERENCE/PPR;
  distance_R = r_enc_count*WHEEL_CIRCUMFERENCE/PPR;  
  distance = (distance_L + distance_R)/2;    
   
  l_enc_count_total += l_enc;
  r_enc_count_total += r_enc;
  distance_total_L = l_enc_count_total*WHEEL_CIRCUMFERENCE/PPR;
  distance_total_R = r_enc_count_total*WHEEL_CIRCUMFERENCE/PPR;
  distance_total = ((distance_total_L + distance_total_R))/2;
   
  
  r_s = r_enc*WHEEL_CIRCUMFERENCE/PPR;
  l_s = l_enc*WHEEL_CIRCUMFERENCE/PPR; 
  delta_x = (l_s + r_s)/2;
  heading = atan2((l_s-r_s)/2, WHEEL_BASE/2);
  
  theta += heading;   
  x += delta_x*cos(theta);
  y += delta_x*sin(theta);
   
  
  
   
// stop-gap fix for skipping counts   
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

  


  
  if (l_enc_count!= old_l_enc_count){
    old_l_enc_count = l_enc_count;
    if (l_enc_count%update_rate==0 && l_enc_count>2){
      duration_L = (micros() - prevmillis_L); // Time difference between revolution in microsecond
      rpm_L = update_rate*(60000000/duration_L)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      prevmillis_L = micros();
        Serial.print("L "); 
        Serial.print(l_enc_count);
        Serial.print("   ");  
        Serial.println(rpm_L);
        Serial.print("   ");
      }
  }
  if (r_enc_count!= old_r_enc_count){
    old_r_enc_count = r_enc_count;
    if(r_enc_count%update_rate==0 && r_enc_count>2){
      duration_R = (micros() - prevmillis_R); // Time difference between revolution in microsecond
      rpm_R = update_rate*(60000000/duration_R)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      old_r_enc_count = r_enc_count;
      prevmillis_R = micros();
        Serial.print("  R ");
        Serial.print(r_enc_count);
        Serial.print("   ");  
        Serial.println(rpm_R);
       Serial.print("   ");
    }
  }
  
}

void ping() {
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
  md.setM2Speed(pwm_L);    
  md.setM1Speed(pwm_R);
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
