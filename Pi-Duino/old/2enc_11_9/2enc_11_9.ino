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
int l_enc_count;
int r_enc_count;
int old_l_enc_count;
int old_r_enc_count;
double delta_x;
double heading;
double ping_duration;
char dataString[5] = {0};
double duration_L;
double duration_R;
double rpm_L;
double rpm_R;
double prevmillis_L = micros();
double prevmillis_R = micros();
double distance_R = 0;
double distance_L=0;
int update_rate = 1;//set from 1 to PPR or maybe more

double error = 0;
double error_dot = 0;
double del_v = 0;
double rpm_target_L;
double rpm_target_R;
double pwm_L = 100;
double pwm_R = 100;
double prev_error = 0;
double distance = 0;
double C = 1;

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
  md.setM1Speed(pwm_R);
  md.setM2Speed(pwm_L);
  Serial.begin(9600);
  // Serial.println("Start");
  delay(000);
}

void loop() {
  // put your main code here, to run repeatedly:
//stopIfFault();
static String opStr;
static unsigned int arg1 = 0;
static unsigned int arg2 = 0;

  if(Serial.available()){
    String input = Serial.readString();
    Serial.print("input: "+input);

    opStr = input.substring(0,3);
    // Serial.println(opStr);

    arg1 = input.substring(3,7).toInt();
    arg2 = input.substring(7,11).toInt();
  }
  else if(distance>500 && micros()>9000000){
    //Serial.println(micros());
    //
    Stop();
  }
  else if(distance>1000){
    //Serial.println(micros());
    //Stop();
    C = 0.6459;
    distance_L = 0;
    distance_R = 0;

    
  }
 
  else{
    
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
  int r_enc = read_encoderR((ENC_PORT & 0b1100000) >> 5);

  r_s = r_enc*WHEEL_CIRCUMFERENCE/PPR;
  l_s = l_enc*WHEEL_CIRCUMFERENCE/PPR;

  l_enc_count += l_enc;
  r_enc_count += r_enc;
  
  // //update the change in avg position and current heading
  //delta_x = (l_s + r_s)/2;
  //heading = atan2((r_s-l_s)/2, WHEEL_BASE/2);

   //update overall global positioning
   //heta += heading;
   //x += delta_x;//*cos(theta);
   //y += delta_x;//*sin(theta);

  distance_L += l_s;
  distance_R += r_s;
  
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
      Serial.println(error);
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
      Serial.print("   ");
      Serial.println(error);
    }
  }

  distance = (distance_L +distance_R)/2;
  theta = atan2((distance_L - distance_R),WHEEL_BASE);
   
  error = (distance_L) - (C*distance_R);
  error_dot = error - prev_error;
  del_v = -(0.05*error) - (0.005*error_dot);
  del_v = (del_v*60)/(70*3.14);
  rpm_target_L = rpm_L + del_v;
  rpm_target_R = rpm_R - del_v;
  pwm_L = 2.114*rpm_target_L + 96.23;
  pwm_R = 2.02*rpm_target_R + 100.9;
  prev_error = error;


  
   //String ret = "";
   //ret = "l_enc: " +String(l_enc_count) + " rpm_L: " + String(rpm_L)  +  " r_enc: " + String(r_enc_count) + " rpm_R: " + String(rpm_R);
   //ret = "error = " + String(error) + ;
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
