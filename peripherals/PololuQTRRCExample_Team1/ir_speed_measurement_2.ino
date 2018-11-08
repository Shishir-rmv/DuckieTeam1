//   Unit of position is pulses, unit of distance is mm

//   Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

// M1 is Right, M2 is left
// Interrupt 3,dig 6 for right, interrupt 2,dig 5 for left


#include <Encoder.h>
#include "DualMC33926MotorShield.h"
#define WHEEL_BASE 137 //current approximation in mm, chassis constant
#define WHEEL_CIRCUMFERENCE 219.9115 //circumference = 2*pi*r, r = 35mm, pi = 3.14
#define PPR 8 //pulses per revolution is = to # of lines per revoluion aka number of white segments which is half of the total segments, this is assumig 32 segments
#define FORWARD +1

DualMC33926MotorShield md;
int runspeed = 400;
int runspeed_L = 400;
int runspeed_R = 400;
double travel_distance = 5000;
long travel_position = FORWARD*(PPR*travel_distance)/WHEEL_CIRCUMFERENCE;
void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

Encoder myEnc_L(2,5);
Encoder myEnc_R(4,6);

void setup() {
  Serial.begin(9600);
  Serial.println("Pulses RPM   Distance(mm)");
  md.init();
  md.setM2Speed(runspeed_L);
  md.setM1Speed(runspeed_R);
}

double prevmillis_L = micros();
double prevmillis_R = micros();
long oldPosition_L  = -999;
long oldPosition_R  = -999;
double distance_R = 0;
double distance_L = 0;
double distance = 0;
long update_rate = PPR*5; //every 5 revolutions

void loop() {    
  stopIfFault();
  long newPosition_L = myEnc_L.read();    
  long newPosition_R = myEnc_R.read();
    
  if (newPosition_L != oldPosition_L) { 
    oldPosition_L = newPosition_L;
    if (oldPosition_L%update_rate==0 && oldPosition_L!=0){
      double duration_L = ( micros() - prevmillis_L); // Time difference between revolution in microsecond
      double rpm_L = update_rate*(60000000/duration_L)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      distance_L += update_rate*WHEEL_CIRCUMFERENCE/PPR;
      prevmillis_L = micros();
      Serial.print("L ");
      Serial.print(newPosition_L);    
      Serial.print("   ");
      Serial.print(rpm_L);   
      Serial.print("  ");
      Serial.println(distance_L);
    }
  }
  if (newPosition_R != oldPosition_R) { 
    oldPosition_R = newPosition_R;
    if (oldPosition_R%update_rate==0 && oldPosition_R!=0){
      double duration_R = ( micros() - prevmillis_R); // Time difference between revolution in microsecond
      double rpm_R = update_rate*(60000000/duration_R)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      distance_R += update_rate*WHEEL_CIRCUMFERENCE/PPR;
      prevmillis_R = micros();
      Serial.print("R ");
      Serial.print(newPosition_R);    
      Serial.print("   ");
      Serial.print(rpm_R);   
      Serial.print("  ");
      Serial.println(distance_R);
    }
  }
  if (oldPosition_L<travel_position){
      md.setM2Speed(0);
      //Serial.println((micros()-prevmillis));
      //if ((micros()-prevmillis)>2000000){
      //  md.setM2Speed(runspeed);
      //  prevmillis = micros();  
      //  oldPosition++;
      }
   if (micros()>12000000){
      md.setM2Speed(0);
       }
  }  
      
    
