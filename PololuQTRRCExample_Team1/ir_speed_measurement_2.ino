/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include "DualMC33926MotorShield.h"
#define WHEEL_BASE 137 //current approximation in mm, chassis constant
#define WHEEL_CIRCUMFERENCE 219.9115 //circumference = 2*pi*r, r = 35mm, pi = 3.14
#define PPR 8 //pulses per revolution is = to # of lines per revoluion aka number of white segments which is half of the total segments, this is assumig 32 segments

DualMC33926MotorShield md;
int runspeed = 400;
void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  md.init();
  md.setM2Speed(runspeed);
  
  
}
float prevmillis=0;
long oldPosition  = -999;
double distance = 0;
void loop() {
    
    stopIfFault();
    long newPosition = myEnc.read();
    if (newPosition != oldPosition) { 
      oldPosition = newPosition;

      double duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
      double rpm = (60000000/duration)/PPR; // rpm = (1/ time millis)*1000*1000*60;
      Serial.println(newPosition);    
      Serial.println(rpm);
      prevmillis = micros();
      distance += WHEEL_CIRCUMFERENCE/PPR;
      Serial.println(distance);
    }
    if (oldPosition%200==0 && oldPosition!=0){
      md.setM2Speed(0);
      //Serial.println((micros()-prevmillis));
      //if ((micros()-prevmillis)>2000000){
      //  md.setM2Speed(runspeed);
      //  prevmillis = micros();  
      //  oldPosition++;
      }
    }  
      
    
