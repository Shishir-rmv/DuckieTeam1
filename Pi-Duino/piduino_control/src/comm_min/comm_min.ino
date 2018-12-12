#include <EnableInterrupt.h>
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

short r_count;
short l_count;
long ping_distance; //cm
double ping_slowdown;

// M1 is Right, M2 is left
// Interrupt 3,dig 6 for right, interrupt 2,dig 5 for left
//Encoder myEnc_L(L_ENC_A,L_ENC_B);
//Encoder myEnc_R(R_ENC_A,R_ENC_B);

str_code hashit (String inString) {
  if (inString == "irr") return irSensor;
  if (inString == "png") return png;
  if (inString == "none") return none;
}

void setup() {
  // put your setup code here, to run once:

  pinMode(L_ENC_A, INPUT);
  digitalWrite(L_ENC_A, HIGH);

  //  enableInterrupt(L_ENC_A, encoder, CHANGE);

  pinMode(L_ENC_B, INPUT);
  digitalWrite(L_ENC_B, HIGH);

  //  enableInterrupt(L_ENC_B, encoder, CHANGE);

  pinMode(R_ENC_A, INPUT);
  pinMode(R_ENC_B, INPUT);
  digitalWrite(R_ENC_A, HIGH);

  //  enableInterrupt(R_ENC_A, encoder, CHANGE);
  digitalWrite(R_ENC_B, HIGH);

  //  enableInterrupt(R_ENC_B, encoder, CHANGE);

  Serial.begin(115200);
  // Serial.println("Start");
}

void loop() {
  // put your main code here, to run repeatedly:
  //stopIfFault();
  //static int prevError = 0;
  static int output;
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
    Serial.println(arg1);
    Serial.println(arg2);
    switch (hashit(opStr)) {
      case none :
        break;

      case irSensor :
        getEnc();
        break;

      
      case png :
        Serial.print("ping duration: ");
        ping();
        break;
      
      default:
        break;
    }
    opStr = "none";
  }
}






void encoder() {
  long l_enc = read_encoderL((ENC_PORT2 & 0b110000) >> 4);
  long r_enc = read_encoderR((ENC_PORT & 0b1100000) >> 5);

  l_count += l_enc;
  r_count += r_enc;
}

void getEnc() {
  char buf[15];
  String str = String(l_count) + "," + String(r_count);
//  Serial.println("yo");
  str.toCharArray(buf,15);
  Serial.write(lowByte(l_count));
  Serial.write(lowByte(r_count));
  Serial.flush();
  l_count = 0;
  r_count = 0;
}

void ping() {
  long ping_duration, startT, endT, Telapsed;
  startT = micros();
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);
  ping_duration = pulseIn(PING_PIN, HIGH, 3000);
  endT = micros();
  ping_distance = microsecondsToInches(ping_duration);
  Telapsed = endT - startT;
  Serial.print("got distance:  ");
  Serial.print(ping_duration);
  Serial.print("us   in  ");
  Serial.print(Telapsed);
  Serial.println("  microseconds");
  if( (ping_duration >= 100) || (ping_duration <= 800) ){
    while( (ping_duration >= 100) || (ping_duration <= 800) ){
      Serial.println("stop");
      pinMode(PING_PIN, OUTPUT);
      digitalWrite(PING_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(PING_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(PING_PIN, LOW);

      pinMode(PING_PIN, INPUT);
      ping_duration = pulseIn(PING_PIN, HIGH, 3000);
    }
  }
  else if( (ping_duration > 800) || (ping_duration <= 1600) ){
    ping_slowdown = ping_duration/1600;
  }
  else{
    ping_slowdown = 1;
  }
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}
int8_t read_encoderL(int8_t new_val)
{
  //defines array that describes moving states of the QE for CW & CCW motion
  static int8_t enc_statesL[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
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
  static int8_t enc_statesR[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t enc_valR = 0;
  static uint8_t stateR = 0;

  enc_valR <<= 2; //preserve old value
  enc_valR |= new_val; //add new value to last 2 LSB
  stateR = enc_valR & 0b1111;
  // Serial.println(stateR, BIN);

  return (enc_statesR[stateR]); //get state and remove old values
}
