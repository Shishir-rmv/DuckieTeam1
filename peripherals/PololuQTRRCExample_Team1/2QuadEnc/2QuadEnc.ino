#define L_ENC_A 5
#define L_ENC_B 4
#define R_ENC_A 7
#define R_ENC_B 6

#define ENC_PORT PIND

#define WHEEL_BASE 137 //current approximation in mm, chassis constant
#define WHEEL_CIRCUMFERENCE 219.9115 //circumference = 2*pi*r, r = 35mm, pi = 3.14
#define PPR 16 //pulses per revolution is = to # of lines per revoluion aka number of white segments which is half of the total segments, this is assumig 32 segments

// set all global position variables to 0
double theta = 0;
double x = 0;
double y = 0;



void setup() {
  // put your setup code here, to run once:

  pinMode(L_ENC_A, INPUT);
  digitalWrite(L_ENC_A, HIGH);
  
  pinMode(L_ENC_B, INPUT);
  digitalWrite(L_ENC_B, HIGH);

  Serial.begin(9200);
  Serial.println("Start");
}

void loop() {
  // put your main code here, to run repeatedly:

  static uint8_t R_counter, L_counter = 0;
  static uint8_t l_enc, r_enc = 0;
  
  int8_t tmpdata;
  double l_s; //left distance changed
  double r_s; //right distance changed
  double delta_x;
  double heading;

  l_enc = read_encoder((ENC_PORT & 0x30));
  r_enc = read_encoder((ENC_PORT & 0xc0));

  if((l_enc || r_enc))
  {
    //print counters for left and right wheels
    Serial.print("Left Counter value: ");
    Serial.println(L_counter, DEC );

    Serial.print("Right Counter value: ");
    Serial.println(R_counter, DEC );

    //increment counter by new encoder value
    L_counter += l_enc;
    R_counter += r_enc;

    //get change in distance
    l_s = WHEEL_CIRCUMFERENCE*(l_enc/PPR);
    r_s = WHEEL_CIRCUMFERENCE*(r_enc/PPR);

    //update the change in avg position and current heading
    delta_x = (l_s + r_s)/2;
    heading = atan2((r_s-l_s)/2, WHEEL_BASE/2);
    
    //update overall global positioning
    theta += heading;
    x += delta_x*cos(theta);
    y += delta_x*sin(theta);
  }
}

int8_t read_encoder(int8_t new_val)
{
  //defines array that describes moving states of the QE for CW & CCW motion
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_val = 0;

  enc_val <<= 2; //preserve old value
  enc_val |= new_val; //add new value to last 2 LSB 

  return (enc_states[(enc_val & 0x0f)]); //get state and remove old values
}
