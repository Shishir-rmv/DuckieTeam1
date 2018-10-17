#define ENC_A 14
#define ENC_B 15
#define ENC_PORT PINC

void setup() {
  // put your setup code here, to run once:
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);
  Serial.begin(9200);
  Serial.println("Start");
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint8_t counter = 0;
  int8_t tmpdata;
  tmpdata = read_encoder();
  if(tmpdata)
  {
    Serial.print("Counter value: ");
    Serial.println(counter, DEC);
    counter += tmpdata;
  }
}

int8_t read_encoder()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  old_AB <<= 2;
  old_AB |= (ENC_PORT & 0x03);
  return (enc_states[(old_AB & 0x0f)]);
}
