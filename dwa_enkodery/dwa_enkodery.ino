const uint8_t enkL_A_pin=2; //enkoder lewego koła
const uint8_t enkL_B_pin=3;
const uint8_t enkP_A_pin=18; //enkoder prawego koła
const uint8_t enkP_B_pin=19;
long int encoderCount=0;
const float promien = 0.045;
const float rozstaw = 0.37;
const unsigned int impulsy_na_obrot = 40000;
unsigned long poprzedni_czas=0;
float Dystans=0;

float dystans(long int liczba_impulsow){
  return 2*3.14*promien*liczba_impulsow/impulsy_na_obrot;
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(enkL_A_pin), enkL_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enkL_B_pin), enkL_B_ISR, CHANGE);
  pinMode(enkL_A_pin, INPUT_PULLUP);
  pinMode(enkL_B_pin, INPUT_PULLUP);
  pinMode(enkP_A_pin, INPUT_PULLUP);
  pinMode(enkP_B_pin, INPUT_PULLUP);

}

void loop(){
  if(millis()-poprzedni_czas>500){
    poprzedni_czas=millis();
    long int count = encoderCount;
    encoderCount=0;
    Dystans += dystans(count);
    float predkosc = dystans(count)/500/10;
    Serial.println(count);
    Serial.println(Dystans);
    Serial.println(predkosc);
  }

}
void enkL_A_ISR(void){
  bool A = digitalRead(enkL_A_pin);
  bool B = digitalRead(enkL_B_pin);
  if (A == B) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void enkL_B_ISR(void){
  bool A = digitalRead(enkL_A_pin);
  bool B = digitalRead(enkL_B_pin);
  if (A != B) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}