#include <Arduino.h>

// PINES MOTORES
const int IN1 = 12;
const int IN2 = 14;
const int IN3 = 26;
const int IN4 = 27;
const int ENA = 25;
const int ENB = 33;

// PWM 
const int freq       = 1000;
const int resolution = 8;
const int canalDer   = 0;
const int canalIzq   = 1;

// ENCODERS 
const int encoderR = 13;
const int encoderL = 35;
const int PPR      = 20;

const unsigned long DEBOUNCE_US = 10000;

// VARIABLES ISR — solo enteros
volatile unsigned long lastTimeR  = 0;
volatile unsigned long lastTimeL  = 0;
volatile unsigned long gapR       = 0; 
volatile unsigned long gapL       = 0;
volatile long          countR     = 0;
volatile long          countL     = 0;

void IRAM_ATTR isrR() {
  unsigned long now = micros();
  unsigned long g   = now - lastTimeR;
  if (g > DEBOUNCE_US) {
    gapR      = g;   
    countR++;
    lastTimeR = now;
  }
}

void IRAM_ATTR isrL() {
  unsigned long now = micros();
  unsigned long g   = now - lastTimeL;
  if (g > DEBOUNCE_US) {
    gapL      = g;
    countL++;
    lastTimeL = now;
  }
}

// MOTORES
void motorR(bool adelante, int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(IN1, adelante ? HIGH : LOW);
  digitalWrite(IN2, adelante ? LOW  : HIGH);
  ledcWrite(canalDer, pwm);
}

void motorL(bool adelante, int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(IN3, adelante ? HIGH : LOW);
  digitalWrite(IN4, adelante ? LOW  : HIGH);
  ledcWrite(canalIzq, pwm);
}

// SETUP
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcSetup(canalDer, freq, resolution);
  ledcSetup(canalIzq, freq, resolution);
  ledcAttachPin(ENA, canalDer);
  ledcAttachPin(ENB, canalIzq);

  pinMode(encoderR, INPUT_PULLUP);
  pinMode(encoderL, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderR), isrR, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderL), isrL, FALLING);
}
void loop() {
  motorR(true, 150);
  motorL(true, 0);

  static unsigned long lastPrint = 0;
  static long lastR = 0, lastL = 0;
  unsigned long now = millis();

  if (now - lastPrint >= 500) { 
    long r, l;
    noInterrupts();
    r = countR;
    l = countL;
    interrupts();

    float dt = (now - lastPrint) / 1000.0;
    
    // Se calcula  pulsos por segundo
    float ppsR = (r - lastR) / dt;
    float ppsL = (l - lastL) / dt;

    // Se convierte a Rad/s: (Pulsos_seg / PPR) * 2 * PI
    float wR = (ppsR / (float)PPR) * 2.0 * PI;
    float wL = (ppsL / (float)PPR) * 2.0 * PI;

    Serial.print("Rad/s -> R: "); Serial.print(wR);
    Serial.print(" | L: "); Serial.println(wL);

    lastR = r;
    lastL = l;
    lastPrint = now;
  }
}