#include <Arduino.h>

// ====== PINES MOTORES ======
const int IN1 = 12; // Motor Derecho
const int IN2 = 14;
const int IN3 = 26; // Motor Izquierdo
const int IN4 = 27;
const int ENA = 25; // PWM Derecho
const int ENB = 33; // PWM Izquierdo

// ====== CONFIGURACIÓN PWM ======
const int freq       = 1000;
const int resolution = 8;
const int canalDer   = 0;
const int canalIzq   = 1;

// ====== CONFIGURACIÓN ENCODERS ======
const int encoderR = 13; 
const int encoderL = 35; 
const int PPR      = 20; 
const unsigned long DEBOUNCE_US = 6000; // Ajustado para captar pulsos rápidos

// ====== VARIABLES DE ESTADO (ISR) ======
volatile long countR = 0;
volatile long countL = 0;
volatile unsigned long lastTimeR = 0;
volatile unsigned long lastTimeL = 0;

// ====== VARIABLES DE CONTROL ======
float setpointW = 12.0; // Radianes por segundo objetivo
float Kp = 0.6;         // Ganancia proporcional (reducida para evitar oscilaciones)
float pwmR = 75.0;      // Pre-carga inicial
float pwmL = 75.0;      // Pre-carga inicial

// ====== INTERRUPCIONES ======
void IRAM_ATTR isrR() {
  unsigned long now = micros();
  if (now - lastTimeR > DEBOUNCE_US) {
    countR++;
    lastTimeR = now;
  }
}

void IRAM_ATTR isrL() {
  unsigned long now = micros();
  if (now - lastTimeL > DEBOUNCE_US) {
    countL++;
    lastTimeL = now;
  }
}

// ====== FUNCIÓN DE CONTROL DE MOTORES ======
void aplicarMotores(int pR, int pL) {
  // Garantizar que el PWM físico nunca sea ilegal
  pR = constrain(pR, 0, 255);
  pL = constrain(pL, 0, 255);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(canalDer, pR);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(canalIzq, pL);
}

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

  Serial.println("--- Sistema de Control Reiniciado (Anti-Saturación) ---");
  delay(1000);
}

void loop() {
  static unsigned long lastControl = 0;
  static long lastCountR = 0;
  static long lastCountL = 0;
  unsigned long now = millis();

  // Ciclo de control cada 50ms
  if (now - lastControl >= 50) {
    long currentR, currentL;

    // Captura segura de contadores
    noInterrupts();
    currentR = countR;
    currentL = countL;
    interrupts();

    float dt = (now - lastControl) / 1000.0;

    // Cálculo de velocidades reales
    float wR = ((currentR - lastCountR) / (float)PPR / dt) * 2.0 * PI;
    float wL = ((currentL - lastCountL) / (float)PPR / dt) * 2.0 * PI;

    // Actualización de PWM con Lazo Cerrado
    pwmR += (setpointW - wR) * Kp;
    pwmL += (setpointW - wL) * Kp;

    // --- BLOQUE ANTI-SATURACIÓN (Indispensable) ---
    // Esto evita que las variables suban a 5000 como te pasó antes
    pwmR = constrain(pwmR, 0, 255);
    pwmL = constrain(pwmL, 0, 255);

    // Enviar señal a los motores
    aplicarMotores((int)pwmR, (int)pwmL);

    // Monitor Serial para depuración
    Serial.print("Target:"); Serial.print(setpointW, 1);
    Serial.print(" | wR:"); Serial.print(wR, 2);
    Serial.print(" | wL:"); Serial.print(wL, 2);
    Serial.print(" | PWM_R:"); Serial.print((int)pwmR);
    Serial.print(" | PWM_L:"); Serial.println((int)pwmL);

    // Guardar historial
    lastCountR = currentR;
    lastCountL = currentL;
    lastControl = now;
  }
}