#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

// ============================================================
// PARÁMETROS FÍSICOS
// ============================================================
const float R = 3.2;      
const float L = 14.0;     
const int PPR = 200;

// ============================================================
// PINES Y PWM
// ============================================================
const int IN1 = 26; const int IN2 = 27;
const int IN3 = 12; const int IN4 = 14;
const int ENA = 33; const int ENB = 25;
const int encoderR = 32; const int encoderL = 13;
const int canalR = 0; const int canalL = 1;
const int freq = 1000; const int resolution = 8;

// ============================================================
// ESTADO, POSICIÓN Y TARGETS
// ============================================================
bool sistemaActivo = false;
int etapa = 0; 
unsigned long tiempoMeta1 = 0;

float x_pos = 0; float y_pos = 0;
float theta = PI / 2.0; 
float targetX = 0; float targetY = 19.0; 

// ============================================================
// CONTROL DE NAVEGACIÓN (AJUSTADO PARA REDUCIR DERIVA Y OSCILACIÓN)
// ============================================================
float Kv = 0.3;      // Reducido ligeramente para mayor suavidad
float Kw = 1.8;      // Bajado un poco para evitar el zigzagueo excesivo al final
float V_MAX = 4.5;    // Bajado para ganar precisión
float V_MIN = 2;    // Bajado para que el arranque sea menos brusco
float W_MAX = 1.5;    

// ============================================================
// PID MOTORES
// ============================================================
float KpR = 10.0; float KiR = 0.55; 
float KpL = 10.0; float KiL = 0.55;
float integralR = 0, integralL = 0;
const float integralMax = 65; 

// AJUSTE DE OFFSETS: Acercados para evitar la curva inicial a la derecha
int offsetR = 105; int offsetL = 96; 

volatile long countR = 0; volatile long countL = 0;
volatile unsigned long lastTimeR = 0; volatile unsigned long lastTimeL = 0;
const unsigned long DEBOUNCE_US = 2000;
int dirR = 1; int dirL = 1;
float gyro_offset = 0;

void IRAM_ATTR isrR() {
    unsigned long now = micros();
    if (now - lastTimeR > DEBOUNCE_US) { countR++; lastTimeR = now; }
}
void IRAM_ATTR isrL() {
    unsigned long now = micros();
    if (now - lastTimeL > DEBOUNCE_US) { countL++; lastTimeL = now; }
}

int aplicarOffsetPWM(int pwm, int offset) {
    if (pwm <= 0) return 0;
    return map(pwm, 0, 255, offset, 255);
}

void motorR(bool adelante, int pwm) {
    dirR = adelante ? 1 : -1;
    ledcWrite(canalR, aplicarOffsetPWM(pwm, offsetR));
    digitalWrite(IN1, adelante ? LOW : HIGH);
    digitalWrite(IN2, adelante ? HIGH : LOW);
}

void motorL(bool adelante, int pwm) {
    dirL = adelante ? 1 : -1;
    ledcWrite(canalL, aplicarOffsetPWM(pwm, offsetL));
    digitalWrite(IN3, adelante ? LOW : HIGH);
    digitalWrite(IN4, adelante ? HIGH : LOW);
}

void detenerRobot() {
    ledcWrite(canalR, 0); ledcWrite(canalL, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    integralR = 0; integralL = 0;
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("Robot_ESP32_V2");
    Wire.begin(18, 19);
    mpu.begin();
    
    float suma = 0;
    for (int i = 0; i < 400; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        suma += g.gyro.x;
        delay(2);
    }
    gyro_offset = suma / 400.0;

    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    ledcSetup(canalR, freq, resolution); ledcSetup(canalL, freq, resolution);
    ledcAttachPin(ENA, canalR); ledcAttachPin(ENB, canalL);

    pinMode(encoderR, INPUT_PULLUP); pinMode(encoderL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderR), isrR, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderL), isrL, RISING);
}

void loop() {
    if (SerialBT.available()) {
        String cmd = SerialBT.readStringUntil('\n');
        cmd.trim();
        if (cmd == "GO") {
            x_pos = 0; y_pos = 0; theta = PI / 2.0;
            targetX = 0; targetY = 17.0;
            etapa = 1; sistemaActivo = true;
        } else if (cmd == "STOP") {
            sistemaActivo = false; detenerRobot();
        }
    }

    if (!sistemaActivo) return;

    static long lastCountR = 0; static long lastCountL = 0;
    static unsigned long lastTime = millis();

    if (etapa == 2) {
        detenerRobot();
        if (millis() - tiempoMeta1 >= 2000) {
            x_pos = 0; y_pos = 0; 
            targetX = 4.5; targetY = 6.4; 
            Kv = 0.30; V_MAX = 5.0; V_MIN = 2; 
            Kw = 1.2; 
            etapa = 3;
        }
        return;
    }

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    if (dt >= 0.02) { 
        noInterrupts();
        long r = countR; long l = countL;
        interrupts();

        float wR = dirR * (((r - lastCountR) * 2.0 * PI) / (PPR * dt));
        float wL = dirL * (((l - lastCountL) * 2.0 * PI) / (PPR * dt));

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        float w_gyro = g.gyro.x - gyro_offset;
        if (abs(w_gyro) < 0.01) w_gyro = 0;

        theta += w_gyro * dt;
        while (theta > PI) theta -= 2 * PI;
        while (theta < -PI) theta += 2 * PI;

        float v_actual = (wR + wL) * R / 2.0;
        x_pos += v_actual * cos(theta) * dt;
        y_pos += v_actual * sin(theta) * dt;

        float dx = targetX - x_pos;
        float dy = targetY - y_pos;
        float dist = sqrt(dx * dx + dy * dy);
        float angleTarget = atan2(dy, dx);
        float errorTheta = angleTarget - theta;
        while (errorTheta > PI) errorTheta -= 2 * PI;
        while (errorTheta < -PI) errorTheta += 2 * PI;

        if (dist < 0.5) { 
            if (etapa == 1) {
                detenerRobot();
                tiempoMeta1 = millis();
                etapa = 2;
                return;
            } else if (etapa == 3) {
                sistemaActivo = false; detenerRobot();
                SerialBT.println("LLEGADA FINAL");
                return;
            }
        }

        // --- CÁLCULO DE VELOCIDADES ---
        float w_des = Kw * errorTheta;
        w_des = constrain(w_des, -W_MAX, W_MAX);

        // Alineación más restrictiva (exponente 6) para asegurar trayectoria recta
        float alineacion = cos(errorTheta);
        if (alineacion < 0.6) alineacion = 0; 
        else alineacion = pow(alineacion, 6); 
        
        float v_des = (Kv * dist * alineacion) + V_MIN; 
        v_des = constrain(v_des, 0, V_MAX);

        // --- PID MOTORES ---
        float wRref = (2.0 * v_des + w_des * L) / (2.0 * R);
        float wLref = (2.0 * v_des - w_des * L) / (2.0 * R);

        float errR = wRref - wR; float errL = wLref - wL;
        integralR = constrain(integralR + errR * dt, -integralMax, integralMax);
        integralL = constrain(integralL + errL * dt, -integralMax, integralMax);

        float salR = (KpR * errR) + (KiR * integralR);
        float salL = (KpL * errL) + (KiL * integralL);

        motorR(salR >= 0, constrain(abs((int)salR), 0, 255));
        motorL(salL >= 0, constrain(abs((int)salL), 0, 255));

        SerialBT.printf("X:%.1f,Y:%.1f,TX:%.1f,TY:%.1f,TH:%.2f,D:%.1f,E:%.2f\n", 
                        x_pos, y_pos, targetX, targetY, theta, dist, errorTheta);

        lastCountR = r; lastCountL = l; lastTime = now;
    }
}