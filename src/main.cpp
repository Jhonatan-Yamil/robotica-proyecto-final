#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth no habilitado
#endif

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

// ============================================================
// PARÁMETROS FÍSICOS
// ============================================================
const float R = 3.2;      // radio rueda [cm]
const float L = 14.0;     // distancia entre ruedas [cm]
const int PPR = 200;

// ============================================================
// PINES
// ============================================================
const int IN1 = 26;
const int IN2 = 27;
const int IN3 = 12;
const int IN4 = 14;

const int ENA = 33;
const int ENB = 25;

const int encoderR = 32;
const int encoderL = 13;

// ============================================================
// PWM
// ============================================================
const int canalR = 0;
const int canalL = 1;
const int freq = 1000;
const int resolution = 8;

// ============================================================
// ESTADO ROBOT
// ============================================================
bool sistemaActivo = false;

float x_pos = 0;
float y_pos = 0;

// Robot inicia mirando hacia +Y
float theta = PI / 2.0;

float targetX = 0;
float targetY = 100;

// ============================================================
// CONTROL
// ============================================================
float Kv = 0.4;
float Kw = 0.55;

float V_MAX = 1;
float W_MAX = 0.6;

// ============================================================
// PID
// ============================================================
float KpR = 0.35;
float KiR = 1;
float KdR = 0;

float KpL = 0.35;
float KiL = 1;
float KdL = 0;

float integralR = 0;
float integralL = 0;

float prevErrR = 0;
float prevErrL = 0;

const float integralMax = 150;

// ============================================================
// OFFSETS
// ============================================================
int offsetR = 95;
int offsetL = 91.4;

// ============================================================
// ENCODERS
// ============================================================
volatile long countR = 0;
volatile long countL = 0;

volatile unsigned long lastTimeR = 0;
volatile unsigned long lastTimeL = 0;

const unsigned long DEBOUNCE_US = 3000;

// ============================================================
// DIRECCIÓN
// ============================================================
int dirR = 1;
int dirL = 1;

// ============================================================
// MPU
// ============================================================
float gyro_offset = 0;

// ============================================================
// INTERRUPCIONES
// ============================================================
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

// ============================================================
// MOTORES
// ============================================================
void motorR(bool adelante, int pwm) {

    dirR = adelante ? 1 : -1;

    int pwmFinal = (pwm > 0)
        ? constrain(pwm + offsetR, 0, 255)
        : 0;

    digitalWrite(IN1, adelante ? LOW : HIGH);
    digitalWrite(IN2, adelante ? HIGH : LOW);

    ledcWrite(canalR, pwmFinal);
}

void motorL(bool adelante, int pwm) {

    dirL = adelante ? 1 : -1;

    int pwmFinal = (pwm > 0)
        ? constrain(pwm + offsetL, 0, 255)
        : 0;

    digitalWrite(IN3, adelante ? LOW : HIGH);
    digitalWrite(IN4, adelante ? HIGH : LOW);

    ledcWrite(canalL, pwmFinal);
}

// ============================================================
// BLUETOOTH
// ============================================================
void leerBT() {

    if (!SerialBT.available()) return;

    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() < 2) return;

    String tipo = cmd.substring(0, 2);
    float valor = cmd.substring(2).toFloat();

    // =====================================
    // START / STOP
    // =====================================
    if (tipo == "GO") {

        if (valor > 0.5) {

            SerialBT.println("INICIO EN 3s");
            delay(3000);

            x_pos = 0;
            y_pos = 0;
            theta = PI / 2.0;

            integralR = 0;
            integralL = 0;

            prevErrR = 0;
            prevErrL = 0;

            sistemaActivo = true;
        }
        else {

            sistemaActivo = false;

            motorR(true, 0);
            motorL(true, 0);

            SerialBT.println("STOP");
        }
    }

    // =====================================
    // TARGETS
    // =====================================
    else if (tipo == "TX") targetX = valor;
    else if (tipo == "TY") targetY = valor;

    // =====================================
    // NAVEGACIÓN
    // =====================================
    else if (tipo == "KV") Kv = valor;
    else if (tipo == "KW") Kw = valor;
    else if (tipo == "VM") V_MAX = valor;
    else if (tipo == "WM") W_MAX = valor;

    // =====================================
    // OFFSETS
    // =====================================
    else if (tipo == "OR") offsetR = valor;
    else if (tipo == "OL") offsetL = valor;

    // =====================================
    // PID
    // =====================================
    else if (tipo == "PR") KpR = valor;
    else if (tipo == "IR") KiR = valor;
    else if (tipo == "DR") KdR = valor;

    else if (tipo == "PL") KpL = valor;
    else if (tipo == "IL") KiL = valor;
    else if (tipo == "DL") KdL = valor;
}

// ============================================================
// SETUP
// ============================================================
void setup() {

    Serial.begin(115200);

    SerialBT.begin("Auto_ESP32");

    Wire.begin(18, 19);

    // =====================================
    // MPU
    // =====================================
    if (!mpu.begin()) {

        Serial.println("ERROR MPU6050");

        while (1);
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    Serial.println("Calibrando MPU...");

    float suma = 0;

    for (int i = 0; i < 500; i++) {

        sensors_event_t a, g, temp;

        mpu.getEvent(&a, &g, &temp);

        // EJE X = GIRO
        suma += g.gyro.x;

        delay(3);
    }

    gyro_offset = suma / 500.0;

    // =====================================
    // MOTORES
    // =====================================
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcSetup(canalR, freq, resolution);
    ledcSetup(canalL, freq, resolution);

    ledcAttachPin(ENA, canalR);
    ledcAttachPin(ENB, canalL);

    // =====================================
    // ENCODERS
    // =====================================
    pinMode(encoderR, INPUT_PULLUP);
    pinMode(encoderL, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(encoderR),
        isrR,
        RISING
    );

    attachInterrupt(
        digitalPinToInterrupt(encoderL),
        isrL,
        RISING
    );

    Serial.println("LISTO");
    SerialBT.println("LISTO");
}

// ============================================================
// LOOP
// ============================================================
void loop() {

    leerBT();

    static long lastCountR = 0;
    static long lastCountL = 0;

    static unsigned long lastTime = millis();

    if (!sistemaActivo) {

        motorR(true, 0);
        motorL(true, 0);

        delay(20);
        return;
    }

    unsigned long now = millis();

    float dt = (now - lastTime) / 1000.0;

    // CONTROL CADA 20 ms
    if (dt >= 0.02) {

        // =====================================
        // ENCODERS
        // =====================================
        noInterrupts();

        long r = countR;
        long l = countL;

        interrupts();

        float wR =
            dirR *
            (((r - lastCountR) * 2.0 * PI) / (PPR * dt));

        float wL =
            dirL *
            (((l - lastCountL) * 2.0 * PI) / (PPR * dt));

        // =====================================
        // MPU
        // =====================================
        sensors_event_t a, g, temp;

        mpu.getEvent(&a, &g, &temp);

        float w_gyro = g.gyro.x - gyro_offset;

        // DEAD BAND
        if (abs(w_gyro) < 0.03) {
            w_gyro = 0;
        }

        // FILTRO
        static float w_filtrado = 0;

        w_filtrado =
            0.8 * w_filtrado +
            0.2 * w_gyro;

        theta += w_filtrado * dt;

        // NORMALIZAR
        while (theta > PI) theta -= 2 * PI;
        while (theta < -PI) theta += 2 * PI;

        // =====================================
        // ODOMETRÍA
        // =====================================
        float v_actual = (wR + wL) * R / 2.0;

        x_pos += v_actual * cos(theta) * dt;
        y_pos += v_actual * sin(theta) * dt;

        // =====================================
        // NAVEGACIÓN
        // =====================================
        float dx = targetX - x_pos;
        float dy = targetY - y_pos;

        float dist = sqrt(dx * dx + dy * dy);

        float angleTarget = atan2(dy, dx);

        float errorTheta = angleTarget - theta;

        while (errorTheta > PI) errorTheta -= 2 * PI;
        while (errorTheta < -PI) errorTheta += 2 * PI;

        float v_des = 0;
        float w_des = 0;

       float errorX = abs(targetX - x_pos);
float errorY = abs(targetY - y_pos);

// ====================================================
// CONDICIÓN DE LLEGADA
// ====================================================

if (errorX > 4 || errorY > 4) {

    // PRIORIDAD DE GIRO
    if (abs(errorTheta) > 0.5) {
        v_des = 0;
    }
    else {
        v_des = Kv * dy;
    }

    w_des = Kw * errorTheta;

    v_des = constrain(v_des, 0, V_MAX);
    w_des = constrain(w_des, -W_MAX, W_MAX);
}
else {

    sistemaActivo = false;

    motorR(true, 0);
    motorL(true, 0);

    SerialBT.println("META ALCANZADA");
}

        // =====================================
        // CINEMÁTICA INVERSA
        // =====================================
        float wRref =
            (2.0 * v_des + w_des * L)
            / (2.0 * R);

        float wLref =
            (2.0 * v_des - w_des * L)
            / (2.0 * R);

        // =====================================
        // PID
        // =====================================
        float errR = wRref - wR;
        float errL = wLref - wL;

        integralR += errR * dt;
        integralL += errL * dt;

        integralR =
            constrain(integralR, -integralMax, integralMax);

        integralL =
            constrain(integralL, -integralMax, integralMax);

        float salR =
            KpR * errR +
            KiR * integralR +
            KdR * ((errR - prevErrR) / dt);

        float salL =
            KpL * errL +
            KiL * integralL +
            KdL * ((errL - prevErrL) / dt);

        motorR(
            salR >= 0,
            constrain(abs((int)salR), 0, 255)
        );

        motorL(
            salL >= 0,
            constrain(abs((int)salL), 0, 255)
        );

        // =====================================
        // TELEMETRÍA
        // =====================================
        SerialBT.printf(
            "X:%.2f,Y:%.2f,TH:%.2f,TX:%.2f,TY:%.2f\n",
            x_pos,
            y_pos,
            theta,
            targetX,
            targetY
        );

        Serial.printf(
            "X=%.2f Y=%.2f TH=%.2f\n",
            x_pos,
            y_pos,
            theta
        );

        // =====================================
        // UPDATE
        // =====================================
        prevErrR = errR;
        prevErrL = errL;

        lastCountR = r;
        lastCountL = l;

        lastTime = now;
    }
}