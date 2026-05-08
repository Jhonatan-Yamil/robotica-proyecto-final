#include <Arduino.h>
// #include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>

WiFiServer server(1234);
WiFiClient client;

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth no habilitado
// #endif

// BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;
TaskHandle_t TareaTelemetria;

// ============================================================
// PARÁMETROS
// ============================================================
const float R = 3.2;      // radio rueda [cm]
const float L = 14.0;     // distancia entre ruedas [cm]

// ============================================================
// ESTADO Y NAVEGACIÓN
// ============================================================
bool sistemaActivo = false;

float targetX = 0.0;
float targetY = 25.0;

float x_pos = 0.0;
float y_pos = 0.0;
float theta = PI / 2.0;

float wRref = 0.0;
float wLref = 0.0;

float gyro_offset = 0.0;

// 🔴 TUS NUEVOS PARÁMETROS
float Kv = 0.30; 
float Kw = 0.10;
float V_MAX = 1.5;
float W_MAX = 1.5;

// ============================================================
// VARIABLES ENVÍO BT
// ============================================================
volatile float env_wRref = 0;
volatile float env_wR = 0;
volatile float env_x = 0;
volatile float env_y = 0;
volatile float env_wL = 0;
volatile float env_wLref = 0;
volatile float env_dist = 0;
volatile float env_errTheta = 0;
volatile float env_wGyro = 0;

volatile bool hayNuevosDatos = false;

// ============================================================
// PINES MOTORES Y OFFSETS
// ============================================================
const int IN1 = 26; const int IN2 = 27;
const int IN3 = 12; const int IN4 = 14;
const int ENA = 33; const int ENB = 25;

const int canalR = 0; const int canalL = 1;
const int freq = 1000; const int resolution = 8;

int dirR = 1; int dirL = 1;

// 🔴 OFFSETS DINÁMICOS (Si necesitas 90, puedes cambiarlo aquí o enviar OR90 / OL90)
int offsetR = 85;
int offsetL = 100;

// ============================================================
// ENCODERS
// ============================================================
const int encoderR = 32; const int encoderL = 13;
const int PPR = 200;

volatile long countR = 0; volatile long countL = 0;
volatile unsigned long lastTimeR = 0; volatile unsigned long lastTimeL = 0;
const unsigned long DEBOUNCE_US = 1500;

// ============================================================
// PID
// ============================================================
float KpR = 0.3; float KiR = 1.2; float KdR = -0.1;
float KpL = 0.5; float KiL = 3.0; float KdL = -0.05;

float integralR = 0; float integralL = 0;
float prev_wRe = 0; float prev_wLe = 0;
const float integralMax = 80;     // Reducido para evitar que el robot "patine"
const float DIST_TOLERANCE = 1.5; // Detenerse a 2.5cm
const float DECEL_DIST = 10.0;    // Empezar a frenar a los 25cm
// ============================================================
// INTERRUPCIONES
// ============================================================
void IRAM_ATTR isrR() {
    unsigned long now = micros();
    if (now - lastTimeR > DEBOUNCE_US) {
        countR++; lastTimeR = now;
    }
}

void IRAM_ATTR isrL() {
    unsigned long now = micros();
    if (now - lastTimeL > DEBOUNCE_US) {
        countL++; lastTimeL = now;
    }
}

// ============================================================
// CONTROL MOTORES
// ============================================================
void motorR(bool adelante, int pwm) {
    dirR = adelante ? 1 : -1;
    int offsetEfectivo = adelante ? offsetR : offsetR + 20;
    int pwmFinal = (pwm > 0) ? constrain(pwm + offsetEfectivo, 0, 255) : 0;
    digitalWrite(IN1, adelante ? LOW : HIGH);
    digitalWrite(IN2, adelante ? HIGH : LOW);
    ledcWrite(canalR, pwmFinal);
}

void motorL(bool adelante, int pwm) {
    dirL = adelante ? 1 : -1;
    int offsetEfectivo = adelante ? offsetL : offsetL + 20; // más empuje en reversa
    int pwmFinal = (pwm > 0) ? constrain(pwm + offsetEfectivo, 0, 255) : 0;
    digitalWrite(IN3, adelante ? LOW : HIGH);
    digitalWrite(IN4, adelante ? HIGH : LOW);
    ledcWrite(canalL, pwmFinal);
}

// ============================================================
// COMANDOS BLUETOOTH
// ============================================================
// void leerComandos() {
//     if (!SerialBT.available()) return;
//     String comando = SerialBT.readStringUntil('\n'); comando.trim();
//     if (comando.length() < 2) return;

//     String tipo = comando.substring(0, 2);
//     float valor = comando.substring(2).toFloat();

//     if (tipo == "GO") {
//         if (valor > 0.5) {
//             SerialBT.println("Iniciando en 3 segundos...");
//             delay(3000);
//             x_pos = 0; y_pos = 0; theta = PI / 2.0;
//             integralR = 0; integralL = 0; prev_wRe = 0; prev_wLe = 0;
//             sistemaActivo = true;
//         } else {
//             sistemaActivo = false;
//             motorR(true, 0); motorL(true, 0);
//             SerialBT.println("Sistema detenido");
//         }
//     }
//     // Coordenadas
//     else if (tipo == "TX") { targetX = valor; SerialBT.println("Target X: " + String(targetX)); }
//     else if (tipo == "TY") { targetY = valor; SerialBT.println("Target Y: " + String(targetY)); }
    
//     // Navegación
//     else if (tipo == "KV") { Kv = valor; SerialBT.println("Kv: " + String(Kv)); }
//     else if (tipo == "KW") { Kw = valor; SerialBT.println("Kw: " + String(Kw)); }
//     else if (tipo == "VM") { V_MAX = valor; SerialBT.println("V_MAX: " + String(V_MAX)); }
//     else if (tipo == "WM") { W_MAX = valor; SerialBT.println("W_MAX: " + String(W_MAX)); }
    
//     // Offsets y PID
//     else if (tipo == "OR") { offsetR = (int)valor; SerialBT.println("Offset R: " + String(offsetR)); }
//     else if (tipo == "OL") { offsetL = (int)valor; SerialBT.println("Offset L: " + String(offsetL)); }
//     else if (tipo == "PR") KpR = valor; else if (tipo == "PL") KpL = valor;
//     else if (tipo == "IR") KiR = valor; else if (tipo == "IL") KiL = valor;
//     else if (tipo == "DR") KdR = valor; else if (tipo == "DL") KdL = valor;
// }

// ============================================================
// TAREA BT
// ============================================================
void tareaBT(void *pvParameters) {
    for (;;) {
        // =====================================
        // RECIBIR COMANDOS WIFI
        // =====================================
        if (client && client.connected() && client.available()) {

            String cmd = client.readStringUntil('\n');
            cmd.trim();

            Serial.println(cmd);

            // START
            if (cmd == "START") {

                x_pos = 0;
                y_pos = 0;
                theta = PI / 2.0;

                integralR = 0;
                integralL = 0;

                prev_wRe = 0;
                prev_wLe = 0;

                sistemaActivo = true;
            }

            // STOP
            else if (cmd == "STOP") {

                sistemaActivo = false;

                motorR(true, 0);
                motorL(true, 0);
            }

            else if (cmd == "RESET") {
                x_pos = 0;
                y_pos = 0;
                theta = PI / 2.0;
                integralR = 0;
                integralL = 0;
                prev_wRe = 0;
                prev_wLe = 0;
                wRref = 0;
                wLref = 0;
                sistemaActivo = false;
                motorR(true, 0);
                motorL(true, 0);
            }

            // TARGET X
            else if (cmd.startsWith("TX:")) {
                targetX = cmd.substring(3).toFloat();
            }

            // TARGET Y
            else if (cmd.startsWith("TY:")) {
                targetY = cmd.substring(3).toFloat();
            }

            // Kv
            else if (cmd.startsWith("KV:")) {
                Kv = cmd.substring(3).toFloat();
            }

            // Kw
            else if (cmd.startsWith("KW:")) {
                Kw = cmd.substring(3).toFloat();
            }

            // V_MAX
            else if (cmd.startsWith("VM:")) {
                V_MAX = cmd.substring(3).toFloat();
            }

            // W_MAX
            else if (cmd.startsWith("WM:")) {
                W_MAX = cmd.substring(3).toFloat();
            }

            // OFFSET R
            else if (cmd.startsWith("OR:")) {
                offsetR = cmd.substring(3).toInt();
            }

            // OFFSET L
            else if (cmd.startsWith("OL:")) {
                offsetL = cmd.substring(3).toInt();
            }
        }
        // =====================================================
        // RECIBIR COMANDOS DESDE SERIAL STUDIO
        // =====================================================
        // if (SerialBT.available()) {

        //     String cmd = SerialBT.readStringUntil('\n');
        //     cmd.trim();

        //     // START
        //     if (cmd == "START") {
        //         x_pos = 0;
        //         y_pos = 0;
        //         theta = PI / 2.0;

        //         integralR = 0;
        //         integralL = 0;

        //         prev_wRe = 0;
        //         prev_wLe = 0;

        //         sistemaActivo = true;

        //         SerialBT.println("{\"status\":\"START\"}");
        //     }

        //     // STOP
        //     else if (cmd == "STOP") {

        //         sistemaActivo = false;

        //         motorR(true, 0);
        //         motorL(true, 0);

        //         SerialBT.println("{\"status\":\"STOP\"}");
        //     }

        //     // KV
        //     else if (cmd.startsWith("KV:")) {
        //         Kv = cmd.substring(3).toFloat();
        //     }

        //     // KW
        //     else if (cmd.startsWith("KW:")) {
        //         Kw = cmd.substring(3).toFloat();
        //     }

        //     // VM
        //     else if (cmd.startsWith("VM:")) {
        //         V_MAX = cmd.substring(3).toFloat();
        //     }

        //     // WM
        //     else if (cmd.startsWith("WM:")) {
        //         W_MAX = cmd.substring(3).toFloat();
        //     }

        //     // TX
        //     else if (cmd.startsWith("TX:")) {
        //         targetX = cmd.substring(3).toFloat();
        //     }

        //     // TY
        //     else if (cmd.startsWith("TY:")) {
        //         targetY = cmd.substring(3).toFloat();
        //     }
        // }

        // =====================================================
        // ENVIAR TELEMETRÍA JSON
        // =====================================================
        // =====================================
        // CONEXIÓN CLIENTE WIFI
        // =====================================
        if (!client || !client.connected()) {
            client = server.available();
        }

        // =====================================
        // ENVIAR TELEMETRÍA
        // =====================================
        if (client && client.connected()) {

            client.printf(
                "{\"x\":%.2f,"
                "\"y\":%.2f,"
                "\"theta\":%.2f,"
                "\"wr\":%.2f,"
                "\"wl\":%.2f,"
                "\"wrref\":%.2f,"
                "\"wgyro\":%.3f,"
                "\"wlref\":%.2f,"
                "\"dist\":%.2f,"
                "\"errtheta\":%.2f,"
                "\"kv\":%.2f,"
                "\"kw\":%.2f,"
                "\"vm\":%.2f,"
                "\"wm\":%.2f,"
                "\"tx\":%.2f,"
                "\"ty\":%.2f}\n",
                x_pos, y_pos, theta,
                env_wR, env_wL,
                env_wRref, env_wGyro, env_wLref,
                env_dist, env_errTheta,
                Kv, Kw, V_MAX, W_MAX,
                targetX, targetY
            );
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    // SerialBT.begin("Auto_ESP32");
    Wire.begin(18, 19);

    if (!mpu.begin()) {
        Serial.println("MPU6050 Error");
        while (1) delay(10);
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    Serial.println("Calibrando gyro...");
    Serial.println("NO mover el robot");
    float suma = 0;
    for (int i = 0; i < 500; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        // 🔴 EJE CORREGIDO PARA CALIBRACIÓN
        suma += g.gyro.x; 
        
        delay(3);
    }
    gyro_offset = suma / 500.0;
    Serial.printf("Gyro offset calculado: %.4f\n", gyro_offset);
    
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    ledcSetup(canalR, freq, resolution); ledcSetup(canalL, freq, resolution);
    ledcAttachPin(ENA, canalR); ledcAttachPin(ENB, canalL);

    pinMode(encoderR, INPUT_PULLUP); pinMode(encoderL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderR), isrR, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderL), isrL, RISING);

    xTaskCreatePinnedToCore(tareaBT, "TareaBT", 4096, NULL, 1, &TareaTelemetria, 0);
    const char* ssid = "RobotESP32";
    const char* password = "12345678";

    WiFi.softAP(ssid, password);
    server.begin();
    
    Serial.println("Servidor TCP iniciado");
    Serial.println("Servidor listo");

    IPAddress IP = WiFi.softAPIP();

    Serial.println(IP);
    Serial.println("Listo");
    // SerialBT.println("Listo");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    // leerComandos();

    static long lastCountR = 0;
    static long lastCountL = 0;
    static unsigned long lastTime = millis();

    if (!sistemaActivo) {
        motorR(true, 0); motorL(true, 0);
        delay(20);
        return;
    }

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    if (dt >= 0.05) {
        noInterrupts();
        long r = countR; long l = countL;
        interrupts();

        float wR = dirR * (((r - lastCountR) * 2.0 * PI) / (PPR * dt));
        float wL = dirL * (((l - lastCountL) * 2.0 * PI) / (PPR * dt));

        // ====================================================
        // LECTURA DEL MPU (EJE X CORREGIDO)
        // ====================================================
        sensors_event_t a, g, temp;
        float w_gyro = 0.0;
        if (mpu.getEvent(&a, &g, &temp)) {
            float raw = g.gyro.x - gyro_offset;
            if (abs(raw) < 0.05) raw = 0.0;
            w_gyro = raw;
        }
        env_wGyro = w_gyro;
        float w_odom = (wR - wL) * R / L;
        float w_fusion = 0.7 * w_odom + 0.3 * w_gyro;
        theta += w_fusion * dt;

        // ====================================================
        // POSICIÓN
        // ====================================================
        float v_actual = (wR + wL) * R / 2.0;
        x_pos += v_actual * cos(theta) * dt;
        y_pos += v_actual * sin(theta) * dt;
        // ====================================================
        // NAVEGACIÓN GO-TO-GOAL
        // ====================================================
        float dx = targetX - x_pos;
        float dy = targetY - y_pos;
        float dist = sqrt(dx * dx + dy * dy);
        
        float angleToTarget = atan2(dy, dx);
        float errorTheta = angleToTarget - theta;
        while (errorTheta > PI) errorTheta -= 2 * PI;
        while (errorTheta < -PI) errorTheta += 2 * PI;
        
        float v_deseada = 0;
        float w_deseada = 0;
        
        if (dist > DIST_TOLERANCE) {
            // Siempre avanza, solo ajusta w según el error angular
            float v_frenado = (dist < DECEL_DIST) ? (Kv * dist) : V_MAX;
            v_deseada = constrain(v_frenado, 0.0, V_MAX);
            w_deseada = Kw * errorTheta;

            // Solo gira en sitio si el error es MUY grande (casi 90°)
            if (abs(errorTheta) > 2.5) {
                v_deseada = 0;
            }

            v_deseada = constrain(v_deseada, 0, V_MAX);
            w_deseada = constrain(w_deseada, -W_MAX, W_MAX);
        }else {
            // META ALCANZADA — parar incondicionalmente
            sistemaActivo = false;
            v_deseada = 0; w_deseada = 0;
            wRref = 0; wLref = 0;
            integralR = 0; integralL = 0;       // ← agregar esto
            prev_wRe = 0; prev_wLe = 0;         // ← y esto
            motorR(true, 0); motorL(true, 0);
            Serial.println(">> META ALCANZADA");
        }

        // Cinemática Inversa
        wRref = (2.0 * v_deseada + w_deseada * L) / (2.0 * R);
        wLref = (2.0 * v_deseada - w_deseada * L) / (2.0 * R);

        // Control PID Clásico (Sin Anti-Windup forzado)
        float wRe = wRref - wR;
        float wLe = wLref - wL;

        // Anti-Windup: Solo suma error a la integral si el motor no está al máximo (255)
        static float salR_prev = 0;
        static float salL_prev = 0;

        if (abs(salR_prev) < 255) {
            integralR = constrain(integralR + wRe * dt, -integralMax, integralMax);
        }
        if (abs(salL_prev) < 255) {
            integralL = constrain(integralL + wLe * dt, -integralMax, integralMax);
        }
        float salR = KpR * wRe + KiR * integralR + KdR * ((wRe - prev_wRe) / dt);
        float salL = KpL * wLe + KiL * integralL + KdL * ((wLe - prev_wLe) / dt);
        salR_prev = salR;
        salL_prev = salL;

        // ← AQUÍ va el bloque nuevo, reemplazando las dos líneas viejas
        static int ciclosSinMovL = 0;
        ciclosSinMovL = (abs(wLref) > 0.1 && abs(wL) < 0.05) ? ciclosSinMovL + 1 : 0;
        int kickL = (ciclosSinMovL > 2) ? 40 : 0;

        motorR(salR >= 0, constrain(abs((int)salR), 0, 255));
        motorL(salL >= 0, constrain(abs((int)salL) + kickL, 0, 255));

        env_wR = wR;
        env_wL = wL;           // ← nuevo
        env_wRref = wRref;
        env_wLref = wLref;     // ← nuevo
        env_dist = dist;       // ← nuevo
        env_errTheta = errorTheta; // ← nuevo
        env_x = x_pos; env_y = y_pos;
        hayNuevosDatos = true;

        lastCountR = r; lastCountL = l;
        lastTime = now;
        prev_wRe = wRe; prev_wLe = wLe;

        Serial.printf(
        "dist=%.2f "
        "theta=%.2f "
        "targetAng=%.2f "
        "errTheta=%.2f "
        "v=%.2f "
        "w=%.2f "
        "wR=%.2f "
        "wL=%.2f "
        "wRref=%.2f "
        "wLref=%.2f\n",
        dist,
        theta,
        angleToTarget,
        errorTheta,
        v_deseada,
        w_deseada,
        wR,
        wL,
        wRref,
        wLref
        );
    }
    delay(10);
}