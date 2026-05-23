#include "esp_camera.h"
#include <WiFi.h>
#include <ESP32Servo.h>
#include <math.h>


#include "board_config.h"
#include "camera_pins.h"


// =====================================
// WiFi
// =====================================
const char *ssid     = "Rodaina's Galaxy A34 5G";
const char *password = "12345678";


// =====================================
// Robot Leg Settings
// =====================================
const float L1 = 15.0;
const float L2 = 10.0;


Servo hipServo;
Servo kneeServo;


const int hipPin  = 14;
const int kneePin = 15;


float currentX = 7.5;
float currentY = -3.0;


// =====================================
// METRICS: FK/IK accuracy tracking
// =====================================
struct IKMetric {
    float target_x;
    float target_y;
    float fk_check_x;
    float fk_check_y;
    float fk_error_cm;
    float hip_deg;
    float knee_deg;
};


#define METRIC_BUF_SIZE 20
IKMetric ikMetrics[METRIC_BUF_SIZE];
int      metricCount = 0;


void logIKMetric(float tx, float ty, float fkx, float fky, float err, float hip, float knee)
{
    int idx = metricCount % METRIC_BUF_SIZE;
    ikMetrics[idx] = { tx, ty, fkx, fky, err, hip, knee };
    metricCount++;
}


void printMetricsSummary()
{
    if (metricCount == 0) {
        Serial.println("[METRICS] No IK solves recorded yet.");
        return;
    }
    int n = min(metricCount, METRIC_BUF_SIZE);
    float sumErr = 0;
    float maxErr = 0;
    for (int i = 0; i < n; i++) {
        sumErr += ikMetrics[i].fk_error_cm;
        if (ikMetrics[i].fk_error_cm > maxErr) maxErr = ikMetrics[i].fk_error_cm;
    }
    Serial.println("\n=== IK/FK Metrics Summary ===");
    Serial.printf("Total IK solves recorded : %d\n", metricCount);
    Serial.printf("Avg FK round-trip error  : %.4f cm\n", sumErr / n);
    Serial.printf("Max FK round-trip error  : %.4f cm\n", maxErr);
    Serial.println("=============================\n");
}


// =====================================
// Forward Kinematics (FK)
// =====================================
void forwardKinematics(float theta1, float theta2, float &x, float &y)
{
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
}


// =====================================
// Inverse Kinematics (IK)
// =====================================
void moveLegTo(float x, float y)
{
    float distSq = (x * x) + (y * y);
    float dist   = sqrt(distSq);


    if (dist > (L1 + L2) || dist < abs(L1 - L2)) {
        Serial.println("Target out of reach");
        return;
    }


    // Law of cosines → knee angle
    float cosTheta2 = (distSq - L1*L1 - L2*L2) / (2 * L1 * L2);
    cosTheta2 = constrain(cosTheta2, -1.0, 1.0);
    float theta2 = acos(cosTheta2);


    // Atan2 → hip angle
    float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));


    // FK validation
    float fkX, fkY;
    forwardKinematics(theta1, theta2, fkX, fkY);
    Serial.printf("IK target: (%.1f, %.1f) | FK check: (%.1f, %.1f)\n", x, y, fkX, fkY);


    float hip_deg  = theta1 * 180.0 / M_PI;
    float knee_deg = theta2 * 180.0 / M_PI;


    // ── METRICS: compute and log FK round-trip error ──
    float fkError = sqrt((fkX - x)*(fkX - x) + (fkY - y)*(fkY - y));
    Serial.printf("[METRICS] FK error: %.4f cm | Hip: %.2f deg | Knee: %.2f deg\n",
                  fkError, hip_deg, knee_deg);
    logIKMetric(x, y, fkX, fkY, fkError, hip_deg, knee_deg);
    // ──────────────────────────────────────────────────


    // Mapping derived from FK of known good angles:
    // wind-up (hip=150,knee=30)  → FK gave (7.5, -3.0)
    // strike  (hip=30, knee=150) → FK gave (7.5, 23.0)
    // hip servo:  0=front, 180=back → 90 - hip_deg
    // knee servo: 0=back,  180=front → 180 - knee_deg
    int hipAngle  = constrain((int)(90 - hip_deg),  0, 180);
    int kneeAngle = constrain((int)(180 - knee_deg), 0, 180);


    Serial.printf("Servo — Hip: %d deg  Knee: %d deg\n", hipAngle, kneeAngle);


    hipServo.write(hipAngle);
    kneeServo.write(kneeAngle);
}


// =====================================
// Smooth IK Motion
// =====================================
void smoothMoveTo(float targetX, float targetY, int steps, int stepDelay)
{
    float startX = currentX;
    float startY = currentY;


    for (int i = 1; i <= steps; i++) {
        float t = (float)i / steps;
        float s = t * t * (3.0 - 2.0 * t);
        moveLegTo(startX + (targetX - startX) * s,
                  startY + (targetY - startY) * s);
        delay(stepDelay);
    }


    currentX = targetX;
    currentY = targetY;
}


// =====================================
// Relax — natural standing pose
// =====================================
void relaxLeg()
{
    hipServo.write(90);
    kneeServo.write(90);
    currentX = 7.5;
    currentY = -3.0;
    Serial.println("LEG: relaxed");
}


// =====================================
// Kick using IK
// =====================================
void performKick()
{
    Serial.println("KICK START");


    // 1. Ensure relaxed first
    relaxLeg();
    delay(300);


    // 2. Wind-up: IK target (7.5, -3.0)
    smoothMoveTo(7.5, -3.0, 20, 10);
    delay(300);


    // 3. Power stroke: IK target (7.5, 23.0) — FAST
    smoothMoveTo(7.5, 23.0, 6, 2);
    delay(150);


    // 4. Recovery
    relaxLeg();


    // ── METRICS: print summary after each kick ────────
    printMetricsSummary();
    // ─────────────────────────────────────────────────


    Serial.println("KICK END");
}


// =====================================
// Camera Server Declaration
// =====================================
void startCameraServer();


// =====================================
// Setup
// =====================================
void setup()
{
    Serial.begin(115200);


    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);


    hipServo.setPeriodHertz(50);
    kneeServo.setPeriodHertz(50);
    hipServo.attach(hipPin,  500, 2400);
    kneeServo.attach(kneePin, 500, 2400);


    // Boot into relaxed pose
    relaxLeg();
    delay(1000);


    // --- Camera Config ---
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0  = Y2_GPIO_NUM;
    config.pin_d1  = Y3_GPIO_NUM;
    config.pin_d2  = Y4_GPIO_NUM;
    config.pin_d3  = Y5_GPIO_NUM;
    config.pin_d4  = Y6_GPIO_NUM;
    config.pin_d5  = Y7_GPIO_NUM;
    config.pin_d6  = Y8_GPIO_NUM;
    config.pin_d7  = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;


    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return;
    }


    // --- WiFi ---
    WiFi.begin(ssid, password);
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }


    startCameraServer();


    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}


void loop()
{
    delay(1);
}
