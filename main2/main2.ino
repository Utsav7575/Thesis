#include <Servo.h>

/* ===================== TUNABLE PARAMETERS ===================== */

// -------- Serial --------
const unsigned long BAUD_RATE = 115200;
const int PACKET_SIZE = 9;

// -------- Servo pins --------
#define MOTOR_X_PIN 9
#define MOTOR_Y_PIN 10
#define LED_PIN 13

// -------- Servo geometry --------
const int SERVO_X_CENTER = 90;   // straight plate position (calibrated)
const int SERVO_Y_CENTER = 90;
const int SERVO_MIN = 45;
const int SERVO_MAX = 135;
const int MAX_SERVO_STEP = 3;    // max degrees per update (smooth motion)

// -------- Timing --------
const unsigned long UPDATE_INTERVAL_MS = 20;   // 50 Hz control
const unsigned long MESSAGE_TIMEOUT_MS = 2000; // ball lost timeout

// -------- Base PID gains --------
float KP_BASE = 12.0;
float KI = 0.05;
float KD = 8.0;

// -------- Adaptive PID (static friction compensation) --------
float KP_MAX = 35.0;
float KP_STEP = 0.3;
float ERROR_ACTIVATE = 0.08;  // when ball is far from center
float MIN_VEL = 0.002;        // ball considered "not moving"

// -------- Deadzone --------
float DEADZONE = 0.05;

/* ===================== GLOBAL STATE ===================== */

Servo servoX, servoY;

// incoming data from Python
float ball_x = 0.0;    // normalized [-1 .. 1]
float ball_y = 0.0;
bool ball_detected = false;

// PID state
float error_x = 0, error_y = 0;
float last_error_x = 0, last_error_y = 0;
float integral_x = 0, integral_y = 0;
float derivative_x = 0, derivative_y = 0;

// adaptive gains
float adaptive_kp_x = KP_BASE;
float adaptive_kp_y = KP_BASE;

// ball motion estimation
float last_ball_x = 0.0;
float last_ball_y = 0.0;

// servo state
int current_x = SERVO_X_CENTER;
int current_y = SERVO_Y_CENTER;
int last_servo_x = SERVO_X_CENTER;
int last_servo_y = SERVO_Y_CENTER;

// timing
unsigned long last_message = 0;
unsigned long last_update = 0;
unsigned long last_debug = 0;

/* ===================== SETUP ===================== */

void setup() {
  Serial.begin(BAUD_RATE);

  servoX.attach(MOTOR_X_PIN);
  servoY.attach(MOTOR_Y_PIN);

  pinMode(LED_PIN, OUTPUT);

  // start from straight/level plate
  servoX.write(SERVO_X_CENTER);
  servoY.write(SERVO_Y_CENTER);

  Serial.println("Arduino ready (binary packets, adaptive PID)");
}

/* ===================== LOOP ===================== */

void loop() {

  // ---- Read binary packet <float x, float y, uint8 detected> ----
  if (Serial.available() >= PACKET_SIZE) {
    uint8_t buf[PACKET_SIZE];
    Serial.readBytes(buf, PACKET_SIZE);

    memcpy(&ball_x, buf, 4);
    memcpy(&ball_y, buf + 4, 4);
    ball_detected = (buf[8] == 1);

    if (ball_detected) last_message = millis();

    digitalWrite(LED_PIN, HIGH);
  }

  // ---- Ball lost handling ----
  if (millis() - last_message > MESSAGE_TIMEOUT_MS) {
    ball_detected = false;
  }

  // ---- Control loop ----
  if (millis() - last_update >= UPDATE_INTERVAL_MS) {
    updateServoControl();
    last_update = millis();
    digitalWrite(LED_PIN, LOW);
  }
}

/* ===================== CONTROL ===================== */

void updateServoControl() {

  // ---- Return to straight position if no ball ----
  if (!ball_detected) {
    moveTowardCenter();
    resetPID();
    return;
  }

  // ---- Ball velocity estimation ----
  float vel_x = ball_x - last_ball_x;
  float vel_y = ball_y - last_ball_y;
  last_ball_x = ball_x;
  last_ball_y = ball_y;

  // ---- Adaptive Kp (static friction compensation) ----
  if (abs(ball_x) > ERROR_ACTIVATE && abs(vel_x) < MIN_VEL)
    adaptive_kp_x = min(adaptive_kp_x + KP_STEP, KP_MAX);
  else
    adaptive_kp_x = KP_BASE;

  if (abs(ball_y) > ERROR_ACTIVATE && abs(vel_y) < MIN_VEL)
    adaptive_kp_y = min(adaptive_kp_y + KP_STEP, KP_MAX);
  else
    adaptive_kp_y = KP_BASE;

  // ---- PID error (signs depend on mechanics) ----
  error_x = -ball_x;
  error_y =  ball_y;

  if (abs(error_x) < DEADZONE) error_x = 0;
  if (abs(error_y) < DEADZONE) error_y = 0;

  // ---- PID math ----
  integral_x += error_x;
  integral_y += error_y;
  integral_x = constrain(integral_x, -50, 50);
  integral_y = constrain(integral_y, -50, 50);

  derivative_x = error_x - last_error_x;
  derivative_y = error_y - last_error_y;

  float output_x = adaptive_kp_x * error_x + KI * integral_x + KD * derivative_x;
  float output_y = adaptive_kp_y * error_y + KI * integral_y + KD * derivative_y;

  last_error_x = error_x;
  last_error_y = error_y;

  // ---- Servo target angles ----
  int servo_x = SERVO_X_CENTER + output_x;
  int servo_y = SERVO_Y_CENTER + output_y;

  servo_x = constrain(servo_x, SERVO_MIN, SERVO_MAX);
  servo_y = constrain(servo_y, SERVO_MIN, SERVO_MAX);

  // ---- Rate limiting ----
  servo_x = constrain(servo_x, last_servo_x - MAX_SERVO_STEP,
                                 last_servo_x + MAX_SERVO_STEP);
  servo_y = constrain(servo_y, last_servo_y - MAX_SERVO_STEP,
                                 last_servo_y + MAX_SERVO_STEP);

  servoX.write(servo_x);
  servoY.write(servo_y);

  last_servo_x = servo_x;
  last_servo_y = servo_y;
  current_x = servo_x;
  current_y = servo_y;
}

/* ===================== HELPERS ===================== */

void moveTowardCenter() {
  if (current_x < SERVO_X_CENTER) current_x++;
  else if (current_x > SERVO_X_CENTER) current_x--;

  if (current_y < SERVO_Y_CENTER) current_y++;
  else if (current_y > SERVO_Y_CENTER) current_y--;

  servoX.write(current_x);
  servoY.write(current_y);

  last_servo_x = current_x;
  last_servo_y = current_y;
}

void resetPID() {
  integral_x = integral_y = 0;
  last_error_x = last_error_y = 0;
  adaptive_kp_x = KP_BASE;
  adaptive_kp_y = KP_BASE;
}
