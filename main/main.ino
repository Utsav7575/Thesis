#include <Servo.h>

// ---------- CONFIG / TUNABLE PARAMETERS ----------
const int MOTOR_X_PIN = 9;
const int MOTOR_Y_PIN = 10;
const int LED_PIN = 13;

// Servo neutral (center) positions
int SERVO_X_OFFSET = 90;
int SERVO_Y_OFFSET = 90;

// Servo limits
const int SERVO_MIN = 45;
const int SERVO_MAX = 135;

// PID gains
float kP_x = 20.0;   // proportional gain X
float kI_x = 0.05;   // integral gain X
float kD_x = 10.0;   // derivative gain X

float kP_y = 20.0;   // proportional gain Y
float kI_y = 0.05;   // integral gain Y
float kD_y = 10.0;   // derivative gain Y

// Ignore small errors
float deadzone = 0.01;

// Maximum servo change per update
const int MAX_SERVO_CHANGE = 4;

// Timing
const unsigned long UPDATE_INTERVAL = 20;       // ms
const unsigned long MESSAGE_TIMEOUT = 2000;     // ms

// ------------------------------------------------

Servo servoX;
Servo servoY;

// PID state variables
float error_x = 0, error_y = 0;
float last_error_x = 0, last_error_y = 0;
float integral_x = 0, integral_y = 0;
float derivative_x = 0, derivative_y = 0;
float output_x = 0, output_y = 0;

// Ball coordinates from Python
float ball_x = 0.0;
float ball_y = 0.0;
bool ball_detected = false;

unsigned long last_message = 0;
unsigned long last_update = 0;
unsigned long last_debug = 0;

void setup() {
  Serial.begin(115200);
  servoX.attach(MOTOR_X_PIN);
  servoY.attach(MOTOR_Y_PIN);
  pinMode(LED_PIN, OUTPUT);

  // Initialize servos to center
  servoX.write(SERVO_X_OFFSET);
  servoY.write(SERVO_Y_OFFSET);

  Serial.println("Arduino ready, waiting for 9-byte packets...");
}

void loop() {
  // --- Read exact 9-byte packet from Python ---
  if (Serial.available() >= 9) {
    byte buffer[9];
    Serial.readBytes(buffer, 9);

    memcpy(&ball_x, buffer, 4);        // float X from Python
    memcpy(&ball_y, buffer + 4, 4);    // float Y from Python
    ball_detected = buffer[8] == 1;    // detected flag

    last_message = millis();

    digitalWrite(LED_PIN, HIGH);
    delay(5);
    digitalWrite(LED_PIN, LOW);

    if (millis() - last_debug > 600) {
      Serial.print("X: "); Serial.print(ball_x, 3);
      Serial.print("  Y: "); Serial.print(ball_y, 3);
      Serial.print("  D: "); Serial.println(ball_detected ? "1" : "0");
      last_debug = millis();
    }
  }

  // --- Ball lost: timeout handling ---
  if (millis() - last_message > MESSAGE_TIMEOUT) {
    ball_detected = false;
  }

  // --- Update servo movement ---
  if (millis() - last_update >= UPDATE_INTERVAL) {
    updateServoControl();
    last_update = millis();
  }
}

void updateServoControl() {
  static int current_x = SERVO_X_OFFSET;
  static int current_y = SERVO_Y_OFFSET;
  static int last_servo_x = SERVO_X_OFFSET;
  static int last_servo_y = SERVO_Y_OFFSET;

  // --- No ball detected → return to center gradually ---
  if (!ball_detected) {
    if (current_x < SERVO_X_OFFSET) current_x++;
    else if (current_x > SERVO_X_OFFSET) current_x--;

    if (current_y < SERVO_Y_OFFSET) current_y++;
    else if (current_y > SERVO_Y_OFFSET) current_y--;

    servoX.write(current_x);
    servoY.write(current_y);

    integral_x = integral_y = 0;
    last_error_x = last_error_y = 0;
    derivative_x = derivative_y = 0;
    return;
  }

  // --- PID calculations ---
  error_x = -ball_x * 1.2;  // adjust sign if needed
  error_y = ball_y * 1.2;

  if (abs(error_x) < deadzone) error_x = 0;
  if (abs(error_y) < deadzone) error_y = 0;

  integral_x += error_x;
  integral_y += error_y;

  derivative_x = error_x - last_error_x;
  derivative_y = error_y - last_error_y;

  output_x = kP_x * error_x + kI_x * integral_x + kD_x * derivative_x;
  output_y = kP_y * error_y + kI_y * integral_y + kD_y * derivative_y;

  // Clamp integral to avoid windup
  integral_x = constrain(integral_x, -50, 50);
  integral_y = constrain(integral_y, -50, 50);

  // Calculate servo angles and clamp to limits
  int servo_x = SERVO_X_OFFSET + constrain(output_x,
                    -(SERVO_X_OFFSET - SERVO_MIN),
                     (SERVO_MAX - SERVO_X_OFFSET));
  int servo_y = SERVO_Y_OFFSET + constrain(output_y,
                    -(SERVO_Y_OFFSET - SERVO_MIN),
                     (SERVO_MAX - SERVO_Y_OFFSET));

  // Rate limiting for smooth motion
  servo_x = constrain(servo_x, last_servo_x - MAX_SERVO_CHANGE, last_servo_x + MAX_SERVO_CHANGE);
  servo_y = constrain(servo_y, last_servo_y - MAX_SERVO_CHANGE, last_servo_y + MAX_SERVO_CHANGE);

  // Write to servos
  servoX.write(servo_x);
  servoY.write(servo_y);

  // Update state
  current_x = servo_x;
  current_y = servo_y;
  last_servo_x = servo_x;
  last_servo_y = servo_y;

  last_error_x = error_x;
  last_error_y = error_y;
}
