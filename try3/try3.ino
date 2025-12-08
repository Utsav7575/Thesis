#include <Servo.h>

// ---------------- CONFIGURATION AND TUNABLE PARAMETERS ----------------
const int MOTOR_X_PIN = 9;       // Pin for X-axis servo
const int MOTOR_Y_PIN = 10;      // Pin for Y-axis servo
const int LED_PIN = 13;          // Onboard LED for debug/status

// Servo neutral (center) positions
const int SERVO_X_OFFSET = 90;
const int SERVO_Y_OFFSET = 90;

// Servo limits
const int SERVO_MIN = 45;
const int SERVO_MAX = 135;

// PID gains for X and Y axes
const float kP_x = 14.0;   // Proportional gain X
const float kI_x = 0.05;   // Integral gain X
const float kD_x = 200.0;  // Derivative gain X

const float kP_y = 14.0;   // Proportional gain Y
const float kI_y = 0.05;   // Integral gain Y
const float kD_y = 200.0;  // Derivative gain Y

// Ignore small errors to prevent jitter
const float deadzone = 0.01;

// Maximum servo movement per update for smooth motion
const int MAX_SERVO_CHANGE = 4;

// Timing intervals (microseconds)
const unsigned long UPDATE_INTERVAL_US = 20000;     // 20 ms per PID loop
const unsigned long MESSAGE_TIMEOUT_US = 2000000;   // 2000 ms for ball lost

// -----------------------------------------------------------------------

Servo servoX;
Servo servoY;

// PID state variables
float error_x = 0, error_y = 0;
float last_error_x = 0, last_error_y = 0;
float integral_x = 0, integral_y = 0;
float derivative_x = 0, derivative_y = 0;
float output_x = 0, output_y = 0;

// Ball coordinates received from Python
float ball_x = 0.0;
float ball_y = 0.0;
bool ball_detected = false;

// Timing variables (using micros() for precise non-blocking control)
unsigned long last_message = 0;
unsigned long last_update = 0;
unsigned long last_debug = 0;

void setup() {
  Serial.begin(115200);       // Initialize serial communication
  servoX.attach(MOTOR_X_PIN); // Attach servos to pins
  servoY.attach(MOTOR_Y_PIN);
  pinMode(LED_PIN, OUTPUT);

  // Initialize servos to neutral positions
  servoX.write(SERVO_X_OFFSET);
  servoY.write(SERVO_Y_OFFSET);

  Serial.println("Arduino ready, waiting for 9-byte packets...");
}

void loop() {
  readSerialPacket();    // Read coordinates from Python (non-blocking)
  checkTimeout();        // Handle lost ball scenario
  runControlLoop();      // Run PID control and move servos
}

// ---------------------- READ SERIAL ----------------------
// Non-blocking read of 9-byte packet (float x, float y, detected flag)
void readSerialPacket() {
  while (Serial.available() >= 9) {
    byte buffer[9];
    Serial.readBytes(buffer, 9);

    memcpy(&ball_x, buffer, 4);        // Copy X coordinate
    memcpy(&ball_y, buffer + 4, 4);    // Copy Y coordinate
    ball_detected = buffer[8] == 1;    // Detection flag

    last_message = micros();           // Update last received time

    // Blink LED for visual feedback (instant, non-blocking)
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);

    // Optional debug prints every 600 ms
    if (micros() - last_debug > 600000) {
      Serial.print("X: "); Serial.print(ball_x, 3);
      Serial.print("  Y: "); Serial.print(ball_y, 3);
      Serial.print("  D: "); Serial.println(ball_detected ? "1" : "0");
      last_debug = micros();
    }
  }
}

// ---------------------- BALL LOST HANDLING ----------------------
void checkTimeout() {
  if (micros() - last_message > MESSAGE_TIMEOUT_US) {
    ball_detected = false; // Mark ball as lost if no data received
  }
}

// ---------------------- PID AND SERVO CONTROL ----------------------
void runControlLoop() {
  unsigned long now = micros();
  if (now - last_update < UPDATE_INTERVAL_US) return; // Wait until next PID cycle
  last_update = now;

  // Current servo positions and last written values
  static int current_x = SERVO_X_OFFSET;
  static int current_y = SERVO_Y_OFFSET;
  static int last_servo_x = SERVO_X_OFFSET;
  static int last_servo_y = SERVO_Y_OFFSET;

  if (!ball_detected) {
    // Gradually return servos to center if ball not detected
    if (current_x < SERVO_X_OFFSET) current_x++;
    else if (current_x > SERVO_X_OFFSET) current_x--;

    if (current_y < SERVO_Y_OFFSET) current_y++;
    else if (current_y > SERVO_Y_OFFSET) current_y--;

    servoX.write(current_x);
    servoY.write(current_y);

    // Reset PID variables
    integral_x = integral_y = 0;
    last_error_x = last_error_y = 0;
    derivative_x = derivative_y = 0;
    return;
  }

  // ---------------- PID CALCULATION ----------------
  error_x = -ball_x * 1.2; // Scale and flip sign if needed
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

  // Calculate new servo angles and clamp to servo limits
  int servo_x = SERVO_X_OFFSET + constrain(output_x,
                    -(SERVO_X_OFFSET - SERVO_MIN),
                     (SERVO_MAX - SERVO_X_OFFSET));
  int servo_y = SERVO_Y_OFFSET + constrain(output_y,
                    -(SERVO_Y_OFFSET - SERVO_MIN),
                     (SERVO_MAX - SERVO_Y_OFFSET));

  // Smooth motion by limiting per-step change
  servo_x = constrain(servo_x, last_servo_x - MAX_SERVO_CHANGE, last_servo_x + MAX_SERVO_CHANGE);
  servo_y = constrain(servo_y, last_servo_y - MAX_SERVO_CHANGE, last_servo_y + MAX_SERVO_CHANGE);

  // Write to servos
  servoX.write(servo_x);
  servoY.write(servo_y);

  // Update internal state
  current_x = servo_x;
  current_y = servo_y;
  last_servo_x = servo_x;
  last_servo_y = servo_y;

  last_error_x = error_x;
  last_error_y = error_y;
}
