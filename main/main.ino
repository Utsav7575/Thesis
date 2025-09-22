#include <Servo.h>

// Servo objects
Servo servoX;
Servo servoY;

// Servo pins
#define MOTOR_X_PIN 9
#define MOTOR_Y_PIN 10
#define LED_PIN 13

// Calibration offsets
int SERVO_X_OFFSET = 90;
int SERVO_Y_OFFSET = 90;

const int SERVO_MIN = 45;
const int SERVO_MAX = 135;

// PID parameters for X and Y axes
float kP_x = 30.0, kI_x = 0.1, kD_x = 0.0;
float kP_y = 25.0, kI_y = 0.05, kD_y = 0.0;
float deadzone = 0.05;

// PID variables
float error_x=0, error_y=0, last_error_x=0, last_error_y=0;
float integral_x=0, integral_y=0, derivative_x=0, derivative_y=0;
float output_x=0, output_y=0;

// Ball position variables
float ball_x=0.0, ball_y=0.0;
bool ball_detected=false;
unsigned long last_message=0;

// Timing
const unsigned long update_interval = 20;
const unsigned long message_timeout = 2000;
unsigned long last_update=0;

// Debug timing
unsigned long last_debug = 0;

void setup() {
  Serial.begin(115200);
  servoX.attach(MOTOR_X_PIN);
  servoY.attach(MOTOR_Y_PIN);
  pinMode(LED_PIN, OUTPUT);
  servoX.write(SERVO_X_OFFSET);
  servoY.write(SERVO_Y_OFFSET);
  Serial.println("✅ Arduino ready, waiting for coordinates...");
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseCoordinates(data);
    last_message = millis();
    digitalWrite(LED_PIN, HIGH);
    delay(20);
    digitalWrite(LED_PIN, LOW);
  }

  if (millis() - last_message > message_timeout) {
    ball_detected = false;
  }

  if (millis() - last_update >= update_interval) {
    updateServoControl();
    last_update = millis();
  }
}

void parseCoordinates(String data) {
  int x_index = data.indexOf("X:");
  int y_index = data.indexOf("Y:");
  int d_index = data.indexOf("D:");
  if (x_index!=-1 && y_index!=-1 && d_index!=-1) {
    int x_end = data.indexOf(",", x_index);
    ball_x = data.substring(x_index+2, x_end).toFloat();
    int y_end = data.indexOf(",", y_index);
    ball_y = data.substring(y_index+2, y_end).toFloat();
    ball_detected = data.substring(d_index+2).toInt() == 1;

    if (millis() - last_debug > 600) {
      Serial.print("Received -> X:");
      Serial.print(ball_x,3);
      Serial.print(" Y:");
      Serial.print(ball_y,3);
      Serial.print(" Detected:");
      Serial.println(ball_detected ? "1":"0");
      last_debug = millis();
    }
  }
}

void updateServoControl() {
  static int current_x = SERVO_X_OFFSET;
  static int current_y = SERVO_Y_OFFSET;

  if (!ball_detected) {
    if (current_x < SERVO_X_OFFSET) current_x++;
    else if (current_x > SERVO_X_OFFSET) current_x--;
    if (current_y < SERVO_Y_OFFSET) current_y++;
    else if (current_y > SERVO_Y_OFFSET) current_y--;
    servoX.write(current_x);
    servoY.write(current_y);
    integral_x=integral_y=0;
    last_error_x=last_error_y=0;
    return;
  }

  // PID calculation using separate constants
  error_x = -ball_x * 1.2;
  error_y = ball_y * 1.2;

  if (abs(error_x)<deadzone) error_x=0;
  if (abs(error_y)<deadzone) error_y=0;

  integral_x += error_x;
  derivative_x = error_x - last_error_x;
  output_x = kP_x*error_x + kI_x*integral_x + kD_x*derivative_x;

  integral_y += error_y;
  derivative_y = error_y - last_error_y;
  output_y = kP_y*error_y + kI_y*integral_y + kD_y*derivative_y;

  integral_x = constrain(integral_x, -50, 50);
  integral_y = constrain(integral_y, -50, 50);

  int servo_x = SERVO_X_OFFSET + constrain(output_x, -(SERVO_X_OFFSET-SERVO_MIN), (SERVO_MAX-SERVO_X_OFFSET));
  int servo_y = SERVO_Y_OFFSET + constrain(output_y, -(SERVO_Y_OFFSET-SERVO_MIN), (SERVO_MAX-SERVO_Y_OFFSET));

  servoX.write(servo_x);
  servoY.write(servo_y);

  current_x = servo_x;
  current_y = servo_y;

  last_error_x = error_x;
  last_error_y = error_y;
}
