// ======================= ARDUINO: receive servo angles only =======================
#include <Servo.h>

// ---------- CONFIG ----------
const int MOTOR_X_PIN = 9;
const int MOTOR_Y_PIN = 10;
const int LED_PIN     = 13;

const int SERVO_MIN = 45;
const int SERVO_MAX = 135;
int SERVO_X_OFFSET  = 90;
int SERVO_Y_OFFSET  = 90;

const int MAX_SERVO_CHANGE = 4;      // smoothing
const unsigned long UPDATE_INTERVAL  = 20;   // ms
const unsigned long MESSAGE_TIMEOUT  = 2000; // ms (safety: return to center)

// ---------- STATE ----------
Servo servoX;
Servo servoY;

int target_x  = SERVO_X_OFFSET;
int target_y  = SERVO_Y_OFFSET;
int current_x = SERVO_X_OFFSET;
int current_y = SERVO_Y_OFFSET;

bool ball_detected = false;
unsigned long last_message = 0;
unsigned long last_update  = 0;
unsigned long last_debug   = 0;

void setup() {
  Serial.begin(115200);
  servoX.attach(MOTOR_X_PIN);
  servoY.attach(MOTOR_Y_PIN);
  pinMode(LED_PIN, OUTPUT);

  servoX.write(SERVO_X_OFFSET);
  servoY.write(SERVO_Y_OFFSET);

  Serial.println("Arduino ready, expecting 3-byte packets: [servoX, servoY, detected]");
}

void loop() {
  // --- read 3-byte packet from Python: uint8 servoX, uint8 servoY, uint8 detected ---
  if (Serial.available() >= 3) {
    byte buffer[3];
    Serial.readBytes(buffer, 3);

    int sx = buffer[0];
    int sy = buffer[1];
    ball_detected = (buffer[2] != 0);

    target_x = constrain(sx, SERVO_MIN, SERVO_MAX);
    target_y = constrain(sy, SERVO_MIN, SERVO_MAX);

    last_message = millis();

    digitalWrite(LED_PIN, HIGH);
    delay(5);
    digitalWrite(LED_PIN, LOW);

    if (millis() - last_debug > 600) {
      Serial.print("ServoX: "); Serial.print(target_x);
      Serial.print("  ServoY: "); Serial.print(target_y);
      Serial.print("  D: ");     Serial.println(ball_detected ? "1" : "0");
      last_debug = millis();
    }
  }

  // --- safety: if no packets for some time, go back to center ---
  if (millis() - last_message > MESSAGE_TIMEOUT) {
    ball_detected = false;
    target_x = SERVO_X_OFFSET;
    target_y = SERVO_Y_OFFSET;
  }

  // --- smooth movement to target position ---
  if (millis() - last_update >= UPDATE_INTERVAL) {
    // X axis
    if (current_x < target_x) {
      current_x = min(current_x + MAX_SERVO_CHANGE, target_x);
    } else if (current_x > target_x) {
      current_x = max(current_x - MAX_SERVO_CHANGE, target_x);
    }

    // Y axis
    if (current_y < target_y) {
      current_y = min(current_y + MAX_SERVO_CHANGE, target_y);
    } else if (current_y > target_y) {
      current_y = max(current_y - MAX_SERVO_CHANGE, target_y);
    }

    current_x = constrain(current_x, SERVO_MIN, SERVO_MAX);
    current_y = constrain(current_y, SERVO_MIN, SERVO_MAX);

    servoX.write(current_x);
    servoY.write(current_y);

    last_update = millis();
  }
}
