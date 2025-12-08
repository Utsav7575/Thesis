#include <Arduino.h>
#include <Servo.h>
#include <AutoPID.h>

// ---------------- CONFIG ----------------
const int SERVO_X_PIN = 9;
const int SERVO_Y_PIN = 10;

const int SERVO_MIN = 45;
const int SERVO_MAX = 135;
const int SERVO_CENTER = 90;

const unsigned long UPDATE_INTERVAL = 20;       // PID update ms
const unsigned long MESSAGE_TIMEOUT = 2000;     // Ball lost timeout

// PID gains
const double KPX = 14.0, KIX = 0.05, KDX = 1200000.0;
const double KPY = 14.0, KIY = 0.05, KDY = 1200000.0;

// PID output limits
const double OUTPUT_MIN = -20, OUTPUT_MAX = 20;

// ---------------- GLOBALS ----------------
Servo servoX, servoY;

double ballX = 0, ballY = 0;
bool ballDetected = false;


bool inputRead = false;

unsigned long lastMessage = 0;
unsigned long lastUpdate = 0;

// PID objects
double setPointX = 0, outputX = 0;
double setPointY = 0, outputY = 0;
AutoPID xPID(&ballX, &setPointX, &outputX, OUTPUT_MIN, OUTPUT_MAX, KPX, KIX, KDX);
AutoPID yPID(&ballY, &setPointY, &outputY, OUTPUT_MIN, OUTPUT_MAX, KPY, KIY, KDY);

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoX.write(SERVO_CENTER);
  servoY.write(SERVO_CENTER);

  xPID.setTimeStep(UPDATE_INTERVAL);
  yPID.setTimeStep(UPDATE_INTERVAL);
}

// ---------------- LOOP ----------------
void loop() {
  // --- Read 9-byte packet from Python ---
  while (Serial.available() >= 9) {
    byte buffer[9];
    Serial.readBytes(buffer, 9);
    memcpy(&ballX, buffer, 4);
    memcpy(&ballY, buffer + 4, 4);
    ballDetected = buffer[8] == 1;
    inputRead = true;
    lastMessage = millis();
  }

  // --- Ball lost: timeout handling ---
  if (millis() - lastMessage > MESSAGE_TIMEOUT) {
    ballDetected = false;
  }

  // --- Update PID and servo ---
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();

    static int currentX = SERVO_CENTER;
    static int currentY = SERVO_CENTER;

    if (ballDetected && inputRead) {
      // Run PID
      xPID.run();
      yPID.run();
      inputRead = false;

      currentX = SERVO_CENTER + outputX; // flip sign if motor reversed
      currentY = SERVO_CENTER - outputY; // flip sign if motor reversed

    } else {
      // Gradually return to center
      if (currentX < SERVO_CENTER) currentX++;
      else if (currentX > SERVO_CENTER) currentX--;
      if (currentY < SERVO_CENTER) currentY++;
      else if (currentY > SERVO_CENTER) currentY--;
    }

    // Clamp and write to servos
    currentX = constrain(currentX, SERVO_MIN, SERVO_MAX);
    currentY = constrain(currentY, SERVO_MIN, SERVO_MAX);
    servoX.write(currentX);
    servoY.write(currentY);
  }
}
