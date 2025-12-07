#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// =============================================================
// IMU Objects
// =============================================================
Adafruit_LSM6DSOX imuA;  // 0x6A
Adafruit_LSM6DSOX imuB;  // 0x6B

float pitchA = 0, rollA = 0;
float pitchB = 0, rollB = 0;
float fusedPitch = 0, fusedRoll = 0;

// Calibration offsets
float pitchOffset = 0;
float rollOffset  = 0;

unsigned long lastTime = 0;

// =============================================================
// LED Pins
// =============================================================
#define LED_GREEN 14
#define LED_YELLOW 32
#define LED_RED 21

// =============================================================
// Start / Recalibration Button
// =============================================================
#define START_BUTTON 27
bool systemActive = false;

unsigned long buttonPressTime = 0;
bool buttonHeld = false;

// Prevent false GREEN on startup
bool firstMeasurement = false;

// =============================================================
// BLE Setup
// =============================================================
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

BLECharacteristic* pCharacteristic;

// =============================================================
// Complementary Filter
// =============================================================
void computeAngles(float ax, float ay, float az,
                   float gx, float gy, float gz,
                   float &pitch, float &roll, float dt)
{
  gx = gx * 180.0 / PI;
  gy = gy * 180.0 / PI;

  pitch += gx * dt;
  roll  += gy * dt;

  float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float rollAcc  = atan2( ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  const float alpha = 0.96;
  pitch = alpha * pitch + (1 - alpha) * pitchAcc;
  roll  = alpha * roll  + (1 - alpha) * rollAcc;
}

// =============================================================
// LED ANIMATIONS
// =============================================================
void startupAnimation() {

  // 1. All LEDs off first (clean start)
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  delay(300);

  // 2. Turn on Green
  digitalWrite(LED_RED, HIGH);
  delay(300);

  // 3. Turn on Yellow
  digitalWrite(LED_RED, LOW); // turn green off so only one is on at a time
  digitalWrite(LED_YELLOW, HIGH);
  delay(300);

  // 4. Turn on Red
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, HIGH);
  delay(300);

  // 5. Turn all LEDs OFF before system begins
  digitalWrite(LED_GREEN, LOW);
  delay(300);
}

void shutdownAnimation() {

  // 1. Immediately turn everything OFF
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  // 2. Delay before animation starts
  delay(300);

  // 3. Shutdown animation: Green → Yellow → Red
  digitalWrite(LED_GREEN, HIGH);
  delay(300);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, HIGH);
  delay(300);

  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, HIGH);
  delay(300);

  // 4. All LEDs OFF at end
  digitalWrite(LED_RED, LOW);
  delay(300);
}

void calibrationAnimation() {

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  delay(300);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(200);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  delay(200);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(200);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  delay(200);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(200);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  delay(200);

  delay(300);
}

// =============================================================
// LED Update
// =============================================================
void updateLEDs(float pitch, float roll)
{
  bool greenZone =
      (pitch >= -18 && pitch <= 18) &&
      (roll >= -10 && roll <= 10);

  bool redZone =
      (pitch >= 30 || pitch <= -30) ||
      (abs(roll) >= 15);

  bool yellowZone = (!greenZone && !redZone);

  digitalWrite(LED_GREEN, greenZone ? HIGH : LOW);
  digitalWrite(LED_YELLOW, yellowZone ? HIGH : LOW);
  digitalWrite(LED_RED, redZone ? HIGH : LOW);
}

// =============================================================
// Posture Classification
// =============================================================
String classifyPosture(float pitch, float roll)
{
  bool greenPitch = (pitch >= -20 && pitch <= 20);
  bool greenRoll  = (roll >= -10 && roll <= 10);

  if (greenPitch && greenRoll)
    return "GOOD ✅";

  bool yellowPitch =
      (pitch > 20 && pitch < 30) ||
      (pitch > -30 && pitch < -20);

  bool yellowRoll =
      (abs(roll) > 10 && abs(roll) < 15);

  if (yellowPitch || yellowRoll)
    return "OKAY 👌";

  return "BAD ❌";
}

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin();

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  pinMode(START_BUTTON, INPUT_PULLUP);

  // -----------------------------------------------------------
  // CLEAN, PROFESSIONAL STARTUP BLOCK (OPTION B)
  // -----------------------------------------------------------
  Serial.println();
  Serial.println("================================");
  Serial.println("        IMU System Boot");
  Serial.println("================================");
  Serial.println();

  // IMU initialization messages
  if (imuA.begin_I2C(0x6A))
      Serial.println("✔ IMU A initialized at 0x6A");
  else
      Serial.println("❌ ERROR: IMU A not detected!");

  if (imuB.begin_I2C(0x6B))
      Serial.println("✔ IMU B initialized at 0x6B");
  else
      Serial.println("❌ ERROR: IMU B not detected!");

  // BLE Init
  BLEDevice::init("IMU System Boot");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEDevice::startAdvertising();

  Serial.println("✔ BLE Advertising Started");
  Serial.println();
  Serial.println("Press button to begin system.");
  Serial.println("Hold button (2s) to recalibrate.");
  Serial.println();

  imuA.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imuB.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imuA.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  imuB.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  imuA.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imuB.setAccelDataRate(LSM6DS_RATE_104_HZ);

  lastTime = micros();
}

// =============================================================
// LOOP
// =============================================================
void loop() {

  // -----------------------------------------------------------
  // BUTTON LOGIC (short press = toggle, long press = recalibrate)
  // -----------------------------------------------------------
  int buttonState = digitalRead(START_BUTTON);

  if (buttonState == LOW) {
    if (buttonPressTime == 0) buttonPressTime = millis();

    // LONG PRESS = recalibrate
    if (!buttonHeld && millis() - buttonPressTime >= 2000) {
      buttonHeld = true;

      pitchOffset = fusedPitch;
      rollOffset  = fusedRoll;

      Serial.println("<<< IMU Recalibrated >>>");
      calibrationAnimation();
    }

  } else {  
    if (buttonPressTime != 0 && !buttonHeld) {

      if (!systemActive) {
        systemActive = true;
        firstMeasurement = true;

      Serial.println("System Activated!");

      // BLE message for activation
      pCharacteristic->setValue("System Activated!");
      pCharacteristic->notify();

      startupAnimation();
}
      else {
        systemActive = false;

        Serial.println("System Deactivated!");

        // BLE message for deactivation
        pCharacteristic->setValue("System Deactivated!");
        pCharacteristic->notify();

        shutdownAnimation();
}

    }
    buttonPressTime = 0;
    buttonHeld = false;
  }

  if (!systemActive) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
    return;
  }

  // -----------------------------------------------------------
  // IMU PROCESSING
  // -----------------------------------------------------------
  sensors_event_t accA, gyroA, tempA;
  sensors_event_t accB, gyroB, tempB;

  imuA.getEvent(&accA, &gyroA, &tempA);
  imuB.getEvent(&accB, &gyroB, &tempB);

  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6;
  lastTime = now;
  if (dt > 0.05) dt = 0.05;

  computeAngles(accA.acceleration.x, accA.acceleration.y, accA.acceleration.z,
                gyroA.gyro.x, gyroA.gyro.y, gyroA.gyro.z,
                pitchA, rollA, dt);

  computeAngles(accB.acceleration.x, accB.acceleration.y, accB.acceleration.z,
                gyroB.gyro.x, gyroB.gyro.y, gyroB.gyro.z,
                pitchB, rollB, dt);

  fusedPitch = 0.5f * (pitchA + pitchB);
  fusedRoll  = 0.5f * (rollA + rollB);

  float calibratedPitch = fusedPitch - pitchOffset;
  float calibratedRoll  = fusedRoll - rollOffset;

  // FIRST MEASUREMENT FIX
  if (firstMeasurement) {
    firstMeasurement = false;
  } else {
    updateLEDs(calibratedPitch, calibratedRoll);
  }

  // SERIAL OUTPUT
  String status = classifyPosture(calibratedPitch, calibratedRoll);

  Serial.print("Pitch: "); Serial.print(calibratedPitch, 1);
  Serial.print("  Roll: "); Serial.print(calibratedRoll, 1);
  Serial.print("  Status: "); Serial.println(status);

  // BLE OUTPUT
  String msg =
      "Pitch: " + String(calibratedPitch, 1) +
      "  Roll: " + String(calibratedRoll, 1) +
      "  Status: " + status;

  pCharacteristic->setValue(msg.c_str());
  pCharacteristic->notify();

  delay(10);
}
