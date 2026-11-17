#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <AccelStepper.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <math.h>

const uint8_t BNO055_ADDRESS = 0x28;

// Nếu không tìm thấy cảm biến, thử đổi 0x28 thành 0x29.
Adafruit_BNO055 bno =
  Adafruit_BNO055(55, BNO055_ADDRESS, &Wire);

const int SDA_PIN = 21;
const int SCL_PIN = 22;

const int IMU_INTERVAL_MS = 20;  // 50 Hz for ROS 2 EKF.
const int CALIBRATION_INTERVAL_MS = 1000;
const int ZERO_SETTLE_MS = 1000;
const int ZERO_SAMPLE_COUNT = 50;
const int ZERO_SAMPLE_INTERVAL_MS = 20;
const int IMU_INVALID_SAMPLE_LIMIT = 10;
const float IMU_MIN_ACCEL_NORM_M_S2 = 1.0f;
const float IMU_MAX_ACCEL_NORM_M_S2 = 30.0f;

// Càng nâng dùng chung ESP32 số 2 với BNO055. Hai công tắc nối
// GPIO xuống GND: bình thường HIGH, chạm công tắc LOW.
// Cau hinh da duoc thu truc tiep voi TB6600 tren xe.
constexpr uint8_t LIFT_STEP_PIN = 23;    // PUL-
constexpr uint8_t LIFT_DIR_PIN = 4;      // DIR-
constexpr uint8_t LIFT_ENABLE_PIN = 19;  // EN-
// Chieu duong/nang canh GPIO33; chieu am/ha/HOME canh GPIO32.
constexpr uint8_t UPPER_LIMIT_PIN = 33;
constexpr uint8_t LOWER_LIMIT_PIN = 32;
constexpr int LIMIT_ACTIVE_LEVEL = LOW;
constexpr long LIFT_STEPS_PER_M = 100000L;  // 1000 step/cm.
constexpr float LIFT_MAX_POSITION_M = 0.52f;
constexpr float LIFT_MAX_SPEED_M_S = 0.01f;        // 1000 step/s.
constexpr float LIFT_ACCELERATION_M_S2 = 0.005f;  // 500 step/s^2.
constexpr float LIFT_HOME_SPEED_M_S = 0.01f;
constexpr uint32_t LIMIT_DEBOUNCE_MS = 25;
constexpr uint32_t LIFT_COMMAND_TIMEOUT_MS = 350;
constexpr uint32_t LIFT_HOME_TIMEOUT_MS = 60000;
constexpr uint32_t LIFT_HOME_UPPER_RELEASE_TIMEOUT_MS = 750;
constexpr long LIFT_HOME_SETTLE_STEPS = LIFT_STEPS_PER_M / 1000L;  // 1 mm
constexpr uint32_t LIFT_HOME_SETTLE_TIMEOUT_MS = 1000;
constexpr uint32_t LIFT_HOME_INPUT_CHECK_TIMEOUT_MS = 1000;
static_assert(LIFT_STEPS_PER_M % 1000L == 0, "1mm must map to whole steps");
constexpr uint32_t LIFT_TELEMETRY_INTERVAL_MS = 100;

AccelStepper liftStepper(AccelStepper::DRIVER, LIFT_STEP_PIN, LIFT_DIR_PIN);

struct DebouncedLimit {
  uint8_t pin;
  bool rawActive;
  bool active;
  uint32_t changedAtMs;
};

DebouncedLimit lowerLimit{LOWER_LIMIT_PIN, false, false, 0};
DebouncedLimit upperLimit{UPPER_LIMIT_PIN, false, false, 0};

enum class LiftMode : uint8_t { UNHOMED, IDLE, MOVING, HOMING, FAULT };
enum class LiftHomeStage : uint8_t { NONE, CHECK_LOWER, SEEK_LOWER, HOME_SETTLE };
LiftHomeStage liftHomeStage = LiftHomeStage::NONE;
long liftHomeStageStartSteps = 0;
uint32_t liftHomeStageStartedMs = 0;

LiftMode liftMode = LiftMode::UNHOMED;
bool liftHomed = false;
uint32_t liftHomeCompletionCount = 0;
bool liftHomeSkippedActive = false;
bool liftHomeLeavingUpper = false;
long liftTargetSteps = 0;
uint32_t liftHomeStartedMs = 0;
uint32_t liftLastCommandMs = 0;
uint32_t liftLastTelemetryMs = 0;
char liftFault[28] = "NONE";
char serialCommandBuffer[96];
size_t serialCommandLength = 0;

enum class LiftCommandType : uint8_t { HOME, STOP, HEARTBEAT, SET_TARGET };

struct LiftCommand {
  LiftCommandType type;
  float targetMetres;
};

struct LiftSnapshot {
  LiftMode mode;
  bool homed;
  long positionSteps;
  long targetSteps;
  float speedStepsPerSecond;
  bool lowerActive;
  bool upperActive;
  uint32_t homeCompletionCount;
  bool homeSkippedActive;
  char fault[28];
};

QueueHandle_t liftCommandQueue = nullptr;
LiftSnapshot sharedLift{
  LiftMode::UNHOMED, false, 0, 0, 0.0f, false, false, 0, false, "NONE"
};
portMUX_TYPE liftSnapshotMux = portMUX_INITIALIZER_UNLOCKED;

void requestImuZeroFromSerial();

long metresToSteps(float metres) {
  return lroundf(metres * static_cast<float>(LIFT_STEPS_PER_M));
}

float stepsToMetres(long steps) {
  return static_cast<float>(steps) / static_cast<float>(LIFT_STEPS_PER_M);
}

const char *liftModeName(LiftMode mode) {
  switch (mode) {
    case LiftMode::UNHOMED: return "UNHOMED";
    case LiftMode::IDLE: return "IDLE";
    case LiftMode::MOVING: return "MOVING";
    case LiftMode::HOMING: return "HOMING";
    case LiftMode::FAULT: return "FAULT";
  }
  return "FAULT";
}

bool enqueueLiftCommand(LiftCommandType type, float targetMetres = 0.0f) {
  const LiftCommand command{type, targetMetres};
  if (liftCommandQueue == nullptr ||
      xQueueSend(liftCommandQueue, &command, 0) != pdTRUE) {
    Serial.println("EVENT,LIFT_REJECTED,COMMAND_QUEUE_FULL");
    return false;
  }
  return true;
}

void updateLimit(DebouncedLimit &input, uint32_t nowMs) {
  const bool raw = digitalRead(input.pin) == LIMIT_ACTIVE_LEVEL;
  if (raw != input.rawActive) {
    input.rawActive = raw;
    input.changedAtMs = nowMs;
  }
  if (input.active != input.rawActive && nowMs - input.changedAtMs >= LIMIT_DEBOUNCE_MS) {
    input.active = input.rawActive;
  }
}

void stopLift(LiftMode nextMode) {
  liftTargetSteps = liftStepper.currentPosition();
  liftStepper.moveTo(liftTargetSteps);
  liftStepper.setSpeed(0.0f);
  liftMode = nextMode;
  liftHomeLeavingUpper = false;
  liftHomeStage = LiftHomeStage::NONE;
}

void setLiftFault(const char *fault) {
  stopLift(LiftMode::FAULT);
  liftHomed = false;
  snprintf(liftFault, sizeof(liftFault), "%s", fault);
  Serial.printf("EVENT,LIFT_FAULT,%s\n", liftFault);
}

void acceptLowerHome(bool alreadyActive = false) {
  liftStepper.setCurrentPosition(0);
  liftTargetSteps = 0;
  liftStepper.moveTo(0);
  liftHomed = true;
  ++liftHomeCompletionCount;
  liftHomeSkippedActive = alreadyActive;
  liftHomeLeavingUpper = false;
  liftHomeStage = LiftHomeStage::NONE;
  liftMode = LiftMode::IDLE;
  snprintf(liftFault, sizeof(liftFault), "NONE");
  Serial.println("EVENT,LIFT_HOME_COMPLETE");
}

void startLiftHome(uint32_t nowMs) {
  updateLimit(lowerLimit, nowMs);
  updateLimit(upperLimit, nowMs);
  if (liftMode == LiftMode::FAULT) {
    if (lowerLimit.active && upperLimit.active) {
      Serial.printf("EVENT,LIFT_REJECTED,FAULT_%s\n", liftFault);
      return;
    }
    snprintf(liftFault, sizeof(liftFault), "NONE");
    liftMode = LiftMode::UNHOMED;
  }
  liftLastCommandMs = nowMs;
  // Check stable inputs with zero pulses first. An already-pressed bottom
  // switch means HOME immediately, as requested for this mechanism.
  liftHomed = false;
  liftHomeLeavingUpper = upperLimit.active;
  liftHomeStartedMs = nowMs;
  liftMode = LiftMode::HOMING;
  liftHomeStage = LiftHomeStage::CHECK_LOWER;
  liftHomeStageStartSteps = liftStepper.currentPosition();
  liftHomeStageStartedMs = nowMs;
  liftStepper.moveTo(liftStepper.currentPosition());
  liftStepper.setSpeed(0.);
  Serial.println(liftHomeLeavingUpper
    ? "EVENT,LIFT_HOME_STARTED_FROM_UPPER"
    : "EVENT,LIFT_HOME_STARTED");
}

void setLiftTarget(float targetMetres, uint32_t nowMs) {
  if (!isfinite(targetMetres) || targetMetres < 0.0f ||
      targetMetres > LIFT_MAX_POSITION_M) {
    Serial.println("EVENT,LIFT_REJECTED,RANGE");
    return;
  }
  if (!liftHomed || liftMode == LiftMode::HOMING || liftMode == LiftMode::FAULT) {
    Serial.println("EVENT,LIFT_REJECTED,NOT_READY");
    return;
  }
  liftLastCommandMs = nowMs;
  liftTargetSteps = metresToSteps(targetMetres);
  liftStepper.moveTo(liftTargetSteps);
  liftMode = liftStepper.distanceToGo() == 0 ? LiftMode::IDLE : LiftMode::MOVING;
}

void handleSerialCommand(char *line, uint32_t nowMs) {
  (void)nowMs;
  if (strcmp(line, "LIFT,INFO") == 0) {
    Serial.println("INFO,LIFT,HOME_SKIP_ACTIVE_1MM_NEW_CONTACT_V3");
    return;
  }
  if (strcmp(line, "LIFT,HOME") == 0) {
    enqueueLiftCommand(LiftCommandType::HOME);
    return;
  }
  if (strcmp(line, "LIFT,STOP") == 0) {
    enqueueLiftCommand(LiftCommandType::STOP);
    return;
  }
  if (strcmp(line, "LIFT,HB") == 0) {
    enqueueLiftCommand(LiftCommandType::HEARTBEAT);
    return;
  }
  if (strncmp(line, "LIFT,SET,", 9) == 0) {
    char *end = nullptr;
    const float target = strtof(line + 9, &end);
    if (end == line + 9 || *end != '\0') {
      Serial.println("EVENT,LIFT_REJECTED,BAD_NUMBER");
      return;
    }
    enqueueLiftCommand(LiftCommandType::SET_TARGET, target);
    return;
  }
  if (strcmp(line, "Z") == 0 || strcmp(line, "z") == 0) {
    requestImuZeroFromSerial();
  }
}

void processLiftCommands(uint32_t nowMs) {
  LiftCommand command;
  while (xQueueReceive(liftCommandQueue, &command, 0) == pdTRUE) {
    switch (command.type) {
      case LiftCommandType::HOME:
        startLiftHome(nowMs);
        break;
      case LiftCommandType::STOP:
        stopLift(liftHomed ? LiftMode::IDLE : LiftMode::UNHOMED);
        liftLastCommandMs = nowMs;
        break;
      case LiftCommandType::HEARTBEAT:
        liftLastCommandMs = nowMs;
        break;
      case LiftCommandType::SET_TARGET:
        setLiftTarget(command.targetMetres, nowMs);
        break;
    }
  }
}

void readSerialCommands(uint32_t nowMs) {
  while (Serial.available() > 0) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\n' || value == '\r') {
      if (serialCommandLength > 0) {
        serialCommandBuffer[serialCommandLength] = '\0';
        handleSerialCommand(serialCommandBuffer, nowMs);
        serialCommandLength = 0;
      }
    } else if (serialCommandLength + 1 < sizeof(serialCommandBuffer)) {
      serialCommandBuffer[serialCommandLength++] = value;
    } else {
      serialCommandLength = 0;
      Serial.println("EVENT,LIFT_REJECTED,LINE_TOO_LONG");
    }
  }
}

void updateLift(uint32_t nowMs) {
  updateLimit(lowerLimit, nowMs);
  updateLimit(upperLimit, nowMs);

  if (lowerLimit.active && upperLimit.active) {
    if (liftMode != LiftMode::FAULT) setLiftFault("BOTH_LIMITS");
    return;
  }

  // Watchdog applies to input checking, seek AND settle, before any pulse.
  if ((liftMode == LiftMode::MOVING || liftMode == LiftMode::HOMING) &&
      nowMs - liftLastCommandMs > LIFT_COMMAND_TIMEOUT_MS) {
    const bool wasHoming = liftMode == LiftMode::HOMING;
    if (wasHoming) liftHomed = false;
    stopLift(liftHomed ? LiftMode::IDLE : LiftMode::UNHOMED);
    Serial.println("EVENT,LIFT_STOPPED,COMM_TIMEOUT");
    return;
  }

  if (liftMode == LiftMode::HOMING) {
    if (nowMs - liftHomeStartedMs > LIFT_HOME_TIMEOUT_MS) {
      setLiftFault("HOME_TIMEOUT");
      return;
    }
    if (liftHomeStage == LiftHomeStage::CHECK_LOWER) {
      if (nowMs-liftHomeStageStartedMs > LIFT_HOME_INPUT_CHECK_TIMEOUT_MS) {
        setLiftFault("HOME_INPUT_UNSTABLE");
        return;
      }
      if (lowerLimit.active != lowerLimit.rawActive ||
          nowMs-lowerLimit.changedAtMs < LIMIT_DEBOUNCE_MS) return;
      if (lowerLimit.active) {
        if (upperLimit.active || upperLimit.rawActive) {
          setLiftFault("BOTH_LIMITS");
          return;
        }
        acceptLowerHome(true);
        return;
      }
      liftHomeStage = LiftHomeStage::SEEK_LOWER;
      liftHomeStageStartedMs = nowMs;
    }
    if (liftHomeStage == LiftHomeStage::HOME_SETTLE) {
      const long travelled = liftHomeStageStartSteps-liftStepper.currentPosition();
      // Count from the first electrical contact, including any switch bounce.
      // Never restart or extend the 100-step allowance on another edge.
      if (upperLimit.active || upperLimit.rawActive || travelled < 0 ||
          travelled > LIFT_HOME_SETTLE_STEPS ||
          nowMs-liftHomeStageStartedMs > LIFT_HOME_SETTLE_TIMEOUT_MS) {
        setLiftFault("HOME_SETTLE_FAILED");
        return;
      }
      if (travelled == LIFT_HOME_SETTLE_STEPS) {
        liftStepper.setSpeed(0.);
        // Bounce may settle while stopped at exactly 1 mm; no extra steps.
        if (lowerLimit.active && lowerLimit.rawActive) acceptLowerHome();
        return;
      }
      liftStepper.setSpeed(-metresToSteps(LIFT_HOME_SPEED_M_S));
      liftStepper.runSpeed();
      return;
    }
  }

  // Nếu reset ngay tại đỉnh, cho phép đi xuống một đoạn ngắn để nhả công tắc.
  // Công tắc không nhả đúng hạn có thể là DIR ngược hoặc công tắc bị kẹt.
  if (liftMode == LiftMode::HOMING && upperLimit.active) {
    if (!liftHomeLeavingUpper) {
      setLiftFault("HOME_HIT_UPPER");
      return;
    }
    if (nowMs - liftHomeStartedMs > LIFT_HOME_UPPER_RELEASE_TIMEOUT_MS) {
      setLiftFault("HOME_UPPER_STUCK");
      return;
    }
  } else if (liftMode == LiftMode::HOMING && liftHomeLeavingUpper) {
    liftHomeLeavingUpper = false;
    Serial.println("EVENT,LIFT_HOME_LEFT_UPPER");
  }

  // A fresh trigger starts the bounded HOME-only exception. Normal downward
  // motion is blocked at the switch and never redefines the settled zero.
  if (lowerLimit.rawActive || lowerLimit.active) {
    if (liftMode == LiftMode::HOMING && liftHomeStage == LiftHomeStage::SEEK_LOWER
        && lowerLimit.rawActive) {
      liftStepper.setSpeed(0.);
      liftHomeStage = LiftHomeStage::HOME_SETTLE;
      liftHomeStageStartSteps = liftStepper.currentPosition();
      liftHomeStageStartedMs = nowMs;
      Serial.println("EVENT,LIFT_HOME_SETTLE_1MM");
      return;
    }
    if (liftMode == LiftMode::MOVING && liftTargetSteps < liftStepper.currentPosition()) {
      stopLift(LiftMode::IDLE);
      return;
    }
  }

  if (upperLimit.active && liftMode == LiftMode::MOVING &&
      liftTargetSteps > liftStepper.currentPosition()) {
    liftStepper.setCurrentPosition(metresToSteps(LIFT_MAX_POSITION_M));
    stopLift(LiftMode::IDLE);
  }

  if (liftMode == LiftMode::HOMING) {
    if (nowMs - liftHomeStartedMs > LIFT_HOME_TIMEOUT_MS) {
      setLiftFault("HOME_TIMEOUT");
      return;
    }
    liftStepper.setSpeed(-metresToSteps(LIFT_HOME_SPEED_M_S));
    liftStepper.runSpeed();
  } else if (liftMode == LiftMode::MOVING) {
    if (!liftStepper.run()) liftMode = LiftMode::IDLE;
  }
}

void updateLiftSnapshot() {
  LiftSnapshot snapshot;
  snapshot.mode = liftMode;
  snapshot.homed = liftHomed;
  snapshot.positionSteps = liftStepper.currentPosition();
  snapshot.targetSteps = liftTargetSteps;
  snapshot.speedStepsPerSecond = liftStepper.speed();
  snapshot.lowerActive = lowerLimit.active;
  snapshot.upperActive = upperLimit.active;
  snapshot.homeCompletionCount = liftHomeCompletionCount;
  snapshot.homeSkippedActive = liftHomeSkippedActive;
  snprintf(snapshot.fault, sizeof(snapshot.fault), "%s", liftFault);
  portENTER_CRITICAL(&liftSnapshotMux);
  sharedLift = snapshot;
  portEXIT_CRITICAL(&liftSnapshotMux);
}

LiftSnapshot getLiftSnapshot() {
  LiftSnapshot snapshot;
  portENTER_CRITICAL(&liftSnapshotMux);
  snapshot = sharedLift;
  portEXIT_CRITICAL(&liftSnapshotMux);
  return snapshot;
}

void publishLiftTelemetry(uint32_t nowMs) {
  if (nowMs - liftLastTelemetryMs < LIFT_TELEMETRY_INTERVAL_MS) return;
  liftLastTelemetryMs = nowMs;
  const LiftSnapshot snapshot = getLiftSnapshot();
  Serial.printf(
    "LIFT,2,%lu,%s,%u,%.5f,%.5f,%.5f,%u,%u,%s,%lu,%u\n",
    static_cast<unsigned long>(nowMs), liftModeName(snapshot.mode),
    snapshot.homed ? 1U : 0U, stepsToMetres(snapshot.positionSteps),
    stepsToMetres(snapshot.targetSteps),
    snapshot.speedStepsPerSecond / static_cast<float>(LIFT_STEPS_PER_M),
    snapshot.lowerActive ? 1U : 0U, snapshot.upperActive ? 1U : 0U,
    snapshot.fault, static_cast<unsigned long>(snapshot.homeCompletionCount),
    snapshot.homeSkippedActive ? 1U : 0U);
}

void TaskLift(void *pvParameters) {
  (void)pvParameters;
  uint32_t lastSnapshotMs = 0;
  for (;;) {
    const uint32_t nowMs = millis();
    processLiftCommands(nowMs);
    updateLift(nowMs);
    if (nowMs - lastSnapshotMs >= 5) {
      lastSnapshotMs = nowMs;
      updateLiftSnapshot();
    }
    // Giống task STEP của firmware Web đã chạy được: không chèn
    // delay cố định và luôn nhường cho scheduler sau mỗi lần run().
    taskYIELD();
  }
}

bool bnoAvailable = false;
bool bnoDataFaultLatched = false;

// ========================================================
// QUATERNION OFFSET
// Q_relative = inverse(Q_offset) × Q_current
// ========================================================
float inv_qw = 1.0f;
float inv_qx = 0.0f;
float inv_qy = 0.0f;
float inv_qz = 0.0f;

// ========================================================
// DỮ LIỆU IMU DÙNG CHUNG GIỮA HAI CORE
// ========================================================
struct IMUData {
  bool sensorOK;

  // Quaternion tương đối
  float qx;
  float qy;
  float qz;
  float qw;

  // Góc Euler, đơn vị độ
  float roll;
  float pitch;
  float yaw;

  // Gia tốc, đơn vị m/s²
  float ax;
  float ay;
  float az;

  // Vận tốc góc, đơn vị rad/s
  float gx;
  float gy;
  float gz;

  // Calibration từ 0 đến 3
  uint8_t calibSystem;
  uint8_t calibGyro;
  uint8_t calibAccel;
  uint8_t calibMag;

  uint32_t sampleCount;
  uint32_t lastUpdateMs;
};

IMUData sharedIMU = {
  false,
  0.0f, 0.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f,
  0, 0, 0, 0,
  0,
  0
};

portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;

// Cờ yêu cầu đặt lại zero từ Serial
bool zeroRequested = false;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;

void normalizeQuaternion(
  float &qw,
  float &qx,
  float &qy,
  float &qz) {
  float magnitude = sqrtf(
    qw * qw + qx * qx + qy * qy + qz * qz);

  if (magnitude < 0.000001f) {
    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
    return;
  }

  qw /= magnitude;
  qx /= magnitude;
  qy /= magnitude;
  qz /= magnitude;
}

// ========================================================
// ĐẶT QUATERNION HIỆN TẠI LÀM OFFSET
// ========================================================
void setQuaternionOffset(
  const imu::Quaternion &quaternion) {
  float qw = quaternion.w();
  float qx = quaternion.x();
  float qy = quaternion.y();
  float qz = quaternion.z();

  normalizeQuaternion(
    qw,
    qx,
    qy,
    qz);

  // Nghịch đảo của quaternion đơn vị
  inv_qw = qw;
  inv_qx = -qx;
  inv_qy = -qy;
  inv_qz = -qz;
}

// Lấy trung bình quaternion sau giai đoạn ổn định. Quaternion q và -q biểu
// diễn cùng một tư thế, vì vậy các mẫu được đưa về cùng bán cầu trước khi cộng.
// Nhờ đó zero không phụ thuộc vào một mẫu tức thời bị nhiễu.
bool setAveragedQuaternionOffset() {
  float sumW = 0.0f;
  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;
  float referenceW = 1.0f;
  float referenceX = 0.0f;
  float referenceY = 0.0f;
  float referenceZ = 0.0f;
  int validSamples = 0;

  for (int sample = 0; sample < ZERO_SAMPLE_COUNT; ++sample) {
    const imu::Quaternion quaternion = bno.getQuat();
    float qw = quaternion.w();
    float qx = quaternion.x();
    float qy = quaternion.y();
    float qz = quaternion.z();
    const float magnitude = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);

    if (isfinite(magnitude) && magnitude > 0.5f) {
      qw /= magnitude;
      qx /= magnitude;
      qy /= magnitude;
      qz /= magnitude;

      if (validSamples == 0) {
        referenceW = qw;
        referenceX = qx;
        referenceY = qy;
        referenceZ = qz;
      } else {
        const float dot = qw * referenceW + qx * referenceX +
                          qy * referenceY + qz * referenceZ;
        if (dot < 0.0f) {
          qw = -qw;
          qx = -qx;
          qy = -qy;
          qz = -qz;
        }
      }

      sumW += qw;
      sumX += qx;
      sumY += qy;
      sumZ += qz;
      ++validSamples;
    }

    vTaskDelay(pdMS_TO_TICKS(ZERO_SAMPLE_INTERVAL_MS));
  }

  if (validSamples < ZERO_SAMPLE_COUNT / 2) {
    return false;
  }

  normalizeQuaternion(sumW, sumX, sumY, sumZ);
  inv_qw = sumW;
  inv_qx = -sumX;
  inv_qy = -sumY;
  inv_qz = -sumZ;
  return true;
}

// ========================================================
// KIỂM TRA YÊU CẦU ĐẶT LẠI ZERO
// ========================================================
bool takeZeroRequest() {
  bool request;

  portENTER_CRITICAL(&commandMux);

  request = zeroRequested;
  zeroRequested = false;

  portEXIT_CRITICAL(&commandMux);

  return request;
}

// ========================================================
// CHUYỂN QUATERNION SANG ROLL, PITCH, YAW
// ========================================================
void quaternionToEuler(
  float qw,
  float qx,
  float qy,
  float qz,
  float &roll,
  float &pitch,
  float &yaw) {
  // Roll quanh trục X
  float sinRoll =
    2.0f * (qw * qx + qy * qz);

  float cosRoll =
    1.0f - 2.0f * (qx * qx + qy * qy);

  roll =
    atan2f(sinRoll, cosRoll);

  // Pitch quanh trục Y
  float sinPitch =
    2.0f * (qw * qy - qz * qx);

  sinPitch =
    constrain(
      sinPitch,
      -1.0f,
      1.0f);

  pitch =
    asinf(sinPitch);

  // Yaw quanh trục Z
  float sinYaw =
    2.0f * (qw * qz + qx * qy);

  float cosYaw =
    1.0f - 2.0f * (qy * qy + qz * qz);

  yaw =
    atan2f(sinYaw, cosYaw);

  const float RAD_TO_DEGREE =
    180.0f / PI;

  roll *= RAD_TO_DEGREE;
  pitch *= RAD_TO_DEGREE;
  yaw *= RAD_TO_DEGREE;
}

// ========================================================
// SAO CHÉP DỮ LIỆU IMU AN TOÀN
// ========================================================
IMUData getIMUSnapshot() {
  IMUData data;

  portENTER_CRITICAL(&imuMux);
  data = sharedIMU;
  portEXIT_CRITICAL(&imuMux);

  return data;
}

void requestImuZeroFromSerial() {
  IMUData data = getIMUSnapshot();
  if (data.sensorOK) {
    portENTER_CRITICAL(&commandMux);
    zeroRequested = true;
    portEXIT_CRITICAL(&commandMux);
    Serial.println("STATUS:ZERO_REQUESTED");
  } else {
    Serial.println("STATUS:BNO055_NOT_READY");
  }
}

bool initializeBNO055() {
  if (!bno.begin()) {
    return false;
  }

  bno.setExtCrystalUse(true);

  // Tổng thời gian khoảng 2 giây: chờ 1 giây, sau đó lấy trung bình 50 mẫu
  // trong 1 giây. Xe chỉ cần đứng yên, không cần lắc/nghiêng.
  Serial.println("STATUS:HOLD_STILL_2S_AUTO_ZERO");
  vTaskDelay(
    pdMS_TO_TICKS(ZERO_SETTLE_MS));

  if (!setAveragedQuaternionOffset()) {
    Serial.println("STATUS:AUTO_ZERO_SAMPLE_FAILED");
    return false;
  }

  return true;
}

// ========================================================
// TASK CORE 0:
// ĐỌC BNO055 VÀ TÍNH TOÁN QUATERNION
// ========================================================
void TaskIMU(void *pvParameters) {
  TickType_t lastWakeTime =
    xTaskGetTickCount();

  const TickType_t frequency =
    pdMS_TO_TICKS(IMU_INTERVAL_MS);

  unsigned long lastRetryTime = 0;
  unsigned long lastCalibrationTime = 0;
  uint8_t systemCalibration = 0;
  uint8_t gyroCalibration = 0;
  uint8_t accelCalibration = 0;
  uint8_t magCalibration = 0;
  int invalidSampleCount = 0;

  for (;;) {
    // Core 0 owns USB parsing/telemetry. Core 1 receives only compact commands
    // through liftCommandQueue and therefore never blocks on Serial output.
    const uint32_t serviceTimeMs = millis();
    readSerialCommands(serviceTimeMs);
    publishLiftTelemetry(serviceTimeMs);

    // Nếu chưa tìm thấy cảm biến, thử lại mỗi 2 giây
    if (!bnoAvailable) {
      // A data-integrity fault must not silently reinitialize and redefine
      // yaw while the robot may be moving. Reset ESP32 number 2 deliberately
      // after stopping the vehicle to clear this latch.
      if (bnoDataFaultLatched) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }

      unsigned long currentTime =
        millis();

      if (
        currentTime - lastRetryTime >= 2000) {
        lastRetryTime = currentTime;

        bnoAvailable =
          initializeBNO055();

        lastWakeTime = xTaskGetTickCount();

        portENTER_CRITICAL(&imuMux);
        sharedIMU.sensorOK =
          bnoAvailable;
        portEXIT_CRITICAL(&imuMux);
      }

      vTaskDelay(
        pdMS_TO_TICKS(100));

      continue;
    }

    // ----------------------------------------
    // Đọc quaternion hiện tại
    // ----------------------------------------
    imu::Quaternion currentQuaternion =
      bno.getQuat();

    float current_qw =
      currentQuaternion.w();

    float current_qx =
      currentQuaternion.x();

    float current_qy =
      currentQuaternion.y();

    float current_qz =
      currentQuaternion.z();

    // ----------------------------------------
    // Đọc gia tốc
    // ----------------------------------------
    sensors_event_t accelData{};
    sensors_event_t gyroData{};

    const bool accelReadOK = bno.getEvent(
      &accelData,
      Adafruit_BNO055::VECTOR_ACCELEROMETER);

    const bool gyroReadOK = bno.getEvent(
      &gyroData,
      Adafruit_BNO055::VECTOR_GYROSCOPE);

    const float quaternionNorm = sqrtf(
      current_qw * current_qw + current_qx * current_qx +
      current_qy * current_qy + current_qz * current_qz);
    const float accelerationNorm = sqrtf(
      accelData.acceleration.x * accelData.acceleration.x +
      accelData.acceleration.y * accelData.acceleration.y +
      accelData.acceleration.z * accelData.acceleration.z);
    const bool sampleValid = accelReadOK && gyroReadOK &&
      isfinite(quaternionNorm) && quaternionNorm > 0.5f &&
      quaternionNorm < 1.5f && isfinite(accelerationNorm) &&
      accelerationNorm >= IMU_MIN_ACCEL_NORM_M_S2 &&
      accelerationNorm <= IMU_MAX_ACCEL_NORM_M_S2 &&
      isfinite(gyroData.gyro.x) && isfinite(gyroData.gyro.y) &&
      isfinite(gyroData.gyro.z);

    if (!sampleValid) {
      invalidSampleCount++;
      if (invalidSampleCount >= IMU_INVALID_SAMPLE_LIMIT) {
        bnoDataFaultLatched = true;
        bnoAvailable = false;
        portENTER_CRITICAL(&imuMux);
        sharedIMU.sensorOK = false;
        portEXIT_CRITICAL(&imuMux);
        Serial.printf(
          "STATUS:BNO055_DATA_INVALID,qnorm=%.3f,anorm=%.3f\n",
          quaternionNorm, accelerationNorm);
      }
      vTaskDelayUntil(&lastWakeTime, frequency);
      continue;
    }
    invalidSampleCount = 0;

    // Apply a requested zero only after validating the complete sensor sample.
    if (takeZeroRequest()) {
      setQuaternionOffset(currentQuaternion);
      Serial.println("STATUS:ZERO_APPLIED");
    }

    normalizeQuaternion(
      current_qw,
      current_qx,
      current_qy,
      current_qz);

    // Q_relative = inverse(Q_offset) x Q_current.
    float relative_qw =
      inv_qw * current_qw - inv_qx * current_qx - inv_qy * current_qy - inv_qz * current_qz;
    float relative_qx =
      inv_qw * current_qx + inv_qx * current_qw + inv_qy * current_qz - inv_qz * current_qy;
    float relative_qy =
      inv_qw * current_qy - inv_qx * current_qz + inv_qy * current_qw + inv_qz * current_qx;
    float relative_qz =
      inv_qw * current_qz + inv_qx * current_qy - inv_qy * current_qx + inv_qz * current_qw;
    normalizeQuaternion(relative_qw, relative_qx, relative_qy, relative_qz);

    // IMU is mounted +90 degrees around Z. Convert it to robot REP-103.
    const float robot_qw = relative_qw;
    const float robot_qx = relative_qy;
    const float robot_qy = -relative_qx;
    const float robot_qz = relative_qz;

    float roll;
    float pitch;
    float yaw;
    quaternionToEuler(
      robot_qw, robot_qx, robot_qy, robot_qz, roll, pitch, yaw);

    // ----------------------------------------
    // Đọc calibration
    // ----------------------------------------
    const unsigned long calibrationTime = millis();
    if (lastCalibrationTime == 0 ||
        calibrationTime - lastCalibrationTime >= CALIBRATION_INTERVAL_MS) {
      bno.getCalibration(
        &systemCalibration,
        &gyroCalibration,
        &accelCalibration,
        &magCalibration);
      lastCalibrationTime = calibrationTime;
    }

    // ----------------------------------------
    // Ghi dữ liệu dùng chung cho loop()
    // ----------------------------------------
    portENTER_CRITICAL(&imuMux);

    sharedIMU.sensorOK = true;

    sharedIMU.qx = robot_qx;
    sharedIMU.qy = robot_qy;
    sharedIMU.qz = robot_qz;
    sharedIMU.qw = robot_qw;

    sharedIMU.roll = roll;
    sharedIMU.pitch = pitch;
    sharedIMU.yaw = yaw;

    sharedIMU.ax =
      accelData.acceleration.y;

    sharedIMU.ay =
      -accelData.acceleration.x;

    sharedIMU.az =
      accelData.acceleration.z;

    sharedIMU.gx = gyroData.gyro.y;
    sharedIMU.gy = -gyroData.gyro.x;
    sharedIMU.gz = gyroData.gyro.z;

    sharedIMU.calibSystem =
      systemCalibration;

    sharedIMU.calibGyro =
      gyroCalibration;

    sharedIMU.calibAccel =
      accelCalibration;

    sharedIMU.calibMag =
      magCalibration;

    sharedIMU.sampleCount++;
    sharedIMU.lastUpdateMs =
      millis();

    const IMUData output = sharedIMU;

    portEXIT_CRITICAL(&imuMux);

    // Protocol consumed by esp32_imu_bridge:
    // IMU,version,sequence,millis,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,S,G,A,M
    Serial.printf(
      "IMU,1,%lu,%lu,%.7f,%.7f,%.7f,%.7f,"
      "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u,%u\n",
      static_cast<unsigned long>(output.sampleCount),
      static_cast<unsigned long>(output.lastUpdateMs),
      output.qx, output.qy, output.qz, output.qw,
      output.gx, output.gy, output.gz,
      output.ax, output.ay, output.az,
      static_cast<unsigned int>(output.calibSystem),
      static_cast<unsigned int>(output.calibGyro),
      static_cast<unsigned int>(output.calibAccel),
      static_cast<unsigned int>(output.calibMag));

    // Nếu một giao dịch I2C bất thường làm task trễ nhiều chu kỳ, bắt đầu lịch
    // mới thay vì chạy liên tiếp nhiều vòng để "bù" và tạo burst trên USB/ROS.
    const TickType_t now = xTaskGetTickCount();
    if (now - lastWakeTime > frequency * 2) {
      lastWakeTime = now;
    }

    // Chu kỳ chính xác khoảng 20 ms (50 Hz).
    vTaskDelayUntil(
      &lastWakeTime,
      frequency);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Khong tao AP/STA va khong chay Bluetooth. Goi ro rang WIFI_OFF de
  // firmware van tat radio ke ca khi thu vien khac duoc them ve sau.
  (void)esp_wifi_stop();
  (void)esp_wifi_deinit();
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    (void)esp_bt_controller_disable();
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
    (void)esp_bt_controller_deinit();
  }

  liftCommandQueue = xQueueCreate(16, sizeof(LiftCommand));
  if (liftCommandQueue == nullptr) {
    Serial.println("STATUS:LIFT_COMMAND_QUEUE_CREATE_FAILED");
    while (true) delay(1000);
  }

  pinMode(LOWER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(UPPER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LIFT_ENABLE_PIN, OUTPUT);
  digitalWrite(LIFT_ENABLE_PIN, HIGH);
  lowerLimit.rawActive = digitalRead(LOWER_LIMIT_PIN) == LIMIT_ACTIVE_LEVEL;
  upperLimit.rawActive = digitalRead(UPPER_LIMIT_PIN) == LIMIT_ACTIVE_LEVEL;
  lowerLimit.changedAtMs = millis();
  upperLimit.changedAtMs = millis();

  // Dao rieng DIR de toa do am cua HOME chay xuong. STEP va EN khong dao.
  liftStepper.setPinsInverted(true, false, false);
  liftStepper.setMaxSpeed(metresToSteps(LIFT_MAX_SPEED_M_S));
  liftStepper.setAcceleration(metresToSteps(LIFT_ACCELERATION_M_S2));
  updateLiftSnapshot();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("BOOT,BNO055_IMU_LIFT,1");
  Serial.println("STATUS:INITIALIZING_BNO055");
  bnoAvailable = initializeBNO055();

  if (bnoAvailable) {
    sharedIMU.sensorOK = true;
    Serial.println("STATUS:BNO055_READY_ZERO_SET");
  } else {
    Serial.println("STATUS:BNO055_NOT_FOUND_RETRYING");
  }

  Serial.println("STATUS:SEND_Z_TO_RESET_ZERO");

  // IMU/USB trên Core 0; STEP có task chuyên dụng trên Core 1.
  BaseType_t imuResult = xTaskCreatePinnedToCore(
    TaskIMU, "IMU_Task", 6144, NULL, 2, NULL, 0);
  BaseType_t liftResult = xTaskCreatePinnedToCore(
    TaskLift, "Lift_Step_Task", 4096, NULL, 2, NULL, 1);

  if (imuResult != pdPASS || liftResult != pdPASS) {
    Serial.println("STATUS:CONTROL_TASK_CREATE_FAILED");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  // Mọi công việc thời gian thực đã nằm trong hai task ghim core.
  delay(1000);
}
