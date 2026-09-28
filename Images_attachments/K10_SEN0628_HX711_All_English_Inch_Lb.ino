#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "unihiker_k10.h"
#include "DFRobot_MatrixLidar.h"
#include "DFRobot_HX711_I2C.h"

UNIHIKER_K10 k10;

// -------------------- Hardware Parameters --------------------
static constexpr uint8_t HX711_I2C_ADDRESS = 0x64;

// Calibration value for the HX711 I2C module.
static constexpr float HX711_CALIBRATION_VALUE = 2236.0f;
static constexpr float WEIGHT_ZERO_BAND_G = 5.0f;

// -------------------- Weight Rules --------------------
// Dimensional weight: kg = L(cm) x W(cm) x H(cm) / 6000.
// Chargeable weight uses the larger of actual and dimensional weight, rounded up to 0.5 kg.
// Shipping cost is calculated by the web interface, not by the K10.
static constexpr float VOLUME_DIVISOR = 6000.0f;
static constexpr float CHARGE_UNIT_KG = 0.5f;
static constexpr float WEIGHT_STABLE_DELTA_G = 3.0f;
static constexpr uint8_t WEIGHT_STABLE_FRAMES = 4;
static constexpr uint32_t WEIGHT_READ_INTERVAL_MS = 100;
static constexpr uint8_t WEIGHT_MEDIAN_WINDOW = 5;
static constexpr float WEIGHT_FAST_JUMP_G = 35.0f;
static constexpr float WEIGHT_MEDIUM_JUMP_G = 10.0f;
static constexpr float WEIGHT_FAST_ALPHA = 0.78f;
static constexpr float WEIGHT_MEDIUM_ALPHA = 0.48f;
static constexpr float WEIGHT_SLOW_ALPHA = 0.22f;
static constexpr float WEIGHT_LOCK_BAND_G = 8.0f;

// Verified SEN0628 I2C address: 0x33.
DFRobot_MatrixLidar_I2C tof(0x33);

// -------------------- Depth and Geometry Parameters --------------------
static constexpr uint8_t GRID = 8;
static constexpr uint8_t ZONES = GRID * GRID;
static constexpr float FOV_X_DEG = 60.0f;
static constexpr float FOV_Y_DEG = 60.0f;

// -------------------- Dimension Calibration --------------------
// Reference box size: 300 x 200 x 150 mm.
// Current stable raw measurement: about 261 x 214 x 156 mm.
// Correct only final L/W/H values; detection logic remains unchanged.
static constexpr float LENGTH_CALIBRATION = 300.0f / 261.0f;
static constexpr float WIDTH_CALIBRATION  = 200.0f / 214.0f;
static constexpr float HEIGHT_CALIBRATION = 150.0f / 156.0f;

static constexpr uint16_t MIN_VALID_MM = 20;
static constexpr uint16_t MAX_VALID_MM = 3900;
static constexpr float FOREGROUND_START_MM = 35.0f;
static constexpr float FOREGROUND_FULL_MM  = 100.0f;
static constexpr float MASK_LEVEL = 0.35f;
static constexpr float EDGE_MIN_LEVEL = 0.50f;
static constexpr float EDGE_FRACTION = 0.60f;
static constexpr uint8_t MIN_COMPONENT_ZONES = 3;
static constexpr float MIN_PACKAGE_HEIGHT_MM = 45.0f;

static constexpr uint8_t BACKGROUND_FRAMES = 36;
static constexpr uint8_t BACKGROUND_MIN_FRAMES = 28;
static constexpr uint8_t BACKGROUND_VALID_ZONES_MIN = 40;
static constexpr uint32_t BACKGROUND_TIMEOUT_MS = 6000;
static constexpr uint8_t DEPTH_FILTER_FRAMES = 3;
static constexpr uint8_t DETECT_CONFIRM_FRAMES = 2;
static constexpr uint8_t REMOVE_CONFIRM_FRAMES = 4;
static constexpr uint8_t MEASURE_FILTER_FRAMES = 3;
static constexpr uint32_t FRAME_INTERVAL_MS = 60;
static constexpr uint32_t DISPLAY_INTERVAL_MS = 120;

// Rotate only the 8x8 display if required by the mounting orientation.
static constexpr bool ROTATE_MATRIX_180 = true;

// -------------------- Display Parameters --------------------
static constexpr uint8_t SCREEN_DIR = 2;
static constexpr int SCREEN_W = 240;
static constexpr int SCREEN_H = 320;
static constexpr int MAP_CELL = 20;
static constexpr int MAP_SIZE = MAP_CELL * GRID;
static constexpr int MAP_X = (SCREEN_W - MAP_SIZE) / 2;
static constexpr int MAP_Y = 40;

static constexpr uint32_t C_BG       = 0xFFFFFF;
static constexpr uint32_t C_HEADER   = 0xFFFFFF;
static constexpr uint32_t C_PANEL    = 0xE8F0F7;
static constexpr uint32_t C_BORDER   = 0xC5D3E0;
static constexpr uint32_t C_TEXT     = 0x1F2937;
static constexpr uint32_t C_MUTED    = 0x64748B;
static constexpr uint32_t C_GRID     = 0x149FE6;
static constexpr uint32_t C_BLUE     = 0x24A7F2;
static constexpr uint32_t C_GREEN    = 0x28D17C;
static constexpr uint32_t C_DONE_GREEN = 0x58E878;
static constexpr uint32_t C_YELLOW   = 0xF4D447;
static constexpr uint32_t C_RED      = 0xFF5B64;
static constexpr uint32_t C_INVALID  = 0x12486B;
static constexpr uint32_t C_HEAT_FRAME = 0x123A5B;
static constexpr uint32_t C_HEAT_GRID = 0x0A2B43;

DFRobot_HX711_I2C scaleI2C(&Wire, HX711_I2C_ADDRESS);
bool scaleConnected = false;
float filteredWeightG = 0.0f;
float previousWeightG = 0.0f;
float previousRobustWeightG = 0.0f;
float stableWeightG = 0.0f;
float weightHistoryG[WEIGHT_MEDIAN_WINDOW] = {0};
uint8_t weightHistoryCount = 0;
uint8_t weightHistoryHead = 0;
uint8_t weightStableFrames = 0;
bool weightLocked = false;
float displayedChargeableKg = 0.0f;
float displayedVolumeWeightKg = 0.0f;

// -------------------- Data and State --------------------
uint16_t rawDepth[ZONES] = {0};
uint16_t filteredDepth[ZONES] = {0};
uint16_t backgroundDepth[ZONES] = {0};
uint16_t backgroundSamples[BACKGROUND_FRAMES][ZONES] = {{0}};
uint16_t depthHistory[DEPTH_FILTER_FRAMES][ZONES] = {{0}};

uint8_t backgroundCount = 0;
uint8_t historyCount = 0;
uint8_t historyHead = 0;
bool backgroundReady = false;
float learningProgressDisplay = 0.0f;
uint32_t backgroundLearningStartMs = 0;

float confidenceMap[ZONES] = {0};
bool componentMask[ZONES] = {false};

struct Measurement {
  float lengthMm = 0;
  float widthMm = 0;
  float heightMm = 0;
  float distanceMm = 0;
  uint8_t zoneCount = 0;
  bool valid = false;
};

Measurement measurementHistory[MEASURE_FILTER_FRAMES];
uint8_t measurementCount = 0;
uint8_t measurementHead = 0;
Measurement displayedMeasurement;

bool packagePresent = false;
uint8_t detectedFrames = 0;
uint8_t emptyFrames = 0;
uint32_t lastFrameMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastSerialMs = 0;
bool previousButtonA = false;
bool previousButtonB = false;

// -------------------- Utilities --------------------
bool validDistance(uint16_t mm) {
  return mm >= MIN_VALID_MM && mm < MAX_VALID_MM;
}

float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float zoneAngleRad(uint8_t index, float fovDeg) {
  const float normalizedCenter = (index + 0.5f) / GRID;
  return (normalizedCenter - 0.5f) * fovDeg * DEG_TO_RAD;
}

float axialDistanceMm(uint16_t radialMm, uint8_t row, uint8_t col) {
  const float tx = tanf(zoneAngleRad(col, FOV_X_DEG));
  const float ty = tanf(zoneAngleRad(row, FOV_Y_DEG));
  return radialMm / sqrtf(1.0f + tx * tx + ty * ty);
}

template <typename T>
void insertionSort(T *values, uint8_t count) {
  for (uint8_t i = 1; i < count; ++i) {
    const T key = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = key;
  }
}

float medianFloat(float *values, uint8_t count) {
  if (count == 0) return 0.0f;
  insertionSort(values, count);
  if (count & 1U) return values[count / 2];
  return 0.5f * (values[count / 2 - 1] + values[count / 2]);
}

uint16_t medianU16(uint16_t *values, uint8_t count) {
  if (count == 0) return 0;
  insertionSort(values, count);
  if (count & 1U) return values[count / 2];
  return static_cast<uint16_t>((static_cast<uint32_t>(values[count / 2 - 1]) +
                                values[count / 2]) / 2U);
}

uint8_t displayIndex(uint8_t row, uint8_t col) {
  if (ROTATE_MATRIX_180) return (GRID - 1 - row) * GRID + (GRID - 1 - col);
  return row * GRID + col;
}

void setRgb(uint32_t color) {
  k10.rgb->setRangeColor(0, 2, color);
}

// -------------------- Background and Depth Filtering --------------------
void resetMeasurementState() {
  packagePresent = false;
  detectedFrames = 0;
  emptyFrames = 0;
  measurementCount = 0;
  measurementHead = 0;
  displayedMeasurement = Measurement();
  displayedChargeableKg = 0.0f;
  displayedVolumeWeightKg = 0.0f;
  weightStableFrames = 0;
}

void startBackgroundLearning() {
  backgroundReady = false;
  backgroundCount = 0;
  learningProgressDisplay = 0.0f;
  backgroundLearningStartMs = millis();
  historyCount = 0;
  historyHead = 0;
  memset(backgroundDepth, 0, sizeof(backgroundDepth));
  memset(backgroundSamples, 0, sizeof(backgroundSamples));
  memset(depthHistory, 0, sizeof(depthHistory));
  resetMeasurementState();
  setRgb(C_YELLOW);
  Serial.println("Background learning started. Keep the platform empty.");
}

void finishBackgroundLearning() {
  for (uint8_t zone = 0; zone < ZONES; ++zone) {
    uint16_t values[BACKGROUND_FRAMES];
    uint8_t count = 0;
    for (uint8_t frame = 0; frame < backgroundCount; ++frame) {
      const uint16_t value = backgroundSamples[frame][zone];
      if (validDistance(value)) values[count++] = value;
    }
    backgroundDepth[zone] = count >= BACKGROUND_FRAMES / 2
                              ? medianU16(values, count)
                              : 0;
  }
  backgroundReady = true;
  historyCount = 0;
  historyHead = 0;
  setRgb(C_BLUE);
  Serial.println("Background learning complete.");
}

void addBackgroundFrame(const uint16_t *frame) {
  if (backgroundCount >= BACKGROUND_FRAMES) return;

  uint8_t validCount = 0;
  for (uint8_t i = 0; i < ZONES; ++i) {
    backgroundSamples[backgroundCount][i] = frame[i];
    if (validDistance(frame[i])) ++validCount;
  }

  // Reject frames with too many invalid zones.
  // A relaxed valid-zone threshold improves learning reliability.
  // Timeout protection prevents the learning progress from stalling.
  if (validCount >= BACKGROUND_VALID_ZONES_MIN) {
    ++backgroundCount;
  }

  if (backgroundCount >= BACKGROUND_FRAMES) {
    finishBackgroundLearning();
    return;
  }

  // Normally 36 frames are collected quickly.
  // If enough samples are available, finish learning after the timeout.
  if (backgroundLearningStartMs != 0 &&
      millis() - backgroundLearningStartMs >= BACKGROUND_TIMEOUT_MS &&
      backgroundCount >= BACKGROUND_MIN_FRAMES) {
    finishBackgroundLearning();
  }
}

void updateDepthMedian(const uint16_t *frame) {
  memcpy(depthHistory[historyHead], frame, sizeof(uint16_t) * ZONES);
  historyHead = (historyHead + 1) % DEPTH_FILTER_FRAMES;
  if (historyCount < DEPTH_FILTER_FRAMES) ++historyCount;

  for (uint8_t zone = 0; zone < ZONES; ++zone) {
    uint16_t values[DEPTH_FILTER_FRAMES];
    uint8_t count = 0;
    for (uint8_t h = 0; h < historyCount; ++h) {
      const uint16_t value = depthHistory[h][zone];
      if (validDistance(value)) values[count++] = value;
    }
    filteredDepth[zone] = medianU16(values, count);
  }
}

// -------------------- Foreground Extraction and Geometry --------------------
void buildConfidenceMap() {
  for (uint8_t row = 0; row < GRID; ++row) {
    for (uint8_t col = 0; col < GRID; ++col) {
      const uint8_t i = row * GRID + col;
      if (!validDistance(backgroundDepth[i]) || !validDistance(filteredDepth[i])) {
        confidenceMap[i] = 0.0f;
        continue;
      }

      const float backgroundZ = axialDistanceMm(backgroundDepth[i], row, col);
      const float currentZ = axialDistanceMm(filteredDepth[i], row, col);
      const float deltaZ = backgroundZ - currentZ;
      confidenceMap[i] = clamp01((deltaZ - FOREGROUND_START_MM) /
                                 (FOREGROUND_FULL_MM - FOREGROUND_START_MM));
    }
  }
}

uint8_t keepLargestComponent() {
  bool visited[ZONES] = {false};
  bool bestMask[ZONES] = {false};
  uint8_t bestCount = 0;

  for (uint8_t seed = 0; seed < ZONES; ++seed) {
    if (visited[seed] || confidenceMap[seed] < MASK_LEVEL) continue;

    uint8_t queue[ZONES];
    uint8_t members[ZONES];
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t memberCount = 0;
    queue[tail++] = seed;
    visited[seed] = true;

    while (head < tail) {
      const uint8_t current = queue[head++];
      members[memberCount++] = current;
      const int row = current / GRID;
      const int col = current % GRID;
      const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
      const int dc[8] = {-1,  0,  1,-1, 1,-1, 0, 1};

      for (uint8_t n = 0; n < 8; ++n) {
        const int nr = row + dr[n];
        const int nc = col + dc[n];
        if (nr < 0 || nr >= GRID || nc < 0 || nc >= GRID) continue;
        const uint8_t next = nr * GRID + nc;
        if (!visited[next] && confidenceMap[next] >= MASK_LEVEL) {
          visited[next] = true;
          queue[tail++] = next;
        }
      }
    }

    if (memberCount > bestCount) {
      memset(bestMask, 0, sizeof(bestMask));
      for (uint8_t i = 0; i < memberCount; ++i) bestMask[members[i]] = true;
      bestCount = memberCount;
    }
  }

  memcpy(componentMask, bestMask, sizeof(componentMask));
  return bestCount;
}

bool profileEdges(const float profile[GRID], float &left, float &right) {
  float peak = 0.0f;
  for (uint8_t i = 0; i < GRID; ++i) peak = max(peak, profile[i]);
  if (peak < EDGE_MIN_LEVEL) return false;

  // Use a stricter interior threshold for the measured boundary. The
  // connected-component mask stays permissive for presence detection, while
  // weak halo cells at the edge are excluded from L/W estimation.
  const float level = max(EDGE_MIN_LEVEL, peak * EDGE_FRACTION);
  int first = -1;
  int last = -1;
  for (uint8_t i = 0; i < GRID; ++i) {
    if (profile[i] >= level) {
      if (first < 0) first = i;
      last = i;
    }
  }
  if (first < 0) return false;

  // Interpolate profile samples to estimate continuous boundaries from 0 to 8.
  const float leftInnerX = first + 0.5f;
  const float leftOuterX = first - 0.5f;
  const float leftInnerV = profile[first];
  const float leftOuterV = first > 0 ? profile[first - 1] : 0.0f;
  const float leftDen = leftInnerV - leftOuterV;
  left = fabsf(leftDen) > 0.0001f
           ? leftOuterX + (level - leftOuterV) / leftDen
           : static_cast<float>(first);

  const float rightInnerX = last + 0.5f;
  const float rightOuterX = last + 1.5f;
  const float rightInnerV = profile[last];
  const float rightOuterV = last < GRID - 1 ? profile[last + 1] : 0.0f;
  const float rightDen = rightOuterV - rightInnerV;
  right = fabsf(rightDen) > 0.0001f
            ? rightInnerX + (level - rightInnerV) / rightDen
            : static_cast<float>(last + 1);

  left = constrain(left, 0.0f, static_cast<float>(GRID));
  right = constrain(right, 0.0f, static_cast<float>(GRID));
  return right > left;
}

float gridBoundaryAngle(float boundary, float fovDeg) {
  return ((boundary / GRID) - 0.5f) * fovDeg * DEG_TO_RAD;
}

Measurement analyzeForeground() {
  Measurement result;

  buildConfidenceMap();
  const uint8_t componentCount = keepLargestComponent();
  if (componentCount < MIN_COMPONENT_ZONES) return result;

  // A summed profile is more stable than taking only the strongest pixel in
  // each row/column. This especially helps when the same box is placed on a
  // different face and occupies fewer 8x8 zones.
  float columnProfile[GRID] = {0};
  float rowProfile[GRID] = {0};

  float allTopZ[ZONES];
  float allHeight[ZONES];
  float allRadial[ZONES];
  uint8_t allCount = 0;

  // Core points are less affected by mixed pixels on package edges.
  float coreTopZ[ZONES];
  float coreHeight[ZONES];
  float coreRadial[ZONES];
  uint8_t coreCount = 0;

  for (uint8_t row = 0; row < GRID; ++row) {
    for (uint8_t col = 0; col < GRID; ++col) {
      const uint8_t i = row * GRID + col;
      if (!componentMask[i]) continue;

      const float confidence = confidenceMap[i];

      columnProfile[col] += confidence;
      rowProfile[row] += confidence;

      const float currentZ =
          axialDistanceMm(filteredDepth[i], row, col);
      const float backgroundZ =
          axialDistanceMm(backgroundDepth[i], row, col);
      const float h = backgroundZ - currentZ;

      allTopZ[allCount] = currentZ;
      allHeight[allCount] = h;
      allRadial[allCount] = filteredDepth[i];
      ++allCount;

      // Prefer the solid interior of the top face for Z/height estimation.
      if (confidence >= 0.72f) {
        coreTopZ[coreCount] = currentZ;
        coreHeight[coreCount] = h;
        coreRadial[coreCount] = filteredDepth[i];
        ++coreCount;
      }
    }
  }

  if (allCount == 0) return result;

  // Normalize each 1D profile to 0..1 before sub-cell edge interpolation.
  float maxColumn = 0.0f;
  float maxRow = 0.0f;

  for (uint8_t i = 0; i < GRID; ++i) {
    maxColumn = max(maxColumn, columnProfile[i]);
    maxRow = max(maxRow, rowProfile[i]);
  }

  if (maxColumn <= 0.0001f || maxRow <= 0.0001f) return result;

  for (uint8_t i = 0; i < GRID; ++i) {
    columnProfile[i] /= maxColumn;
    rowProfile[i] /= maxRow;
  }

  float left = 0, right = 0, top = 0, bottom = 0;

  if (!profileEdges(columnProfile, left, right) ||
      !profileEdges(rowProfile, top, bottom)) {
    return result;
  }

  // If at least two solid interior zones exist, use them for top distance and
  // height. Otherwise fall back to the whole connected component.
  const bool useCore = coreCount >= 2;

  const float topZ =
      useCore ? medianFloat(coreTopZ, coreCount)
              : medianFloat(allTopZ, allCount);

  const float height =
      useCore ? medianFloat(coreHeight, coreCount)
              : medianFloat(allHeight, allCount);

  const float radial =
      useCore ? medianFloat(coreRadial, coreCount)
              : medianFloat(allRadial, allCount);

  if (topZ <= 0.0f || height < MIN_PACKAGE_HEIGHT_MM) return result;

  const float angleLeft =
      gridBoundaryAngle(left, FOV_X_DEG);
  const float angleRight =
      gridBoundaryAngle(right, FOV_X_DEG);
  const float angleTop =
      gridBoundaryAngle(top, FOV_Y_DEG);
  const float angleBottom =
      gridBoundaryAngle(bottom, FOV_Y_DEG);

  const float sizeX =
      topZ * fabsf(tanf(angleRight) - tanf(angleLeft));

  const float sizeY =
      topZ * fabsf(tanf(angleBottom) - tanf(angleTop));

  // Apply three-axis calibration for the current fixed installation.
  result.lengthMm = max(sizeX, sizeY) * LENGTH_CALIBRATION;
  result.widthMm  = min(sizeX, sizeY) * WIDTH_CALIBRATION;
  result.heightMm = height * HEIGHT_CALIBRATION;
  result.distanceMm = radial;
  result.zoneCount = componentCount;

  result.valid =
      result.lengthMm >= 20.0f &&
      result.widthMm >= 20.0f;

  return result;
}

void addMeasurement(const Measurement &measurement) {
  // If the parcel has clearly changed orientation, discard the old short
  // history so the display does not drag the previous pose for several frames.
  if (displayedMeasurement.valid) {
    const float dL =
        fabsf(measurement.lengthMm - displayedMeasurement.lengthMm);
    const float dW =
        fabsf(measurement.widthMm - displayedMeasurement.widthMm);
    const float dH =
        fabsf(measurement.heightMm - displayedMeasurement.heightMm);

    if (dL > 45.0f || dW > 45.0f || dH > 35.0f) {
      measurementCount = 0;
      measurementHead = 0;
    }
  }

  measurementHistory[measurementHead] = measurement;
  measurementHead = (measurementHead + 1) % MEASURE_FILTER_FRAMES;

  if (measurementCount < MEASURE_FILTER_FRAMES) {
    ++measurementCount;
  }

  float lengths[MEASURE_FILTER_FRAMES];
  float widths[MEASURE_FILTER_FRAMES];
  float heights[MEASURE_FILTER_FRAMES];
  float distances[MEASURE_FILTER_FRAMES];

  for (uint8_t i = 0; i < measurementCount; ++i) {
    lengths[i] = measurementHistory[i].lengthMm;
    widths[i] = measurementHistory[i].widthMm;
    heights[i] = measurementHistory[i].heightMm;
    distances[i] = measurementHistory[i].distanceMm;
  }

  const float newLength =
      medianFloat(lengths, measurementCount);
  const float newWidth =
      medianFloat(widths, measurementCount);
  const float newHeight =
      medianFloat(heights, measurementCount);
  const float newDistance =
      medianFloat(distances, measurementCount);

  if (!displayedMeasurement.valid) {
    displayedMeasurement.lengthMm = newLength;
    displayedMeasurement.widthMm = newWidth;
    displayedMeasurement.heightMm = newHeight;
    displayedMeasurement.distanceMm = newDistance;
  } else {
    // Deadband suppresses tiny 8x8 edge jitter, while larger changes respond
    // immediately enough for flipping the same parcel onto another face.
    if (fabsf(newLength - displayedMeasurement.lengthMm) > 2.5f) {
      displayedMeasurement.lengthMm =
          0.35f * displayedMeasurement.lengthMm +
          0.65f * newLength;
    }

    if (fabsf(newWidth - displayedMeasurement.widthMm) > 2.5f) {
      displayedMeasurement.widthMm =
          0.35f * displayedMeasurement.widthMm +
          0.65f * newWidth;
    }

    if (fabsf(newHeight - displayedMeasurement.heightMm) > 2.0f) {
      displayedMeasurement.heightMm =
          0.30f * displayedMeasurement.heightMm +
          0.70f * newHeight;
    }

    if (fabsf(newDistance - displayedMeasurement.distanceMm) > 2.0f) {
      displayedMeasurement.distanceMm =
          0.35f * displayedMeasurement.distanceMm +
          0.65f * newDistance;
    }
  }

  displayedMeasurement.zoneCount = measurement.zoneCount;
  displayedMeasurement.valid = true;
}

void updatePresenceState(const Measurement &current) {
  if (current.valid) {
    emptyFrames = 0;
    if (detectedFrames < DETECT_CONFIRM_FRAMES) ++detectedFrames;
    if (detectedFrames >= DETECT_CONFIRM_FRAMES) {
      if (!packagePresent) {
        packagePresent = true;
        measurementCount = 0;
        measurementHead = 0;
        setRgb(C_GREEN);
      }
      // Keep updating while the parcel remains in view. Removal is confirmed
      // only after several invalid frames below.
      addMeasurement(current);
    }
  } else {
    detectedFrames = 0;
    if (packagePresent) {
      if (emptyFrames < REMOVE_CONFIRM_FRAMES) ++emptyFrames;
      if (emptyFrames >= REMOVE_CONFIRM_FRAMES) {
        resetMeasurementState();
        memset(componentMask, 0, sizeof(componentMask));
        setRgb(C_BLUE);
      }
    }
  }
}

// -------------------- Weight --------------------
void resetWeightFilterState() {
  filteredWeightG = 0.0f;
  previousWeightG = 0.0f;
  previousRobustWeightG = 0.0f;
  stableWeightG = 0.0f;

  weightHistoryCount = 0;
  weightHistoryHead = 0;
  weightStableFrames = 0;
  weightLocked = false;

  for (uint8_t i = 0; i < WEIGHT_MEDIAN_WINDOW; ++i) {
    weightHistoryG[i] = 0.0f;
  }
}

float robustWeightMedian() {
  if (weightHistoryCount == 0) return 0.0f;

  float values[WEIGHT_MEDIAN_WINDOW];

  for (uint8_t i = 0; i < weightHistoryCount; ++i) {
    values[i] = weightHistoryG[i];
  }

  return medianFloat(values, weightHistoryCount);
}

void initializeScale() {
  scaleConnected = scaleI2C.begin();

  if (!scaleConnected) {
    Serial.println("HX711 not detected; dimensions remain available.");
    return;
  }

  scaleI2C.setCalibration(HX711_CALIBRATION_VALUE);
  scaleI2C.peel();
  resetWeightFilterState();

  Serial.println("HX711 I2C ready and tared.");
}

void updateWeight() {
  if (!scaleConnected) return;

  static uint32_t lastWeightReadMs = 0;
  const uint32_t now = millis();

  if (lastWeightReadMs != 0 &&
      now - lastWeightReadMs < WEIGHT_READ_INTERVAL_MS) {
    return;
  }

  lastWeightReadMs = now;

  // Four-corner platform: use a small sensor-side average first, then a
  // 5-sample median to reject mechanical spikes from the four feet.
  float grams = fabsf(scaleI2C.readWeight(4));

  if (grams < WEIGHT_ZERO_BAND_G) {
    grams = 0.0f;
  }

  weightHistoryG[weightHistoryHead] = grams;
  weightHistoryHead =
      (weightHistoryHead + 1) % WEIGHT_MEDIAN_WINDOW;

  if (weightHistoryCount < WEIGHT_MEDIAN_WINDOW) {
    ++weightHistoryCount;
  }

  const float robustG = robustWeightMedian();

  // If a previously locked reading changes meaningfully, immediately unlock.
  if (weightLocked &&
      fabsf(robustG - stableWeightG) > WEIGHT_LOCK_BAND_G) {
    weightLocked = false;
    weightStableFrames = 0;
  }

  if (!weightLocked) {
    if (weightHistoryCount == 1) {
      filteredWeightG = robustG;
    } else {
      const float diff =
          fabsf(robustG - filteredWeightG);

      float alpha = WEIGHT_SLOW_ALPHA;

      if (diff >= WEIGHT_FAST_JUMP_G) {
        alpha = WEIGHT_FAST_ALPHA;
      } else if (diff >= WEIGHT_MEDIUM_JUMP_G) {
        alpha = WEIGHT_MEDIUM_ALPHA;
      }

      filteredWeightG +=
          alpha * (robustG - filteredWeightG);
    }

    if (filteredWeightG < WEIGHT_ZERO_BAND_G) {
      filteredWeightG = 0.0f;
    }

    // Stability is judged from the robust sensor value, not from the EMA.
    // This avoids locking while the displayed value is still slowly climbing.
    if (weightHistoryCount >= 3 &&
        fabsf(robustG - previousRobustWeightG)
            <= WEIGHT_STABLE_DELTA_G) {

      if (weightStableFrames < WEIGHT_STABLE_FRAMES) {
        ++weightStableFrames;
      }
    } else {
      weightStableFrames = 0;
    }

    if (weightStableFrames >= WEIGHT_STABLE_FRAMES) {
      stableWeightG = filteredWeightG;
      weightLocked = true;
      filteredWeightG = stableWeightG;
    }
  } else {
    // Hold a stable number instead of letting the display creep by 1 g.
    filteredWeightG = stableWeightG;
  }

  previousRobustWeightG = robustG;
  previousWeightG = filteredWeightG;
}

// -------------------- Display: Simple UI V4 --------------------
void fillScreen(uint32_t color) {
  k10.canvas->canvasRectangle(0, 0, SCREEN_W, SCREEN_H, color, color, true);
}

void drawText(const String &text, int x, int y, uint32_t color,
              Canvas::eFontSize_t font = Canvas::eCNAndENFont16) {
  k10.canvas->canvasText(text, x, y, color, font, 30, false);
}

String formatInchNumber(float mm) {
  return String(mm / 25.4f, 1);
}

String formatActualWeightLb() {
  if (!scaleConnected) return "--";
  return String(max(0.0f, filteredWeightG) / 453.59237f, 2) + " lb";
}

String formatDimensionalWeightLb() {
  return String(displayedVolumeWeightKg * 2.20462262f, 2) + " lb";
}

uint32_t matrixCellColor(uint8_t rawIndex) {
  // Waiting: all cells are blue.
  if (!packagePresent && detectedFrames == 0) {
    return C_BLUE;
  }

  // Scanning: detected package cells are yellow.
  if (!packagePresent && detectedFrames > 0) {
    return componentMask[rawIndex] ? C_YELLOW : C_BLUE;
  }

  // Completed: detected package cells are green.
  if (packagePresent) {
    return componentMask[rawIndex] ? C_DONE_GREEN : C_BLUE;
  }

  return C_BLUE;
}

void drawStateMatrix() {
  // Keep the visual style close to the earlier 8x8 heatmap:
  // solid-filled cells with only a thin dark grid between them.
  const int frameX = MAP_X - 2;
  const int frameY = MAP_Y - 2;

  k10.canvas->canvasSetLineWidth(1);

  k10.canvas->canvasRectangle(
      frameX, frameY, MAP_SIZE + 4, MAP_SIZE + 4,
      C_HEAT_FRAME, C_HEAT_FRAME, true
  );

  for (uint8_t row = 0; row < GRID; ++row) {
    for (uint8_t col = 0; col < GRID; ++col) {
      const uint8_t rawIndex = displayIndex(row, col);
      const int x = MAP_X + col * MAP_CELL;
      const int y = MAP_Y + row * MAP_CELL;
      const uint32_t fillColor = matrixCellColor(rawIndex);

      // Dark outer cell.
      k10.canvas->canvasRectangle(
          x, y, MAP_CELL, MAP_CELL,
          C_HEAT_GRID, C_HEAT_GRID, true
      );

      // Fill almost the entire cell, leaving a 1 px grid line.
      k10.canvas->canvasRectangle(
          x + 1, y + 1, MAP_CELL - 2, MAP_CELL - 2,
          fillColor, fillColor, true
      );
    }
  }
}

void drawTitle() {
  drawText("Package Size", 48, 7, C_TEXT, Canvas::eCNAndENFont24);
}

void drawLearningScreen() {
  fillScreen(C_BG);

  // Show only the calibration prompt during background learning.
  drawText("Learning", 76, 72,
           C_BLUE, Canvas::eCNAndENFont24);

  // Center the instruction below the title.
  drawText("Keep platform empty", 39, 132,
           C_MUTED, Canvas::eCNAndENFont16);

  const float targetProgress =
      backgroundCount * 100.0f / BACKGROUND_FRAMES;

  learningProgressDisplay +=
      (targetProgress - learningProgressDisplay) * 0.32f;

  if (fabsf(targetProgress - learningProgressDisplay) < 0.2f) {
    learningProgressDisplay = targetProgress;
  }

  int progress =
      static_cast<int>(learningProgressDisplay + 0.5f);

  if (backgroundReady) progress = 100;
  progress = constrain(progress, 0, 100);

  k10.canvas->canvasRectangle(
      32, 190, 176, 14,
      C_BORDER, C_PANEL, true
  );

  k10.canvas->canvasRectangle(
      34, 192,
      172 * progress / 100,
      10,
      C_BLUE, C_BLUE, true
  );

  drawText(
      String(progress) + "%",
      progress >= 100 ? 86 : 94,
      220,
      C_TEXT,
      Canvas::eCNAndENFont24
  );

  k10.canvas->updateCanvas();
}

float roundChargeWeight(float kg) {
  if (kg <= 0.0f) return 0.0f;
  return ceilf(kg / CHARGE_UNIT_KG - 0.0001f) * CHARGE_UNIT_KG;
}

void updateBilling() {
  if (!packagePresent || !displayedMeasurement.valid) {
    displayedChargeableKg = 0.0f;
    displayedVolumeWeightKg = 0.0f;
    return;
  }

  const float actualWeightKg =
      max(0.0f, filteredWeightG) / 1000.0f;

  const float lengthCm =
      displayedMeasurement.lengthMm / 10.0f;
  const float widthCm =
      displayedMeasurement.widthMm / 10.0f;
  const float heightCm =
      displayedMeasurement.heightMm / 10.0f;

  displayedVolumeWeightKg =
      (lengthCm * widthCm * heightCm) / VOLUME_DIVISOR;

  displayedChargeableKg =
      roundChargeWeight(
          max(actualWeightKg, displayedVolumeWeightKg)
      );
}

void drawDimensionLine() {
  String sizeText = "-- x -- x -- in";

  if (packagePresent && displayedMeasurement.valid) {
    sizeText =
        formatInchNumber(displayedMeasurement.lengthMm) +
        " x " +
        formatInchNumber(displayedMeasurement.widthMm) +
        " x " +
        formatInchNumber(displayedMeasurement.heightMm) +
        " in";
  }

  // Keep the whole dimension string inside the screen.
  // Use a smaller horizontal offset and center according to text length.
  int x = 6;

  if (sizeText.length() <= 16) {
    x = 22;
  } else if (sizeText.length() <= 20) {
    x = 12;
  }

  drawText(
      sizeText,
      x, 202,
      C_TEXT,
      Canvas::eCNAndENFont24
  );
}

void drawWeightCards() {
  // Left card: volumetric weight.
  k10.canvas->canvasRectangle(
      12, 245, 102, 62,
      C_PANEL, C_PANEL, true
  );

  drawText("Dim. Weight", 24, 252, C_MUTED);

  if (packagePresent && displayedMeasurement.valid) {
    drawText(
        formatDimensionalWeightLb(),
        20, 274, C_TEXT, Canvas::eCNAndENFont24
    );
  } else {
    drawText(
        "--",
        49, 274, C_MUTED, Canvas::eCNAndENFont24
    );
  }

  // Right card: actual weight.
  k10.canvas->canvasRectangle(
      126, 245, 102, 62,
      0xECFBF3, 0xECFBF3, true
  );

  drawText("Actual Weight", 132, 252, C_MUTED);

  drawText(
      formatActualWeightLb(),
      143, 274,
      scaleConnected ? C_GREEN : C_RED,
      Canvas::eCNAndENFont24
  );
}

void drawMainScreen(const Measurement &current) {
  (void)current;

  const uint32_t now = millis();

  if (lastDisplayMs != 0 &&
      now - lastDisplayMs < DISPLAY_INTERVAL_MS) {
    return;
  }

  lastDisplayMs = now;

  fillScreen(C_BG);
  drawTitle();
  drawStateMatrix();
  drawDimensionLine();
  drawWeightCards();

  k10.canvas->updateCanvas();
}

void drawSensorError() {
  fillScreen(C_BG);

  drawText(
      "Sensor Error",
      60, 82,
      C_RED,
      Canvas::eCNAndENFont24
  );

  drawText("SEN0628 not found", 45, 142, C_TEXT);
  drawText("Check I2C wiring", 50, 178, C_MUTED);
  drawText("Retrying...", 77, 214, C_MUTED);

  k10.canvas->updateCanvas();
}

// -------------------- Initialization and Buttons --------------------
void handleButtons() {
  const bool buttonA = k10.buttonA->isPressed();
  const bool buttonB = k10.buttonB->isPressed();

  if (buttonA && !previousButtonA) startBackgroundLearning();

  if (buttonB && !previousButtonB && scaleConnected) {
    fillScreen(C_BG);
    drawText("Tare Scale", 60, 128, C_YELLOW, Canvas::eCNAndENFont24);
    drawText("Keep platform empty", 39, 178, C_TEXT);
    k10.canvas->updateCanvas();
    scaleI2C.peel();
    scaleConnected = true;
    resetWeightFilterState();
    Serial.println("HX711 tare complete.");
  }

  previousButtonA = buttonA;
  previousButtonB = buttonB;
}

void printDiagnostics(const Measurement &current) {
  if (millis() - lastSerialMs < 500) return;
  lastSerialMs = millis();

  Serial.print("state=");
  Serial.print(backgroundReady ? (packagePresent ? "PRESENT" : "WAITING") : "LEARNING");
  Serial.print(" zones=");
  Serial.print(current.zoneCount);
  Serial.print(" L/W/H(mm)=");

  if (displayedMeasurement.valid) {
    Serial.print(displayedMeasurement.lengthMm, 1);
    Serial.print('/');
    Serial.print(displayedMeasurement.widthMm, 1);
    Serial.print('/');
    Serial.print(displayedMeasurement.heightMm, 1);
  } else {
    Serial.print("--/--/--");
  }

  Serial.print(" weight(g)=");
  Serial.print(filteredWeightG, 1);
  Serial.print(" volumeWeight(kg)=");
  Serial.print(displayedVolumeWeightKg, 2);
  Serial.print(" chargeWeight(kg)=");
  Serial.print(displayedChargeableKg, 1);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("K10 + SEN0628 + HX711 Parcel System");

  k10.begin();
  k10.initScreen(SCREEN_DIR);
  k10.creatCanvas();
  k10.setScreenBackground(C_BG);
  k10.rgb->brightness(20);
  setRgb(C_YELLOW);

  // SEN0628 initialization uses the API syntax verified in Arduino IDE.
  int beginResult = tof.begin();
  Serial.print("tof.begin result = ");
  Serial.println(beginResult);

  if (beginResult != 0) {
    Serial.println("SEN0628 BEGIN FAILED");
    drawSensorError();
    while (1) delay(100);
  }
  Serial.println("SEN0628 BEGIN SUCCESS");

  int modeResult = tof.setRangingMode(eMatrix_8X8);
  Serial.print("setRangingMode result = ");
  Serial.println(modeResult);

  if (modeResult != 0) {
    Serial.println("8x8 MODE FAILED");
    drawSensorError();
    while (1) delay(100);
  }
  Serial.println("8x8 MODE SUCCESS");

  initializeScale();
  startBackgroundLearning();
}

void loop() {
  handleButtons();
  updateWeight();

  const uint32_t now = millis();
  if (now - lastFrameMs < FRAME_INTERVAL_MS) {
    delay(2);
    return;
  }
  lastFrameMs = now;

  uint8_t readResult = tof.getAllData(rawDepth);
  if (readResult != 0) {
    Serial.print("getAllData failed: ");
    Serial.println(readResult);
    delay(20);
    return;
  }

  if (!backgroundReady) {
    addBackgroundFrame(rawDepth);
    drawLearningScreen();
    return;
  }

  updateDepthMedian(rawDepth);
  Measurement current;
  if (historyCount >= 3) current = analyzeForeground();
  updatePresenceState(current);
  updateBilling();
  drawMainScreen(current);
  printDiagnostics(current);
}
