#include "scanner.h"
#include "project_config.h"
#include "debug.h"
#include "servo.h"
#include "srf05.h"
#include <math.h>
#include <stdbool.h>

typedef struct
{
  uint8_t angleDeg;
  float d05Cm;
  float d04Cm;
  ScanTargetType type;
} ScanPoint;

typedef struct
{
  uint8_t startDeg;
  uint8_t endDeg;
  uint8_t centerDeg;
  uint8_t count;
  float clusterDist;
} ScanCluster;

ScanTargetType scanClassifyTarget(float d05RawCm, float d04RawCm)
{
  if (d05RawCm < SCAN_MIN_RANGE_CM || d05RawCm > SCAN_MAX_RANGE_CM)
    return TARGET_NONE;

  if (d04RawCm == ECHO_ERROR_TIMEOUT)
    return TARGET_NONE;

  if (d04RawCm == ECHO_ERROR_TOO_CLOSE ||
      (d04RawCm > 0.0f && d04RawCm < SCAN_MIN_RANGE_CM))
    return TARGET_OBSTACLE;

  if (d04RawCm > SRF04_MAX_RANGE_CM)
    return TARGET_OBJECT;

  if (d04RawCm >= SCAN_MIN_RANGE_CM)
  {
    float d04AdjCm = d04RawCm - SENSOR_DEPTH_OFFSET_CM;
    if (d04AdjCm <= (d05RawCm + SCAN_OBSTACLE_TOLERANCE_CM))
      return TARGET_OBSTACLE;
  }
  return TARGET_OBJECT;
}

// Tính trung vị của mảng số thực (sắp xếp tại chỗ)
static float computeMedian(float *vals, uint8_t count)
{
  if (count == 0u)
    return ECHO_ERROR_TIMEOUT;

  for (uint8_t i = 1u; i < count; i++)
  {
    float key = vals[i];
    uint8_t j = i;
    while (j > 0u && vals[j - 1u] > key)
    {
      vals[j] = vals[j - 1u];
      j--;
    }
    vals[j] = key;
  }

  if ((count & 1u) != 0u)
    return vals[count / 2u];
  return (vals[count / 2u - 1u] + vals[count / 2u]) * 0.5f;
}

// Quay servo cho mỗi bước quét: bước nhỏ nhảy thẳng để đỡ rung, góc lớn dùng moveServoSmooth
static void moveServoForScanStep(uint8_t targetDeg)
{
  int16_t diff = (int16_t)targetDeg - (int16_t)getServoAngle();
  if (diff < 0)
    diff = -diff;

  if (diff <= (int16_t)SERVO_DIRECT_STEP_MAX_DEG)
  {
    setServoAngle(targetDeg);
    HAL_Delay(SCAN_STEP_SETTLE_MS);
  }
  else
  {
    moveServoSmooth(targetDeg);
    HAL_Delay(SCAN_SETTLE_MS);
  }
}

// Quét trọn cung và lưu profile góc; áp dụng Confirm-on-Hit lọc 2/3 tại các điểm có tín hiệu
static uint8_t captureScanProfile(uint8_t arcStartDeg, uint8_t arcEndDeg, uint8_t stepDeg,
                                  ScanPoint *profile, uint8_t maxPoints)
{
  uint8_t count = 0u;
  uint8_t seqObjHits = 0u;
  uint8_t srf04ErrCount = 0u;

  debugDecision("SCANNING");
  moveServoSmooth(arcStartDeg);
  HAL_Delay(SCAN_SETTLE_MS);

  int16_t dir = (arcStartDeg <= arcEndDeg) ? (int16_t)stepDeg : -(int16_t)stepDeg;
  for (int16_t a = (int16_t)arcStartDeg;
       ((dir > 0) ? (a <= (int16_t)arcEndDeg) : (a >= (int16_t)arcEndDeg)) && count < maxPoints;
       a += dir)
  {
    if (count > 0u)
      moveServoForScanStep((uint8_t)a);

    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);

    bool prevWasObj = (count > 0u && profile[count - 1u].type == TARGET_OBJECT);
    bool d05Hit = (d05RawCm >= SCAN_MIN_RANGE_CM && d05RawCm <= SCAN_MAX_RANGE_CM);
    bool d05Err = (d05RawCm == ECHO_ERROR_TIMEOUT || d05RawCm == ECHO_ERROR_TOO_CLOSE);
    if (d05Hit || d05Err || prevWasObj)
      measureDualDistanceConfirmCM(d05RawCm, d04RawCm, &d05RawCm, &d04RawCm);

    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);
    profile[count].angleDeg = (uint8_t)a;
    profile[count].d05Cm = d05RawCm;
    profile[count].d04Cm = d04RawCm;
    profile[count].type = type;
    count++;

    if (type == TARGET_OBJECT)
      seqObjHits++;
    else
      seqObjHits = 0u;

    if (d05RawCm >= SCAN_MIN_RANGE_CM && d05RawCm <= SCAN_MAX_RANGE_CM &&
        d04RawCm == ECHO_ERROR_TIMEOUT)
      srf04ErrCount++;

    debugScan(d05RawCm, d04RawCm, (uint8_t)a, seqObjHits, type);
  }

  if (srf04ErrCount >= SCAN_CONFIRM_COUNT)
    debugLog("SENSOR_WARN srf04_err=%u\r\n", srf04ErrCount);

  return count;
}

// Kiểm tra điểm OBSTACLE có thật sự thuộc thân tường/vật cản (tránh điểm giao thoa vật gần + tường xa xóa mất vật)
static bool isObstacleAnchor(const ScanPoint *profile, uint8_t count, uint8_t j)
{
  if (profile[j].type != TARGET_OBSTACLE)
    return false;

  float d05 = profile[j].d05Cm;
  float d04 = profile[j].d04Cm;
  if (d04 < SCAN_MIN_RANGE_CM)
    return true;
  if ((d04 - SENSOR_DEPTH_OFFSET_CM - d05) <= SCAN_OBS_GUARD_DEPTH_CM)
    return true;

  if (j > 0u && profile[j - 1u].type == TARGET_OBSTACLE &&
      fabsf(profile[j - 1u].d05Cm - d05) <= SCAN_OBS_GUARD_DEPTH_CM)
    return true;
  if ((j + 1u) < count && profile[j + 1u].type == TARGET_OBSTACLE &&
      fabsf(profile[j + 1u].d05Cm - d05) <= SCAN_OBS_GUARD_DEPTH_CM)
    return true;

  return false;
}

// Xóa điểm OBJECT giả nằm sát hai mép tường hoặc vật cản cao ở cùng độ sâu
static void filterObstacleHalo(const ScanPoint *profile, ScanTargetType *filtered, uint8_t count)
{
  for (uint8_t i = 0u; i < count; i++)
    filtered[i] = profile[i].type;

  for (uint8_t i = 0u; i < count; i++)
  {
    if (profile[i].type != TARGET_OBJECT)
      continue;

    uint8_t left = (i > SCAN_OBS_GUARD_STEPS) ? (uint8_t)(i - SCAN_OBS_GUARD_STEPS) : 0u;
    uint8_t right = (uint8_t)(i + SCAN_OBS_GUARD_STEPS);
    if (right >= count)
      right = (uint8_t)(count - 1u);

    for (uint8_t j = left; j <= right; j++)
    {
      if (isObstacleAnchor(profile, count, j) &&
          fabsf(profile[i].d05Cm - profile[j].d05Cm) <= SCAN_OBS_GUARD_DEPTH_CM)
      {
        filtered[i] = TARGET_NONE;
        break;
      }
    }
  }
}

static void addCandidate(ScanCluster *candidates, uint8_t *candCount, uint8_t maxCandidates,
                         const ScanCluster *cand)
{
  if (*candCount < maxCandidates)
  {
    candidates[(*candCount)++] = *cand;
    return;
  }

  uint8_t worstIdx = 0u;
  for (uint8_t i = 1u; i < *candCount; i++)
  {
    if (candidates[i].clusterDist > candidates[worstIdx].clusterDist)
      worstIdx = i;
  }
  if (cand->clusterDist < candidates[worstIdx].clusterDist)
    candidates[worstIdx] = *cand;
}

// Đóng cụm, loại cụm quá rộng (tường nghiêng), rồi tính tâm và khoảng cách từ vùng lõi gần nhất
static void finalizeCluster(const ScanPoint *profile, const ScanTargetType *filtered,
                            uint8_t startIdx, uint8_t endIdx,
                            ScanCluster *candidates, uint8_t *candCount, uint8_t maxCandidates)
{
  float allDists[SCAN_MAX_POINTS];
  uint8_t objCount = 0u;
  float minDist = SCAN_MAX_RANGE_CM + 1.0f;

  for (uint8_t k = startIdx; k <= endIdx; k++)
  {
    if (filtered[k] == TARGET_OBJECT)
    {
      float d = profile[k].d05Cm;
      allDists[objCount++] = d;
      if (d < minDist)
        minDist = d;
    }
  }

  if (objCount < SCAN_CONFIRM_COUNT)
    return;

  uint8_t startDeg = profile[startIdx].angleDeg;
  uint8_t endDeg = profile[endIdx].angleDeg;
  uint8_t spanDeg = (startDeg >= endDeg) ? (uint8_t)(startDeg - endDeg) : (uint8_t)(endDeg - startDeg);
  float allMedian = computeMedian(allDists, objCount);
  if (allMedian > SCAN_CLUSTER_MAX_SPAN_NEAR_CM && spanDeg > SCAN_CLUSTER_MAX_SPAN_DEG)
  {
    debugLog("CLUSTER_DROP_WIDE start=%u end=%u span=%u d=%s\r\n",
             startDeg, endDeg, spanDeg, debugF1(allMedian));
    return;
  }

  float coreDists[SCAN_MAX_POINTS];
  uint8_t coreCount = 0u;
  uint8_t coreStartDeg = 0u;
  uint8_t coreEndDeg = 0u;

  for (uint8_t k = startIdx; k <= endIdx; k++)
  {
    if (filtered[k] == TARGET_OBJECT &&
        profile[k].d05Cm <= (minDist + SCAN_CORE_DEPTH_MARGIN_CM))
    {
      if (coreCount == 0u)
        coreStartDeg = profile[k].angleDeg;
      coreEndDeg = profile[k].angleDeg;
      coreDists[coreCount++] = profile[k].d05Cm;
    }
  }

  if (coreCount == 0u)
    return;

  ScanCluster cand;
  cand.startDeg = startDeg;
  cand.endDeg = endDeg;
  cand.centerDeg = (uint8_t)(((uint16_t)coreStartDeg + (uint16_t)coreEndDeg) / 2u);
  cand.count = objCount;
  cand.clusterDist = computeMedian(coreDists, coreCount);
  addCandidate(candidates, candCount, maxCandidates, &cand);
}

// Gom cụm chống trôi độ sâu từ profile đã lọc và sắp xếp ứng viên từ gần tới xa
static uint8_t extractClusters(const ScanPoint *profile, const ScanTargetType *filtered, uint8_t count,
                               ScanCluster *candidates, uint8_t maxCandidates)
{
  bool isBridge[SCAN_MAX_POINTS];
  for (uint8_t i = 0u; i < count; i++)
    isBridge[i] = false;

  for (uint8_t i = 1u; (i + 1u) < count; i++)
  {
    if (filtered[i] == TARGET_NONE &&
        filtered[i - 1u] == TARGET_OBJECT &&
        filtered[i + 1u] == TARGET_OBJECT &&
        fabsf(profile[i - 1u].d05Cm - profile[i + 1u].d05Cm) <= SCAN_CLUSTER_DEPTH_JUMP_CM)
    {
      isBridge[i] = true;
    }
  }

  uint8_t candCount = 0u;
  bool inCluster = false;
  uint8_t curStartIdx = 0u;
  uint8_t curEndIdx = 0u;
  float curMinDist = 0.0f;
  float curMaxDist = 0.0f;
  float curLastDist = 0.0f;

  for (uint8_t i = 0u; i < count; i++)
  {
    if (filtered[i] == TARGET_OBJECT)
    {
      float d = profile[i].d05Cm;
      if (inCluster)
      {
        float nextMin = (d < curMinDist) ? d : curMinDist;
        float nextMax = (d > curMaxDist) ? d : curMaxDist;
        if (fabsf(d - curLastDist) > SCAN_CLUSTER_DEPTH_JUMP_CM ||
            (nextMax - nextMin) > SCAN_CLUSTER_MAX_DEPTH_SPAN_CM)
        {
          finalizeCluster(profile, filtered, curStartIdx, curEndIdx, candidates, &candCount, maxCandidates);
          inCluster = false;
        }
      }

      if (!inCluster)
      {
        inCluster = true;
        curStartIdx = i;
        curMinDist = d;
        curMaxDist = d;
      }
      else
      {
        if (d < curMinDist) curMinDist = d;
        if (d > curMaxDist) curMaxDist = d;
      }
      curEndIdx = i;
      curLastDist = d;
    }
    else if (inCluster && !isBridge[i])
    {
      finalizeCluster(profile, filtered, curStartIdx, curEndIdx, candidates, &candCount, maxCandidates);
      inCluster = false;
    }
  }

  if (inCluster)
    finalizeCluster(profile, filtered, curStartIdx, curEndIdx, candidates, &candCount, maxCandidates);

  for (uint8_t i = 1u; i < candCount; i++)
  {
    ScanCluster key = candidates[i];
    uint8_t j = i;
    while (j > 0u && candidates[j - 1u].clusterDist > key.clusterDist)
    {
      candidates[j] = candidates[j - 1u];
      j--;
    }
    candidates[j] = key;
  }

  return candCount;
}

// Bù độ rơ SG90 theo chiều quét rồi đo xác minh 5 mẫu tại góc tâm ứng viên
static bool verifyClusterCenter(const ScanCluster *cand, int16_t scanDir, float *outDistCm)
{
  int16_t preload = (scanDir < 0)
                        ? ((int16_t)cand->centerDeg + (int16_t)SCAN_BACKLASH_PRELOAD_DEG)
                        : ((int16_t)cand->centerDeg - (int16_t)SCAN_BACKLASH_PRELOAD_DEG);
  if (preload < (int16_t)SERVO_SAFE_MIN_DEG)
    preload = (int16_t)SERVO_SAFE_MIN_DEG;
  if (preload > (int16_t)SERVO_SAFE_MAX_DEG)
    preload = (int16_t)SERVO_SAFE_MAX_DEG;

  if (getServoAngle() != (uint8_t)preload)
  {
    moveServoSmooth((uint8_t)preload);
    HAL_Delay(SCAN_STEP_SETTLE_MS);
  }
  moveServoSmooth(cand->centerDeg);
  HAL_Delay(SCAN_SETTLE_MS);

  float validDists[SCAN_VERIFY_SAMPLES];
  uint8_t objHits = 0u;
  uint8_t obsHits = 0u;

  for (uint8_t i = 0u; i < SCAN_VERIFY_SAMPLES; i++)
  {
    float d05 = ECHO_ERROR_TIMEOUT;
    float d04 = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05, &d04);
    ScanTargetType type = scanClassifyTarget(d05, d04);
    if (type == TARGET_OBSTACLE)
    {
      obsHits++;
    }
    else if (type == TARGET_OBJECT &&
             fabsf(d05 - cand->clusterDist) <= SCAN_VERIFY_DEPTH_TOL_CM)
    {
      validDists[objHits++] = d05;
    }
  }

  if (obsHits <= SCAN_VERIFY_MAX_OBS_HITS && objHits >= SCAN_VERIFY_MIN_HITS)
  {
    *outDistCm = computeMedian(validDists, objHits);
    return true;
  }

  debugLog("CLUSTER_REJECT center=%u d=%s obj=%u/%u obs=%u\r\n",
           cand->centerDeg, debugF1(cand->clusterDist), objHits, SCAN_VERIFY_SAMPLES, obsHits);
  return false;
}

// Xác nhận tường thẳng trước mũi (80°..100°) bằng trung vị thực tế của cả SRF05 và SRF04
static bool detectFrontWallFromProfile(const ScanPoint *profile, uint8_t count,
                                       float *outD05Cm, float *outD04Cm)
{
  uint8_t wallIdx[SCAN_MAX_POINTS];
  float buf[SCAN_MAX_POINTS];
  uint8_t wallCount = 0u;

  for (uint8_t i = 0u; i < count; i++)
  {
    if (profile[i].angleDeg >= SCAN_WALL_MIN_DEG &&
        profile[i].angleDeg <= SCAN_WALL_MAX_DEG &&
        profile[i].type == TARGET_OBSTACLE)
    {
      wallIdx[wallCount] = i;
      buf[wallCount] = profile[i].d05Cm;
      wallCount++;
    }
  }

  if (wallCount < SCAN_CONFIRM_COUNT)
    return false;

  float med05 = computeMedian(buf, wallCount);
  uint8_t consCount = 0u;
  for (uint8_t i = 0u; i < wallCount; i++)
  {
    uint8_t idx = wallIdx[i];
    if (fabsf(profile[idx].d05Cm - med05) <= SCAN_WALL_DEPTH_SPREAD_CM)
      buf[consCount++] = profile[idx].d05Cm;
  }

  if (consCount < SCAN_CONFIRM_COUNT)
    return false;

  *outD05Cm = computeMedian(buf, consCount);
  consCount = 0u;
  for (uint8_t i = 0u; i < wallCount; i++)
  {
    uint8_t idx = wallIdx[i];
    if (fabsf(profile[idx].d05Cm - med05) <= SCAN_WALL_DEPTH_SPREAD_CM)
      buf[consCount++] = profile[idx].d04Cm;
  }
  *outD04Cm = computeMedian(buf, consCount);
  return true;
}

// Quét toàn cung chỉ định, lọc mép tường, gom cụm chống trôi và xác minh đa cụm gần -> xa
static ScanStatus scanArcCluster(uint8_t arcStartDeg, uint8_t arcEndDeg, uint8_t stepDeg,
                                 float *distanceCm, uint8_t *angleDeg)
{
  ScanPoint profile[SCAN_MAX_POINTS];
  ScanTargetType filtered[SCAN_MAX_POINTS];
  ScanCluster candidates[SCAN_MAX_CLUSTERS];

  uint8_t count = captureScanProfile(arcStartDeg, arcEndDeg, stepDeg, profile, SCAN_MAX_POINTS);
  filterObstacleHalo(profile, filtered, count);
  uint8_t candCount = extractClusters(profile, filtered, count, candidates, SCAN_MAX_CLUSTERS);
  int16_t scanDir = (arcStartDeg <= arcEndDeg) ? (int16_t)stepDeg : -(int16_t)stepDeg;

  for (uint8_t i = 0u; i < candCount; i++)
  {
    float verifiedDist = ECHO_ERROR_TIMEOUT;
    if (verifyClusterCenter(&candidates[i], scanDir, &verifiedDist))
    {
      *angleDeg = candidates[i].centerDeg;
      *distanceCm = verifiedDist;
      debugLog("CLUSTER arc=[%u..%u] start=%u end=%u hits=%u -> center=%u d=%s\r\n",
               arcStartDeg, arcEndDeg, candidates[i].startDeg, candidates[i].endDeg,
               candidates[i].count, candidates[i].centerDeg, debugF1(verifiedDist));
      HAL_Delay(SCAN_FOUND_HOLD_MS);
      return SCAN_OBJECT_FOUND;
    }
  }

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  float wallD05 = ECHO_ERROR_TIMEOUT;
  float wallD04 = ECHO_ERROR_TIMEOUT;
  if (detectFrontWallFromProfile(profile, count, &wallD05, &wallD04))
  {
    *distanceCm = wallD05;
    *angleDeg = SCAN_HOME_ANGLE;
    debugWallDetected(wallD05, wallD04);
    HAL_Delay(WALL_HOLD_MS);
    return SCAN_WALL_FOUND;
  }

  debugLog("SCAN_EMPTY arc=[%u..%u] cands=%u\r\n", arcStartDeg, arcEndDeg, candCount);
  return ERROR_NO_OBJECTIVE;
}

void scanInit(void)
{
  servoInit();
}

ScanStatus scanSideArc(ScanSide side, float *distanceCm, uint8_t *angleDeg)
{
  if (side == SCAN_SIDE_LEFT)
    return scanArcCluster(SCAN_MAX_DEG, SCAN_HOME_ANGLE, SCAN_STEP_DEG, distanceCm, angleDeg);
  if (side == SCAN_SIDE_RIGHT)
    return scanArcCluster(SCAN_HOME_ANGLE, SCAN_MIN_DEG, SCAN_STEP_DEG, distanceCm, angleDeg);
  return scanArcCluster(SCAN_MAX_DEG, SCAN_MIN_DEG, SCAN_STEP_DEG, distanceCm, angleDeg);
}

ScanStatus scanForObject(float *distanceCm, uint8_t *angleDeg)
{
  return scanArcCluster(SCAN_MAX_DEG, SCAN_MIN_DEG, SCAN_STEP_DEG, distanceCm, angleDeg);
}

ScanStatus scanFine(float *distanceCm, uint8_t *angleDeg)
{
  ScanStatus status = scanArcCluster(SCAN_FINE_MAX_DEG, SCAN_FINE_MIN_DEG, SCAN_FINE_STEP_DEG,
                                     distanceCm, angleDeg);
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  return status;
}
