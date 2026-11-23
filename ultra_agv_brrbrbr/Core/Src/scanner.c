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
  bool isWallBump;
} ScanCluster;

static bool lastFrontClear = false;
static bool lastLeftWall = false;
static bool lastRightWall = false;
static bool curLeftWallHint = false;
static bool curRightWallHint = false;
static bool curWideFrontWall = false;
static float curWideWallD05 = 0.0f;

static bool isValidD05(float d)
{
  return (d >= SCAN_MIN_RANGE_CM && d <= SCAN_MAX_RANGE_CM);
}

ScanTargetType scanClassifyTarget(float d05RawCm, float d04RawCm)
{
  if (d04RawCm == ECHO_ERROR_TOO_CLOSE ||
      (d04RawCm > 0.0f && d04RawCm < SCAN_MIN_RANGE_CM))
    return TARGET_OBSTACLE;

  if (d04RawCm >= SCAN_MIN_RANGE_CM && d04RawCm <= SRF04_MAX_RANGE_CM)
  {
    if (d05RawCm == ECHO_ERROR_TOO_CLOSE ||
        (d05RawCm > 0.0f && d05RawCm < SCAN_MIN_RANGE_CM))
      return TARGET_OBSTACLE;
    if (d05RawCm == ECHO_ERROR_TIMEOUT && d04RawCm <= SCAN_CLOSE_WALL_SRF04_CM)
      return TARGET_OBSTACLE;
    if (d05RawCm > SCAN_MAX_RANGE_CM)
      return TARGET_OBSTACLE;
  }

  if (d05RawCm < SCAN_MIN_RANGE_CM || d05RawCm > SCAN_MAX_RANGE_CM)
    return TARGET_NONE;

  if (d04RawCm == ECHO_ERROR_TIMEOUT)
    return TARGET_NONE;

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

// Quy đổi cự ly tường hiệu dụng từ SRF05 hoặc bù lùi SRF04 khi SRF05 bị mù do áp sát tường
static float effectiveWallDistCm(float d05Cm, float d04Cm)
{
  if (isValidD05(d05Cm))
    return d05Cm;
  if (d04Cm == ECHO_ERROR_TOO_CLOSE || (d04Cm > 0.0f && d04Cm < SCAN_MIN_RANGE_CM))
    return 0.0f;
  if (d04Cm >= SCAN_MIN_RANGE_CM && d04Cm <= SRF04_MAX_RANGE_CM)
  {
    float adj = d04Cm - SENSOR_DEPTH_OFFSET_CM;
    return (adj > 0.0f) ? adj : 0.0f;
  }
  return 0.0f;
}

// Đổi điểm cực (góc servo, cự ly cm) sang tọa độ 2D Descartes (cm)
static void pointToXY(uint8_t angleDeg, float distCm, float *outX, float *outY)
{
  float rad = (float)angleDeg * (MATH_PI / 180.0f);
  *outX = distCm * cosf(rad);
  *outY = distCm * sinf(rad);
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
    bool d05Hit = isValidD05(d05RawCm);
    bool d05Err = (d05RawCm == ECHO_ERROR_TIMEOUT || d05RawCm == ECHO_ERROR_TOO_CLOSE);
    bool d04Hit = (d04RawCm == ECHO_ERROR_TOO_CLOSE ||
                   (d04RawCm > 0.0f && d04RawCm <= SRF04_MAX_RANGE_CM));
    if (d05Hit || d05Err || d04Hit || prevWasObj)
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

    if (isValidD05(d05RawCm) && d04RawCm == ECHO_ERROR_TIMEOUT)
      srf04ErrCount++;

    debugScan(d05RawCm, d04RawCm, (uint8_t)a, seqObjHits, type);
  }

  if (srf04ErrCount >= SCAN_CONFIRM_COUNT)
    debugLog("SENSOR_WARN srf04_err=%u\r\n", srf04ErrCount);

  return count;
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

// Tính dây cung 2D và hướng pháp tuyến hướng về gốc cảm biến (0, 0) giữa 2 điểm
static float initChordBasis(const ScanPoint *pA, const ScanPoint *pB,
                            float *x0, float *y0, float *ux, float *uy, float *inwardSign)
{
  float x1 = 0.0f, y1 = 0.0f;
  pointToXY(pA->angleDeg, pA->d05Cm, x0, y0);
  pointToXY(pB->angleDeg, pB->d05Cm, &x1, &y1);
  float chordCm = hypotf(x1 - *x0, y1 - *y0);
  if (chordCm <= 0.0f)
    return 0.0f;

  *ux = (x1 - *x0) / chordCm;
  *uy = (y1 - *y0) / chordCm;
  if (inwardSign)
  {
    float origCross = (-(*x0)) * (*uy) - (-(*y0)) * (*ux);
    *inwardSign = (origCross >= 0.0f) ? 1.0f : -1.0f;
  }
  return chordCm;
}

// Đo bề ngang 2D và độ lệch tối đa khỏi dây cung toàn đoạn
static void computeLineMetrics(const ScanPoint *profile, const uint8_t *idx, uint8_t n,
                               float *outChordCm, float *outMaxErrCm)
{
  *outChordCm = 0.0f;
  *outMaxErrCm = 0.0f;
  if (n < 2u)
    return;

  float x0 = 0.0f, y0 = 0.0f, ux = 0.0f, uy = 0.0f;
  float chordCm = initChordBasis(&profile[idx[0]], &profile[idx[n - 1u]],
                                 &x0, &y0, &ux, &uy, NULL);
  *outChordCm = chordCm;
  if (chordCm <= 0.0f)
    return;

  for (uint8_t k = 0u; k < n; k++)
  {
    float xk = 0.0f, yk = 0.0f;
    pointToXY(profile[idx[k]].angleDeg, profile[idx[k]].d05Cm, &xk, &yk);
    float absErr = fabsf((xk - x0) * uy - (yk - y0) * ux);
    if (absErr > *outMaxErrCm)
      *outMaxErrCm = absErr;
  }
}

// Đo độ nhô cục bộ về phía xe so với dây cung hai vai (±24°) để bắt vật sát tường
static void computeLocalBumpMetrics(const ScanPoint *profile, const uint8_t *idx, uint8_t n,
                                    float *outMaxProtrudeCm, uint8_t *outPeakPos,
                                    float *outProtrudeArr)
{
  for (uint8_t k = 0u; k < n; k++)
    outProtrudeArr[k] = 0.0f;
  *outMaxProtrudeCm = 0.0f;
  *outPeakPos = 0u;
  if (n < 5u)
    return;

  for (uint8_t k = 2u; (k + 2u) < n; k++)
  {
    uint8_t l = (k > SCAN_WALL_BUMP_HALF_STEPS) ? (uint8_t)(k - SCAN_WALL_BUMP_HALF_STEPS) : 0u;
    uint8_t r = ((uint16_t)k + SCAN_WALL_BUMP_HALF_STEPS < n)
                    ? (uint8_t)(k + SCAN_WALL_BUMP_HALF_STEPS)
                    : (uint8_t)(n - 1u);
    float x0 = 0.0f, y0 = 0.0f, ux = 0.0f, uy = 0.0f, inwardSign = 1.0f;
    if (initChordBasis(&profile[idx[l]], &profile[idx[r]], &x0, &y0, &ux, &uy, &inwardSign) <= 0.0f)
      continue;

    float xk = 0.0f, yk = 0.0f;
    pointToXY(profile[idx[k]].angleDeg, profile[idx[k]].d05Cm, &xk, &yk);
    float protrude = ((xk - x0) * uy - (yk - y0) * ux) * inwardSign;
    outProtrudeArr[k] = protrude;
    if (protrude > *outMaxProtrudeCm)
    {
      *outMaxProtrudeCm = protrude;
      *outPeakPos = k;
    }
  }
}

// Kiểm tra đoạn d05 có tăng/giảm đơn điệu đủ lớn (tường nghiêng khi SRF04 trượt gương)
static bool isMonotonicRamp(const ScanPoint *profile, const uint8_t *idx, uint8_t n)
{
  if (n < 4u)
    return false;

  float dFirst = profile[idx[0]].d05Cm;
  float dLast = profile[idx[n - 1u]].d05Cm;
  if (fabsf(dLast - dFirst) < SCAN_WALL_RAMP_MIN_DELTA_CM)
    return false;

  bool incOk = true;
  bool decOk = true;
  for (uint8_t i = 0u; (i + 1u) < n; i++)
  {
    float d0 = profile[idx[i]].d05Cm;
    float d1 = profile[idx[i + 1u]].d05Cm;
    if (d1 < (d0 - SCAN_WALL_RAMP_MONO_TOL_CM))
      incOk = false;
    if (d1 > (d0 + SCAN_WALL_RAMP_MONO_TOL_CM))
      decOk = false;
  }
  return incOk || decOk;
}

// Tạo ứng viên cụm từ dải chỉ số lõi [first..last] của đoạn
static void buildCandidateFromRange(const ScanPoint *profile, const uint8_t *idx,
                                    uint8_t first, uint8_t last, bool isWallBump,
                                    ScanCluster *candidates, uint8_t *candCount, uint8_t maxCandidates)
{
  float minDist = SCAN_MAX_RANGE_CM + 1.0f;
  for (uint8_t i = first; i <= last; i++)
  {
    float d = profile[idx[i]].d05Cm;
    if (d < minDist)
      minDist = d;
  }

  float coreDists[SCAN_MAX_POINTS];
  uint8_t coreCount = 0u;
  uint8_t coreStartDeg = 0u;
  uint8_t coreEndDeg = 0u;
  for (uint8_t i = first; i <= last; i++)
  {
    float d = profile[idx[i]].d05Cm;
    if (d <= (minDist + SCAN_CORE_DEPTH_MARGIN_CM))
    {
      if (coreCount == 0u)
        coreStartDeg = profile[idx[i]].angleDeg;
      coreEndDeg = profile[idx[i]].angleDeg;
      coreDists[coreCount++] = d;
    }
  }
  if (coreCount == 0u)
    return;

  ScanCluster cand;
  cand.startDeg = profile[idx[first]].angleDeg;
  cand.endDeg = profile[idx[last]].angleDeg;
  cand.centerDeg = (uint8_t)(((uint16_t)coreStartDeg + (uint16_t)coreEndDeg) / 2u);
  cand.count = (uint8_t)(last - first + 1u);
  cand.clusterDist = computeMedian(coreDists, coreCount);
  cand.isWallBump = isWallBump;
  addCandidate(candidates, candCount, maxCandidates, &cand);
}

// Ghi nhận dải chắn rộng (>= 66°) cắt qua mũi xe để không trả về NONE khi SRF04 mất sóng
static void recordWideFrontWallIfSpan(const ScanPoint *profile, const uint8_t *idx, uint8_t n,
                                      uint8_t segStartDeg, uint8_t segEndDeg, uint8_t segSpanDeg)
{
  if (segSpanDeg < SCAN_WALL_MIN_SPAN_DEG)
    return;

  uint8_t minAng = (segStartDeg <= segEndDeg) ? segStartDeg : segEndDeg;
  uint8_t maxAng = (segStartDeg >= segEndDeg) ? segStartDeg : segEndDeg;
  if (minAng > SCAN_WALL_MAX_DEG || maxAng < SCAN_WALL_MIN_DEG)
    return;

  float buf[SCAN_MAX_POINTS];
  uint8_t cnt = 0u;
  for (uint8_t i = 0u; i < n; i++)
  {
    uint8_t ang = profile[idx[i]].angleDeg;
    if (ang >= SCAN_WALL_MIN_DEG && ang <= SCAN_WALL_MAX_DEG)
      buf[cnt++] = profile[idx[i]].d05Cm;
  }
  if (cnt == 0u)
  {
    for (uint8_t i = 0u; i < n; i++)
      buf[cnt++] = profile[idx[i]].d05Cm;
  }
  if (cnt == 0u)
    return;

  curWideFrontWall = true;
  curWideWallD05 = computeMedian(buf, cnt);
}

// Phân loại đoạn 2D: tường phẳng/nghiêng, cục lồi vật sát tường, hoặc cụm vật độc lập
static void processSegment(const ScanPoint *profile, const uint8_t *idx, uint8_t n,
                           ScanCluster *candidates, uint8_t *candCount, uint8_t maxCandidates)
{
  if (n == 0u)
    return;

  uint8_t obsCount = 0u;
  uint8_t objCount = 0u;
  for (uint8_t i = 0u; i < n; i++)
  {
    if (profile[idx[i]].type == TARGET_OBSTACLE)
      obsCount++;
    else if (profile[idx[i]].type == TARGET_OBJECT)
      objCount++;
  }

  float chordCm = 0.0f;
  float maxErrCm = 0.0f;
  computeLineMetrics(profile, idx, n, &chordCm, &maxErrCm);

  uint8_t segStartDeg = profile[idx[0]].angleDeg;
  uint8_t segEndDeg = profile[idx[n - 1u]].angleDeg;
  uint8_t segSpanDeg = (segStartDeg >= segEndDeg)
                           ? (uint8_t)(segStartDeg - segEndDeg)
                           : (uint8_t)(segEndDeg - segStartDeg);

  bool hasWallObs = (obsCount >= ((n >= 10u) ? SCAN_WALL_OBS_MIN_HITS : SCAN_CONFIRM_COUNT));
  bool isWideFlat = (n >= 4u) && (segSpanDeg >= SCAN_WALL_MIN_SPAN_DEG) && (maxErrCm <= SCAN_WALL_LINE_ERR_CM);
  bool isAngledRamp = (n >= 4u) && (chordCm >= SCAN_WALL_RAMP_MIN_WIDTH_CM) &&
                      (maxErrCm <= SCAN_WALL_LINE_ERR_CM) && isMonotonicRamp(profile, idx, n);
  bool isWallSeg = hasWallObs || isWideFlat || isAngledRamp;

  if (isWallSeg)
  {
    if (segStartDeg >= SCAN_SIDE_WALL_LEFT_MIN_DEG)
      curLeftWallHint = true;
    if (segEndDeg <= SCAN_SIDE_WALL_RIGHT_MAX_DEG)
      curRightWallHint = true;

    float maxProtrudeCm = 0.0f;
    // Tách vật nhỏ đặt sát mặt tường dựa vào cục lồi cục bộ so với dây cung hai vai
    if (n >= 7u)
    {
      uint8_t peakPos = 0u;
      float protrudeArr[SCAN_MAX_POINTS];
      computeLocalBumpMetrics(profile, idx, n, &maxProtrudeCm, &peakPos, protrudeArr);

      if (maxProtrudeCm >= SCAN_WALL_BUMP_MIN_CM &&
          peakPos >= 2u && peakPos <= (uint8_t)(n - 3u) &&
          profile[idx[peakPos]].d05Cm >= WALL_BACKUP_TRIGGER_CM)
      {
        float halfProt = 0.5f * maxProtrudeCm;
        uint8_t bL = peakPos;
        uint8_t bR = peakPos;
        while (bL > 1u && protrudeArr[bL - 1u] >= halfProt)
          bL--;
        while (bR < (uint8_t)(n - 2u) && protrudeArr[bR + 1u] >= halfProt)
          bR++;

        uint8_t bCount = (uint8_t)(bR - bL + 1u);
        uint8_t angL = profile[idx[bL]].angleDeg;
        uint8_t angR = profile[idx[bR]].angleDeg;
        uint8_t bSpanDeg = (angL >= angR) ? (uint8_t)(angL - angR) : (uint8_t)(angR - angL);

        if (bCount >= SCAN_CONFIRM_COUNT && bSpanDeg <= SCAN_BUMP_MAX_SPAN_DEG)
        {
          debugLog("WALL_BUMP start=%u end=%u prot=%s span=%u\r\n",
                   angL, angR, debugF1(maxProtrudeCm), bSpanDeg);
          buildCandidateFromRange(profile, idx, bL, bR, true, candidates, candCount, maxCandidates);
          return;
        }
      }
    }

    recordWideFrontWallIfSpan(profile, idx, n, segStartDeg, segEndDeg, segSpanDeg);
    debugLog("SEG_DROP_WALL start=%u end=%u n=%u obs=%u span=%u w=%s err=%s prot=%s\r\n",
             segStartDeg, segEndDeg, n, obsCount, segSpanDeg,
             debugF1(chordCm), debugF1(maxErrCm), debugF1(maxProtrudeCm));
    return;
  }

  if (objCount < SCAN_CONFIRM_COUNT)
    return;

  if (n >= 4u && segSpanDeg > SCAN_OBJ_MAX_SPAN_DEG)
  {
    recordWideFrontWallIfSpan(profile, idx, n, segStartDeg, segEndDeg, segSpanDeg);
    debugLog("SEG_DROP_WIDE start=%u end=%u span=%u\r\n", segStartDeg, segEndDeg, segSpanDeg);
    return;
  }

  buildCandidateFromRange(profile, idx, 0u, (uint8_t)(n - 1u), false, candidates, candCount, maxCandidates);
}

// Gom các đoạn bề mặt liên tục theo d05 (bắc cầu tối đa 1 điểm rớt) và lọc ứng viên gần -> xa
static uint8_t extractSegmentsAndCandidates(const ScanPoint *profile, uint8_t count,
                                            ScanCluster *candidates, uint8_t maxCandidates)
{
  bool isBridge[SCAN_MAX_POINTS];
  for (uint8_t i = 0u; i < count; i++)
    isBridge[i] = false;

  for (uint8_t i = 1u; (i + 1u) < count; i++)
  {
    if (!isValidD05(profile[i].d05Cm) &&
        isValidD05(profile[i - 1u].d05Cm) &&
        isValidD05(profile[i + 1u].d05Cm) &&
        fabsf(profile[i - 1u].d05Cm - profile[i + 1u].d05Cm) <= SCAN_SEG_JUMP_CM)
    {
      isBridge[i] = true;
    }
  }

  uint8_t candCount = 0u;
  uint8_t segIdx[SCAN_MAX_POINTS];
  uint8_t segLen = 0u;
  float lastD05 = 0.0f;

  for (uint8_t i = 0u; i < count; i++)
  {
    float d = profile[i].d05Cm;
    if (isValidD05(d))
    {
      if (segLen > 0u && fabsf(d - lastD05) > SCAN_SEG_JUMP_CM)
      {
        processSegment(profile, segIdx, segLen, candidates, &candCount, maxCandidates);
        segLen = 0u;
      }
      segIdx[segLen++] = i;
      lastD05 = d;
    }
    else if (segLen > 0u && !isBridge[i])
    {
      processSegment(profile, segIdx, segLen, candidates, &candCount, maxCandidates);
      segLen = 0u;
    }
  }

  if (segLen > 0u)
    processSegment(profile, segIdx, segLen, candidates, &candCount, maxCandidates);

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

    if (cand->isWallBump)
    {
      // Vật nằm sát tường có thể làm SRF04 bắt mép tường sau -> xác minh theo độ ổn định d05 tại đỉnh lồi
      if (isValidD05(d05) && fabsf(d05 - cand->clusterDist) <= SCAN_BUMP_VERIFY_TOL_CM)
        validDists[objHits++] = d05;
    }
    else if (type == TARGET_OBSTACLE)
    {
      obsHits++;
    }
    else if (type == TARGET_OBJECT &&
             fabsf(d05 - cand->clusterDist) <= SCAN_VERIFY_DEPTH_TOL_CM)
    {
      validDists[objHits++] = d05;
    }
  }

  uint8_t maxObs = (cand->count <= 1u) ? 0u : SCAN_VERIFY_MAX_OBS_HITS;
  if (obsHits <= maxObs && objHits >= SCAN_VERIFY_MIN_HITS)
  {
    *outDistCm = computeMedian(validDists, objHits);
    return true;
  }

  debugLog("CLUSTER_REJECT center=%u d=%s obj=%u/%u obs=%u\r\n",
           cand->centerDeg, debugF1(cand->clusterDist), objHits, SCAN_VERIFY_SAMPLES, obsHits);
  return false;
}

// Xác nhận tường áp sát rộng (60°..120°), mù kép hoặc tường thẳng trước mũi (80°..100°)
static bool detectFrontWallFromProfile(const ScanPoint *profile, uint8_t count,
                                       float *outD05Cm, float *outD04Cm)
{
  uint8_t closeIdx[SCAN_MAX_POINTS];
  float closeBuf[SCAN_MAX_POINTS];
  uint8_t closeCount = 0u;
  uint8_t blindCount = 0u;

  for (uint8_t i = 0u; i < count; i++)
  {
    uint8_t ang = profile[i].angleDeg;
    if (ang >= SCAN_CLOSE_WALL_MIN_DEG && ang <= SCAN_CLOSE_WALL_MAX_DEG &&
        profile[i].type == TARGET_OBSTACLE)
    {
      float ew = effectiveWallDistCm(profile[i].d05Cm, profile[i].d04Cm);
      if (ew < WALL_BACKUP_TRIGGER_CM)
      {
        closeIdx[closeCount] = i;
        closeBuf[closeCount] = ew;
        closeCount++;
      }
    }
    if (ang >= SCAN_WALL_MIN_DEG && ang <= SCAN_WALL_MAX_DEG)
    {
      float d05 = profile[i].d05Cm;
      float d04 = profile[i].d04Cm;
      bool isBlind = (d05 == ECHO_ERROR_TOO_CLOSE) ||
                     (d05 > 0.0f && d05 < SCAN_MIN_RANGE_CM) ||
                     (d05 == ECHO_ERROR_TIMEOUT &&
                      (d04 <= SCAN_CLOSE_WALL_SRF04_CM ||
                       d04 == ECHO_ERROR_TIMEOUT ||
                       d04 == ECHO_ERROR_TOO_CLOSE));
      if (isBlind)
        blindCount++;
    }
  }

  if (closeCount >= SCAN_CONFIRM_COUNT)
  {
    *outD05Cm = computeMedian(closeBuf, closeCount);
    for (uint8_t i = 0u; i < closeCount; i++)
      closeBuf[i] = profile[closeIdx[i]].d04Cm;
    *outD04Cm = computeMedian(closeBuf, closeCount);
    return true;
  }

  if (blindCount >= SCAN_FRONT_BLIND_WALL_HITS)
  {
    *outD05Cm = ECHO_ERROR_TOO_CLOSE;
    *outD04Cm = ECHO_ERROR_TIMEOUT;
    return true;
  }

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
      buf[wallCount] = effectiveWallDistCm(profile[i].d05Cm, profile[i].d04Cm);
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
    float ew = effectiveWallDistCm(profile[idx].d05Cm, profile[idx].d04Cm);
    if (fabsf(ew - med05) <= SCAN_WALL_DEPTH_SPREAD_CM)
      buf[consCount++] = ew;
  }

  if (consCount < SCAN_CONFIRM_COUNT)
    return false;

  *outD05Cm = computeMedian(buf, consCount);
  consCount = 0u;
  for (uint8_t i = 0u; i < wallCount; i++)
  {
    uint8_t idx = wallIdx[i];
    float ew = effectiveWallDistCm(profile[idx].d05Cm, profile[idx].d04Cm);
    if (fabsf(ew - med05) <= SCAN_WALL_DEPTH_SPREAD_CM)
      buf[consCount++] = profile[idx].d04Cm;
  }
  *outD04Cm = computeMedian(buf, consCount);
  return true;
}

// Tổng hợp gợi ý độ thông thoáng phía trước và tường hai bên hành lang từ profile quét
static void updateCorridorHints(const ScanPoint *profile, uint8_t count)
{
  bool frontClear = true;
  bool leftWall = curLeftWallHint;
  bool rightWall = curRightWallHint;

  for (uint8_t i = 0u; i < count; i++)
  {
    uint8_t ang = profile[i].angleDeg;
    if (ang >= SCAN_FRONT_CLEAR_MIN_DEG && ang <= SCAN_FRONT_CLEAR_MAX_DEG)
    {
      if (profile[i].d05Cm <= SCAN_EXPAND_CLEAR_CM || profile[i].d04Cm <= SCAN_EXPAND_CLEAR_CM)
        frontClear = false;
    }
    if (profile[i].type == TARGET_OBSTACLE)
    {
      if (ang >= SCAN_SIDE_WALL_LEFT_MIN_DEG) leftWall = true;
      if (ang <= SCAN_SIDE_WALL_RIGHT_MAX_DEG) rightWall = true;
    }
  }

  lastFrontClear = frontClear;
  lastLeftWall = leftWall;
  lastRightWall = rightWall;
}

// Quét toàn cung chỉ định, gom đoạn 2D, lọc tường/tách cục lồi và xác minh ứng viên gần -> xa
static ScanStatus scanArcCluster(uint8_t arcStartDeg, uint8_t arcEndDeg, uint8_t stepDeg,
                                 bool fineMode, float *distanceCm, uint8_t *angleDeg)
{
  ScanPoint profile[SCAN_MAX_POINTS];
  ScanCluster candidates[SCAN_MAX_CLUSTERS];

  curLeftWallHint = false;
  curRightWallHint = false;
  curWideFrontWall = false;
  curWideWallD05 = 0.0f;

  uint8_t count = captureScanProfile(arcStartDeg, arcEndDeg, stepDeg, profile, SCAN_MAX_POINTS);
  uint8_t candCount = extractSegmentsAndCandidates(profile, count, candidates, SCAN_MAX_CLUSTERS);
  if (!fineMode)
    updateCorridorHints(profile, count);

  int16_t scanDir = (arcStartDeg <= arcEndDeg) ? (int16_t)stepDeg : -(int16_t)stepDeg;

  float wallD05 = ECHO_ERROR_TIMEOUT;
  float wallD04 = ECHO_ERROR_TIMEOUT;
  bool hasFrontWall = detectFrontWallFromProfile(profile, count, &wallD05, &wallD04);
  if (!hasFrontWall && curWideFrontWall)
  {
    hasFrontWall = true;
    wallD05 = curWideWallD05;
    wallD04 = ECHO_OUT_OF_RANGE_CM;
  }

  if (hasFrontWall && wallD05 < WALL_BACKUP_TRIGGER_CM &&
      (candCount == 0u || candidates[0].clusterDist >= wallD05))
  {
    moveServoSmooth(SCAN_HOME_ANGLE);
    HAL_Delay(SCAN_SETTLE_MS);
    *distanceCm = wallD05;
    *angleDeg = SCAN_HOME_ANGLE;
    debugWallDetected(wallD05, wallD04);
    HAL_Delay(WALL_HOLD_MS);
    return SCAN_WALL_FOUND;
  }

  for (uint8_t i = 0u; i < candCount; i++)
  {
    if (!candidates[i].isWallBump && hasFrontWall &&
        candidates[i].clusterDist >= (wallD05 - SCAN_SEG_JUMP_CM))
      continue;

    if (fineMode)
    {
      if (candidates[i].centerDeg < SCAN_FINE_ACCEPT_MIN_DEG ||
          candidates[i].centerDeg > SCAN_FINE_ACCEPT_MAX_DEG)
        continue;
    }

    float verifiedDist = ECHO_ERROR_TIMEOUT;
    if (verifyClusterCenter(&candidates[i], scanDir, &verifiedDist))
    {
      *angleDeg = candidates[i].centerDeg;
      *distanceCm = verifiedDist;
      debugLog("CLUSTER arc=[%u..%u] start=%u end=%u hits=%u bump=%u -> center=%u d=%s\r\n",
               arcStartDeg, arcEndDeg, candidates[i].startDeg, candidates[i].endDeg,
               candidates[i].count, candidates[i].isWallBump ? 1u : 0u,
               candidates[i].centerDeg, debugF1(verifiedDist));
      HAL_Delay(SCAN_FOUND_HOLD_MS);
      return SCAN_OBJECT_FOUND;
    }
  }

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);

  if (hasFrontWall)
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
  lastFrontClear = false;
  lastLeftWall = false;
  lastRightWall = false;
}

ScanStatus scanSideArc(ScanSide side, float *distanceCm, uint8_t *angleDeg)
{
  if (side == SCAN_SIDE_LEFT)
    return scanArcCluster(SCAN_MAX_DEG, SCAN_HOME_ANGLE, SCAN_STEP_DEG, false, distanceCm, angleDeg);
  if (side == SCAN_SIDE_RIGHT)
    return scanArcCluster(SCAN_HOME_ANGLE, SCAN_MIN_DEG, SCAN_STEP_DEG, false, distanceCm, angleDeg);
  return scanArcCluster(SCAN_MAX_DEG, SCAN_MIN_DEG, SCAN_STEP_DEG, false, distanceCm, angleDeg);
}

ScanStatus scanForObject(float *distanceCm, uint8_t *angleDeg)
{
  return scanArcCluster(SCAN_MAX_DEG, SCAN_MIN_DEG, SCAN_STEP_DEG, false, distanceCm, angleDeg);
}

ScanStatus scanFine(float *distanceCm, uint8_t *angleDeg)
{
  ScanStatus status = scanArcCluster(SCAN_FINE_MAX_DEG, SCAN_FINE_MIN_DEG, SCAN_FINE_STEP_DEG,
                                     true, distanceCm, angleDeg);
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  return status;
}

void scanGetLastCorridorHint(bool *frontClear, bool *leftWall, bool *rightWall)
{
  if (frontClear) *frontClear = lastFrontClear;
  if (leftWall) *leftWall = lastLeftWall;
  if (rightWall) *rightWall = lastRightWall;
}

ScanStatus scanSimpleForObject(float *distanceCm, uint8_t *angleDeg)
{
  bool fromLeft = (getServoAngle() >= SCAN_HOME_ANGLE);
  uint8_t startDeg = fromLeft ? SCAN_MAX_DEG : SCAN_MIN_DEG;
  uint8_t endDeg = fromLeft ? SCAN_MIN_DEG : SCAN_MAX_DEG;
  int16_t dir = fromLeft ? -(int16_t)SCAN_STEP_DEG : (int16_t)SCAN_STEP_DEG;

  float sumAngle = 0.0f;
  float sumDist = 0.0f;
  uint8_t hitCount = 0u;

  debugDecision("SCANNING");
  moveServoSmooth(startDeg);
  HAL_Delay(SCAN_SETTLE_MS);

  for (int16_t a = (int16_t)startDeg;
       (dir > 0) ? (a <= (int16_t)endDeg) : (a >= (int16_t)endDeg);
       a += dir)
  {
    if (a != (int16_t)startDeg)
      moveServoForScanStep((uint8_t)a);

    float d05RawCm = ECHO_ERROR_TIMEOUT;
    float d04RawCm = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05RawCm, &d04RawCm);
    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);

    if (type == TARGET_OBJECT)
    {
      sumAngle += (float)a;
      sumDist += d05RawCm;
      hitCount++;
      debugScan(d05RawCm, d04RawCm, (uint8_t)a, hitCount, type);
    }
    else
    {
      debugScan(d05RawCm, d04RawCm, (uint8_t)a, 0u, type);
      if (hitCount >= SCAN_CONFIRM_COUNT)
        break;
      hitCount = 0u;
      sumAngle = 0.0f;
      sumDist = 0.0f;
    }
  }

  if (hitCount >= SCAN_CONFIRM_COUNT)
  {
    *angleDeg = (uint8_t)lroundf(sumAngle / (float)hitCount);
    *distanceCm = sumDist / (float)hitCount;
    debugLog("SIMPLE_SCAN hits=%u avg_ang=%u avg_d=%s\r\n",
             hitCount, *angleDeg, debugF1(*distanceCm));
    return SCAN_OBJECT_FOUND;
  }

  return ERROR_NO_OBJECTIVE;
}
