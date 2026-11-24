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
static bool curScanFineMode = false;
static ScanExtraInfo lastExtra = {false, false, false, SCAN_HOME_ANGLE, 0.0f};
static ScanWallSegLocal lastWallSegs[SCAN_MAX_WALL_SEGS_LOCAL];
static uint8_t lastWallSegCount = 0u;

static void addLocalWallSeg(uint8_t startDeg, uint8_t endDeg, float startDistCm, float endDistCm)
{
  if (lastWallSegCount >= SCAN_MAX_WALL_SEGS_LOCAL)
    return;
  lastWallSegs[lastWallSegCount].startDeg = startDeg;
  lastWallSegs[lastWallSegCount].endDeg = endDeg;
  lastWallSegs[lastWallSegCount].startDistCm = startDistCm;
  lastWallSegs[lastWallSegCount].endDistCm = endDistCm;
  lastWallSegCount++;
}

static void resetLastExtra(void)
{
  lastExtra.isConfirmedWideWall = false;
  lastExtra.isWallBump = false;
  lastExtra.hasSuspect = false;
  lastExtra.suspectAngleDeg = SCAN_HOME_ANGLE;
  lastExtra.suspectDistCm = 0.0f;
  lastWallSegCount = 0u;
}

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
    // Chỉ kết luận tường khi d05 quá sát (-2/<3 cm) VÀ d04 cũng nằm sát mặt tường (<= 13 cm); nếu d04 xa thì là vật sát mũi
    if ((d05RawCm == ECHO_ERROR_TOO_CLOSE ||
         (d05RawCm > 0.0f && d05RawCm < SCAN_MIN_RANGE_CM)) &&
        d04RawCm <= (2.0f + SENSOR_DEPTH_OFFSET_CM + SCAN_OBSTACLE_TOLERANCE_CM))
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

// Đổi điểm cực (góc servo, cự ly từ mặt đầu dò cm) sang tọa độ 2D Descartes quanh trục servo (cm)
static void pointToXY(uint8_t angleDeg, float distCm, float *outX, float *outY)
{
  float rad = (float)angleDeg * (MATH_PI / 180.0f);
  float rCm = SENSOR_FACE_OFFSET_CM + distCm;
  *outX = rCm * cosf(rad);
  *outY = rCm * sinf(rad);
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
  uint8_t topErrCount = 0u;

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
      topErrCount++;

    debugScan(d05RawCm, d04RawCm, (uint8_t)a, seqObjHits, type);
  }

  if (topErrCount >= SCAN_CONFIRM_COUNT)
    debugLog("SENSOR_WARN top_err=%u\r\n", topErrCount);

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
  float dx = x1 - *x0;
  float dy = y1 - *y0;
  float chordCm = sqrtf(dx * dx + dy * dy);
  if (chordCm <= 0.0f)
    return 0.0f;

  *ux = dx / chordCm;
  *uy = dy / chordCm;
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

// Đo độ nhô cục bộ về phía xe so với dây cung hai vai (±15°) để bắt vật sát tường
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

// Kiểm tra đoạn d05 có tăng/giảm đơn điệu đủ lớn (tường nghiêng khi SRF05 trên trượt gương)
static bool isMonotonicRamp(const ScanPoint *profile, const uint8_t *idx, uint8_t n)
{
  if (n < 3u)
    return false;

  float dFirst = profile[idx[0]].d05Cm;
  float dLast = profile[idx[n - 1u]].d05Cm;
  if (fabsf(dLast - dFirst) < SCAN_WALL_RAMP_MIN_DELTA_CM)
    return false;

  bool incOk = true;
  bool decOk = true;
  uint8_t incSteps = 0u;
  uint8_t decSteps = 0u;
  for (uint8_t i = 0u; (i + 1u) < n; i++)
  {
    float d0 = profile[idx[i]].d05Cm;
    float d1 = profile[idx[i + 1u]].d05Cm;
    if (d1 < (d0 - SCAN_WALL_RAMP_MONO_TOL_CM))
      incOk = false;
    if (d1 > (d0 + SCAN_WALL_RAMP_MONO_TOL_CM))
      decOk = false;
    if ((d1 - d0) >= 1.5f)
      incSteps++;
    if ((d0 - d1) >= 1.5f)
      decSteps++;
  }
  return (incOk && incSteps >= 2u) || (decOk && decSteps >= 2u);
}

// Tạo ứng viên cụm từ dải chỉ số lõi [first..last] của đoạn
static void buildCandidateFromRange(const ScanPoint *profile, const uint8_t *idx,
                                    uint8_t first, uint8_t last, bool isWallBump,
                                    ScanCluster *candidates, uint8_t *candCount, uint8_t maxCandidates)
{
  float allDists[SCAN_MAX_POINTS];
  uint8_t allCount = 0u;
  float minDist = SCAN_MAX_RANGE_CM + 1.0f;
  uint8_t minPos = first;
  for (uint8_t i = first; i <= last; i++)
  {
    float d = profile[idx[i]].d05Cm;
    allDists[allCount++] = d;
    if (d < minDist)
    {
      minDist = d;
      minPos = i;
    }
  }

  // Mở rộng lõi liên tiếp quanh đỉnh gần nhất, tránh bắc cầu qua khe lõm nối nhầm sang mép tường bên cạnh
  float coreDists[SCAN_MAX_POINTS];
  uint8_t coreL = minPos;
  uint8_t coreR = minPos;
  while (coreL > first && profile[idx[coreL - 1u]].d05Cm <= (minDist + SCAN_CORE_DEPTH_MARGIN_CM))
    coreL--;
  while (coreR < last && profile[idx[coreR + 1u]].d05Cm <= (minDist + SCAN_CORE_DEPTH_MARGIN_CM))
    coreR++;

  uint8_t coreCount = 0u;
  uint8_t coreStartDeg = profile[idx[coreL]].angleDeg;
  uint8_t coreEndDeg = profile[idx[coreR]].angleDeg;
  for (uint8_t i = coreL; i <= coreR; i++)
    coreDists[coreCount++] = profile[idx[i]].d05Cm;
  // Tránh 1 điểm nhiễu/mép tường gần hơn chiếm quyền tâm cụm của cả nhóm điểm vật ổn định
  if (coreCount < SCAN_CONFIRM_COUNT && allCount >= SCAN_CONFIRM_COUNT)
  {
    float segMed = computeMedian(allDists, allCount);
    coreCount = 0u;
    for (uint8_t i = first; i <= last; i++)
    {
      float d = profile[idx[i]].d05Cm;
      if (fabsf(d - segMed) <= SCAN_CORE_DEPTH_MARGIN_CM)
      {
        if (coreCount == 0u)
          coreStartDeg = profile[idx[i]].angleDeg;
        coreEndDeg = profile[idx[i]].angleDeg;
        coreDists[coreCount++] = d;
      }
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
  bool isAngledRamp = (n >= 3u) && (chordCm >= SCAN_WALL_RAMP_MIN_WIDTH_CM) &&
                      (maxErrCm <= SCAN_WALL_LINE_ERR_CM) && isMonotonicRamp(profile, idx, n);
  bool touchesEdge = (segStartDeg >= SCAN_MAX_DEG || segEndDeg <= SCAN_MIN_DEG);
  bool isEdgeWallStub = (n == 2u) && touchesEdge;
  bool isWallSeg = hasWallObs || isWideFlat || isAngledRamp || isEdgeWallStub;

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
          if (bL >= 2u)
            addLocalWallSeg(segStartDeg, profile[idx[bL - 1u]].angleDeg,
                            profile[idx[0]].d05Cm, profile[idx[bL - 1u]].d05Cm);
          if ((uint8_t)(n - 1u - bR) >= 2u)
            addLocalWallSeg(profile[idx[bR + 1u]].angleDeg, segEndDeg,
                            profile[idx[bR + 1u]].d05Cm, profile[idx[n - 1u]].d05Cm);
          debugLog("WALL_BUMP start=%u end=%u prot=%s span=%u\r\n",
                   angL, angR, debugF1(maxProtrudeCm), bSpanDeg);
          buildCandidateFromRange(profile, idx, bL, bR, true, candidates, candCount, maxCandidates);
          return;
        }
      }
    }

    {
      float segDists[SCAN_MAX_POINTS];
      for (uint8_t i = 0u; i < n; i++)
        segDists[i] = profile[idx[i]].d05Cm;
      float medDist = computeMedian(segDists, n);
      uint8_t centerDeg = (uint8_t)(((uint16_t)segStartDeg + (uint16_t)segEndDeg) / 2u);

      // Tách riêng cụm hẹp nằm gọn trong cung ở xa (>24 cm) bị SRF05 trên loe chùm để kiểm chứng (bỏ mảng tường bị cắt biên 45°/135°)
      bool touchesArcEdge = (segStartDeg >= SCAN_MAX_DEG || segEndDeg <= SCAN_MIN_DEG);
      if (!curScanFineMode && !touchesArcEdge && obsCount > 0u &&
          chordCm <= SCAN_SUSPECT_MAX_CHORD_CM &&
          segSpanDeg < SCAN_WALL_WIDE_SPAN_DEG &&
          medDist > AGV_WALL_CONFIRM_DIST_CM)
      {
        if (!lastExtra.hasSuspect || medDist < lastExtra.suspectDistCm)
        {
          lastExtra.hasSuspect = true;
          lastExtra.suspectAngleDeg = centerDeg;
          lastExtra.suspectDistCm = medDist;
        }
        debugLog("SEG_SUSPECT ang=%u d=%s w=%s\r\n", centerDeg, debugF1(medDist), debugF1(chordCm));
        return;
      }
    }

    if (chordCm >= SCAN_WALL_WIDE_CHORD_CM || segSpanDeg >= SCAN_WALL_WIDE_SPAN_DEG)
      lastExtra.isConfirmedWideWall = true;
    if (n >= 2u)
      addLocalWallSeg(segStartDeg, segEndDeg, profile[idx[0]].d05Cm, profile[idx[n - 1u]].d05Cm);

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
    float segDists[SCAN_MAX_POINTS];
    for (uint8_t i = 0u; i < n; i++)
      segDists[i] = profile[idx[i]].d05Cm;
    // Vật rất gần đầu dò bị chùm siêu âm loe rộng góc -> không loại nhầm thành tường rộng
    if (computeMedian(segDists, n) > (AGV_GRAB_DIST_MAX_CM + 5.0f))
    {
      lastExtra.isConfirmedWideWall = true;
      addLocalWallSeg(segStartDeg, segEndDeg, profile[idx[0]].d05Cm, profile[idx[n - 1u]].d05Cm);
      recordWideFrontWallIfSpan(profile, idx, n, segStartDeg, segEndDeg, segSpanDeg);
      debugLog("SEG_DROP_WIDE start=%u end=%u span=%u\r\n", segStartDeg, segEndDeg, segSpanDeg);
      return;
    }
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
        profile[i - 1u].type == profile[i + 1u].type &&
        fabsf(profile[i - 1u].d05Cm - profile[i + 1u].d05Cm) <= SCAN_CORE_DEPTH_MARGIN_CM)
    {
      isBridge[i] = true;
    }
  }

  uint8_t candCount = 0u;
  uint8_t segIdx[SCAN_MAX_POINTS];
  uint8_t segLen = 0u;
  float lastD05 = 0.0f;
  ScanTargetType lastType = TARGET_NONE;

  for (uint8_t i = 0u; i < count; i++)
  {
    float d = profile[i].d05Cm;
    if (isValidD05(d))
    {
      // Luôn tách ranh giới vật (OBJ) <-> tường (WALL) để tường gần bên không nuốt cụm vật (kể cả khi cùng cự ly d05)
      bool typeSplit = (lastType != profile[i].type);
      if (segLen > 0u && (fabsf(d - lastD05) > SCAN_SEG_JUMP_CM || typeSplit))
      {
        processSegment(profile, segIdx, segLen, candidates, &candCount, maxCandidates);
        segLen = 0u;
      }
      segIdx[segLen++] = i;
      lastD05 = d;
      lastType = profile[i].type;
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

// Quét đối chứng ngược chiều quanh ứng viên (triệt tiêu độ rơ SG90 và lọc mép tường 1 chiều) rồi đo 5 mẫu chốt tâm
static bool verifyClusterCenter(const ScanCluster *cand, int16_t scanDir,
                                uint8_t *outAngleDeg, float *outDistCm, float *outD04Cm)
{
  uint8_t verifyAng = cand->centerDeg;

  if (!cand->isWallBump)
  {
    // Quét ngược chiều cửa sổ hẹp ±10° quanh tâm ứng viên để đối chứng 2 chiều
    int16_t revDir = (scanDir < 0) ? (int16_t)SCAN_STEP_DEG : -(int16_t)SCAN_STEP_DEG;
    int16_t aLo = (int16_t)cand->centerDeg - (int16_t)(2u * SCAN_STEP_DEG);
    int16_t aHi = (int16_t)cand->centerDeg + (int16_t)(2u * SCAN_STEP_DEG);
    if (aLo < (int16_t)SERVO_SAFE_MIN_DEG) aLo = (int16_t)SERVO_SAFE_MIN_DEG;
    if (aHi > (int16_t)SERVO_SAFE_MAX_DEG) aHi = (int16_t)SERVO_SAFE_MAX_DEG;
    int16_t revStart = (revDir > 0) ? aLo : aHi;
    int16_t revEnd = (revDir > 0) ? aHi : aLo;

    moveServoSmooth((uint8_t)revStart);
    HAL_Delay(SCAN_SETTLE_MS);

    uint8_t revObjCount = 0u;
    uint8_t revObsCore = 0u;
    uint8_t revFirstDeg = 0u;
    uint8_t revLastDeg = 0u;
    for (int16_t a = revStart; (revDir > 0) ? (a <= revEnd) : (a >= revEnd); a += revDir)
    {
      if (a != revStart)
        moveServoForScanStep((uint8_t)a);
      float rd05 = ECHO_ERROR_TIMEOUT;
      float rd04 = ECHO_ERROR_TIMEOUT;
      measureDualDistanceCM(&rd05, &rd04);
      if (isValidD05(rd05) || (rd04 > 0.0f && rd04 <= SRF04_MAX_RANGE_CM))
        measureDualDistanceConfirmCM(rd05, rd04, &rd05, &rd04);
      ScanTargetType rType = scanClassifyTarget(rd05, rd04);
      int16_t dAng = a - (int16_t)cand->centerDeg;
      if (dAng < 0) dAng = -dAng;
      if (rType == TARGET_OBSTACLE && dAng <= (int16_t)SCAN_STEP_DEG &&
          fabsf(effectiveWallDistCm(rd05, rd04) - cand->clusterDist) <= SCAN_VERIFY_DEPTH_TOL_CM)
      {
        revObsCore++;
      }
      else if (rType == TARGET_OBJECT && fabsf(rd05 - cand->clusterDist) <= SCAN_VERIFY_DEPTH_TOL_CM)
      {
        if (revObjCount == 0u)
          revFirstDeg = (uint8_t)a;
        revLastDeg = (uint8_t)a;
        revObjCount++;
      }
    }

    if (revObsCore >= SCAN_CONFIRM_COUNT || revObjCount < SCAN_CONFIRM_COUNT)
    {
      debugLog("CLUSTER_REJECT_REV center=%u d=%s revObj=%u revObs=%u\r\n",
               cand->centerDeg, debugF1(cand->clusterDist), revObjCount, revObsCore);
      return false;
    }
    uint8_t revCenter = (uint8_t)(((uint16_t)revFirstDeg + (uint16_t)revLastDeg) / 2u);
    verifyAng = (uint8_t)(((uint16_t)cand->centerDeg + (uint16_t)revCenter) / 2u);
  }

  int16_t preload = (scanDir < 0)
                        ? ((int16_t)verifyAng + (int16_t)SCAN_BACKLASH_PRELOAD_DEG)
                        : ((int16_t)verifyAng - (int16_t)SCAN_BACKLASH_PRELOAD_DEG);
  if (preload < (int16_t)SERVO_SAFE_MIN_DEG)
    preload = (int16_t)SERVO_SAFE_MIN_DEG;
  if (preload > (int16_t)SERVO_SAFE_MAX_DEG)
    preload = (int16_t)SERVO_SAFE_MAX_DEG;

  if (getServoAngle() != (uint8_t)preload)
  {
    moveServoSmooth((uint8_t)preload);
    HAL_Delay(SCAN_STEP_SETTLE_MS);
  }
  moveServoSmooth(verifyAng);
  HAL_Delay(SCAN_SETTLE_MS);

  float validDists[SCAN_VERIFY_SAMPLES];
  float validD04[SCAN_VERIFY_SAMPLES];
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
      {
        validD04[objHits] = d04;
        validDists[objHits++] = d05;
      }
    }
    else if (type == TARGET_OBSTACLE)
    {
      obsHits++;
    }
    else if (type == TARGET_OBJECT &&
             fabsf(d05 - cand->clusterDist) <= SCAN_VERIFY_DEPTH_TOL_CM)
    {
      validD04[objHits] = d04;
      validDists[objHits++] = d05;
    }
  }

  uint8_t maxObs = (cand->count <= 1u) ? 0u : SCAN_VERIFY_MAX_OBS_HITS;
  if (obsHits <= maxObs && objHits >= SCAN_VERIFY_MIN_HITS)
  {
    *outAngleDeg = verifyAng;
    *outDistCm = computeMedian(validDists, objHits);
    *outD04Cm = computeMedian(validD04, objHits);
    return true;
  }

  debugLog("CLUSTER_REJECT center=%u d=%s obj=%u/%u obs=%u\r\n",
           verifyAng, debugF1(cand->clusterDist), objHits, SCAN_VERIFY_SAMPLES, obsHits);
  return false;
}

// Kiểm tra tường trước mũi có thực sự chắn đường tới ứng viên hay không (không loại nhầm vật gần hơn hoặc nằm ở phía thoáng đối diện)
static bool isCandBlockedByFrontWall(const ScanCluster *cand, bool hasFrontWall,
                                     float wallD05, uint8_t wallMinDeg, uint8_t wallMaxDeg)
{
  if (!hasFrontWall || cand->isWallBump)
    return false;
  if (wallD05 >= AGV_GRAB_DIST_MAX_CM)
  {
    bool wallRightOnly = (wallMaxDeg < SCAN_WALL_MIN_DEG);
    bool wallLeftOnly = (wallMinDeg > SCAN_WALL_MAX_DEG);
    if ((wallRightOnly && cand->centerDeg >= SCAN_HOME_ANGLE) ||
        (wallLeftOnly && cand->centerDeg <= SCAN_HOME_ANGLE))
      return false;
  }
  return cand->clusterDist >= (wallD05 - SCAN_CORE_DEPTH_MARGIN_CM);
}

// Xác nhận tường áp sát rộng (60°..120°), mù kép hoặc tường thẳng trước mũi (80°..100°)
static bool detectFrontWallFromProfile(const ScanPoint *profile, uint8_t count,
                                       float *outD05Cm, float *outD04Cm,
                                       uint8_t *outMinDeg, uint8_t *outMaxDeg)
{
  uint8_t closeIdx[SCAN_MAX_POINTS];
  float closeBuf[SCAN_MAX_POINTS];
  uint8_t closeCount = 0u;
  uint8_t blindCount = 0u;
  *outMinDeg = SCAN_WALL_MIN_DEG;
  *outMaxDeg = SCAN_WALL_MAX_DEG;

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
      bool d04CloseOrErr = (d04 <= SCAN_CLOSE_WALL_SRF04_CM ||
                            d04 == ECHO_ERROR_TIMEOUT ||
                            d04 == ECHO_ERROR_TOO_CLOSE);
      bool isBlind = d04CloseOrErr &&
                     ((d05 == ECHO_ERROR_TOO_CLOSE) ||
                      (d05 > 0.0f && d05 < SCAN_MIN_RANGE_CM) ||
                      (d05 == ECHO_ERROR_TIMEOUT));
      if (isBlind)
        blindCount++;
    }
  }

  if (closeCount >= SCAN_CONFIRM_COUNT)
  {
    uint8_t a0 = profile[closeIdx[0]].angleDeg;
    uint8_t a1 = profile[closeIdx[closeCount - 1u]].angleDeg;
    *outMinDeg = (a0 <= a1) ? a0 : a1;
    *outMaxDeg = (a0 >= a1) ? a0 : a1;
    *outD05Cm = computeMedian(closeBuf, closeCount);
    // Ghi nhận đoạn tường áp sát vào bản đồ 2D để đường về không cắt xuyên qua vùng mù sát mũi
    if (lastWallSegCount == 0u)
      addLocalWallSeg(a0, a1, *outD05Cm, *outD05Cm);
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
  curScanFineMode = fineMode;
  resetLastExtra();

  uint8_t count = captureScanProfile(arcStartDeg, arcEndDeg, stepDeg, profile, SCAN_MAX_POINTS);
  uint8_t candCount = extractSegmentsAndCandidates(profile, count, candidates, SCAN_MAX_CLUSTERS);
  if (!fineMode)
    updateCorridorHints(profile, count);

  int16_t scanDir = (arcStartDeg <= arcEndDeg) ? (int16_t)stepDeg : -(int16_t)stepDeg;

  float wallD05 = ECHO_ERROR_TIMEOUT;
  float wallD04 = ECHO_ERROR_TIMEOUT;
  uint8_t wallMinDeg = SCAN_WALL_MIN_DEG;
  uint8_t wallMaxDeg = SCAN_WALL_MAX_DEG;
  bool hasFrontWall = detectFrontWallFromProfile(profile, count, &wallD05, &wallD04, &wallMinDeg, &wallMaxDeg);
  if (!hasFrontWall && curWideFrontWall)
  {
    hasFrontWall = true;
    wallD05 = curWideWallD05;
    wallD04 = ECHO_OUT_OF_RANGE_CM;
    wallMinDeg = SCAN_WALL_MIN_DEG;
    wallMaxDeg = SCAN_WALL_MAX_DEG;
  }

  if (hasFrontWall && wallD05 < WALL_BACKUP_TRIGGER_CM &&
      (candCount == 0u || isCandBlockedByFrontWall(&candidates[0], true, wallD05, wallMinDeg, wallMaxDeg)))
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
    if (isCandBlockedByFrontWall(&candidates[i], hasFrontWall, wallD05, wallMinDeg, wallMaxDeg))
      continue;

    if (fineMode)
    {
      if (candidates[i].centerDeg < SCAN_FINE_ACCEPT_MIN_DEG ||
          candidates[i].centerDeg > SCAN_FINE_ACCEPT_MAX_DEG)
        continue;
    }

    uint8_t verifiedAng = candidates[i].centerDeg;
    float verifiedDist = ECHO_ERROR_TIMEOUT;
    float verifiedD04 = ECHO_ERROR_TIMEOUT;
    if (verifyClusterCenter(&candidates[i], scanDir, &verifiedAng, &verifiedDist, &verifiedD04))
    {
      *angleDeg = verifiedAng;
      *distanceCm = verifiedDist;
      lastExtra.isWallBump = candidates[i].isWallBump;
      bool topInRng = (verifiedD04 >= SCAN_MIN_RANGE_CM && verifiedD04 <= SRF04_MAX_RANGE_CM);
      float diffCm = topInRng ? ((verifiedD04 - SENSOR_DEPTH_OFFSET_CM) - verifiedDist)
                              : (SRF04_MAX_RANGE_CM - SENSOR_DEPTH_OFFSET_CM - verifiedDist);
      debugLog("CLUSTER arc=[%u..%u] start=%u end=%u hits=%u bump=%u -> center=%u d05=%s d04=%s diff=%s\r\n",
               arcStartDeg, arcEndDeg, candidates[i].startDeg, candidates[i].endDeg,
               candidates[i].count, candidates[i].isWallBump ? 1u : 0u,
               verifiedAng, debugF1(verifiedDist), debugF1(verifiedD04), debugF1(diffCm));
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
  curScanFineMode = false;
  resetLastExtra();
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

void scanGetLastExtraInfo(ScanExtraInfo *outInfo)
{
  if (outInfo)
    *outInfo = lastExtra;
}

uint8_t scanGetLastWallSegments(ScanWallSegLocal *outSegs, uint8_t maxSegs)
{
  if (!outSegs || maxSegs == 0u)
    return 0u;
  uint8_t n = (lastWallSegCount < maxSegs) ? lastWallSegCount : maxSegs;
  for (uint8_t i = 0u; i < n; i++)
    outSegs[i] = lastWallSegs[i];
  return n;
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
