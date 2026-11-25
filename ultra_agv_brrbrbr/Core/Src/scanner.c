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
  float distCm;
  float minDistCm;
  float startDistCm;
  float endDistCm;
  ScanTargetType type;
} ScanCluster;

static bool lastFrontClear = false;
static bool lastLeftWall = false;
static bool lastRightWall = false;
static ScanExtraInfo lastExtra = {false, false, false, SCAN_HOME_ANGLE, 0.0f};
static ScanWallSegLocal lastWallSegs[SCAN_MAX_WALL_SEGS_LOCAL];
static uint8_t lastWallSegCount = 0u;

static void addLocalWallSeg(const ScanCluster *c)
{
  if (lastWallSegCount >= SCAN_MAX_WALL_SEGS_LOCAL)
    return;
  lastWallSegs[lastWallSegCount].startDeg = c->startDeg;
  lastWallSegs[lastWallSegCount].endDeg = c->endDeg;
  lastWallSegs[lastWallSegCount].startDistCm = c->startDistCm;
  lastWallSegs[lastWallSegCount].endDistCm = c->endDistCm;
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

static bool isFrontAngle(uint8_t angleDeg)
{
  return (angleDeg >= SCAN_CLOSE_WALL_MIN_DEG && angleDeg <= SCAN_CLOSE_WALL_MAX_DEG);
}

// Mục tiêu nghi ngờ nằm chính diện mũi xe (chạy thẳng tới soi được, không cần xoay)
static bool isSuspectFront(uint8_t angleDeg)
{
  int16_t err = (int16_t)angleDeg - (int16_t)SCAN_HOME_ANGLE;
  if (err < 0)
    err = -err;
  return err <= (int16_t)SCAN_SUSPECT_FRONT_HALF_DEG;
}

// Ghi nhận mục tiêu nghi ngờ: ưu tiên chính diện, cùng loại thì giữ cụm gặp trước (cụm đã sắp gần -> xa)
static void noteSuspect(uint8_t angleDeg, float distCm)
{
  if (lastExtra.hasSuspect && (isSuspectFront(lastExtra.suspectAngleDeg) || !isSuspectFront(angleDeg)))
    return;
  lastExtra.hasSuspect = true;
  lastExtra.suspectAngleDeg = angleDeg;
  lastExtra.suspectDistCm = distCm;
}

ScanTargetType scanClassifyTarget(float d05RawCm, float d04RawCm)
{
  if (d04RawCm == ECHO_ERROR_TOO_CLOSE ||
      (d04RawCm > 0.0f && d04RawCm < SCAN_MIN_RANGE_CM))
    return TARGET_OBSTACLE;

  if (d04RawCm >= SCAN_MIN_RANGE_CM && d04RawCm <= SRF04_MAX_RANGE_CM)
  {
    // Chỉ kết luận tường khi d05 quá sát (-2/<2 cm) VÀ d04 cũng nằm sát mặt tường; nếu d04 xa thì là vật sát mũi
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

  // Chỉ SRF05 dưới thấy: vật thấp khi đủ gần (SRF05 trên không với tới đỉnh vật), xa hơn là mép/cạnh vật cản phản xạ yếu
  if (d04RawCm > SRF04_MAX_RANGE_CM)
    return (d05RawCm <= SCAN_TOP_BLIND_MAX_CM) ? TARGET_OBJECT : TARGET_OBSTACLE;

  // Tường: SRF05 trên (đã bù lùi) gần bằng SRF05 dưới; vật thấp: SRF05 trên chỉ thấy đỉnh vật nên xa hơn
  float d04AdjCm = d04RawCm - SENSOR_DEPTH_OFFSET_CM;
  if (d04AdjCm <= (d05RawCm + SCAN_OBSTACLE_TOLERANCE_CM))
    return TARGET_OBSTACLE;
  return TARGET_OBJECT;
}

// Quy đổi cự ly tường hiệu dụng từ SRF05 hoặc bù lùi SRF05 trên khi SRF05 dưới bị mù do áp sát tường
static float effectiveWallDistCm(float d05Cm, float d04Cm)
{
  if (isValidD05(d05Cm))
    return d05Cm;
  if (d04Cm >= SCAN_MIN_RANGE_CM && d04Cm <= SRF04_MAX_RANGE_CM)
  {
    float adj = d04Cm - SENSOR_DEPTH_OFFSET_CM;
    return (adj > 0.0f) ? adj : 0.0f;
  }
  return 0.0f;
}

// Sắp xếp tăng dần mảng số thực tại chỗ
static void sortFloats(float *vals, uint8_t count)
{
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
}

// Tính trung vị của mảng số thực (sắp xếp tại chỗ)
static float computeMedian(float *vals, uint8_t count)
{
  if (count == 0u)
    return ECHO_ERROR_TIMEOUT;

  sortFloats(vals, count);
  if ((count & 1u) != 0u)
    return vals[count / 2u];
  return (vals[count / 2u - 1u] + vals[count / 2u]) * 0.5f;
}

// Phân loại 1 nhóm mẫu cùng mặt phản xạ theo tứ phân vị dưới
// của chênh lệch (SRF05 trên chỉ đo dài ra khi tiếng vọng yếu ở mép chùm/mặt nghiêng, không bao giờ ngắn đi)
static ScanTargetType classifySamples(const float *d05Cm, const float *d04Cm, uint8_t count, float *outDiffCm)
{
  float diffs[SCAN_MAX_POINTS];
  uint8_t diffCount = 0u;
  for (uint8_t i = 0u; i < count; i++)
  {
    if (d04Cm[i] >= SCAN_MIN_RANGE_CM && d04Cm[i] <= SRF04_MAX_RANGE_CM)
      diffs[diffCount++] = d04Cm[i] - SENSOR_DEPTH_OFFSET_CM - d05Cm[i];
  }

  *outDiffCm = 0.0f;
  if (count == 0u)
    return TARGET_NONE;
  // SRF05 trên thấy dưới 1/2 số mẫu: chưa kết luận, để nơi gọi xét cự ly và bằng chứng xung quanh
  if ((uint8_t)(diffCount * 2u) < count)
    return TARGET_NONE;

  sortFloats(diffs, diffCount);
  *outDiffCm = diffs[(diffCount - 1u) / 4u];
  return (*outDiffCm <= SCAN_OBSTACLE_TOLERANCE_CM) ? TARGET_OBSTACLE : TARGET_OBJECT;
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

// Quét trọn cung và lưu profile góc; đo lại lọc 2/3 tại các điểm có tín hiệu
static uint8_t captureScanProfile(uint8_t arcStartDeg, uint8_t arcEndDeg, uint8_t stepDeg,
                                  ScanPoint *profile, uint8_t maxPoints)
{
  uint8_t count = 0u;
  uint8_t seqObjHits = 0u;

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

    bool d05Hit = isValidD05(d05RawCm) || d05RawCm == ECHO_ERROR_TIMEOUT || d05RawCm == ECHO_ERROR_TOO_CLOSE;
    bool d04Hit = (d04RawCm == ECHO_ERROR_TOO_CLOSE ||
                   (d04RawCm > 0.0f && d04RawCm <= SRF04_MAX_RANGE_CM));
    if (d05Hit || d04Hit)
      measureDualDistanceConfirmCM(d05RawCm, d04RawCm, &d05RawCm, &d04RawCm);

    ScanTargetType type = scanClassifyTarget(d05RawCm, d04RawCm);
    profile[count].angleDeg = (uint8_t)a;
    profile[count].d05Cm = d05RawCm;
    profile[count].d04Cm = d04RawCm;
    profile[count].type = type;
    count++;

    seqObjHits = (type == TARGET_OBJECT) ? (uint8_t)(seqObjHits + 1u) : 0u;
    debugScan(d05RawCm, d04RawCm, (uint8_t)a, seqObjHits, type);
  }

  return count;
}

// Dựng cụm từ đoạn liên tục: lõi là các điểm liền kề quanh cự ly nhỏ nhất (trong biên SCAN_CORE_DEPTH_MARGIN_CM),
// chỉ lõi được dùng phân loại vì ở mép cụm chênh lệch 2 cảm biến không tin được
static void buildCluster(const ScanPoint *profile, const uint8_t *idx, uint8_t n, ScanCluster *out)
{
  uint8_t minPos = 0u;
  for (uint8_t i = 1u; i < n; i++)
  {
    if (profile[idx[i]].d05Cm < profile[idx[minPos]].d05Cm)
      minPos = i;
  }
  float limitCm = profile[idx[minPos]].d05Cm + SCAN_CORE_DEPTH_MARGIN_CM;
  uint8_t coreL = minPos;
  uint8_t coreR = minPos;
  while (coreL > 0u && profile[idx[coreL - 1u]].d05Cm <= limitCm)
    coreL--;
  while ((uint8_t)(coreR + 1u) < n && profile[idx[coreR + 1u]].d05Cm <= limitCm)
    coreR++;

  // Mặt tường cho cự ly gần như phẳng nên lõi có thể lan ra tận mép chùm: chỉ giữ ±SCAN_CORE_HALF_DEG quanh giữa lõi
  int16_t midDeg = ((int16_t)profile[idx[coreL]].angleDeg + (int16_t)profile[idx[coreR]].angleDeg) / 2;
  float d05s[SCAN_MAX_POINTS];
  float d04s[SCAN_MAX_POINTS];
  uint8_t coreCount = 0u;
  for (uint8_t i = coreL; i <= coreR; i++)
  {
    int16_t offDeg = (int16_t)profile[idx[i]].angleDeg - midDeg;
    if (offDeg > (int16_t)SCAN_CORE_HALF_DEG || offDeg < -(int16_t)SCAN_CORE_HALF_DEG)
      continue;
    d05s[coreCount] = profile[idx[i]].d05Cm;
    d04s[coreCount] = profile[idx[i]].d04Cm;
    coreCount++;
  }

  float diffCm = 0.0f;
  out->type = classifySamples(d05s, d04s, coreCount, &diffCm);
  out->startDeg = profile[idx[0]].angleDeg;
  out->endDeg = profile[idx[n - 1u]].angleDeg;
  out->centerDeg = (uint8_t)midDeg;
  out->startDistCm = profile[idx[0]].d05Cm;
  out->endDistCm = profile[idx[n - 1u]].d05Cm;
  out->distCm = computeMedian(d05s, coreCount);
  out->minDistCm = profile[idx[minPos]].d05Cm;

  // SRF05 trên thấy đỉnh vật ở điểm cùng độ sâu lõi chỗ khác trong cụm (lõi có thể rơi vào vùng SRF05 trên mất sóng)
  uint8_t objHits = 0u;
  for (uint8_t i = 0u; i < n; i++)
  {
    float d05 = profile[idx[i]].d05Cm;
    float d04 = profile[idx[i]].d04Cm;
    if (fabsf(d05 - out->distCm) <= SCAN_WALL_EVIDENCE_DEPTH_CM && d04 >= SCAN_MIN_RANGE_CM &&
        d04 <= SRF04_MAX_RANGE_CM && (d04 - SENSOR_DEPTH_OFFSET_CM - d05) > SCAN_OBSTACLE_TOLERANCE_CM)
      objHits++;
  }
  // Lõi không có SRF05 trên: vật thấp nếu đủ gần hoặc SRF05 trên thấy đỉnh vật ở chỗ khác, không thì là mép vật cản
  if (out->type == TARGET_NONE)
    out->type = (out->distCm <= SCAN_TOP_BLIND_MAX_CM || objHits >= SCAN_CONFIRM_COUNT) ? TARGET_OBJECT : TARGET_OBSTACLE;

  // Tường gấp khúc (góc chữ L): vùng gần nhất cục bộ khác trong cụm, cùng độ sâu lõi, mang dấu hiệu tường thì cả cụm là tường.
  // Không xét sườn của lõi vì vật nhìn lệch trục θ có chênh lệch nhỏ đi 7(1 - cosθ) trông giống tường
  for (uint8_t i = 0u; i < n && out->type == TARGET_OBJECT; i++)
  {
    int16_t offDeg = (int16_t)profile[idx[i]].angleDeg - midDeg;
    float dMin = profile[idx[i]].d05Cm;
    if (offDeg <= (int16_t)SCAN_OTHER_MIN_SEP_DEG && offDeg >= -(int16_t)SCAN_OTHER_MIN_SEP_DEG)
      continue;
    if (fabsf(dMin - out->distCm) > SCAN_WALL_EVIDENCE_DEPTH_CM)
      continue;

    bool isLocalMin = true;
    uint8_t m = 0u;
    for (uint8_t j = 0u; j < n; j++)
    {
      int16_t dj = (int16_t)profile[idx[j]].angleDeg - (int16_t)profile[idx[i]].angleDeg;
      if (dj < 0)
        dj = -dj;
      if (dj <= (int16_t)SCAN_LOCAL_MIN_HALF_DEG && profile[idx[j]].d05Cm < dMin)
        isLocalMin = false;
      if (dj <= (int16_t)SCAN_CORE_HALF_DEG && profile[idx[j]].d05Cm <= dMin + SCAN_CORE_DEPTH_MARGIN_CM)
      {
        d05s[m] = profile[idx[j]].d05Cm;
        d04s[m] = profile[idx[j]].d04Cm;
        m++;
      }
    }
    float otherDiffCm = 0.0f;
    if (isLocalMin && classifySamples(d05s, d04s, m, &otherDiffCm) == TARGET_OBSTACLE)
      out->type = TARGET_OBSTACLE;
  }

  debugLog("SEG start=%u end=%u n=%u core=%u center=%u d=%s diff=%s type=%s\r\n",
           out->startDeg, out->endDeg, n, coreCount, out->centerDeg, debugF1(out->distCm), debugF1(diffCm),
           (out->type == TARGET_OBSTACLE) ? "WALL" : "OBJ");
}

// Tách các đoạn bề mặt liên tục theo d05 (bắc cầu 1 điểm rớt sóng), dựng cụm và sắp gần -> xa
static uint8_t extractClusters(const ScanPoint *profile, uint8_t count, ScanCluster *clusters, uint8_t maxClusters)
{
  uint8_t clusterCount = 0u;
  uint8_t segIdx[SCAN_MAX_POINTS];
  uint8_t segLen = 0u;

  for (uint8_t i = 0u; i <= count; i++)
  {
    bool valid = (i < count) && isValidD05(profile[i].d05Cm);
    bool isBridge = (i < count) && !valid && segLen > 0u && (uint8_t)(i + 1u) < count &&
                    isValidD05(profile[i + 1u].d05Cm) &&
                    fabsf(profile[i + 1u].d05Cm - profile[segIdx[segLen - 1u]].d05Cm) <= SCAN_SEG_JUMP_CM;
    if (isBridge)
      continue;

    bool split = !valid ||
                 (segLen > 0u && fabsf(profile[i].d05Cm - profile[segIdx[segLen - 1u]].d05Cm) > SCAN_SEG_JUMP_CM);
    if (split && segLen > 0u)
    {
      if (segLen >= SCAN_CONFIRM_COUNT && clusterCount < maxClusters)
        buildCluster(profile, segIdx, segLen, &clusters[clusterCount++]);
      segLen = 0u;
    }
    if (valid)
      segIdx[segLen++] = i;
  }

  for (uint8_t i = 1u; i < clusterCount; i++)
  {
    ScanCluster key = clusters[i];
    uint8_t j = i;
    while (j > 0u && clusters[j - 1u].distCm > key.distCm)
    {
      clusters[j] = clusters[j - 1u];
      j--;
    }
    clusters[j] = key;
  }
  return clusterCount;
}

// Vật dựa sát tường nghiêng bị gộp vào cụm tường: tìm dải >= SCAN_OBJ_RUN_MIN điểm liền kề trong tầm phân loại có chênh lệch
// kiểu vật ổn định (mép tường cũng cho chênh dương nhưng dao động mạnh); bỏ dải chạm mép cung. Trả về dải gần nhất
static bool findObjectRun(const ScanPoint *profile, uint8_t count, ScanCluster *out)
{
  bool found = false;
  uint8_t runStart = 0u;
  uint8_t runLen = 0u;
  float minDiff = 0.0f;
  float maxDiff = 0.0f;

  for (uint8_t i = 0u; i <= count; i++)
  {
    bool ok = false;
    float diff = 0.0f;
    if (i < count)
    {
      float d05 = profile[i].d05Cm;
      float d04 = profile[i].d04Cm;
      diff = d04 - SENSOR_DEPTH_OFFSET_CM - d05;
      ok = d05 >= SCAN_MIN_RANGE_CM && d05 <= SCAN_CLASSIFY_MAX_CM &&
           d04 >= SCAN_MIN_RANGE_CM && d04 <= SRF04_MAX_RANGE_CM &&
           diff >= SCAN_OBJ_DIFF_MIN_CM && diff <= SCAN_OBJ_DIFF_MAX_CM;
    }
    if (ok)
    {
      if (runLen == 0u)
      {
        runStart = i;
        minDiff = diff;
        maxDiff = diff;
      }
      if (diff < minDiff) minDiff = diff;
      if (diff > maxDiff) maxDiff = diff;
      runLen++;
      continue;
    }
    // 2 điểm kề dải phải có SRF05 trên: ở rìa vùng phủ SRF05 trên đo dài ra (giống vật) ngay trước khi mất sóng
    // Điểm kề cũng không được là mép cung (vật lấn mép đã có luật báo nghi ngờ riêng)
    bool edgesSeen = runStart > 1u && (uint8_t)(runStart + runLen + 1u) < count &&
                     profile[runStart - 1u].d04Cm >= SCAN_MIN_RANGE_CM &&
                     profile[runStart - 1u].d04Cm <= SRF04_MAX_RANGE_CM &&
                     profile[runStart + runLen].d04Cm >= SCAN_MIN_RANGE_CM &&
                     profile[runStart + runLen].d04Cm <= SRF04_MAX_RANGE_CM;
    if (runLen >= SCAN_OBJ_RUN_MIN && (maxDiff - minDiff) <= SCAN_OBJ_RUN_SPREAD_CM && edgesSeen)
    {
      float d05s[SCAN_MAX_POINTS];
      for (uint8_t k = 0u; k < runLen; k++)
        d05s[k] = profile[runStart + k].d05Cm;
      float distCm = computeMedian(d05s, runLen);
      if (!found || distCm < out->distCm)
      {
        found = true;
        out->startDeg = profile[runStart].angleDeg;
        out->endDeg = profile[runStart + runLen - 1u].angleDeg;
        out->centerDeg = (uint8_t)(((uint16_t)out->startDeg + (uint16_t)out->endDeg) / 2u);
        out->distCm = distCm;
        out->minDistCm = d05s[0];
        out->startDistCm = profile[runStart].d05Cm;
        out->endDistCm = profile[runStart + runLen - 1u].d05Cm;
        out->type = TARGET_OBJECT;
      }
    }
    runLen = 0u;
  }
  return found;
}

// Tường áp sát mũi làm SRF05 dưới mù (-2/-1/<2 cm): đếm điểm OBSTACLE không có d05 hợp lệ ở cung trước mũi
static bool detectBlindFrontWall(const ScanPoint *profile, uint8_t count, float *outD05Cm, float *outD04Cm)
{
  float d05s[SCAN_MAX_POINTS];
  float d04s[SCAN_MAX_POINTS];
  uint8_t blindCount = 0u;
  for (uint8_t i = 0u; i < count; i++)
  {
    float d05 = profile[i].d05Cm;
    bool d05Blind = (d05 == ECHO_ERROR_TIMEOUT || d05 == ECHO_ERROR_TOO_CLOSE || (d05 > 0.0f && d05 < SCAN_MIN_RANGE_CM));
    if (isFrontAngle(profile[i].angleDeg) && profile[i].type == TARGET_OBSTACLE && d05Blind)
    {
      d05s[blindCount] = effectiveWallDistCm(profile[i].d05Cm, profile[i].d04Cm);
      d04s[blindCount] = profile[i].d04Cm;
      blindCount++;
    }
  }
  if (blindCount < SCAN_FRONT_BLIND_WALL_HITS)
    return false;
  *outD05Cm = computeMedian(d05s, blindCount);
  *outD04Cm = computeMedian(d04s, blindCount);
  return true;
}

// Đến đúng tâm cụm (lấy đà bù độ rơ SG90) rồi đo lại nhiều mẫu để chốt cự ly và phân loại vật
static bool verifyClusterCenter(const ScanCluster *c, int16_t scanDir,
                                uint8_t *outAngleDeg, float *outDistCm, float *outD04Cm)
{
  int16_t preload = (scanDir < 0)
                        ? ((int16_t)c->centerDeg + (int16_t)SCAN_BACKLASH_PRELOAD_DEG)
                        : ((int16_t)c->centerDeg - (int16_t)SCAN_BACKLASH_PRELOAD_DEG);
  if (preload < (int16_t)SERVO_SAFE_MIN_DEG)
    preload = (int16_t)SERVO_SAFE_MIN_DEG;
  if (preload > (int16_t)SERVO_SAFE_MAX_DEG)
    preload = (int16_t)SERVO_SAFE_MAX_DEG;

  moveServoSmooth((uint8_t)preload);
  HAL_Delay(SCAN_STEP_SETTLE_MS);
  moveServoSmooth(c->centerDeg);
  HAL_Delay(SCAN_SETTLE_MS);

  float d05s[SCAN_VERIFY_SAMPLES];
  float d04s[SCAN_VERIFY_SAMPLES];
  uint8_t hits = 0u;
  for (uint8_t i = 0u; i < SCAN_VERIFY_SAMPLES; i++)
  {
    float d05 = ECHO_ERROR_TIMEOUT;
    float d04 = ECHO_ERROR_TIMEOUT;
    measureDualDistanceCM(&d05, &d04);
    if (isValidD05(d05) && fabsf(d05 - c->distCm) <= SCAN_VERIFY_DEPTH_TOL_CM)
    {
      d05s[hits] = d05;
      d04s[hits] = d04;
      hits++;
    }
  }

  // Chỉ xác minh cự ly SRF05 dưới ổn định tại tâm; không phân loại lại bằng mẫu thô 1 góc vì SRF05 trên
  // có thể bắt tiếng vọng tường ngay sau vật (đo ngắn đi) — phân loại đã chốt từ lõi cụm lúc quét
  if (hits >= SCAN_VERIFY_MIN_HITS)
  {
    *outAngleDeg = c->centerDeg;
    *outDistCm = computeMedian(d05s, hits);
    *outD04Cm = computeMedian(d04s, hits);
    return true;
  }

  debugLog("CLUSTER_REJECT center=%u d=%s hits=%u/%u\r\n",
           c->centerDeg, debugF1(c->distCm), hits, SCAN_VERIFY_SAMPLES);
  return false;
}

// Tổng hợp gợi ý độ thông thoáng phía trước và tường hai bên hành lang từ profile quét
static void updateCorridorHints(const ScanPoint *profile, uint8_t count)
{
  bool frontClear = true;
  bool leftWall = false;
  bool rightWall = false;

  for (uint8_t i = 0u; i < count; i++)
  {
    uint8_t ang = profile[i].angleDeg;
    if (ang >= SCAN_FRONT_CLEAR_MIN_DEG && ang <= SCAN_FRONT_CLEAR_MAX_DEG)
    {
      if (profile[i].d05Cm <= SCAN_EXPAND_CLEAR_CM || profile[i].d04Cm <= SCAN_EXPAND_CLEAR_CM)
        frontClear = false;
    }
    if (profile[i].type == TARGET_OBSTACLE && effectiveWallDistCm(profile[i].d05Cm, profile[i].d04Cm) <= SCAN_CLASSIFY_MAX_CM)
    {
      if (ang >= SCAN_SIDE_WALL_LEFT_MIN_DEG) leftWall = true;
      if (ang <= SCAN_SIDE_WALL_RIGHT_MAX_DEG) rightWall = true;
    }
  }

  lastFrontClear = frontClear;
  lastLeftWall = leftWall;
  lastRightWall = rightWall;
}

static ScanStatus reportWall(float wallD05, float wallD04, float *distanceCm, uint8_t *angleDeg)
{
  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  lastExtra.isConfirmedWideWall = true;
  *distanceCm = wallD05;
  *angleDeg = SCAN_HOME_ANGLE;
  debugWallDetected(wallD05, wallD04);
  HAL_Delay(WALL_HOLD_MS);
  return SCAN_WALL_FOUND;
}

// Quét cung, gom cụm theo SRF05 dưới, phân loại tâm cụm trong tầm tin cậy; cụm xa chỉ báo nghi ngờ để xe tiến lại gần
static ScanStatus scanArcCluster(uint8_t arcStartDeg, uint8_t arcEndDeg, uint8_t stepDeg,
                                 bool fineMode, float *distanceCm, uint8_t *angleDeg)
{
  ScanPoint profile[SCAN_MAX_POINTS];
  ScanCluster clusters[SCAN_MAX_CLUSTERS];

  resetLastExtra();
  uint8_t count = captureScanProfile(arcStartDeg, arcEndDeg, stepDeg, profile, SCAN_MAX_POINTS);
  uint8_t clusterCount = extractClusters(profile, count, clusters, SCAN_MAX_CLUSTERS);
  if (!fineMode)
    updateCorridorHints(profile, count);
  int16_t scanDir = (arcStartDeg <= arcEndDeg) ? (int16_t)stepDeg : -(int16_t)stepDeg;

  float wallD05 = ECHO_ERROR_TIMEOUT;
  float wallD04 = ECHO_ERROR_TIMEOUT;
  if (detectBlindFrontWall(profile, count, &wallD05, &wallD04))
    return reportWall(wallD05, wallD04, distanceCm, angleDeg);

  const ScanCluster *frontWall = NULL;
  for (uint8_t i = 0u; i < clusterCount; i++)
  {
    const ScanCluster *c = &clusters[i];
    if (c->distCm > SCAN_CLASSIFY_MAX_CM)
    {
      // Ngoài tầm phân loại tin cậy: ghi nhận cụm xa để xe tiến lại soi
      if (!fineMode)
        noteSuspect(c->centerDeg, c->distCm);
      continue;
    }

    if (c->type == TARGET_OBSTACLE)
    {
      addLocalWallSeg(c);
      // Tường nghiêng có lõi lệch ra mép cung nhưng thân vẫn chắn ngang mũi: xét cả bề rộng cụm
      uint8_t loDeg = (c->startDeg < c->endDeg) ? c->startDeg : c->endDeg;
      uint8_t hiDeg = (c->startDeg < c->endDeg) ? c->endDeg : c->startDeg;
      if (frontWall == NULL && hiDeg >= SCAN_CLOSE_WALL_MIN_DEG && loDeg <= SCAN_CLOSE_WALL_MAX_DEG)
        frontWall = c;
      continue;
    }

    if (fineMode && (c->centerDeg < SCAN_FINE_ACCEPT_MIN_DEG || c->centerDeg > SCAN_FINE_ACCEPT_MAX_DEG))
      continue;

    // Điểm gần nhất nằm ngay mép cung: tâm vật có thể ở ngoài cung, chỉ thấy mép chùm sóng -> báo nghi ngờ để xe xoay lại soi
    bool atArcEdge = (c->startDeg == arcStartDeg && c->startDistCm <= c->minDistCm + SCAN_EDGE_MIN_TOL_CM) ||
                     (c->endDeg == arcEndDeg && c->endDistCm <= c->minDistCm + SCAN_EDGE_MIN_TOL_CM);
    if (atArcEdge)
    {
      if (!fineMode)
        noteSuspect((c->startDistCm <= c->endDistCm) ? c->startDeg : c->endDeg, c->minDistCm);
      debugLog("CLUSTER_EDGE center=%u d=%s\r\n", c->centerDeg, debugF1(c->distCm));
      continue;
    }

    uint8_t verifiedAng = c->centerDeg;
    float verifiedDist = ECHO_ERROR_TIMEOUT;
    float verifiedD04 = ECHO_ERROR_TIMEOUT;
    if (verifyClusterCenter(c, scanDir, &verifiedAng, &verifiedDist, &verifiedD04))
    {
      *angleDeg = verifiedAng;
      *distanceCm = verifiedDist;
      debugLog("CLUSTER arc=[%u..%u] start=%u end=%u -> center=%u d05=%s d04=%s\r\n",
               arcStartDeg, arcEndDeg, c->startDeg, c->endDeg,
               verifiedAng, debugF1(verifiedDist), debugF1(verifiedD04));
      HAL_Delay(SCAN_FOUND_HOLD_MS);
      return SCAN_OBJECT_FOUND;
    }
    // Xác minh không ổn định: không bỏ qua mà báo nghi ngờ để xe soi lại
    if (!fineMode)
      noteSuspect(c->centerDeg, c->distCm);
  }

  // Lượt soát cuối: vật dựa sát tường bị gộp vào cụm tường (trừ khi tường chắn mũi đã quá sát)
  ScanCluster runCand;
  bool wallTooClose = (frontWall != NULL && frontWall->distCm < WALL_BACKUP_TRIGGER_CM);
  if (!wallTooClose && findObjectRun(profile, count, &runCand) &&
      !(fineMode && (runCand.centerDeg < SCAN_FINE_ACCEPT_MIN_DEG || runCand.centerDeg > SCAN_FINE_ACCEPT_MAX_DEG)))
  {
    uint8_t verifiedAng = runCand.centerDeg;
    float verifiedDist = ECHO_ERROR_TIMEOUT;
    float verifiedD04 = ECHO_ERROR_TIMEOUT;
    if (verifyClusterCenter(&runCand, scanDir, &verifiedAng, &verifiedDist, &verifiedD04))
    {
      *angleDeg = verifiedAng;
      *distanceCm = verifiedDist;
      lastExtra.isWallBump = true;
      debugLog("CLUSTER_RUN start=%u end=%u -> center=%u d05=%s d04=%s\r\n",
               runCand.startDeg, runCand.endDeg, verifiedAng, debugF1(verifiedDist), debugF1(verifiedD04));
      HAL_Delay(SCAN_FOUND_HOLD_MS);
      return SCAN_OBJECT_FOUND;
    }
  }

  if (frontWall != NULL)
    return reportWall(frontWall->distCm, ECHO_OUT_OF_RANGE_CM, distanceCm, angleDeg);

  moveServoSmooth(SCAN_HOME_ANGLE);
  HAL_Delay(SCAN_SETTLE_MS);
  debugLog("SCAN_EMPTY arc=[%u..%u] clusters=%u suspect=%u ang=%u d=%s\r\n",
           arcStartDeg, arcEndDeg, clusterCount, lastExtra.hasSuspect ? 1u : 0u,
           lastExtra.suspectAngleDeg, debugF1(lastExtra.suspectDistCm));
  return ERROR_NO_OBJECTIVE;
}

void scanInit(void)
{
  servoInit();
  lastFrontClear = false;
  lastLeftWall = false;
  lastRightWall = false;
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
