#include "destination.h"
#include "debug.h"
#include "encoder.h"
#include "imu.h"
#include "motion.h"
#include "motor.h"
#include "project_config.h"
#include "servo.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

typedef enum
{
  DEST_ACT_SPIN = 0,
  DEST_ACT_DRIVE
} DestActionType;

typedef struct
{
  uint8_t type;
  int16_t value;        // SPIN: độ (>0 CW, <0 CCW); DRIVE: cm (>0 tiến, <0 lùi)
  float prevHeadingDeg; // Hướng IMU (CCW+) trước khi thực hiện bước
} DestAction;

typedef struct
{
  float x1Cm;
  float y1Cm;
  float x2Cm;
  float y2Cm;
  uint8_t startDeg;
  uint8_t endDeg;
} WallSeg2D;

typedef struct
{
  float xCm;
  float yCm;
} Waypoint2D;

static DestAction actionStack[DEST_MAX_ACTIONS];
static uint8_t actionCount = 0u;
static bool stackOverflow = false;

// shortcut: giới hạn tĩnh 16 đoạn tường trong RAM, nâng lên thay thế vòng tròn nếu sa bàn > 4 phòng
static WallSeg2D wallSegs[DEST_MAX_WALL_SEGS];
static uint8_t wallSegCount = 0u;
static bool wallOverflow = false;

// shortcut: giới hạn tĩnh 24 mốc đường đi, tự rơi về actionStack nếu hành trình dài vượt mảng
static Waypoint2D waypoints[DEST_MAX_WAYPOINTS];
static uint8_t waypointCount = 0u;
static bool waypointOverflow = false;

static Waypoint2D bypassMarks[DEST_MAX_BYPASS_MARKS];
static uint8_t bypassMarkCount = 0u;

float destinationNormalizeDeg(float deg)
{
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

static int16_t normalizeSpinDeg(int32_t deg)
{
  while (deg > 180) deg -= 360;
  while (deg <= -180) deg += 360;
  return (int16_t)deg;
}

static float minF(float a, float b) { return (a < b) ? a : b; }
static float maxF(float a, float b) { return (a > b) ? a : b; }
static float distSq2d(float dx, float dy) { return dx * dx + dy * dy; }

static float pointToSegDistSq(float px, float py, float x1, float y1, float x2, float y2)
{
  float vx = x2 - x1;
  float vy = y2 - y1;
  float l2 = vx * vx + vy * vy;
  if (l2 < 1e-5f)
    return distSq2d(px - x1, py - y1);
  float t = ((px - x1) * vx + (py - y1) * vy) / l2;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return distSq2d(px - (x1 + t * vx), py - (y1 + t * vy));
}

static int8_t orient2d(float ax, float ay, float bx, float by, float cx, float cy)
{
  float v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  if (fabsf(v) < 1e-5f)
    return 0;
  return (v > 0.0f) ? 1 : -1;
}

static bool onSeg2d(float ax, float ay, float bx, float by, float px, float py)
{
  return (px >= minF(ax, bx) - 1e-4f && px <= maxF(ax, bx) + 1e-4f &&
          py >= minF(ay, by) - 1e-4f && py <= maxF(ay, by) + 1e-4f);
}

static float segToSegDistSq(float ax, float ay, float bx, float by,
                          float cx, float cy, float dx, float dy)
{
  int8_t o1 = orient2d(ax, ay, bx, by, cx, cy);
  int8_t o2 = orient2d(ax, ay, bx, by, dx, dy);
  int8_t o3 = orient2d(cx, cy, dx, dy, ax, ay);
  int8_t o4 = orient2d(cx, cy, dx, dy, bx, by);
  if (o1 != o2 && o3 != o4)
    return 0.0f;
  if (o1 == 0 && onSeg2d(ax, ay, bx, by, cx, cy)) return 0.0f;
  if (o2 == 0 && onSeg2d(ax, ay, bx, by, dx, dy)) return 0.0f;
  if (o3 == 0 && onSeg2d(cx, cy, dx, dy, ax, ay)) return 0.0f;
  if (o4 == 0 && onSeg2d(cx, cy, dx, dy, bx, by)) return 0.0f;

  float d1 = pointToSegDistSq(ax, ay, cx, cy, dx, dy);
  float d2 = pointToSegDistSq(bx, by, cx, cy, dx, dy);
  float d3 = pointToSegDistSq(cx, cy, ax, ay, bx, by);
  float d4 = pointToSegDistSq(dx, dy, ax, ay, bx, by);
  return minF(minF(d1, d2), minF(d3, d4));
}

void destinationExecSpin(int16_t spinCmdDeg, NavPose *pose, bool recordHistory)
{
  if (spinCmdDeg == 0)
    return;

  float prevTheta = pose ? pose->thetaDeg : 0.0f;
  servoStop();
  motionSpinAnglePid(spinCmdDeg, PID_TARGET_SPIN_RPM);
  if (!pose)
    return;

  if (imuIsReady())
  {
    pose->thetaDeg = imuGetHeadingDeg();
  }
  else
  {
    uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
    float actualDeg = ((float)avgPulses * 360.0f) / (float)ENC_SPIN_360_PULSES;
    pose->thetaDeg = destinationNormalizeDeg(pose->thetaDeg + ((spinCmdDeg < 0) ? actualDeg : -actualDeg));
  }
  if (recordHistory)
  {
    float deltaMathDeg = destinationNormalizeDeg(pose->thetaDeg - prevTheta);
    destinationRecordSpin((int16_t)lroundf(-deltaMathDeg), prevTheta);
  }
}

float destinationExecDrive(int16_t driveCm, NavPose *pose, bool recordHistory)
{
  if (driveCm == 0)
    return 0.0f;

  float prevTheta = pose ? pose->thetaDeg : 0.0f;
  servoStop();
  motionDriveDistancePid(driveCm, PID_TARGET_DRIVE_RPM);
  if (!pose)
    return 0.0f;

  uint32_t avgPulses = (encoderGetPulses(MOTOR_LEFT) + encoderGetPulses(MOTOR_RIGHT)) / 2u;
  float actualDistCm = ((float)avgPulses * WHEEL_CIRCUMFERENCE_CM) / (float)ENC_PULSES_PER_REV;
  float sign = (driveCm > 0) ? 1.0f : -1.0f;
  float newTheta = imuIsReady() ? imuGetHeadingDeg() : prevTheta;
  float midTheta = destinationNormalizeDeg(prevTheta + 0.5f * destinationNormalizeDeg(newTheta - prevTheta));
  float rad = midTheta * (MATH_PI / 180.0f);
  pose->xCm += sign * actualDistCm * cosf(rad);
  pose->yCm += sign * actualDistCm * sinf(rad);
  pose->thetaDeg = newTheta;
  if (recordHistory)
  {
    destinationRecordDrive((int16_t)lroundf(sign * actualDistCm), prevTheta);
    destinationRecordPose(pose);
  }
  return actualDistCm;
}

void destinationReset(void)
{
  actionCount = 0u;
  stackOverflow = false;
  wallSegCount = 0u;
  wallOverflow = false;
  bypassMarkCount = 0u;
  waypointOverflow = false;
  waypoints[0].xCm = 0.0f;
  waypoints[0].yCm = 0.0f;
  waypointCount = 1u;
}

void destinationRecordSpin(int16_t actualSpinDeg, float prevHeadingDeg)
{
  int16_t normSpin = normalizeSpinDeg((int32_t)actualSpinDeg);
  if (normSpin == 0)
    return;

  // Gộp các bước xoay liên tiếp (giữ nguyên mốc prevHeadingDeg ban đầu của cụm xoay)
  if (actionCount > 0u && actionStack[actionCount - 1u].type == DEST_ACT_SPIN)
  {
    int16_t merged = normalizeSpinDeg((int32_t)actionStack[actionCount - 1u].value + (int32_t)normSpin);
    if (abs(merged) <= DEST_MIN_SPIN_DEG)
      actionCount--;
    else
      actionStack[actionCount - 1u].value = merged;
    return;
  }

  if (abs(normSpin) <= DEST_MIN_SPIN_DEG)
    return;

  if (actionCount < DEST_MAX_ACTIONS)
  {
    actionStack[actionCount].type = DEST_ACT_SPIN;
    actionStack[actionCount].value = normSpin;
    actionStack[actionCount].prevHeadingDeg = prevHeadingDeg;
    actionCount++;
  }
  else if (!stackOverflow)
  {
    stackOverflow = true;
    debugLog("DEST_OVERFLOW max=%u\r\n", DEST_MAX_ACTIONS);
  }
}

void destinationRecordDrive(int16_t actualDriveCm, float prevHeadingDeg)
{
  if (abs(actualDriveCm) < DEST_MIN_DRIVE_CM)
    return;

  // Gộp các bước chạy thẳng/lùi liên tiếp để triệt tiêu các nhịp vi chỉnh thừa
  if (actionCount > 0u && actionStack[actionCount - 1u].type == DEST_ACT_DRIVE)
  {
    int32_t merged = (int32_t)actionStack[actionCount - 1u].value + (int32_t)actualDriveCm;
    if (merged > 32767) merged = 32767;
    if (merged < -32768) merged = -32768;
    if (abs((int)merged) < DEST_MIN_DRIVE_CM)
      actionCount--;
    else
      actionStack[actionCount - 1u].value = (int16_t)merged;
    return;
  }

  if (actionCount < DEST_MAX_ACTIONS)
  {
    actionStack[actionCount].type = DEST_ACT_DRIVE;
    actionStack[actionCount].value = actualDriveCm;
    actionStack[actionCount].prevHeadingDeg = prevHeadingDeg;
    actionCount++;
  }
  else if (!stackOverflow)
  {
    stackOverflow = true;
    debugLog("DEST_OVERFLOW max=%u\r\n", DEST_MAX_ACTIONS);
  }
}

uint8_t destinationGetCount(void)
{
  return actionCount;
}

void destinationRecordPose(const NavPose *pose)
{
  if (!pose || waypointOverflow)
    return;

  float dx = pose->xCm - waypoints[waypointCount - 1u].xCm;
  float dy = pose->yCm - waypoints[waypointCount - 1u].yCm;
  if (distSq2d(dx, dy) < (DEST_WAYPOINT_MIN_DIST_CM * DEST_WAYPOINT_MIN_DIST_CM))
    return;

  if (waypointCount < DEST_MAX_WAYPOINTS)
  {
    waypoints[waypointCount].xCm = pose->xCm;
    waypoints[waypointCount].yCm = pose->yCm;
    waypointCount++;
  }
  else
  {
    waypointOverflow = true;
    debugLog("WP_OVERFLOW max=%u\r\n", DEST_MAX_WAYPOINTS);
  }
}

void destinationAddWallSeg(float x1Cm, float y1Cm, float x2Cm, float y2Cm,
                           uint8_t startDeg, uint8_t endDeg)
{
  if (distSq2d(x2Cm - x1Cm, y2Cm - y1Cm) < (DEST_WALL_MIN_LEN_CM * DEST_WALL_MIN_LEN_CM))
    return;

  const float mergeSq = DEST_WALL_MERGE_DIST_CM * DEST_WALL_MERGE_DIST_CM;
  float nvx = x2Cm - x1Cm;
  float nvy = y2Cm - y1Cm;
  float nLen = sqrtf(distSq2d(nvx, nvy));
  for (uint8_t i = 0u; i < wallSegCount; i++)
  {
    WallSeg2D *e = &wallSegs[i];
    if (pointToSegDistSq(x1Cm, y1Cm, e->x1Cm, e->y1Cm, e->x2Cm, e->y2Cm) < mergeSq &&
        pointToSegDistSq(x2Cm, y2Cm, e->x1Cm, e->y1Cm, e->x2Cm, e->y2Cm) < mergeSq)
    {
      if (distSq2d(x1Cm - e->x1Cm, y1Cm - e->y1Cm) < mergeSq)
        e->startDeg = startDeg;
      if (distSq2d(x2Cm - e->x2Cm, y2Cm - e->y2Cm) < mergeSq)
        e->endDeg = endDeg;
      return;
    }
    // Gộp kéo dài 2 đoạn tường cùng phương chồng lấn để không tràn 16 slot trên hành lang dài
    float evx = e->x2Cm - e->x1Cm;
    float evy = e->y2Cm - e->y1Cm;
    float eLen = sqrtf(distSq2d(evx, evy));
    if (eLen > 1e-3f && nLen > 1e-3f)
    {
      float ux = evx / eLen, uy = evy / eLen;
      float cosA = fabsf((nvx * ux + nvy * uy) / nLen);
      float p1Perp = fabsf((x1Cm - e->x1Cm) * uy - (y1Cm - e->y1Cm) * ux);
      float p2Perp = fabsf((x2Cm - e->x1Cm) * uy - (y2Cm - e->y1Cm) * ux);
      if (cosA >= 0.94f && p1Perp <= DEST_BYPASS_COLLINEAR_DIST_CM && p2Perp <= DEST_BYPASS_COLLINEAR_DIST_CM)
      {
        float t1 = (x1Cm - e->x1Cm) * ux + (y1Cm - e->y1Cm) * uy;
        float t2 = (x2Cm - e->x1Cm) * ux + (y2Cm - e->y1Cm) * uy;
        float tMin = minF(t1, t2), tMax = maxF(t1, t2);
        if (tMax >= -10.0f && tMin <= (eLen + 10.0f))
        {
          float uMin = minF(0.0f, tMin), uMax = maxF(eLen, tMax);
          float bx = e->x1Cm, by = e->y1Cm;
          e->x1Cm = bx + uMin * ux;
          e->y1Cm = by + uMin * uy;
          e->x2Cm = bx + uMax * ux;
          e->y2Cm = by + uMax * uy;
          if (uMin < 0.0f) e->startDeg = (t1 < t2) ? startDeg : endDeg;
          if (uMax > eLen) e->endDeg = (t1 > t2) ? startDeg : endDeg;
          return;
        }
      }
    }
  }

  if (wallSegCount < DEST_MAX_WALL_SEGS)
  {
    wallSegs[wallSegCount].x1Cm = x1Cm;
    wallSegs[wallSegCount].y1Cm = y1Cm;
    wallSegs[wallSegCount].x2Cm = x2Cm;
    wallSegs[wallSegCount].y2Cm = y2Cm;
    wallSegs[wallSegCount].startDeg = startDeg;
    wallSegs[wallSegCount].endDeg = endDeg;
    wallSegCount++;
  }
  else if (!wallOverflow)
  {
    wallOverflow = true;
    debugLog("WALL_MAP_FULL max=%u\r\n", DEST_MAX_WALL_SEGS);
  }
}

bool destinationIsPathClear(float axCm, float ayCm, float bxCm, float byCm, float clearanceCm)
{
  float clearSq = clearanceCm * clearanceCm;
  for (uint8_t i = 0u; i < wallSegCount; i++)
  {
    const WallSeg2D *w = &wallSegs[i];
    if (segToSegDistSq(axCm, ayCm, bxCm, byCm, w->x1Cm, w->y1Cm, w->x2Cm, w->y2Cm) < clearSq)
      return false;
  }
  return true;
}

bool destinationPlanWallBypass(const NavPose *pose, NavPose *outTarget, float *outLookHeadingDeg)
{
  if (!pose || !outTarget || !outLookHeadingDeg)
    return false;

  float thRad = pose->thetaDeg * (MATH_PI / 180.0f);
  float rayAx = pose->xCm + DEST_BODY_RADIUS_CM * cosf(thRad);
  float rayAy = pose->yCm + DEST_BODY_RADIUS_CM * sinf(thRad);
  float frontX = pose->xCm + DEST_BYPASS_FRONT_RAY_CM * cosf(thRad);
  float frontY = pose->yCm + DEST_BYPASS_FRONT_RAY_CM * sinf(thRad);

  bool found = false;
  float bestMoveSq = 1e12f;
  float bestBx = 0.0f;
  float bestBy = 0.0f;
  float bestLookDeg = 0.0f;

  const float minSegSq = DEST_BYPASS_MIN_SEG_CM * DEST_BYPASS_MIN_SEG_CM;
  const float maxTipSq = DEST_BYPASS_MAX_TIP_DIST_CM * DEST_BYPASS_MAX_TIP_DIST_CM;

  for (uint8_t i = 0u; i < wallSegCount; i++)
  {
    const WallSeg2D *w = &wallSegs[i];
    if (segToSegDistSq(rayAx, rayAy, frontX, frontY,
                       w->x1Cm, w->y1Cm, w->x2Cm, w->y2Cm) > (DEST_BYPASS_FRONT_TOL_CM * DEST_BYPASS_FRONT_TOL_CM))
      continue;

    float vx = w->x2Cm - w->x1Cm;
    float vy = w->y2Cm - w->y1Cm;
    float lenSq = distSq2d(vx, vy);
    if (lenSq < minSegSq)
      continue;

    float len = sqrtf(lenSq);
    float ux = vx / len;
    float uy = vy / len;
    float nx = -uy;
    float ny = ux;
    float mx = 0.5f * (w->x1Cm + w->x2Cm);
    float my = 0.5f * (w->y1Cm + w->y2Cm);
    if (((pose->xCm - mx) * nx + (pose->yCm - my) * ny) < 0.0f)
    {
      nx = -nx;
      ny = -ny;
    }

    for (uint8_t k = 0u; k < 2u; k++)
    {
      float ex = (k == 0u) ? w->x1Cm : w->x2Cm;
      float ey = (k == 0u) ? w->y1Cm : w->y2Cm;
      float outX = (k == 0u) ? -ux : ux;
      float outY = (k == 0u) ? -uy : uy;
      uint8_t ang = (k == 0u) ? w->startDeg : w->endDeg;

      if (ang < DEST_BYPASS_MIN_ANG_DEG || ang > DEST_BYPASS_MAX_ANG_DEG)
        continue;
      if (distSq2d(ex - pose->xCm, ey - pose->yCm) > maxTipSq)
        continue;

      // Bỏ qua đầu mút nếu chạm tường khác hoặc bị đoạn tường dài cùng phương nối tiếp phủ qua
      bool coveredByOther = false;
      for (uint8_t m = 0u; m < wallSegCount; m++)
      {
        if (m == i)
          continue;
        const WallSeg2D *o = &wallSegs[m];
        float dTipSq = pointToSegDistSq(ex, ey, o->x1Cm, o->y1Cm, o->x2Cm, o->y2Cm);
        if (dTipSq >= (DEST_BODY_RADIUS_CM * DEST_BODY_RADIUS_CM))
          continue;
        float mvx = o->x2Cm - o->x1Cm;
        float mvy = o->y2Cm - o->y1Cm;
        float dot = ux * mvx + uy * mvy;
        if ((dot * dot) < ((DEST_BYPASS_COLLINEAR_DOT * DEST_BYPASS_COLLINEAR_DOT) * distSq2d(mvx, mvy)))
        {
          coveredByOther = true;
          break;
        }
        if (dTipSq < (DEST_BYPASS_COLLINEAR_DIST_CM * DEST_BYPASS_COLLINEAR_DIST_CM))
        {
          float p1 = (o->x1Cm - ex) * outX + (o->y1Cm - ey) * outY;
          float p2 = (o->x2Cm - ex) * outX + (o->y2Cm - ey) * outY;
          if (p1 > DEST_BYPASS_COLLINEAR_EXT_CM || p2 > DEST_BYPASS_COLLINEAR_EXT_CM)
          {
            coveredByOther = true;
            break;
          }
        }
      }
      if (coveredByOther)
        continue;

      float bx = ex + DEST_BYPASS_SIDE_CM * outX + DEST_BYPASS_BACK_CM * nx;
      float by = ey + DEST_BYPASS_SIDE_CM * outY + DEST_BYPASS_BACK_CM * ny;
      float dMoveSq = distSq2d(bx - pose->xCm, by - pose->yCm);
      if (dMoveSq < (DEST_BYPASS_MIN_MOVE_CM * DEST_BYPASS_MIN_MOVE_CM) ||
          dMoveSq > (DEST_BYPASS_MAX_MOVE_CM * DEST_BYPASS_MAX_MOVE_CM))
        continue;

      bool visited = false;
      for (uint8_t b = 0u; b < bypassMarkCount; b++)
      {
        if (distSq2d(bypassMarks[b].xCm - bx, bypassMarks[b].yCm - by) <
            (DEST_SAFE_CLEARANCE_CM * DEST_SAFE_CLEARANCE_CM))
        {
          visited = true;
          break;
        }
      }
      if (visited)
        continue;

      if (!destinationIsPathClear(pose->xCm, pose->yCm, bx, by, DEST_SAFE_CLEARANCE_CM))
        continue;

      if (!found || dMoveSq < bestMoveSq)
      {
        found = true;
        bestMoveSq = dMoveSq;
        bestBx = bx;
        bestBy = by;
        bestLookDeg = atan2f(-ny, -nx) * (180.0f / MATH_PI);
      }
    }
  }

  if (!found)
    return false;

  if (bypassMarkCount < DEST_MAX_BYPASS_MARKS)
  {
    bypassMarks[bypassMarkCount].xCm = bestBx;
    bypassMarks[bypassMarkCount].yCm = bestBy;
    bypassMarkCount++;
  }
  outTarget->xCm = bestBx;
  outTarget->yCm = bestBy;
  outTarget->thetaDeg = atan2f(bestBy - pose->yCm, bestBx - pose->xCm) * (180.0f / MATH_PI);
  *outLookHeadingDeg = bestLookDeg;
  return true;
}

// Xoay xe về góc đích tuyệt đối nếu độ lệch vượt ngưỡng tolDeg
static void spinToHeading(float targetDeg, float tolDeg, NavPose *pose)
{
  float turnMath = destinationNormalizeDeg(targetDeg - pose->thetaDeg);
  if (fabsf(turnMath) <= tolDeg)
    return;
  int16_t spinCmd = (int16_t)lroundf(-turnMath);
  if (spinCmd != 0)
  {
    destinationExecSpin(spinCmd, pose, false);
    HAL_Delay(NAV_STEP_PAUSE_MS);
  }
}

// Chạy lùi một đoạn về tọa độ đích để bảo vệ vật đang nằm trong càng gắp phía trước
static void driveBackToPoint(float tgtX, float tgtY, uint8_t stepIdx, NavPose *pose)
{
  float dx = tgtX - pose->xCm;
  float dy = tgtY - pose->yCm;
  if (distSq2d(dx, dy) < (float)(DEST_MIN_DRIVE_CM * DEST_MIN_DRIVE_CM))
    return;

  float backHeading = destinationNormalizeDeg(atan2f(dy, dx) * (180.0f / MATH_PI) + 180.0f);
  spinToHeading(backHeading, DEST_RETURN_ALIGN_TOL_DEG, pose);

  dx = tgtX - pose->xCm;
  dy = tgtY - pose->yCm;
  float thRad = pose->thetaDeg * (MATH_PI / 180.0f);
  float backProj = -(dx * cosf(thRad) + dy * sinf(thRad));
  float cmdDist = (backProj > 1.0f) ? backProj : sqrtf(distSq2d(dx, dy));
  int16_t revCm = (int16_t)(-lroundf(cmdDist));
  if (abs(revCm) >= DEST_MIN_DRIVE_CM)
  {
    debugLog("DEST_STEP i=%u d=%d\r\n", stepIdx, revCm);
    destinationExecDrive(revCm, pose, false);
    HAL_Delay(NAV_STEP_PAUSE_MS);
  }
}

void destinationReturnHome(NavPose *pose)
{
  debugLog("DEST_RET n=%u ov=%u\r\n", actionCount, stackOverflow ? 1u : 0u);
  debugState("RETURN", "GO_HOME");

  if (pose && !stackOverflow && !wallOverflow && !waypointOverflow)
  {
    Waypoint2D rawChain[DEST_MAX_WAYPOINTS + 2u];
    Waypoint2D optNodes[DEST_MAX_WAYPOINTS + 2u];
    uint8_t rawLen = 0u;
    rawChain[rawLen].xCm = pose->xCm;
    rawChain[rawLen].yCm = pose->yCm;
    rawLen++;

    for (int16_t i = (int16_t)waypointCount - 1; i >= 0; i--)
    {
      float lx = rawChain[rawLen - 1u].xCm;
      float ly = rawChain[rawLen - 1u].yCm;
      if (distSq2d(waypoints[i].xCm - lx, waypoints[i].yCm - ly) >=
          (DEST_CHAIN_NODE_MIN_DIST_CM * DEST_CHAIN_NODE_MIN_DIST_CM))
        rawChain[rawLen++] = waypoints[i];
    }
    if (distSq2d(rawChain[rawLen - 1u].xCm, rawChain[rawLen - 1u].yCm) >
        (DEST_CHAIN_HOME_EPS_CM * DEST_CHAIN_HOME_EPS_CM))
    {
      rawChain[rawLen].xCm = 0.0f;
      rawChain[rawLen].yCm = 0.0f;
      rawLen++;
    }
    else
    {
      rawChain[rawLen - 1u].xCm = 0.0f;
      rawChain[rawLen - 1u].yCm = 0.0f;
    }

    uint8_t optLen = 0u;
    optNodes[optLen++] = rawChain[0];
    uint8_t idx = 0u;
    while ((uint8_t)(idx + 1u) < rawLen)
    {
      uint8_t farthest = (uint8_t)(idx + 1u);
      for (uint8_t j = (uint8_t)(rawLen - 1u); j > idx; j--)
      {
        if (destinationIsPathClear(rawChain[idx].xCm, rawChain[idx].yCm,
                                   rawChain[j].xCm, rawChain[j].yCm,
                                   DEST_SAFE_CLEARANCE_CM))
        {
          farthest = j;
          break;
        }
      }
      optNodes[optLen++] = rawChain[farthest];
      idx = farthest;
    }

    debugLog("DEST_SHORTCUT raw=%u opt=%u\r\n", rawLen, optLen);
    for (uint8_t i = 1u; i < optLen; i++)
    {
      driveBackToPoint(optNodes[i].xCm, optNodes[i].yCm, i, pose);
    }

    if (distSq2d(pose->xCm, pose->yCm) >= (DEST_HOME_TOL_CM * DEST_HOME_TOL_CM) &&
        destinationIsPathClear(pose->xCm, pose->yCm, 0.0f, 0.0f, DEST_BODY_RADIUS_CM))
    {
      driveBackToPoint(0.0f, 0.0f, optLen, pose);
    }
  }
  else
  {
    for (int16_t i = (int16_t)actionCount - 1; i >= 0; i--)
    {
      DestAction act = actionStack[i];

      if (act.type == DEST_ACT_DRIVE)
      {
        if (imuIsReady() && pose)
          spinToHeading(act.prevHeadingDeg, (float)DEST_HEADING_TRIM_DEG, pose);
        destinationExecDrive((int16_t)(-act.value), pose, false);
        HAL_Delay(NAV_STEP_PAUSE_MS);
      }
      else
      {
        int16_t revSpin = imuIsReady()
                              ? (int16_t)lroundf(-destinationNormalizeDeg(act.prevHeadingDeg - imuGetHeadingDeg()))
                              : (int16_t)(-act.value);
        if (abs(revSpin) > DEST_MIN_SPIN_DEG)
        {
          destinationExecSpin(revSpin, pose, false);
          HAL_Delay(NAV_STEP_PAUSE_MS);
        }
      }
    }
  }

  if (pose && !stackOverflow)
    spinToHeading(0.0f, (float)DEST_MIN_SPIN_DEG - 0.5f, pose);

  destinationReset();
  motorStopAll();
  debugLog("DEST_DONE (%s,%s)\r\n",
           debugF1(pose ? pose->xCm : 0.0f), debugF1(pose ? pose->yCm : 0.0f));
}
