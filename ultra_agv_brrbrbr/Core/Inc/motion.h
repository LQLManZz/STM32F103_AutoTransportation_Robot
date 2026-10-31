#ifndef MOTION_H
#define MOTION_H

#include "motor.h"

void motionInit(void);

// Dieu khien chuyen dong truc tiep (phuc vu PID va vong lap thoi gian thuc)
void motionForward(uint8_t speedPercent);
void motionBackward(uint8_t speedPercent);
void motionBrake(void);
void motionStop(void);

// Xoay truc tiep tai cho (danh cho vong lap can huong chu dong)
void motionSpinLeft(uint8_t speedPercent);
void motionSpinRight(uint8_t speedPercent);

// Xoay tai cho theo goc dinh luong: > 0 sang phai, < 0 sang trai
void motionSpinAngle(int16_t angleDeg, uint8_t speedPercent);

// Chay thang theo cu ly co tang/giam toc: > 0 tien, < 0 lui
void motionDriveDistance(int16_t distanceCm, uint8_t speedPercent);

#endif // MOTION_H
