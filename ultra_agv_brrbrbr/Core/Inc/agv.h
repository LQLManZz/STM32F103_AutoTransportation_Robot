#ifndef AGV_H
#define AGV_H

#include <stdint.h>

// Khoi tao toan bo module cua xe
void agvInit(void);

// Mot vong hoat dong: quet tim vat -> can huong chu dong -> tiep can bang PID
void agvRun(void);

// Can huong xe chu dong co phan hoi sieu am khoa muc tieu (a -> 0)
uint8_t agvAlignHeadingActive(float headingDeg, float expectedDistCm);

// Tiep can muc tieu bang thuat toan PID cu ly sieu am trong dai dung sai
void agvDrivePID(float targetDistanceCm);

#endif // AGV_H
