#ifndef AGV_H
#define AGV_H

// Khởi tạo toàn bộ module của xe
void agvInit(void);

// Một vòng hoạt động: quét tìm vật (tọa độ cực) -> xoay hở theo góc -> tiến tới bằng PID cự ly; tới vật rồi thì đứng yên
void agvRun(void);

#endif // AGV_H
