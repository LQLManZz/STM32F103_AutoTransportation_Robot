#ifndef AGV_H
#define AGV_H

// Khởi tạo toàn bộ module của xe
void agvInit(void);

// Một vòng hoạt động: quét tọa độ cực tìm vật -> xoay + vi chỉnh hướng -> tiến bằng PID -> căn chỉnh vào tay gắp phải
void agvRun(void);

#endif // AGV_H
