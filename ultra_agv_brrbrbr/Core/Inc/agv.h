#ifndef AGV_H
#define AGV_H

// Khởi tạo toàn bộ module của xe
void agvInit(void);

// Một vòng hoạt động: quét tìm vật/tường -> xoay + vi chỉnh hướng -> chạy tới gần vật (hoặc né tường)
void agvRun(void);

#endif // AGV_H
