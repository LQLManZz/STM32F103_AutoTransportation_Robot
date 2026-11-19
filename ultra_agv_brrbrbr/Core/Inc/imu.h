#ifndef IMU_H
#define IMU_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

// Khởi tạo MPU6050 trên I2C2 (trả về true nếu tìm thấy cảm biến)
bool imuInit(void);

// Kiểm tra MPU6050 đã sẵn sàng sử dụng hay chưa
bool imuIsReady(void);

// Hiệu chuẩn sai số điểm 0 (bias) trục Z khi xe đứng yên hoàn toàn (~1 giây)
void imuCalibrate(void);

// Đặt lại góc Yaw tích phân về 0° và reset mốc thời gian đo
void imuResetYaw(void);

// Đọc tốc độ góc trục Z và tích phân cập nhật góc Yaw (độ: + quay trái CCW, - quay phải CW)
float imuUpdate(void);

// Lấy góc Yaw tương đối tính từ lần imuResetYaw() gần nhất (độ)
float imuGetYawDeg(void);

// Lấy góc hướng hiện tại của xe tính từ lúc calib ban đầu, chuẩn hóa (-180°, 180°]
float imuGetHeadingDeg(void);

// Lấy tốc độ góc trục Z hiện tại (°/s)
float imuGetRateDps(void);

#endif // IMU_H
