#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

// Khởi tạo bộ đệm ghi log blackbox trong RAM
void blackboxInit(void);
// Bắt đầu thu thập dữ liệu printf vào bộ đệm RAM
void blackboxStartCapture(void);
// Dừng thu thập log vào bộ đệm RAM
void blackboxStopCapture(void);
// Ghi 1 ký tự vào ring buffer RAM khi đang capture
void blackboxPutChar(char ch);
// Kiểm tra Flash có đang lưu log blackbox hợp lệ không
bool blackboxHasSavedLog(void);
// Lưu phần log trong RAM xuống 8 KB cuối Flash
bool blackboxFlush(void);
// Xóa 8 page Flash lưu log blackbox
bool blackboxErase(void);
// Xuất toàn bộ log đã lưu trong Flash ra UART1
void blackboxDumpToUart(void);
// Chờ lệnh UART khi khởi động (DD: đọc lại, CC: xóa log, HH: giữ xe đứng yên)
void blackboxServiceBootCmd(uint32_t waitMs);

#endif // BLACKBOX_H
