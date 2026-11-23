#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

// Khởi tạo bộ đệm ghi log blackbox trong RAM
void blackboxInit(void);
// Bắt đầu thu thập dữ liệu printf vào bộ đệm RAM
void blackboxStartCapture(void);
// Tiếp tục thu thập log vào RAM mà không xóa dữ liệu đang có
void blackboxResumeCapture(void);
// Dừng thu thập log vào bộ đệm RAM
void blackboxStopCapture(void);
// Kiểm tra trạng thái đang thu thập log vào RAM
bool blackboxIsCapturing(void);
// Ghi 1 ký tự vào ring buffer RAM khi đang capture
void blackboxPutChar(char ch);
// Ghi thẳng chuỗi vào RAM blackbox mà không xuất UART
void blackboxWriteStr(const char *str);
// Kiểm tra Flash có đang lưu log blackbox hợp lệ không (dở dang hoặc hoàn tất)
bool blackboxHasSavedLog(void);
// Kiểm tra Flash có đang lưu bản log đã hoàn tất trọn bộ chưa
bool blackboxHasCompleteLog(void);
// Lưu phần log trong RAM xuống 8 KB cuối Flash (trạng thái đang chạy)
bool blackboxFlush(void);
// Lưu log xuống Flash và đánh dấu đã hoàn tất trọn bộ (bảo vệ khỏi bị ghi đè khi Reset)
bool blackboxFlushComplete(void);
// Xóa 8 page Flash lưu log blackbox
bool blackboxErase(void);
// Xuất toàn bộ log đã lưu trong Flash ra UART1
void blackboxDumpToUart(void);
// Xuất log đang nằm trong RAM ra UART1
void blackboxDumpRamToUart(void);
// Chờ lệnh UART khi khởi động (DD/RR/FF/CC/HH/GG)
void blackboxServiceBootCmd(uint32_t waitMs);
// Kiểm tra lệnh UART không chặn trong lúc chờ/rảnh (trả true nếu nhận GG)
bool blackboxPollUartCmd(void);

#endif // BLACKBOX_H
