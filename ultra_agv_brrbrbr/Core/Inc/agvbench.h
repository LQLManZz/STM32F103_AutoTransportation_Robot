#ifndef AGVBENCH_H
#define AGVBENCH_H

// Đọc 1 dòng lệnh bench tích hợp scanner + motor từ UART1 rồi thực thi (P, H, D, T, G, N), kết thúc in BENCH_DONE
void agvBenchRun(void);

#endif // AGVBENCH_H
