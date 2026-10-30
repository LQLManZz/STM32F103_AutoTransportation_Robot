# Bảng đấu dây – Phiên bản 1 – 1 STM32

Bố trí Blue Pill: MCU: cột 44–63, đầu USB ở cột 63 (quay về mũi xe), hàng A (B12…3V3) cắm hàng g, hàng B (VB…G) hàng c.
Rail T+ = 5V (từ cổng 5V của L298N, giữ jumper 5V-EN), rail T− = GND. Dài = chiều dài thân dây tối thiểu.

| # | Net | Từ | Tới | Loại | Màu | Dài (mm) |
|---|---|---|---|---|---|---|
| 1 | VBAT+ | PIN + | L298N 12V | dây hộp pin | đỏ | 137 |
| 2 | VBAT- | PIN - | L298N GND | dây hộp pin | đen | 129 |
| 3 | M_T+ | MOTOR_T + | L298N OUT1 | dây motor | đỏ | 163 |
| 4 | M_T- | MOTOR_T - | L298N OUT2 | dây motor | đen | 154 |
| 5 | M_P+ | MOTOR_P + | L298N OUT3 | dây motor | đỏ | 123 |
| 6 | M_P- | MOTOR_P - | L298N OUT4 | dây motor | đen | 129 |
| 7 | 5V | L298N 5V | lỗ T+10 (rail +5V) | M-M | đỏ | 94 |
| 8 | GND | L298N GND | lỗ T-11 (rail GND) | M-M | đen | 91 |
| 9 | 5V | lỗ i46 (Blue Pill 5V) | lỗ T+37 (rail +5V) | M-M | đỏ | 54 |
| 10 | GND | lỗ i45 (Blue Pill G_A) | lỗ T-36 (rail GND) | M-M | đen | 52 |
| 11 | 5V | lỗ T+40 (rail +5V) | lỗ B+40 (rail +5V) | M-M | đỏ | 63 |
| 12 | GND | lỗ T-40 (rail GND) | lỗ B-40 (rail GND) | M-M | đen | 63 |
| 13 | 5V | SRF05 VCC | lỗ B+49 (rail +5V) | F-M | đỏ | 126 |
| 14 | TRIG | SRF05 TRIG | lỗ i60 (Blue Pill PB15) | F-M | xanh lá | 129 |
| 15 | ECHO | SRF05 ECHO | lỗ i59 (Blue Pill PA8) | F-M | xanh dương | 131 |
| 16 | GND | SRF05 GND | lỗ B-49 (rail GND) | F-M | đen | 125 |
| 17 | SERVO | SG90 SIG | lỗ a58 (Blue Pill PA1) | M-M | vàng | 70 |
| 18 | 5V | SG90 VCC | lỗ B+48 (rail +5V) | M-M | đỏ | 39 |
| 19 | GND | SG90 GND | lỗ B-48 (rail GND) | M-M | đen | 39 |
| 20 | ENA | L298N ENA | lỗ a53 (Blue Pill PA6) | F-M | cam | 144 |
| 21 | ENB | L298N ENB | lỗ a52 (Blue Pill PA7) | F-M | cam | 153 |
| 22 | IN1 | L298N IN1 | lỗ i63 (Blue Pill PB12) | F-M | trắng | 159 |
| 23 | IN2 | L298N IN2 | lỗ i62 (Blue Pill PB13) | F-M | trắng | 159 |
| 24 | IN3 | L298N IN3 | lỗ i61 (Blue Pill PB14) | F-M | trắng | 159 |
| 25 | IN4 | L298N IN4 | lỗ a50 (Blue Pill PB1) | F-M | trắng | 146 |
