# Bảng đấu dây – Phiên bản 2 – 2 STM32 (MCU_DV định vị, MCU_LAI lái)

Bố trí Blue Pill: MCU_DV: cột 44–63, đầu USB ở cột 63 (quay về mũi xe), hàng A (B12…3V3) cắm hàng g, hàng B (VB…G) hàng c; MCU_LAI: cột 16–35, đầu USB ở cột 16 (quay về đuôi xe), hàng A (B12…3V3) cắm hàng c, hàng B (VB…G) hàng g.
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
| 9 | 5V | lỗ i46 (MCU_DV 5V) | lỗ T+37 (rail +5V) | M-M | đỏ | 54 |
| 10 | GND | lỗ i45 (MCU_DV G_A) | lỗ T-36 (rail GND) | M-M | đen | 52 |
| 11 | 5V | lỗ a33 (MCU_LAI 5V) | lỗ T+26 (rail +5V) | M-M | đỏ | 29 |
| 12 | GND | lỗ a34 (MCU_LAI G_A) | lỗ T-27 (rail GND) | M-M | đen | 26 |
| 13 | 5V | lỗ T+40 (rail +5V) | lỗ B+40 (rail +5V) | M-M | đỏ | 63 |
| 14 | GND | lỗ T-40 (rail GND) | lỗ B-40 (rail GND) | M-M | đen | 63 |
| 15 | 5V | SRF05 VCC | lỗ B+49 (rail +5V) | F-M | đỏ | 126 |
| 16 | TRIG | SRF05 TRIG | lỗ i60 (MCU_DV PB15) | F-M | xanh lá | 129 |
| 17 | ECHO | SRF05 ECHO | lỗ i59 (MCU_DV PA8) | F-M | xanh dương | 131 |
| 18 | GND | SRF05 GND | lỗ B-49 (rail GND) | F-M | đen | 125 |
| 19 | SERVO | SG90 SIG | lỗ a58 (MCU_DV PA1) | M-M | vàng | 70 |
| 20 | 5V | SG90 VCC | lỗ B+48 (rail +5V) | M-M | đỏ | 39 |
| 21 | GND | SG90 GND | lỗ B-48 (rail GND) | M-M | đen | 39 |
| 22 | ENA | L298N ENA | lỗ i26 (MCU_LAI PA6) | F-M | cam | 75 |
| 23 | ENB | L298N ENB | lỗ i27 (MCU_LAI PA7) | F-M | cam | 86 |
| 24 | IN1 | L298N IN1 | lỗ a16 (MCU_LAI PB12) | F-M | trắng | 88 |
| 25 | IN2 | L298N IN2 | lỗ a17 (MCU_LAI PB13) | F-M | trắng | 89 |
| 26 | IN3 | L298N IN3 | lỗ a18 (MCU_LAI PB14) | F-M | trắng | 90 |
| 27 | IN4 | L298N IN4 | lỗ i29 (MCU_LAI PB1) | F-M | trắng | 88 |
| 28 | UART_DV_TX | lỗ i58 (MCU_DV PA9) | lỗ a22 (MCU_LAI PA10) | M-M | tím | 115 |
| 29 | UART_LAI_TX | lỗ a21 (MCU_LAI PA9) | lỗ i57 (MCU_DV PA10) | M-M | tím | 115 |
