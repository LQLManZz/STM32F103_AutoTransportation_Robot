# Setup sensor siêu âm SRF05
## Tổng quan
![[Pasted image 20261016002405.png]]

Khởi động SysTick đếm số chu kỳ CPU:
```c
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
```

Kích xung $10\mathrm{\mu s}$ cho chân Trigger:
```c
  // Đưa Trigger xuống mức thấp trước để tạo cạnh lên sạch
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);
  delay_us(4);
  // Xung Trigger dương 10 us: SRF05 yêu cầu tối thiểu 10 us
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, GPIO_PIN_RESET);
```

Sử dụng hàm PulseIn() để lấy độ dài xung:
```c
uint32_t pulseIn(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state, uint32_t timeout_us)
{
  uint32_t timeout_ticks = timeout_us * (SystemCoreClock / 1000000UL);
  uint32_t min_ticks = 50UL * (SystemCoreClock / 1000000UL); // Bỏ qua xung < 50 us (nhiễu)
  uint32_t start_wait = DWT->CYCCNT;
  while ((DWT->CYCCNT - start_wait) < timeout_ticks)
  {
    // Chờ chân chuyển sang trạng thái khác "state" (bắt đầu từ trạng thái hiện tại)
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == state)
    {
      if ((DWT->CYCCNT - start_wait) > timeout_ticks)
        return 0;
    }
    // Chờ chân lên đúng "state": bắt đầu xung cần đo
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != state)
    {
      if ((DWT->CYCCNT - start_wait) > timeout_ticks)
        return 0;
    }
    // Đo thời gian chân giữ "state"
    uint32_t pulse_start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == state)
    {
      if ((DWT->CYCCNT - pulse_start) > timeout_ticks)
        return 0;
    }
    uint32_t pulse_ticks = DWT->CYCCNT - pulse_start;
    if (pulse_ticks >= min_ticks)
    {
      return pulse_ticks / (SystemCoreClock / 1000000UL); // Đổi chu kỳ CPU sang us
    }
  }
  return 0;
}
```

Dạng sóng để dễ hình dung:
![[pulseIn_waveform.drawio.png]]

Tính khoảng cách:
- Tốc độ âm thanh trong không khí: $343\mathrm{m/s} \to 0.0343\mathrm{cm/\mu s}$
- Công thức:$$\text{distance}=\frac{\text{pulseIn()}\cdot 0.0343}{2}$$
## STM32CubeMX
Constraints:
- `PA08`: Echo
- `PB15`: Trigger
- `VCC`: 5V
# Setup servo SG90
## Tổng quan
![[Pasted image 20261016002647.png]]
**Chỉnh sửa đối với servo đang dùng:** 
- Góc $0\degree$ tương ứng với xung $0.5\mathrm{ms}$
- Góc $180\degree$ tương ứng với xung $2.5\mathrm{ms}$
Mặc định sẽ góc quay tối đa $0\degree \to 180\degree$
Công thức để ánh xạ từ độ sang độ rộng xung:$$\text{pulse servo}(\mu  s)=500+\frac{\mathrm{angle\degree}\cdot (2500-500)}{180}$$
Ánh xạ ngược từ độ rộng xung sang độ:
$$\mathrm{angle\degree}=\frac{(\text{pulse servo}(\mu s)-500)*180}{2500-500}+0.5$$
Cộng thêm một lượng $0.5$ để làm tròn độ, tránh việc ánh xạ xuống sẽ làm lệch đi 1 độ so với góc thực tế.

Đặt giá trị cho CCR để quay servo tới góc mong muốn:
```c
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pulse_servo);
```

Đọc giá trị từ CCR để lấy góc hiện tại của servo:
```c
	__HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
```

## STM32CubeMX
Constraints:
- `PA01`: Timer Channel 2, PWM Generation
- `VCC`: 5V
Timer 2:
- `PWM Mode 1`
- `Prescale`: 71
- `ARR`: 19999
