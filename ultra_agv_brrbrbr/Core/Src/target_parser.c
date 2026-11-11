/**
  ******************************************************************************
  * @file    target_parser.c
  * @brief   Hiện thực nhận ngắt UART và giải mã chuỗi tọa độ (x, y) từ cảm biến âm
  ******************************************************************************
  */

#include "target_parser.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern UART_HandleTypeDef huart1;

static uint8_t rx_single_byte = 0;
static char rx_line_buffer[UART_RX_BUFFER_SIZE];
static uint8_t rx_index = 0;

static volatile bool has_new_data = false;
static float current_x = 0.0f;
static float current_y = 0.0f;

void TargetParser_Init(void)
{
  rx_index = 0;
  has_new_data = false;
  current_x = 0.0f;
  current_y = 0.0f;

  // Bật ngắt NVIC cho USART1
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  // Kích hoạt nhận byte đầu tiên qua ngắt
  HAL_UART_Receive_IT(&huart1, &rx_single_byte, 1);
}

bool TargetParser_HasNewTarget(void)
{
  return has_new_data;
}

bool TargetParser_GetTarget(float *out_x, float *out_y)
{
  if (out_x == NULL || out_y == NULL)
  {
    return false;
  }

  *out_x = current_x;
  *out_y = current_y;
  has_new_data = false; // Đã đọc xong tọa độ mới
  return true;
}

static void TargetParser_ProcessLine(char *line)
{
  float parsed_x = 0.0f;
  float parsed_y = 0.0f;
  int items = 0;

  // 1. Kiểm tra định dạng Góc và Khoảng cách (Polar): "ANG:30 DIST:40" hoặc "A:30 D:40"
  char *pa = strstr(line, "ANG:");
  if (!pa) pa = strstr(line, "ang:");
  if (!pa) pa = strstr(line, "A:");
  if (!pa) pa = strstr(line, "a:");

  char *pd = strstr(line, "DIST:");
  if (!pd) pd = strstr(line, "dist:");
  if (!pd) pd = strstr(line, "D:");
  if (!pd) pd = strstr(line, "d:");

  if (pa && pd)
  {
    float ang = 0.0f, dist = 0.0f;
    while (*pa && (*pa < '0' || *pa > '9') && *pa != '-' && *pa != '+') pa++;
    while (*pd && (*pd < '0' || *pd > '9') && *pd != '-' && *pd != '+') pd++;
    if (sscanf(pa, "%f", &ang) == 1 && sscanf(pd, "%f", &dist) == 1)
    {
      float rad = ang * (3.14159265f / 180.0f);
      current_x = dist * sinf(rad);
      current_y = dist * cosf(rad);
      has_new_data = true;
      printf("PARSE_OK (Polar): Ang=%.1f deg, Dist=%.1f cm -> X=%.1f, Y=%.1f\r\n",
             ang, dist, current_x, current_y);
      return;
    }
  }

  // 2. Định dạng nhãn: "X:15.5 Y:30.2" hoặc "x=15.5 y=30.2"
  char *px = strstr(line, "X:");
  if (!px) px = strstr(line, "x:");
  if (!px) px = strstr(line, "x=");
  if (!px) px = strstr(line, "X=");

  char *py = strstr(line, "Y:");
  if (!py) py = strstr(line, "y:");
  if (!py) py = strstr(line, "y=");
  if (!py) py = strstr(line, "Y=");

  if (px && py)
  {
    while (*px && (*px < '0' || *px > '9') && *px != '-' && *px != '+') px++;
    while (*py && (*py < '0' || *py > '9') && *py != '-' && *py != '+') py++;
    if (sscanf(px, "%f", &parsed_x) == 1 && sscanf(py, "%f", &parsed_y) == 1)
    {
      current_x = parsed_x;
      current_y = parsed_y;
      has_new_data = true;
      printf("PARSE_OK (Cartesian): X=%.1f cm, Y=%.1f cm\r\n", current_x, current_y);
      return;
    }
  }

  // 3. Định dạng mở ngoặc: (x, y)
  if (line[0] == '(')
  {
    items = sscanf(line, "(%f,%f)", &parsed_x, &parsed_y);
    if (items == 2)
    {
      current_x = parsed_x;
      current_y = parsed_y;
      has_new_data = true;
      printf("PARSE_OK (Bracket): X=%.1f cm, Y=%.1f cm\r\n", current_x, current_y);
      return;
    }
  }

  // 4. Định dạng phân tách bằng dấu phẩy: x,y
  items = sscanf(line, "%f,%f", &parsed_x, &parsed_y);
  if (items == 2)
  {
    current_x = parsed_x;
    current_y = parsed_y;
    has_new_data = true;
    printf("PARSE_OK (Comma): X=%.1f cm, Y=%.1f cm\r\n", current_x, current_y);
    return;
  }

  // 5. Định dạng phân tách bằng khoảng trắng: x y
  items = sscanf(line, "%f %f", &parsed_x, &parsed_y);
  if (items == 2)
  {
    current_x = parsed_x;
    current_y = parsed_y;
    has_new_data = true;
    printf("PARSE_OK (Space): X=%.1f cm, Y=%.1f cm\r\n", current_x, current_y);
    return;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rx_single_byte == '\n' || rx_single_byte == '\r')
    {
      if (rx_index > 0)
      {
        rx_line_buffer[rx_index] = '\0';
        TargetParser_ProcessLine(rx_line_buffer);
        rx_index = 0;
      }
    }
    else
    {
      if (rx_index < (UART_RX_BUFFER_SIZE - 1))
      {
        rx_line_buffer[rx_index++] = (char)rx_single_byte;
      }
      else
      {
        rx_index = 0; // Tràn bộ đệm, reset
      }
    }

    // Tiếp tục nhận byte kế tiếp qua ngắt
    HAL_UART_Receive_IT(&huart1, &rx_single_byte, 1);
  }
}
