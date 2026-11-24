#include "lcd.h"
#include "project_config.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define LCD_BIT_RS 0x01u        // P0 PCF8574: 1 = ghi dữ liệu, 0 = ghi lệnh
#define LCD_BIT_EN 0x04u        // P2 PCF8574: xung Enable chốt nibble
#define LCD_BIT_BL 0x08u        // P3 PCF8574: bật đèn nền
#define LCD_CMD_CLEAR 0x01u     // Xóa màn hình, con trỏ về đầu
#define LCD_CMD_ENTRY 0x06u     // Con trỏ tự tăng, không cuộn màn hình
#define LCD_CMD_DISPLAY 0x0Cu   // Bật hiển thị, tắt con trỏ
#define LCD_CMD_FUNCTION 0x28u  // Bus 4 bit, 2 dòng, font 5x8
#define LCD_CMD_DDRAM 0x80u     // Đặt địa chỉ ô ghi tiếp theo
#define LCD_ROW1_ADDR 0x40u     // Địa chỉ ô đầu dòng 1
#define LCD_BYTES_PER_NIBBLE 3u // Mỗi nibble: đặt dữ liệu, EN cao, EN thấp
#define LCD_BYTES_PER_CHAR 6u   // Mỗi byte HD44780 = 2 nibble

extern I2C_HandleTypeDef hi2c2;

static char lineCache[LCD_ROWS][LCD_COLS + 1];
static bool lcdFault = false;
static uint16_t lcdAddr = 0u;

static bool lcdWrite(uint8_t *data, uint16_t len)
{
  if (lcdFault)
    return false;

  // PCF8574 không có thanh ghi: dùng byte đầu làm "địa chỉ ô nhớ" để dùng chung hàm Mem_Write với IMU (tiết kiệm Flash)
  if (HAL_I2C_Mem_Write(&hi2c2, lcdAddr, data[0], I2C_MEMADD_SIZE_8BIT,
                        &data[1], len - 1u, LCD_I2C_TIMEOUT_MS) == HAL_OK)
    return true;

  // Thử gửi lại 1 lần trước khi tạm ngắt LCD (không giải kẹt bus: PCF8574 NACK giao dịch đầu sau khi reset I2C2)
  if (HAL_I2C_Mem_Write(&hi2c2, lcdAddr, data[0], I2C_MEMADD_SIZE_8BIT,
                        &data[1], len - 1u, LCD_I2C_TIMEOUT_MS) == HAL_OK)
    return true;

  lcdFault = true;
  return false;
}

// Thêm 1 nibble (4 bit cao của bits): đặt RS/dữ liệu trước rồi mới tạo xung EN để HD44780 chốt đúng
static void lcdPutNibble(uint8_t *buf, uint16_t *len, uint8_t bits)
{
  buf[(*len)++] = bits;
  buf[(*len)++] = bits | LCD_BIT_EN;
  buf[(*len)++] = bits;
}

// Thêm 1 byte (lệnh hoặc ký tự) vào bộ đệm dạng 2 nibble
static void lcdPutByte(uint8_t *buf, uint16_t *len, uint8_t value, uint8_t rs)
{
  lcdPutNibble(buf, len, (uint8_t)(value & 0xF0u) | rs | LCD_BIT_BL);
  lcdPutNibble(buf, len, (uint8_t)(value << 4) | rs | LCD_BIT_BL);
}

static void lcdSendCmd(uint8_t cmd)
{
  uint8_t buf[LCD_BYTES_PER_CHAR];
  uint16_t len = 0u;

  lcdPutByte(buf, &len, cmd, 0u);
  lcdWrite(buf, len);
}

// Dò địa chỉ module trả ACK: PCF8574 0x27..0x20 rồi PCF8574A 0x3F..0x38 (tùy cầu hàn A0..A2)
// Gửi 0x00 tắt đèn nền, lúc khởi tạo bật lại: đèn nền chớp tắt là module đã nhận lệnh
static uint8_t lcdFindAddr(void)
{
  uint8_t bl = 0u;

  for (uint8_t i = 0u; i < 2u * LCD_I2C_ADDR_COUNT; i++)
  {
    uint8_t addr = (i < LCD_I2C_ADDR_COUNT) ? (uint8_t)(LCD_I2C_ADDR_TOP - i)
                                            : (uint8_t)(LCD_I2C_ADDR_A_TOP - (i - LCD_I2C_ADDR_COUNT));
    if (HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(addr << 1), bl, I2C_MEMADD_SIZE_8BIT,
                          &bl, 1u, LCD_I2C_TIMEOUT_MS) == HAL_OK)
      return addr;
  }
  return 0u;
}

bool lcdInit(uint8_t *addr7)
{
  // Chuỗi nibble đưa HD44780 về chế độ 4 bit: 3 lần 0x3 rồi 0x2
  static const uint8_t initNibbles[] = {0x30u, 0x30u, 0x30u, 0x20u};

  lcdFault = false;
  memset(lineCache, 0, sizeof(lineCache));

  // Không module nào trả lời thì tắt LCD, tránh treo vòng lặp vì timeout I2C
  *addr7 = lcdFindAddr();
  lcdAddr = (uint16_t)(*addr7 << 1);
  if (*addr7 == 0u)
  {
    lcdFault = true;
    return false;
  }

  HAL_Delay(LCD_POWER_UP_MS);
  for (uint8_t i = 0u; i < sizeof(initNibbles); i++)
  {
    uint8_t buf[LCD_BYTES_PER_NIBBLE];
    uint16_t len = 0u;

    lcdPutNibble(buf, &len, initNibbles[i] | LCD_BIT_BL);
    if (!lcdWrite(buf, len))
      return false;
    HAL_Delay(LCD_INIT_STEP_MS);
  }

  lcdSendCmd(LCD_CMD_FUNCTION);
  lcdSendCmd(LCD_CMD_DISPLAY);
  lcdSendCmd(LCD_CMD_ENTRY);
  lcdSendCmd(LCD_CMD_CLEAR);
  HAL_Delay(LCD_CLEAR_MS);
  return !lcdFault;
}

void lcdPrintLine(uint8_t row, const char *format, ...)
{
  if (row >= LCD_ROWS)
    return;

  char text[LCD_COLS + 1];
  va_list args;

  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);

  // Đệm khoảng trắng để xóa ký tự cũ còn sót cuối dòng
  size_t n = strlen(text);
  memset(&text[n], ' ', LCD_COLS - n);
  text[LCD_COLS] = '\0';

  if (strcmp(text, lineCache[row]) == 0)
    return;

  // Gửi lệnh đặt con trỏ + 16 ký tự trong 1 lần truyền I2C
  uint8_t buf[(LCD_COLS + 1u) * LCD_BYTES_PER_CHAR];
  uint16_t len = 0u;

  lcdPutByte(buf, &len, LCD_CMD_DDRAM | (row ? LCD_ROW1_ADDR : 0u), 0u);
  for (uint8_t i = 0u; i < LCD_COLS; i++)
    lcdPutByte(buf, &len, (uint8_t)text[i], LCD_BIT_RS);

  if (lcdWrite(buf, len))
    strcpy(lineCache[row], text);
}
