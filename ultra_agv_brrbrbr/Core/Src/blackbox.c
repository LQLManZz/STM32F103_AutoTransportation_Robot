#include "blackbox.h"
#include "project_config.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
  uint32_t magic;
  uint32_t length;
} BlackboxHeader;

extern UART_HandleTypeDef huart1;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;

static uint8_t ramBuf[BLACKBOX_MAX_DATA_LEN];
static uint16_t writeIdx = 0u;
static bool wrapped = false;
static bool capturing = false;
static bool dirty = false;
static uint8_t flushCount = 0u;

// Đảm bảo vùng Flash lưu log không chồng lấn lên mã firmware hiện tại
static bool isFlashRegionSafe(void)
{
  uintptr_t fwEnd = (uintptr_t)&_sidata + ((uintptr_t)&_edata - (uintptr_t)&_sdata);
  return fwEnd <= (uintptr_t)BLACKBOX_FLASH_ADDR;
}

static void uartSendRaw(const uint8_t *buf, uint16_t len)
{
  if (!buf || len == 0u)
    return;
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000u);
}

static void uartSendStr(const char *str)
{
  if (!str)
    return;
  uartSendRaw((const uint8_t *)str, (uint16_t)strlen(str));
}

static uint8_t ringByteAt(uint16_t startIdx, uint16_t offset)
{
  uint32_t idx = ((uint32_t)startIdx + (uint32_t)offset) % BLACKBOX_MAX_DATA_LEN;
  return ramBuf[idx];
}

static bool eraseFlashPages(void)
{
  FLASH_EraseInitTypeDef eraseInit = {0};
  uint32_t pageError = 0u;

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

  eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInit.Banks = FLASH_BANK_1;
  eraseInit.PageAddress = BLACKBOX_FLASH_ADDR;
  eraseInit.NbPages = BLACKBOX_FLASH_PAGES;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
  if (status != HAL_OK)
  {
    HAL_FLASH_Lock();
    return false;
  }
  return true;
}

// Đọc 1 byte UART sạch, tự động loại bỏ nhiễu khung/nhiễu điện khi chân PA10 thả nổi
static bool uartTryReadCleanChar(char *outCh)
{
  uint32_t sr = huart1.Instance->SR;
  if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0u)
  {
    (void)huart1.Instance->DR;
    return false;
  }
  if ((sr & USART_SR_RXNE) != 0u)
  {
    *outCh = (char)(huart1.Instance->DR & 0xFFu);
    return true;
  }
  return false;
}

void blackboxInit(void)
{
  writeIdx = 0u;
  wrapped = false;
  capturing = false;
  dirty = false;
  flushCount = 0u;
}

void blackboxStartCapture(void)
{
  writeIdx = 0u;
  wrapped = false;
  dirty = false;
  capturing = true;
}

void blackboxStopCapture(void)
{
  capturing = false;
}

void blackboxPutChar(char ch)
{
  if (!capturing)
    return;

  ramBuf[writeIdx++] = (uint8_t)ch;
  if (writeIdx >= BLACKBOX_MAX_DATA_LEN)
  {
    writeIdx = 0u;
    wrapped = true;
  }
  dirty = true;
}

bool blackboxHasSavedLog(void)
{
  const volatile BlackboxHeader *hdr = (const volatile BlackboxHeader *)BLACKBOX_FLASH_ADDR;
  return (hdr->magic == BLACKBOX_MAGIC) &&
         (hdr->length > 0u) &&
         (hdr->length <= BLACKBOX_MAX_DATA_LEN);
}

bool blackboxErase(void)
{
  if (!isFlashRegionSafe())
  {
    uartSendStr("BLACKBOX_ERR fw_overlap\r\n");
    return false;
  }

  bool prevCap = capturing;
  capturing = false;

  bool ok = eraseFlashPages();
  HAL_FLASH_Lock();
  if (ok)
    dirty = false;

  capturing = prevCap;
  return ok;
}

bool blackboxFlush(void)
{
  if (!dirty)
    return true;
  if (flushCount >= BLACKBOX_MAX_FLUSHES)
    return false;

  uint16_t totalLen = wrapped ? (uint16_t)BLACKBOX_MAX_DATA_LEN : writeIdx;
  if (totalLen == 0u)
    return true;

  if (!isFlashRegionSafe())
  {
    uartSendStr("BLACKBOX_ERR fw_overlap\r\n");
    return false;
  }

  bool prevCap = capturing;
  capturing = false;

  // Nếu ring buffer đã quay vòng, bỏ qua phần đầu bị cắt ngang cho tới hết ký tự '\n' đầu tiên
  uint16_t startIdx = wrapped ? writeIdx : 0u;
  if (wrapped)
  {
    uint16_t scanLimit = (totalLen < 160u) ? totalLen : 160u;
    for (uint16_t skip = 0u; skip < scanLimit; skip++)
    {
      if (ringByteAt(startIdx, skip) == (uint8_t)'\n')
      {
        uint16_t drop = (uint16_t)(skip + 1u);
        if (drop < totalLen)
        {
          startIdx = (uint16_t)(((uint32_t)startIdx + (uint32_t)drop) % BLACKBOX_MAX_DATA_LEN);
          totalLen -= drop;
        }
        break;
      }
    }
  }

  bool ok = eraseFlashPages();
  if (ok)
  {
    uint32_t dstAddr = BLACKBOX_FLASH_ADDR + BLACKBOX_HEADER_SIZE;
    for (uint16_t i = 0u; i < totalLen; i += 2u)
    {
      uint8_t b0 = ringByteAt(startIdx, i);
      uint8_t b1 = ((uint16_t)(i + 1u) < totalLen) ? ringByteAt(startIdx, (uint16_t)(i + 1u)) : 0xFFu;
      uint16_t halfWord = (uint16_t)b0 | ((uint16_t)b1 << 8u);
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, dstAddr, halfWord) != HAL_OK)
      {
        ok = false;
        break;
      }
      dstAddr += 2u;
    }
  }

  // Ghi length trước rồi mới ghi magic để mất điện giữa chừng không tạo header giả hợp lệ
  if (ok)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BLACKBOX_FLASH_ADDR + 4u, (uint32_t)totalLen) != HAL_OK)
      ok = false;
  }
  if (ok)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BLACKBOX_FLASH_ADDR, BLACKBOX_MAGIC) != HAL_OK)
      ok = false;
  }

  HAL_FLASH_Lock();
  if (ok)
  {
    dirty = false;
    flushCount++;
  }

  capturing = prevCap;
  return ok;
}

void blackboxDumpToUart(void)
{
  bool prevCap = capturing;
  capturing = false;

  if (!blackboxHasSavedLog())
  {
    uartSendStr("\r\n=== BLACKBOX EMPTY (chua co log) ===\r\n");
  }
  else
  {
    const volatile BlackboxHeader *hdr = (const volatile BlackboxHeader *)BLACKBOX_FLASH_ADDR;
    uint32_t len = hdr->length;
    char hdrLine[80];
    snprintf(hdrLine, sizeof(hdrLine),
             "\r\n=== BLACKBOX LOG START (len=%lu bytes) ===\r\n",
             (unsigned long)len);
    uartSendStr(hdrLine);

    const volatile uint8_t *src = (const volatile uint8_t *)(BLACKBOX_FLASH_ADDR + BLACKBOX_HEADER_SIZE);
    uint8_t chunk[64];
    uint32_t offset = 0u;
    while (offset < len)
    {
      uint16_t n = (uint16_t)((len - offset > sizeof(chunk)) ? sizeof(chunk) : (len - offset));
      for (uint16_t i = 0u; i < n; i++)
        chunk[i] = src[offset + i];
      uartSendRaw(chunk, n);
      offset += n;
    }

    uartSendStr("\r\n=== BLACKBOX LOG END ===\r\n");
  }

  uartSendStr("=== CMD: gui 'DD'=doc lai, 'CC'=xoa log, 'HH'=giu xe dung yen ===\r\n");
  capturing = prevCap;
}

void blackboxServiceBootCmd(uint32_t waitMs)
{
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_FEFLAG(&huart1);
  __HAL_UART_CLEAR_NEFLAG(&huart1);
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    (void)huart1.Instance->DR;

  uint32_t startMs = HAL_GetTick();
  bool holdMode = false;
  char prevCmd = '\0';
  uint32_t prevCmdMs = 0u;

  while (holdMode || ((HAL_GetTick() - startMs) < waitMs))
  {
    char ch = '\0';
    if (!uartTryReadCleanChar(&ch))
    {
      HAL_Delay(5u);
      continue;
    }

    if (ch >= 'a' && ch <= 'z')
      ch = (char)(ch - 'a' + 'A');
    if (ch == '\r' || ch == '\n' || ch == ' ')
      continue;

    uint32_t now = HAL_GetTick();
    if (ch == 'D' || ch == 'C' || ch == 'H')
    {
      // Yêu cầu 2 ký tự giống nhau (DD/CC/HH) trong 1.5 giây để chống nhiễu khi PA10 thả nổi
      if ((ch == prevCmd) && ((now - prevCmdMs) <= 1500u))
      {
        prevCmd = '\0';
        if (ch == 'D')
        {
          blackboxDumpToUart();
          startMs = HAL_GetTick();
        }
        else if (ch == 'C')
        {
          if (blackboxErase())
            uartSendStr("\r\nBLACKBOX_ERASED OK\r\n");
          else
            uartSendStr("\r\nBLACKBOX_ERASED ERR\r\n");
          startMs = HAL_GetTick();
        }
        else if (ch == 'H')
        {
          holdMode = true;
          uartSendStr("\r\nBLACKBOX_HOLD: xe dung yen. Gui 'DD' de doc lai, 'CC' de xoa, bam RST de chay xe.\r\n");
        }
      }
      else
      {
        prevCmd = ch;
        prevCmdMs = now;
      }
    }
    else
    {
      prevCmd = '\0';
    }
  }
}
