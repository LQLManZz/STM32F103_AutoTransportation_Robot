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
static bool holdMode = false;
static char prevCmd = '\0';
static uint32_t prevCmdMs = 0u;

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
  holdMode = false;
  prevCmd = '\0';
  prevCmdMs = 0u;
}

void blackboxStartCapture(void)
{
  writeIdx = 0u;
  wrapped = false;
  dirty = false;
  capturing = true;
}

void blackboxResumeCapture(void)
{
  capturing = true;
}

void blackboxStopCapture(void)
{
  capturing = false;
}

bool blackboxIsCapturing(void)
{
  return capturing;
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

void blackboxWriteStr(const char *str)
{
  if (!str)
    return;
  while (*str)
    blackboxPutChar(*str++);
}

bool blackboxHasCompleteLog(void)
{
  const volatile BlackboxHeader *hdr = (const volatile BlackboxHeader *)BLACKBOX_FLASH_ADDR;
  return (hdr->magic == BLACKBOX_MAGIC_DONE) &&
         (hdr->length > 0u) &&
         (hdr->length <= BLACKBOX_MAX_DATA_LEN);
}

bool blackboxHasSavedLog(void)
{
  const volatile BlackboxHeader *hdr = (const volatile BlackboxHeader *)BLACKBOX_FLASH_ADDR;
  return ((hdr->magic == BLACKBOX_MAGIC) || (hdr->magic == BLACKBOX_MAGIC_DONE)) &&
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

// Lấy vị trí bắt đầu và độ dài hợp lệ trong ring RAM (căn sau dòng cắt nếu đã quay vòng)
static void getRamSpan(uint16_t *outStartIdx, uint16_t *outTotalLen)
{
  uint16_t totalLen = wrapped ? (uint16_t)BLACKBOX_MAX_DATA_LEN : writeIdx;
  uint16_t startIdx = wrapped ? writeIdx : 0u;
  if (wrapped && totalLen > 0u)
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
  *outStartIdx = startIdx;
  *outTotalLen = totalLen;
}

static bool flushWithMagic(uint32_t magic, bool forceUpgrade)
{
  if (!dirty && !(forceUpgrade && !blackboxHasCompleteLog()))
    return true;
  if (flushCount >= BLACKBOX_MAX_FLUSHES)
    return false;

  uint16_t startIdx = 0u;
  uint16_t totalLen = 0u;
  getRamSpan(&startIdx, &totalLen);
  if (totalLen == 0u)
    return true;

  if (!isFlashRegionSafe())
  {
    uartSendStr("BLACKBOX_ERR fw_overlap\r\n");
    return false;
  }

  bool prevCap = capturing;
  capturing = false;

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
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BLACKBOX_FLASH_ADDR, magic) != HAL_OK)
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

bool blackboxFlush(void)
{
  return flushWithMagic(BLACKBOX_MAGIC, false);
}

bool blackboxFlushComplete(void)
{
  return flushWithMagic(BLACKBOX_MAGIC_DONE, true);
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
    const char *stateStr = (hdr->magic == BLACKBOX_MAGIC_DONE) ? "COMPLETE" : "PARTIAL";
    char hdrLine[96];
    snprintf(hdrLine, sizeof(hdrLine),
             "\r\n=== BLACKBOX LOG START (len=%lu bytes, state=%s) ===\r\n",
             (unsigned long)len, stateStr);
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

  uartSendStr("=== CMD: gui 'DD'=Flash, 'RR'=RAM, 'FF'=luu, 'CC'=xoa, 'HH'=dung, 'GG'=chay ===\r\n");
  capturing = prevCap;
}

void blackboxDumpRamToUart(void)
{
  bool prevCap = capturing;
  capturing = false;

  uint16_t startIdx = 0u;
  uint16_t len = 0u;
  getRamSpan(&startIdx, &len);

  if (len == 0u)
  {
    uartSendStr("\r\n=== BLACKBOX RAM EMPTY ===\r\n");
  }
  else
  {
    char hdrLine[80];
    snprintf(hdrLine, sizeof(hdrLine),
             "\r\n=== BLACKBOX RAM START (len=%lu bytes) ===\r\n",
             (unsigned long)len);
    uartSendStr(hdrLine);

    uint8_t chunk[64];
    uint16_t offset = 0u;
    while (offset < len)
    {
      uint16_t n = (uint16_t)((len - offset > sizeof(chunk)) ? sizeof(chunk) : (len - offset));
      for (uint16_t i = 0u; i < n; i++)
        chunk[i] = ringByteAt(startIdx, (uint16_t)(offset + i));
      uartSendRaw(chunk, n);
      offset += n;
    }

    uartSendStr("\r\n=== BLACKBOX RAM END ===\r\n");
  }

  capturing = prevCap;
}

// Giải mã cặp lệnh lặp 2 ký tự (trong 1.5s) để chống nhiễu trên chân PA10 thả nổi
static void processUartCmdChar(char ch, bool *outGoNow, bool *outResetTimeout)
{
  if (ch >= 'a' && ch <= 'z')
    ch = (char)(ch - 'a' + 'A');
  if (ch == '\r' || ch == '\n' || ch == ' ')
    return;

  if (ch != 'D' && ch != 'R' && ch != 'F' && ch != 'C' && ch != 'H' && ch != 'G')
  {
    prevCmd = '\0';
    return;
  }

  uint32_t now = HAL_GetTick();
  if ((ch == prevCmd) && ((now - prevCmdMs) <= 1500u))
  {
    prevCmd = '\0';
    if (ch == 'D')
    {
      blackboxDumpToUart();
      if (outResetTimeout)
        *outResetTimeout = true;
    }
    else if (ch == 'R')
    {
      blackboxDumpRamToUart();
      if (outResetTimeout)
        *outResetTimeout = true;
    }
    else if (ch == 'F')
    {
      uartSendStr(blackboxFlush() ? "\r\nBLACKBOX_FLUSH OK\r\n" : "\r\nBLACKBOX_FLUSH ERR\r\n");
      if (outResetTimeout)
        *outResetTimeout = true;
    }
    else if (ch == 'C')
    {
      uartSendStr(blackboxErase() ? "\r\nBLACKBOX_ERASED OK\r\n" : "\r\nBLACKBOX_ERASED ERR\r\n");
      if (outResetTimeout)
        *outResetTimeout = true;
    }
    else if (ch == 'H')
    {
      holdMode = true;
      uartSendStr("\r\nBLACKBOX_HOLD: xe dung yen. Gui 'DD'=Flash, 'RR'=RAM, 'FF'=luu, 'CC'=xoa, 'GG'=chay.\r\n");
    }
    else if (ch == 'G')
    {
      holdMode = false;
      if (outGoNow)
        *outGoNow = true;
      uartSendStr("\r\nBLACKBOX_GO\r\n");
    }
  }
  else
  {
    prevCmd = ch;
    prevCmdMs = now;
  }
}

void blackboxServiceBootCmd(uint32_t waitMs)
{
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_FEFLAG(&huart1);
  __HAL_UART_CLEAR_NEFLAG(&huart1);
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    (void)huart1.Instance->DR;

  uint32_t startMs = HAL_GetTick();
  prevCmd = '\0';

  while (holdMode || ((HAL_GetTick() - startMs) < waitMs))
  {
    bool goNow = false;
    bool resetTimeout = false;
    char ch = '\0';
    while (uartTryReadCleanChar(&ch))
      processUartCmdChar(ch, &goNow, &resetTimeout);

    if (resetTimeout)
      startMs = HAL_GetTick();
    if (goNow && !holdMode)
      break;
    HAL_Delay(5u);
  }
}

bool blackboxPollUartCmd(void)
{
  bool goNow = false;
  char ch = '\0';

  while (uartTryReadCleanChar(&ch))
    processUartCmdChar(ch, &goNow, NULL);

  while (holdMode)
  {
    while (uartTryReadCleanChar(&ch))
      processUartCmdChar(ch, &goNow, NULL);
    HAL_Delay(5u);
  }

  return goNow;
}
