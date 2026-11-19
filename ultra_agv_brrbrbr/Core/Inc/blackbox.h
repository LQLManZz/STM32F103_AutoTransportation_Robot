#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

void blackboxInit(void);
void blackboxStartCapture(void);
void blackboxStopCapture(void);
void blackboxPutChar(char ch);
bool blackboxHasSavedLog(void);
bool blackboxFlush(void);
bool blackboxErase(void);
void blackboxDumpToUart(void);
void blackboxServiceBootCmd(uint32_t waitMs);

#endif // BLACKBOX_H
