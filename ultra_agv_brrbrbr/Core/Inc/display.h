/**
 * display.h - Hien thi trang thai FSM va khoang cach len LCD 16x2
 * Phu trach: Nguyen Quang Chinh
 */
#ifndef DISPLAY_H
#define DISPLAY_H

#include "main.h"

#define DISPLAY_REFRESH_MS   250    /* chu ky lam moi LCD */

/* Goi sau LCD_Init(): hien man hinh chao */
void Display_Init(void);

/* Goi lien tuc trong while(1): tu lam moi LCD moi 250 ms */
void Display_Update(void);

#endif /* DISPLAY_H */
