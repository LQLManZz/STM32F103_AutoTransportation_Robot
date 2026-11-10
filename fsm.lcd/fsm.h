/**
 * fsm.h - May trang thai (FSM) dieu phoi toan he thong robot
 * Phu trach: Nguyen Quang Chinh
 *
 * Phien ban KHONG co nut bam: FSM tu bat dau sau khi cap nguon (FSM_AUTO_START_MS),
 * hoac nhom goi FSM_EventStart() / FSM_EventStop() tu noi khac.
 */
#ifndef FSM_H
#define FSM_H

#include <stdint.h>

typedef enum {
    FSM_IDLE = 0,     /* Cho lenh bat dau                      */
    FSM_SEARCH,       /* Servo quet tim vat                    */
    FSM_APPROACH,     /* Chay toi vat                          */
    FSM_GRIP,         /* Dong tay gap                          */
    FSM_TRANSPORT,    /* Cho vat toi dich                      */
    FSM_RELEASE,      /* Nha vat                               */
    FSM_ERROR         /* Dung an toan, cho FSM_EventStop() hoac nhan RESET tren board */
} FSM_State_t;

/* ===== THAM SO CHINH (sua tai day, khong sua trong fsm.c) ===== */
#define FSM_AUTO_START_MS       3000    /* Tu chay sau khi bat nguon ngan ay ms (de kip dat robot). 0 = khong tu chay, cho FSM_EventStart() */
#define FSM_AUTO_RESTART        0       /* 1 = xong RELEASE tu chay lai vong moi; 0 = dung o IDLE (nhan RESET de chay lai) */

#define FSM_SPEED_APPROACH      50      /* % toc do khi tien toi vat           */
#define FSM_SPEED_TRANSPORT     60      /* % toc do khi cho vat                */
#define FSM_GRIP_DISTANCE_CM    8       /* <= khoang cach nay thi bat dau gap  */
#define FSM_SEARCH_TIMEOUT_MS   20000   /* quet qua lau ma khong thay -> loi   */
#define FSM_LOST_TIMEOUT_MS     1000    /* mat vat qua lau -> quay lai SEARCH  */
#define FSM_GRIP_TIMEOUT_MS     5000    /* gap qua lau -> loi                  */
#define FSM_GRIP_MAX_RETRY      3       /* gap hut toi da may lan              */
#define FSM_TRANSPORT_TIME_MS   5000    /* TAM THOI: chay 5 s toi dich (chua chot cach xac dinh dich) */
#define FSM_RELEASE_MIN_MS      1000    /* thoi gian toi thieu o trang thai nha */

void         FSM_Init(void);
void         FSM_Update(void);          /* goi lien tuc trong while(1)         */

void         FSM_EventStart(void);      /* ra lenh bat dau (tu code khac, UART, nut cua ban khac...) */
void         FSM_EventStop(void);       /* ra lenh dung, ve IDLE               */

FSM_State_t  FSM_GetState(void);
const char  *FSM_GetStateName(FSM_State_t s);

#endif /* FSM_H */
