/**
 * robot_if.h - GIAO DIEN CHUNG giua FSM va cac module cua cac ban khac
 *
 * Gui file nay cho ca nhom. Moi nguoi viet ham dung TEN + KIEU nhu duoi day.
 * Khi ghep, ham that cua ban ay se tu dong thay the ham gia (stub) trong robot_stub.c
 */
#ifndef ROBOT_IF_H
#define ROBOT_IF_H

#include <stdint.h>

/* ===== Le Quang Long: Servo quet + SRF05 ===== */
void     Scan_Start(void);            /* bat dau quet servo tim vat          */
void     Scan_Stop(void);             /* dung quet, giu servo huong vao vat  */
uint8_t  Sensor_ObjectFound(void);    /* 1 = dang thay vat, 0 = khong        */
uint16_t Sensor_GetDistanceCm(void);  /* khoang cach toi vat (cm)            */

/* ===== Vuong Thanh Luan: Driver dong co 2WD ===== */
void Motor_Forward(uint8_t speed_pct);    /* 0..100 % */
void Motor_Backward(uint8_t speed_pct);
void Motor_TurnLeft(uint8_t speed_pct);
void Motor_TurnRight(uint8_t speed_pct);
void Motor_Stop(void);

/* ===== Hoang Vu Quang: Tay gap ===== */
void    Gripper_Close(void);          /* gap vat                        */
void    Gripper_Open(void);           /* nha vat                        */
uint8_t Gripper_IsDone(void);         /* 1 = da xong thao tac gap/nha   */
uint8_t Gripper_HasObject(void);      /* 1 = dang cam vat               */

#endif /* ROBOT_IF_H */
