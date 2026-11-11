#ifndef ROBOT_IF_H
#define ROBOT_IF_H

#include <stdint.h>

// Giao diện Servo quét + SRF05
void Scan_Start(void);
void Scan_Stop(void);
uint8_t Sensor_ObjectFound(void);
uint16_t Sensor_GetDistanceCm(void);

// Giao diện động cơ 2WD
void Motor_Forward(uint8_t speed_pct);
void Motor_Backward(uint8_t speed_pct);
void Motor_TurnLeft(uint8_t speed_pct);
void Motor_TurnRight(uint8_t speed_pct);
void Motor_Stop(void);

// Giao diện tay gắp
void Gripper_Close(void);
void Gripper_Open(void);
uint8_t Gripper_IsDone(void);
uint8_t Gripper_HasObject(void);

#endif // ROBOT_IF_H
