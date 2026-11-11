#include "robot_if.h"
#include "main.h"

#define WEAK __attribute__((weak))

static uint8_t sim_scanning, sim_found, sim_moving, sim_holding, sim_grip_busy;
static uint32_t sim_t_scan, sim_t_move, sim_t_grip;
static uint16_t sim_dist = 60;

WEAK void Scan_Start(void) { sim_scanning = 1; sim_t_scan = HAL_GetTick(); }
WEAK void Scan_Stop(void)  { sim_scanning = 0; }

WEAK uint8_t Sensor_ObjectFound(void)
{
  if (sim_scanning && !sim_found && (HAL_GetTick() - sim_t_scan > 3000))
    sim_found = 1;
  return sim_found;
}

WEAK uint16_t Sensor_GetDistanceCm(void)
{
  if (sim_moving && !sim_holding && sim_found && (HAL_GetTick() - sim_t_move >= 200))
  {
    sim_t_move = HAL_GetTick();
    if (sim_dist > 5) sim_dist--;
  }
  return sim_found ? sim_dist : 0;
}

WEAK void Motor_Forward(uint8_t s)   { (void)s; sim_moving = 1; sim_t_move = HAL_GetTick(); }
WEAK void Motor_Backward(uint8_t s)  { (void)s; sim_moving = 0; }
WEAK void Motor_TurnLeft(uint8_t s)  { (void)s; sim_moving = 0; }
WEAK void Motor_TurnRight(uint8_t s) { (void)s; sim_moving = 0; }
WEAK void Motor_Stop(void)           { sim_moving = 0; }

WEAK void Gripper_Close(void)
{
  sim_grip_busy = 1; sim_holding = 1; sim_t_grip = HAL_GetTick();
}

WEAK void Gripper_Open(void)
{
  sim_grip_busy = 1; sim_holding = 0; sim_t_grip = HAL_GetTick();
  sim_found = 0; sim_dist = 60;
}

WEAK uint8_t Gripper_IsDone(void)
{
  if (sim_grip_busy && (HAL_GetTick() - sim_t_grip > 1000))
    sim_grip_busy = 0;
  return !sim_grip_busy;
}

WEAK uint8_t Gripper_HasObject(void) { return sim_holding; }
