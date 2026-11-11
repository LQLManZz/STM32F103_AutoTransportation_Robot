#include "fsm.h"
#include "robot_if.h"
#include "main.h"

static FSM_State_t state;
static uint32_t t_enter, t_lost;
static uint8_t grip_retry, auto_armed;
static volatile uint8_t ev_start, ev_stop;

static const char *const state_names[] = {
    "IDLE", "SEARCH", "APPROACH", "GRIP", "TRANSPORT", "RELEASE", "ERROR"};

const char *FSM_GetStateName(FSM_State_t s) { return (s <= FSM_ERROR) ? state_names[s] : "?"; }
FSM_State_t FSM_GetState(void)              { return state; }
void FSM_EventStart(void)                   { ev_start = 1; }
void FSM_EventStop(void)                    { ev_stop = 1; }

static void change_state(FSM_State_t next)
{
  Motor_Stop();
  state = next;
  t_enter = HAL_GetTick();
  t_lost = 0;

  switch (next)
  {
  case FSM_IDLE:
    Scan_Stop();
    grip_retry = 0;
    if (FSM_AUTO_RESTART) auto_armed = 1;
    break;
  case FSM_SEARCH:
    Scan_Start();
    break;
  case FSM_APPROACH:
    Scan_Stop();
    Motor_Forward(FSM_SPEED_APPROACH);
    break;
  case FSM_GRIP:
    Gripper_Close();
    break;
  case FSM_TRANSPORT:
    Motor_Forward(FSM_SPEED_TRANSPORT);
    break;
  case FSM_RELEASE:
    Gripper_Open();
    break;
  case FSM_ERROR:
    Scan_Stop();
    break;
  }
}

void FSM_Init(void)
{
  ev_start = ev_stop = 0;
  auto_armed = 1;
  change_state(FSM_IDLE);
}

void FSM_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - t_enter;

  if (ev_stop)
  {
    ev_stop = ev_start = 0;
    if (state != FSM_IDLE) change_state(FSM_IDLE);
    auto_armed = 0;
    return;
  }

  switch (state)
  {
  case FSM_IDLE:
    if (ev_start || (auto_armed && FSM_AUTO_START_MS > 0 && elapsed >= FSM_AUTO_START_MS))
    {
      ev_start = 0;
      auto_armed = FSM_AUTO_RESTART ? 1 : 0;
      grip_retry = 0;
      change_state(FSM_SEARCH);
    }
    break;

  case FSM_SEARCH:
    ev_start = 0;
    if (Sensor_ObjectFound()) change_state(FSM_APPROACH);
    else if (elapsed > FSM_SEARCH_TIMEOUT_MS) change_state(FSM_ERROR);
    break;

  case FSM_APPROACH:
    if (Sensor_ObjectFound())
    {
      uint16_t d = Sensor_GetDistanceCm();
      t_lost = 0;
      if (d > 0 && d <= FSM_GRIP_DISTANCE_CM) change_state(FSM_GRIP);
    }
    else if (t_lost == 0)
    {
      t_lost = now;
    }
    else if (now - t_lost > FSM_LOST_TIMEOUT_MS)
    {
      change_state(FSM_SEARCH);
    }
    break;

  case FSM_GRIP:
    if (Gripper_IsDone() && elapsed > 300u)
    {
      if (Gripper_HasObject()) change_state(FSM_TRANSPORT);
      else if (++grip_retry >= FSM_GRIP_MAX_RETRY) change_state(FSM_ERROR);
      else { Gripper_Open(); change_state(FSM_SEARCH); }
    }
    else if (elapsed > FSM_GRIP_TIMEOUT_MS)
    {
      change_state(FSM_ERROR);
    }
    break;

  case FSM_TRANSPORT:
    if (elapsed >= FSM_TRANSPORT_TIME_MS) change_state(FSM_RELEASE);
    break;

  case FSM_RELEASE:
    if (Gripper_IsDone() && elapsed >= FSM_RELEASE_MIN_MS) change_state(FSM_IDLE);
    break;

  case FSM_ERROR:
    break;
  }
}
