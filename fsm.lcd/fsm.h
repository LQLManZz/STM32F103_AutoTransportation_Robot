#ifndef FSM_H
#define FSM_H

#include <stdint.h>

typedef enum
{
  FSM_IDLE = 0,
  FSM_SEARCH,
  FSM_APPROACH,
  FSM_GRIP,
  FSM_TRANSPORT,
  FSM_RELEASE,
  FSM_ERROR
} FSM_State_t;

#define FSM_AUTO_START_MS 3000u
#define FSM_AUTO_RESTART 0u
#define FSM_SPEED_APPROACH 50u
#define FSM_SPEED_TRANSPORT 60u
#define FSM_GRIP_DISTANCE_CM 8u
#define FSM_SEARCH_TIMEOUT_MS 20000u
#define FSM_LOST_TIMEOUT_MS 1000u
#define FSM_GRIP_TIMEOUT_MS 5000u
#define FSM_GRIP_MAX_RETRY 3u
#define FSM_TRANSPORT_TIME_MS 5000u
#define FSM_RELEASE_MIN_MS 1000u

void FSM_Init(void);
void FSM_Update(void);
void FSM_EventStart(void);
void FSM_EventStop(void);
FSM_State_t FSM_GetState(void);
const char *FSM_GetStateName(FSM_State_t s);

#endif // FSM_H
