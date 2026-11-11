/**
 * fsm.c - May trang thai dieu phoi robot (khong dung nut bam)
 *
 * IDLE -> SEARCH -> APPROACH -> GRIP -> TRANSPORT -> RELEASE -> IDLE
 *            ^          |         |
 *            +----------+---------+  (mat vat / gap hut thi quay lai SEARCH)
 * Loi (timeout) -> ERROR.  FSM_EventStop() o bat ky trang thai nao -> IDLE.
 *
 * Bat dau: tu dong sau FSM_AUTO_START_MS, hoac khi ai do goi FSM_EventStart().
 */
#include "fsm.h"
#include "robot_if.h"
#include "main.h"

static FSM_State_t state;
static uint32_t    t_enter;       /* thoi diem vao trang thai hien tai */
static uint32_t    t_lost;        /* thoi diem bat dau mat vat (0 = chua mat) */
static uint8_t     grip_retry;
static uint8_t     auto_armed;    /* 1 = con duoc phep tu bat dau o IDLE */
static volatile uint8_t ev_start, ev_stop;

static const char *const state_names[] = {
    "IDLE", "SEARCH", "APPROACH", "GRIP", "TRANSPORT", "RELEASE", "ERROR"
};

const char *FSM_GetStateName(FSM_State_t s)
{
    return (s <= FSM_ERROR) ? state_names[s] : "?";
}

FSM_State_t FSM_GetState(void) { return state; }
void FSM_EventStart(void)      { ev_start = 1; }
void FSM_EventStop(void)       { ev_stop  = 1; }

/* ---------- Chuyen trang thai + hanh dong khi VAO trang thai ---------- */
static void change_state(FSM_State_t next)
{
    Motor_Stop();                       /* an toan: dung xe truoc moi lan chuyen */
    state   = next;
    t_enter = HAL_GetTick();
    t_lost  = 0;

    switch (next) {
    case FSM_IDLE:
        Scan_Stop();
        grip_retry = 0;
        if (FSM_AUTO_RESTART) auto_armed = 1;
        break;
    case FSM_SEARCH:
        Scan_Start();
        break;
    case FSM_APPROACH:
        Scan_Stop();                    /* giu servo huong vao vat */
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

/* ---------- Vong cap nhat FSM ---------- */
void FSM_Update(void)
{
    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - t_enter;

    /* Lenh Stop co do uu tien cao nhat o moi trang thai */
    if (ev_stop) {
        ev_stop = 0;
        ev_start = 0;
        if (state != FSM_IDLE) change_state(FSM_IDLE);
        auto_armed = 0;                 /* da dung thi khong tu chay lai */
        return;
    }

    switch (state) {

    case FSM_IDLE:
        if (ev_start ||
            (auto_armed && FSM_AUTO_START_MS > 0 && elapsed >= FSM_AUTO_START_MS)) {
            ev_start = 0;
            auto_armed = FSM_AUTO_RESTART ? 1 : 0;
            grip_retry = 0;
            change_state(FSM_SEARCH);
        }
        break;

    case FSM_SEARCH:
        ev_start = 0;
        if (Sensor_ObjectFound())
            change_state(FSM_APPROACH);
        else if (elapsed > FSM_SEARCH_TIMEOUT_MS)
            change_state(FSM_ERROR);
        break;

    case FSM_APPROACH: {
        uint16_t d = Sensor_GetDistanceCm();
        if (Sensor_ObjectFound()) {
            t_lost = 0;
            if (d > 0 && d <= FSM_GRIP_DISTANCE_CM)
                change_state(FSM_GRIP);
        } else {
            if (t_lost == 0) t_lost = now;
            else if (now - t_lost > FSM_LOST_TIMEOUT_MS)
                change_state(FSM_SEARCH);
        }
        break;
    }

    case FSM_GRIP:
        if (Gripper_IsDone() && elapsed > 300) {    /* 300 ms cho tay gap kip nhan lenh */
            if (Gripper_HasObject()) {
                change_state(FSM_TRANSPORT);
            } else if (++grip_retry >= FSM_GRIP_MAX_RETRY) {
                change_state(FSM_ERROR);
            } else {
                Gripper_Open();
                change_state(FSM_SEARCH);
            }
        } else if (elapsed > FSM_GRIP_TIMEOUT_MS) {
            change_state(FSM_ERROR);
        }
        break;

    case FSM_TRANSPORT:
        /* TODO: thay bang dieu kien "da toi dich" khi nhom chot (cam bien vach / vach den / ...) */
        if (elapsed >= FSM_TRANSPORT_TIME_MS)
            change_state(FSM_RELEASE);
        break;

    case FSM_RELEASE:
        if (Gripper_IsDone() && elapsed >= FSM_RELEASE_MIN_MS)
            change_state(FSM_IDLE);
        break;

    case FSM_ERROR:
        /* Dung yen, cho FSM_EventStop() (xu ly o dau ham) hoac nhan RESET */
        break;
    }
}
