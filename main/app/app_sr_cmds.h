// app_sr_cmds.h
#ifndef _APP_SR_CMDS_H_
#define _APP_SR_CMDS_H_
#include "esp_mn_speech_commands.h"

typedef enum {
    SR_LANG_EN,
    SR_LANG_CN,
    SR_LANG_MAX,
} sr_language_t;

typedef enum {
    SR_CMD_MOVE_FORWARD   = 100,
    SR_CMD_MOVE_BACKWARD,
    SR_CMD_TURN_LEFT,
    SR_CMD_TURN_RIGHT,
    SR_CMD_SPIN_AROUND,
    SR_CMD_STOP,
    SR_CMD_REPORT_TEMPHUMI, 
    SR_CMD_WALK_AROUND,   
} sr_robot_cmd_t;

typedef struct sr_cmd {
    SLIST_ENTRY(sr_cmd) next;
    uint32_t id;
    sr_robot_cmd_t cmd_id;
    sr_language_t lang;
    int param;
    const char* str;
    const char* phoneme;
    const char** next_cmds; 
} sr_cmd_t;

#define ROBOT_CMDS_EN_COUNT  8  
extern const sr_cmd_t g_robot_cmds_en[ROBOT_CMDS_EN_COUNT];

#endif // _APP_SR_CMDS_H_