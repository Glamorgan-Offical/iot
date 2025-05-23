// app_sr_cmds.c
#include "esp_mn_speech_commands.h"
#include "app_sr.h"
#include "app_sr_cmds.h"

static const char* no_next_cmds[] = { NULL };

const sr_cmd_t g_robot_cmds_en[] = {
    { {NULL},0, SR_CMD_MOVE_FORWARD,  SR_LANG_EN, 0,
      "move forward",  "M UW V F AO R W ER D",  no_next_cmds },

    { {NULL},0, SR_CMD_MOVE_BACKWARD, SR_LANG_EN, 0,
      "move backward", "M UW V B AE K W ER D",  no_next_cmds },

    { {NULL},0, SR_CMD_TURN_LEFT,     SR_LANG_EN, 0,
      "turn left",     "T ER N L EH F T",        no_next_cmds },

    { {NULL},0, SR_CMD_TURN_RIGHT,    SR_LANG_EN, 0,
      "turn right",    "T ER N R AY T",          no_next_cmds },

    { {NULL},0, SR_CMD_SPIN_AROUND,   SR_LANG_EN, 0,
      "spin around",   "S P IH N ER AW N D",     no_next_cmds },
    { {NULL}, 0, SR_CMD_STOP,          SR_LANG_EN, 0,
    "stop",           "S T AA P",             no_next_cmds },
    { {NULL},0, SR_CMD_REPORT_TEMPHUMI, SR_LANG_EN, 0,
      "environment report", "EH N V AY R AH N M AH N T R IH P AO R T", no_next_cmds },
    { {NULL},0, SR_CMD_WALK_AROUND,    SR_LANG_EN, 0,
      "walk around",    "W AO K ER AW N D",     no_next_cmds },
};