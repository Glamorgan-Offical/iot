// app_sr_cmds.c
#include "esp_mn_speech_commands.h"
#include "app_sr.h"
#include "app_sr_cmds.h"

// 如果不需要后续命令，就将next_cmds设为NULL
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
};