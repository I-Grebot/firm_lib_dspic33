#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

// Include main board configuration file
#include "board_cfg.h"

// Prototypes
// ----------

void scheduler_init(void);
uint8_t scheduler_ready(void);
void scheduler_start(void);
void scheduler_stop(void);
uint16_t scheduler_task_ready(uint16_t _rate);

#endif // !_SCHEDULER_H_
