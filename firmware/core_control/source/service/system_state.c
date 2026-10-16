/**
 * @brief system state
 * @author Davy Refss
 *
 */
 
/* include files*/
#include "system_state.h"

static volatile running_state_t running_state = STATE_INIT;

running_state_t system_state_get(void)
{
    return running_state;
}

void system_state_set(running_state_t state)
{
    running_state = state;
}
