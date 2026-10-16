/**
 * @file bsp_can.h
 * @author Davyrefss
 * @brief can bsp
 *
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

/* include files */
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* defines */
#define CAN_STANDARD_ID  (0x000Fu)

/* typedefs */
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

/* functions decalration */
bool can_config_init(void);
bool can_transmit_data(const can_frame_t* frame);
bool can_receive_data(const can_frame_t* frame);
void can_rx_task(void);
void can_parse_data(const can_frame_t* frame);   
void can_test_send(void);

#endif /* BSP_CAN_H */
