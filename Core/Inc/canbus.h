/* insert copyright  */

#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_

#include <stdint.h> // uint32_t

typedef struct __canbus_msg {
	uint32_t ts_rx;
	uint32_t ts_tx;
    uint32_t id;
    uint32_t dlc;
    union {
        uint8_t data[8];
        uint32_t data32[2];
    };
} canbus_msg;

#endif /* INC_CANBUS_H_ */
