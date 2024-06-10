/* insert copyright  */

#include "app.h"
#include "main.h"

/* extern variables */
extern CAN_HandleTypeDef hcan;
canbus_msg msg = {.id=0x7E, .dlc=8};

void init_app(void)
{
    init_canhw(&hcan);
    msg.data32[1] = 0;
    msg.data32[0] = 0xFF;
}

void deinit_app(void)
{
    deinit_canhw(&hcan);
}

void task_app()
{
    task_canhw(&hcan);
    canhw_send(&hcan, &msg);
}

void cb_app(canbus_msg *rxmsg, CAN_RxHeaderTypeDef *rxHeader)
{

}
