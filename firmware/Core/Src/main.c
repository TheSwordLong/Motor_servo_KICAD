#include "gd32f30x.h"
#include "systick.h"
#include "robot_port.h"
#include "capture.h"
int main(void)
{
    robot_emergency_stop();
    systick_config();
    robot_port_init();
    for(;;) {
        robot_port_update();
        capture_debug_update();
    }
}
