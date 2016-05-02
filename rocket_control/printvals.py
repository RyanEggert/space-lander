import time
from usb_vendor import PIC_USB
import traceback
# Product IDs: Master PIC is 0x0004, Rocket PIC is 0x0005, Barge PIC is 0x0006
comms = PIC_USB(0x0005)

def main():
    print("START")
    loop_time = .2  # How often to run the main loop, in seconds
    while True:
        start_time = time.clock()
        # print(chr(27) + "[2J")
        # quad_info()
        try:
            debug_uart_buffers()
            debug_uart_status()
            rocket_info()
            endstops()
            # debug_oc_status()
        except Exception, e:
            print "Error occurred. {}".format(e)
            traceback.print_exc()
            print "Retrying..."
            comms = PIC_USB(0x0005)
        while (time.clock() - start_time) < loop_time:
            pass

def rocket_info():
    info = comms.get_rocket_info()
    print "Rocket Tilt {} | Rocket Speed {} | Throttle {} | Motor Speed {} | Motor Thrust {} | Stepper Speed {} | Tilt Angle {} | Tilt Direction {} | Rocket State {}".format(
        info["tilt"],
        info["speed"],
        info["throttle"],
        info["motor_speed"],
        info["motor_thrust"],
        info["tilt_ang"],
        info["tilt_dir"],
        info["stepper_speed"],
        info["rocket_state"]
    )


def debug_uart_buffers():
    info = comms.debug_uart_buffers()
    rx = info["rx"]
    tx = info["tx"]
    print "TX_head {} | TX_tail {} | TX_count {} || RX_head {} | RX_tail {} | RX_count {}".format(
        tx["head"],
        tx["tail"],
        tx["count"],
        rx["head"],
        rx["tail"],
        rx["count"],
    )

def debug_uart_status():
    info = comms.debug_uart_status()
    print "URXDA: {} | OERR {} | FERR {} || PERR {} | RIDLE {} | ADDEN {}".format(
        info["URXDA"],
        info["OERR"],
        info["FERR"],
        info["PERR"],
        info["RIDLE"],
        info["ADDEN"]
    )

def debug_oc_status():
    info = comms.debug_oc_status()
    print "DC_OCM0 {} | DC_OCM1 {} | DC_OCM2 {} | DC_OCTSEL {} | DC_OCFLT {}".format(
        info["DC_OCM0"],
        info["DC_OCM1"],
        info["DC_CM2"],
        info["DC_OCTSEL"],
        info["DC_OCFLT"],
    )
    print "ST_OCM0 {} | ST_OCM1 {} | ST_OCM2 {} | ST_OCTSEL {} | ST_OCFLT {}".format(
        info["ST_OCM0"],
        info["ST_OCM1"],
        info["ST_CM2"],
        info["ST_OCTSEL"],
        info["ST_OCFLT"]
    )


def quad_info():
    info = comms.get_quad_info()
    print "Quad Counter {} | Overflow {}".format(
        info["counter"],
        info["overflow"],
    )

def endstops():
    """
    Reads the system's endstops.
    """
    info = comms.get_limit_sw_info()
    print("Y_BOT {} | Y_TOP {} | X_L {} | X_R {} | BARGE {} ".format(
    info["Y_BOT"],
    info["Y_TOP"],
    info["X_L"],
    info["X_R"],
    info["BARGE"])
    )

if __name__ == '__main__':
    main()