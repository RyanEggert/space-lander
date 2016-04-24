import time
from usb_vendor import PIC_USB
# Product IDs: Master PIC is 0x0004, Rocket PIC is 0x0005, Barge PIC is 0x0006
comms = PIC_USB(0x0005)

def main():
    print("START")
    loop_time = .05  # How often to run the main loop, in seconds
    while True:
        start_time = time.clock()
        # print(chr(27) + "[2J")
        # quad_info()
<<<<<<< HEAD
        # rocket_info()
        debug_uart_buffers()
=======
        try:
            debug_uart_buffers()
            debug_uart_status()
            rocket_info()
        except Exception, e:
            print "Error occurred."
            print e
            print "Retrying..."
>>>>>>> 829e9a890ded95b5f48aeefdba69ec5561bc3a4a
        while (time.clock() - start_time) < loop_time:
            pass

def rocket_info():
    info = comms.get_rocket_info()
    print "Rocket Tilt {} | Rocket Speed {} | Throttle {} | Motor Speed {} | Motor Thrust {} | Tilt Angle {} | Tilt Direction {}".format(
        info["tilt"],
        info["speed"],
        info["throttle"],
        info["motor_speed"],
        info["motor_thrust"],
        info["tilt_ang"],
        info["tilt_dir"]
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

def quad_info():
    info = comms.get_quad_info()
    print "Quad Counter {} | Overflow {}".format(
        info["counter"],
        info["overflow"],
    )


if __name__ == '__main__':
    main()