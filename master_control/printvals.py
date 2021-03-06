import time
from usb_vendor import PIC_USB
# Product IDs: Master PIC is 0x0004, Rocket PIC is 0x0005, Barge PIC is 0x0006
comms = PIC_USB(0x0004)  

def main():
    print("START")
    loop_time = .25  # How often to run the main loop, in seconds
    while True:
        start_time = time.clock()
        # rocket_info()
        debug_uart_buffers()
        debug_uart_status()

        while (time.clock() - start_time) < loop_time:
            pass

def rocket_info():
    info = comms.get_rocket_info()
    print "Rocket Tilt {} | Rocket Speed {} | Rocket State {} | Motor Speed {} | Stepper Speed {}".format(
        info["tilt"],
        info["speed"],
        info["state"],
        info["motor_speed"],
        info["stepper_speed"]
    )

def debug_uart_status():
    info = comms.debug_uart_status()
    uart1 = info["uart1"]
    uart2 = info["uart2"]
    print "[UART1] URXDA: {} | OERR {} | FERR {} || PERR {} | RIDLE {} | ADDEN {}".format(
        uart1["URXDA"],
        uart1["OERR"],
        uart1["FERR"],
        uart1["PERR"],
        uart1["RIDLE"],
        uart1["ADDEN"]
    )
    print "[UART2] URXDA: {} | OERR {} | FERR {} || PERR {} | RIDLE {} | ADDEN {}".format(
        uart2["URXDA"],
        uart2["OERR"],
        uart2["FERR"],
        uart2["PERR"],
        uart2["RIDLE"],
        uart2["ADDEN"]
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

if __name__ == '__main__':
    main()
    # debug_uart_buffers()