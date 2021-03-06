import time
from usb_vendor import PIC_USB
comms = PIC_USB()

def main():
    print("START")
    loop_time = .25  # How often to run the main loop, in seconds
    while True:
        start_time = time.clock()
        rocket_info()

        while (time.clock() - start_time) < loop_time:
            pass

def rocket_info():
    info = comms.get_rocket_info()
    print "Rocket Tilt {} | Rocket Speed {} | Rocket State {}".format(
        info["tilt"],
        info["speed"],
        info["state"]
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