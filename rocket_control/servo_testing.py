from usb_vendor import PIC_USB
import time
def main():
    c = PIC_USB(0x0005)
    c.debug_servo_set_freq(50)
    c.debug_servo_set_pos(7, 120)
    time.sleep(1)
    # c.debug_servo_set_pos(8, 450)

    for pos in range(120, 450, 3):
        print pos
        c.debug_servo_set_pos(7, pos)

        c.debug_servo_set_pos(8, pos)
        time.sleep(.05)

if __name__ == '__main__':
    # import argparse
    # parser = argparse.ArgumentParser()
    # parser.add_argument('speed', metavar='<command>', help='command servo', choices=[""])

    # args = parser.parse_args()
    # # print(args)

    # main(args.speed, args.direction)
    main()