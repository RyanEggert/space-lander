from usb_vendor import PIC_USB

def main():
    c = PIC_USB(0x0005)
    c.debug_servo_set_pos(200)
    c.debug_servo_set_freq(50)

if __name__ == '__main__':
    # import argparse
    # parser = argparse.ArgumentParser()
    # parser.add_argument('speed', metavar='<command>', help='command servo', choices=[""])

    # args = parser.parse_args()
    # # print(args)

    # main(args.speed, args.direction)
    main()