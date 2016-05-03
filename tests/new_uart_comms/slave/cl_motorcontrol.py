# cl_motorcontrol.py
from usb_vendor import PIC_USB


def main(speed, direction):
    comms = PIC_USB(0x0005)
    comms.command_dcmotor(speed, direction)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('speed', metavar='<motor speed>', type=int, help='Speed of motor (0-65535)')
    parser.add_argument('direction', metavar='<motor direction>', type=int, help='Direction of motor (0 or 1)')

    args = parser.parse_args()
    # print(args)

    main(args.speed, args.direction)