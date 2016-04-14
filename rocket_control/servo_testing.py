from usb_vendor import PIC_USB
import time

def return_reverse(a_list):
    a_list.reverse()
    return a_list

def main():
    SERVO_MIN = 120
    SERVO_MAX = 450
    SERVO_CENTER = int(((SERVO_MAX - SERVO_MIN)/2.) + SERVO_MIN)
    c = PIC_USB(0x0005)
    c.debug_servo_set_freq(50)
    time.sleep(.5)
    c.debug_servo_set_pos(7, SERVO_CENTER)
    time.sleep(.1)
    for pos in return_reverse(range(SERVO_MIN, SERVO_CENTER, 2)):
            print pos
            c.debug_servo_set_pos(7, pos)
            c.debug_servo_set_pos(8, pos)
            time.sleep(.08)
    # c.debug_servo_set_pos(8, 450)
    time.sleep(1)
    for i in xrange(3):
        for pos in range(SERVO_MIN, SERVO_MAX, 3):
            print pos
            c.debug_servo_set_pos(7, pos)
            c.debug_servo_set_pos(8, pos)
            time.sleep(.03)
        for pos in return_reverse(range(SERVO_MIN, SERVO_MAX, 5)):
            print pos
            c.debug_servo_set_pos(7, pos)
            c.debug_servo_set_pos(8, pos)
            time.sleep(.01)
        time.sleep(1)

    c.debug_servo_set_pos(7, SERVO_CENTER)

if __name__ == '__main__':
    # import argparse
    # parser = argparse.ArgumentParser()
    # parser.add_argument('speed', metavar='<command>', help='command servo', choices=[""])

    # args = parser.parse_args()
    # # print(args)

    # main(args.speed, args.direction)
    main()