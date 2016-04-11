import usb.core

class PIC_USB(object):
    def __init__(self, product_id):
        super(PIC_USB, self).__init__()
        self.SET_STATE = 0
        self.GET_VALS = 1
        self.GET_ROCKET_INFO = 2
        self.DEBUG_UART_BUFFERS = 3
        self.GET_QUAD_INFO = 4
        self.COMMAND_DCMOTOR = 5

        self.DEBUG_SERVO_SET_POS = 60
        self.DEBUG_SERVO_SET_FREQ = 61
        self.DEBUG_SERVO_SLEEP = 62
        self.DEBUG_SERVO_WAKE = 63
        self.DEBUG_SERVO_RESET = 64

        self.vendor_id = 0x6666
        self.product_id = product_id
        self.dev = usb.core.find(idVendor=self.vendor_id, idProduct=self.product_id)
        if self.dev is None:
            raise ValueError('No USB device found with idVendor=0x{0:04x} and idProduct=0x{1:04x}.'.format(self.vendor_id, self.product_id))
        try:
            self.dev.set_configuration()
        except usb.core.USBError, e:
            print("ERR:\nError connecting to found device.\n" 
                "If permissions issue, try adding this device to /etc/udev/rules.d/usb_prototype_devices.rules. Do so by adding the following line--\n"
                "SUBSYSTEM==\"usb\", ATTRS{{idVendor}}==\"{:04x}\", ATTRS{{idProduct}}==\"{:04x}\", MODE==\"0666\"\n\n".format(self.vendor_id, self.product_id))
            raise(e)

    def close(self):
        self.dev = None

    @staticmethod
    def parse16(ret, start_index):
        return int(ret[start_index])+int(ret[start_index + 1])*256

    @staticmethod
    def parse32(ret, start_index):
        return int(ret[start_index]) + int(ret[start_index + 1]) * 2**8 + int(ret[start_index + 2]) * 2**16 + int(ret[start_index + 3]) * 2**24

    @staticmethod
    def parse_gen(ret, start_index, end_index):
        running_sum = 0 
        selection = ret[start_index:end_index + 1]
        for i, val in enumerate(selection):
            running_sum += int(val) * (2 ** (8* i))
        return running_sum

    # Handlers
    def set_state(self, state):
        try:
            self.dev.ctrl_transfer(0x40, self.SET_STATE, int(state))
        except usb.core.USBError:
            print "Could not send SET_STATE vendor request."

    def get_vals(self):
        """

        """
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.GET_VALS, 0, 0, 12)
        except usb.core.USBError:
            print "Could not send GET_VALS vendor request."
        else:
            return [int(ret[0])+int(ret[1])*256, int(ret[2])+int(ret[3])*256,
            int(ret[4])+int(ret[5])*256,int(ret[6])+int(ret[7])*256,
            int(ret[8])+int(ret[9])*256, int(ret[10])+int(ret[11])*256]

    def debug_uart_buffers(self):
        """
        Reads information about the head, tail, and count of the transmit ("tx")
        and receive ("rx") software UART buffers.
        """
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.DEBUG_UART_BUFFERS, 0, 0, 12)
        except usb.core.USBError:
            print "Could not send DEBUG_UART_BUFFERS vendor request."
        else:
            txbuf = {}
            rxbuf = {}
            out = {}
            txbuf["head"] = self.parse16(ret, 0)
            txbuf["tail"] = self.parse16(ret, 2)
            txbuf["count"] = self.parse16(ret, 4)

            rxbuf["head"] = self.parse16(ret, 6)
            rxbuf["tail"] = self.parse16(ret, 8)
            rxbuf["count"] = self.parse16(ret, 10)
            out["tx"] = txbuf
            out["rx"] = rxbuf
            return out

    def get_rocket_info(self):
        """
        Reads the rocket's current measured tilt, measured speed, and state.
        """
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.GET_ROCKET_INFO, 0, 0, 6)
        except usb.core.USBError:
            print "Could not send GET_ROCKET_INFO vendor request."
        else:
            out = {}
            out["tilt"] = self.parse16(ret, 0)
            out["speed"] = self.parse16(ret, 2)
            out["state"] = self.parse16(ret, 4)
            return out

    def get_quad_info(self):
        """
        Reads the latest data from the DC motor's quadrature encoder. Returned
        is the counter (4 bytes) and an overflow/underflow counter.
        """
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.GET_QUAD_INFO, 0, 0, 8)
        except usb.core.USBError:
            print "Could not send GET_QUAD_INFO vendor request."
        else:
            print ret
            out = {}
            out["counter"] = self.parse32(ret, 0)
            out["overflow"] = self.parse16(ret, 4)
            out["diff"] = self.parse16(ret, 6)
            return out

    def command_dcmotor(self, speed, direction):
        try:
            self.dev.ctrl_transfer(0x40, self.COMMAND_DCMOTOR, int(speed), int(direction))
        except usb.core.USBError:
            print "Could not send COMMAND_DCMOTOR vendor request."

    def debug_servo_set_pos(self, pos):
        try:
            self.dev.ctrl_transfer(0x40, self.DEBUG_SERVO_SET_POS, int(pos))
        except usb.core.USBError:
            print "Could not send DEBUG_SERVO_SET_POS vendor request."

    def debug_servo_set_freq(self, freq):
        shift_freq = freq * 10
        try:
            self.dev.ctrl_transfer(0x40, self.DEBUG_SERVO_SET_FREQ, int(shift_freq))
        except usb.core.USBError:
            print "Could not send DEBUG_SERVO_SET_FREQ vendor request."

    def debug_servo_sleep(self):
        try:
            self.dev.ctrl_transfer(0x40, self.DEBUG_SERVO_SLEEP, 0)
        except usb.core.USBError:
            print "Could not send DEBUG_SERVO_SLEEP vendor request."

    def debug_servo_wake(self):
        try:
            self.dev.ctrl_transfer(0x40, self.DEBUG_SERVO_WAKE, 0)
        except usb.core.USBError:
            print "Could not send DEBUG_SERVO_WAKE vendor request."

    def debug_servo_reset(self):
        try:
            self.dev.ctrl_transfer(0x40, self.DEBUG_SERVO_RESET, 0)
        except usb.core.USBError:
            print "Could not send DEBUG_SERVO_RESET vendor request."