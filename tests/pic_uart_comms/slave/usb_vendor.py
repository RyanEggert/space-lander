import usb.core

class PIC_USB(object):

    def __init__(self):
        super(PIC_USB, self).__init__()
        self.SET_STATE = 0
        self.GET_VALS = 1
        self.GET_ROCKET_INFO = 2
        self.DEBUG_UART_BUFFERS = 3

        self.dev = usb.core.find(idVendor = 0x6666, idProduct = 0x0003)
        if self.dev is None:
            raise ValueError('no USB device found matching idVendor = 0x6666 and idProduct = 0x0003')
        self.dev.set_configuration()

    def close(self):
        self.dev = None
    @staticmethod
    def parse16(ret, start_index):
        return int(ret[start_index])+int(ret[start_index + 1])*256

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
