
import usb.core

class hellousb:

    def __init__(self):
        self.SET_STATE = 0
        self.GET_VALS = 1
        self.GET_ROCKET_INFO = 2

        self.dev = usb.core.find(idVendor = 0x6666, idProduct = 0x0003)
        if self.dev is None:
            raise ValueError('no USB device found matching idVendor = 0x6666 and idProduct = 0x0003')
        self.dev.set_configuration()

    def close(self):
        self.dev = None

    def set_state(self, state):
        try:
            self.dev.ctrl_transfer(0x40, self.SET_STATE, int(state))
        except usb.core.USBError:
            print "Could not send SET_VALS vendor request."

    def get_vals(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.GET_VALS, 0, 0, 12)
        except usb.core.USBError:
            print "Could not send GET_VALS vendor request."
        else:
            return [int(ret[0])+int(ret[1])*256, int(ret[2])+int(ret[3])*256,
            int(ret[4])+int(ret[5])*256,int(ret[6])+int(ret[7])*256,
            int(ret[8])+int(ret[9])*256, int(ret[10])+int(ret[11])*256]

    def get_rocket_info(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.GET_ROCKET_INFO, 0, 0, 6)
        except usb.core.USBError:
            print "Could not send GET_ROCKET_INFO vendor request."
        else:
            return [int(ret[0])+int(ret[1])*256, int(ret[2])+int(ret[3])*256,
            int(ret[4])+int(ret[5])*256]
