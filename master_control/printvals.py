import usb_vendor as h
test = h.hellousb()

while(1):
	# [rockettilt, TXtail, TXcount, RXhead, RXtail, RXcount] = test.get_vals()
	# print "Rocket_Tilt {} | TX_tail {} | TX_count {} || RX_head {} | RX_tail {} | RX_count {}".format(rockettilt, TXtail, TXcount, RXhead, RXtail, RXcount)
    [r_tilt, r_speed, r_state] = test.get_rocket_info()
    print "Rocket Tilt {} | Rocket Speed {} | Rocket State {}".format(r_tilt, r_speed, r_state)