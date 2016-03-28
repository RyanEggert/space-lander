import usb_vendor as h
test = h.hellousb()

while(1):
	[TXhead, TXtail, TXcount, RXhead, RXtail, RXcount] = test.get_vals()
	print "TX_head {} | TX_tail {} | TX_count {} || RX_head {} | RX_tail {} | RX_count {}".format(TXhead, TXtail, TXcount, RXhead, RXtail, RXcount)