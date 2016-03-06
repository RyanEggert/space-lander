import joystick as h
import matplotlib.pyplot as plt
import csv
import numpy as np
import time


test = h.hellousb()
res = []
counter = 0
start = time.time()

while(counter != 4000):
	[val1,val2] = test.get_vals()
	# convert int to float
	frac=0
	for i in range(0,7):
		check = (val2 >> i) & 1
		den = 2.0**(7-i)
		# print i, check, bin(den)
		if (check):
			frac += 1.0/den

	frac = val2 & 127
	whole = val2 >> 7
	valShift = val2
	# append whole number + fraction
	res.append(whole)
	counter += 1

end = time.time()
elapsed = end - start
time_vec = np.linspace(0,elapsed,3000)
x = np.array(res)
deriv = np.diff(x)
# plt.plot(time_vec[1:],deriv)
plt.plot(time_vec,x)
plt.title('Virtual Wall')
plt.ylabel('Shaft Position (deg)')
plt.xlabel('Time (s)')
plt.show()
