import joystick as h
import matplotlib.pyplot as plt
import csv
import numpy as np
import time
from scipy.signal import butter, lfilter, freqz


test = h.hellousb()
test.set_vals(0)
shaftang = []
current = []
counter = 0
start = time.time()

while(counter != 4000):
	[val1,val2] = test.get_vals()
	shaftang.append(val1 >> 7)
	current.append((1.65 - (val2*3.3/(2**10)))/.75)
	counter += 1

end = time.time()
elapsed = end - start
time_vec = np.linspace(0,elapsed,4000)
x = np.array(shaftang)
x2 = np.array(current)
# plt.plot(time_vec[1:],deriv)

# def butter_lowpass(cutoff, order=5):
#     normal_cutoff = cutoff / nyq
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     return b, a

# def butter_lowpass_filter(data, cutoff, order=5):
#     b, a = butter_lowpass(cutoff, order=order)
#     y = lfilter(b, a, data)
#     return y

# filt = butter_lowpass_filter(x2,150)


plt.subplot(2, 1, 1)
plt.plot(time_vec,x)
plt.title('Spring: Displacement')
plt.ylabel('Shaft Position (deg)')
plt.xlabel('Time (s)')

plt.subplot(2, 1, 2)
plt.plot(time_vec,x2)
plt.title('Spring: Current')
plt.ylabel('Current (A)')
plt.xlabel('Time (s)')
plt.show()
