import math as ma
import matplotlib.pyplot as plt

length = 10
precision = 100

pi = ma.pi

arr_len = length * precision

time = [x / precision for x in range(arr_len * 2)]
sin1 = [ma.sin(2 * pi * x / (precision * length)) for x in range(arr_len)]
sin2 = [ma.sin(4 * pi * x / (precision * length)) for x in range(arr_len)]

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(time, sin1 + sin1)
plt.subplot(2, 1, 2)
plt.plot(time, sin2 + sin2)
plt.show()
