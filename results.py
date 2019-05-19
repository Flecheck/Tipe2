import numpy as np
import matplotlib.pyplot as plt

data = np.fromfile("output/first.bin", dtype=np.float32)
original = np.fromfile("output/third.bin", dtype=np.float32)
original2 = np.fromfile("output/second.bin", dtype=np.float32)
to_show = (len(data) - 1) // (8 * 32)

for i in range(to_show):
    original[i] = 0.0013 * (original[i] + original2[i])

plt.figure()
plt.plot(range(to_show), data[:to_show], label="result")
plt.plot(range(to_show), original[:to_show], label="original")
plt.legend()
plt.show()