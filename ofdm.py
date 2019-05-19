import numpy as np
import matplotlib.pyplot as plt

one = np.fromfile("output/ofdm_rec.bin", dtype=np.float32)
two = np.fromfile("output/ofdm_emit.bin", dtype=np.float32)
to_show = len(one) // 16

plt.figure()
plt.subplot(1, 2, 2)
plt.plot(range(to_show), one[:to_show] * 1000, label="output")
plt.legend()
plt.subplot(1, 2, 1)
plt.plot(range(to_show), two[:to_show], label="input")
plt.legend()
plt.show()