import numpy as np
import matplotlib.pyplot as plt

data1 = np.fromfile("output/battements_with.bin", dtype=np.float32)
data2 = np.fromfile("output/battements_without.bin", dtype=np.float32)
to_show = (len(data1) - 1) // 8

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(range(to_show), data1[:to_show])
plt.title("Avec cube")
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(range(to_show), data2[:to_show])
plt.title("Sans cube")
plt.legend()

plt.show()