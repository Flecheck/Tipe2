import numpy as np
import matplotlib.pyplot as plt

data = np.fromfile("output/first.bin", dtype=np.float32)
to_show = (len(data) - 1) // 4

plt.figure()
plt.plot(range(to_show), data[:to_show])
plt.show()