import numpy as np
import matplotlib.pyplot as plt

one = np.fromfile("output/ofdm_rec.bin", dtype=np.float32)
two = np.fromfile("output/ofdm_emit.bin", dtype=np.float32)
to_show = len(one) // 12

fft = np.imag(np.fft.fft(two[:2048]))
fft2 = np.imag(np.fft.fft(one[1903:1903+2048]))

def autolabel(rects):
    """
    Attach a text label above each bar displaying its height
    """
    for rect in rects:
        color = "white"
        if rect.get_height() < 0:
            if rect.get_height() > -0.5: color = "black"
            plt.text(rect.get_x() + rect.get_width()/2, min(rect.get_height()/2, -0.5), "1", ha='center', va='center', color=color)
        else:
            if rect.get_height() < 0.5: color = "black"
            plt.text(rect.get_x() + rect.get_width()/2, max(rect.get_height()/2, 0.5), "0", ha='center', va='center', color=color)

plt.figure()
plt.subplots_adjust(hspace=0.3)
plt.subplot(2, 2, 2)
plt.plot(range(to_show), one[1903:1903+to_show])
plt.title("Signal reçu")
plt.xlabel("Temps")
plt.legend()
plt.subplot(2, 2, 1)
plt.plot(range(to_show), two[:to_show])
plt.title("Signal émis")
plt.xlabel("Temps")
plt.legend()
plt.subplot(2, 2, 3)
autolabel(plt.bar([x+1 for x in range(8)], fft[1:9], width=0.8))
plt.title("Parties réelles du premier symbole émis")
plt.xlabel("Harmoniques")
plt.legend()
plt.subplot(2, 2, 4)
autolabel(plt.bar([x+1 for x in range(8)], fft2[1:9], width=0.8))
plt.title("Parties réelles du premier symbole reçu")
plt.xlabel("Harmoniques")
plt.legend()
plt.show()