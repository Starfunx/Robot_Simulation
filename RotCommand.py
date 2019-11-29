#coding utf-8

import numpy as np
import matplotlib.pyplot as plt

alpha = np.arange(-np.pi, np.pi, 0.01)

Kp = 15
Kalpha = 10

Command = Kp*np.sin(alpha)*np.cos(alpha) + Kalpha*alpha
Command2 = Kp*np.sin(2*alpha)/2

plt.plot(alpha, Command)
plt.plot(alpha, Command2)
plt.show()
