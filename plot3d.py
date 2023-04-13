import numpy as np
import matplotlib.pyplot as plt
import time
import sys

fig = plt.figure()
ax = plt.axes(projection='3d')


i = 0
ax = plt.gca()

for line in sys.stdin:
	if i == 0:
		x = float(line)
	if i == 1:
		y = float(line)
	if i == 2:
		z = float(line)
		ax.scatter(x, y, z)

	i += 1
	i %= 3

plt.show()
