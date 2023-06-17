from spline.spline import Spline, Spline2D
import matplotlib.pyplot as plt
import numpy as np


a = [0, 0]
b = [10, 20]
c = [15, 4]
d = [20, -5]
e = [15, 3]
f = [10, 3]
g = [5, 3]

# p = [a, b, c, d, e, f]
p = [a, b, c, d, e, f, g]

# para 2D, temos x(t) e y(t)

# logo, os pontos
# a = [0, 0]
# b = [-2, 2]
# c = [0, 4]
# sao representados como:

# ta = 0
# tb = 100
# tc = 200

# ax = [ta, 0]
# bx = [tb, -2]
# cx = [tc, 0]

# ay = [ta, 0]
# by = [tb, 2]
# cy = [tc, 4]

# px = [ax, bx, cx]
# py = [ay, by, cy]

# sx = Spline(px, resolution=0.01, precision=4)
# sy = Spline(py, resolution=0.01, precision=4)

# sx.calculate()
# sy.calculate()

s = Spline2D(p, 0.01, 1)
s.calculate()

# print(len(sx.points_spline_y))
# print(len(sy.points_spline_y))
# plt.plot(s.points_spline_x, s.points_spline_y, marker="o", markersize=0.01)
# plt.scatter(sx.points_spline_y, sy.points_spline_y, s=2)
plt.scatter(s.points_spline_x, s.points_spline_y, s=2)
plt.plot(a[0], a[1], marker="o", color="red")
plt.plot(b[0], b[1], marker="o", color="red")
plt.plot(c[0], c[1], marker="o", color="red")
plt.plot(d[0], d[1], marker="o", color="red")
plt.plot(e[0], e[1], marker="o", color="red")
plt.plot(f[0], f[1], marker="o", color="red")
plt.show()