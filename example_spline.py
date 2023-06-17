from spline.spline import Spline
import matplotlib.pyplot as plt
import numpy as np

a = [0, 3]
b = [0.5, 10]
c = [1, 5]
d = [1.5, 7]

p = [a, b, c, d]

s = Spline(p, precision=4)
sol = s.calculate()

print(sol)

curve_resolution = 0.01

x = []
y = []

# from point b to point c
for i in np.arange(a[0], b[0], curve_resolution):

    x.append(i)
    y.append(sol[0][0] + sol[0][1]*i + sol[0][2]*i**2 + sol[0][3]*i**3)

# from point b to point c
for i in np.arange(b[0], c[0], curve_resolution):

    x.append(i)
    y.append(sol[1][0] + sol[1][1]*i + sol[1][2]*i**2 + sol[1][3]*i**3)

# from point c to point d
for i in np.arange(c[0], d[0], curve_resolution):

    x.append(i)
    y.append(sol[2][0] + sol[2][1]*i + sol[2][2]*i**2 + sol[2][3]*i**3)

plt.plot(x, y)
plt.plot(a[0], a[1], marker="o", color="red")
plt.plot(b[0], b[1], marker="o", color="red")
plt.plot(c[0], c[1], marker="o", color="red")
plt.plot(d[0], d[1], marker="o", color="red")
plt.show()