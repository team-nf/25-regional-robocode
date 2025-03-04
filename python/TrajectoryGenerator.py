import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from numpy.polynomial.polynomial import Polynomial

import json

# Zaman vektörü
t = np.linspace(0, 1, 100)

# Kullanıcının belirlediği x ve y hedef noktaları
x_points = [1, 2, 3, 4]
y_points = [5, 6, 8, 16]
phi_points = [0.698, 0.9, 1.2, 1.57]

# Zaman noktaları
t_points = np.linspace(0, 1, len(x_points))

# 5. dereceden polinom ile fit etme
poly_x = Polynomial.fit(t_points, x_points, 5)
poly_y = Polynomial.fit(t_points, y_points, 5)

# x ve y değerlerini hesapla
x_vals = poly_x(t)
y_vals = poly_y(t)

# Robot kol parametreleri
L = 5  # Kol uzunluğu

# Açılar ve yükseklik hesaplama
theta_vals = np.arccos(x_vals/L)
h_vals = y_vals - np.sin(theta_vals)*L

poly_phi = Polynomial.fit(t_points, phi_points, 5)
phi_vals = poly_phi(t)

# Animasyon çizimi
fig, ax = plt.subplots()
ax.set_xlim(0, 16)
ax.set_ylim(0, 16)
line, = ax.plot([], [], 'ro-', lw=3)
path, = ax.plot([], [], 'b--', lw=1)  # This will plot the path
path_x, path_y = [], []  # Store the path coordinates

# Initialize function
def init():
    line.set_data([], [])
    path.set_data([], [])
    return line, path

def update(frame):
    theta = theta_vals[frame]
    h = h_vals[frame]
    phi = phi_vals[frame]
    
    # Robot kolunun uç noktasını hesapla
    x_end = L * np.cos(theta)
    y_end = L * np.sin(theta) + h
    
    line.set_data([0, x_end], [h, y_end])
    # Append current position to the path
    path_x.append(x_end)
    path_y.append(y_end)

    
    # Update the path plot
    path.set_data(path_x, path_y)
    print(np.sqrt(x_end**2 + (y_end-h)**2))
    return line, path

def post():
    format = {}
    format["theta"] = {f"{i}": theta for i, theta in enumerate(theta_vals)} 
    format["h"] = {f"{i}": h for i, h in enumerate(h_vals)}
    format["phi"] = {f"{i}": phi for i, phi in enumerate(phi_vals)}

    with open('trajectory.json', 'w') as f:
        json.dump(format, f, separators=(',\n', ': \n'))

ani = animation.FuncAnimation(fig, update, frames=len(t), init_func=init, blit=True, interval=50)
post()
plt.show()
