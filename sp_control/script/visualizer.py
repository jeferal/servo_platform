import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Define the desired system characteristics
desired_overshoot = 0  # No overshoot for critically damped response (ζ = 1)

m = 1.0  # Mass in kg
k = 4.0  # Spring constant in N/m
c = 2.0  # Damping coefficient in N/(m/s)

ωn = np.sqrt(k / m)
ζ = c / (2 * np.sqrt(k * m))

num = [ωn**2]
den = [1, 2 * ζ * ωn, ωn**2]
system = signal.TransferFunction(num, den)

# Time vector
t = np.linspace(0, 10, 1000)

# Simulate the system's response to a unit step input
t, y = signal.step(system, T=t)

# Plot the response
plt.figure()
plt.plot(t, y)
plt.title('Critically Damped Response')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.grid(True)
plt.show()
