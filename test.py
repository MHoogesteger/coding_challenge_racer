import matplotlib.pyplot as plt
import numpy as np


s_r = 0.6
alpha_range = np.linspace(0, 2*np.pi,100) 
n_range = np.arange(0,100)

a_grid,n_grid = np.meshgrid(alpha_range,n_range)
plt.scatter(a_grid, ((1-s_r) * (a_grid + 2*n_grid*np.pi))%(2*np.pi))
plt.show()


