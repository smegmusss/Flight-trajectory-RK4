# Here a simple script to visualize the results of the simulator

import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('trajectory_data.csv')

plt.figure(figsize=(8, 5))
plt.plot(data['x'], data['y'], label='Trajectory with drag', color='red')
plt.title('Projectile Trajectory (RK4)')
plt.xlabel('Distance X (m)')
plt.ylabel('Altitude Y (m)')
plt.grid(True)
plt.legend()
plt.savefig('trajectory.png')
plt.show()
