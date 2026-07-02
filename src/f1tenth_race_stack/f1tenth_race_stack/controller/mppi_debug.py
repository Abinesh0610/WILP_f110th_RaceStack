import numpy as np
from mppi_controller import MPPIController

import rclpy
rclpy.init()
c = MPPIController()

# Set mock state
c._state = np.array([0.0, 0.0, 0.0, 1.0])
c._obstacle_points = np.array([[5.0, 5.0]]) # Far away
c._racing_line = np.array([[i*0.1, 0.0, 0.0] for i in range(100)])
c.u_sequence = np.zeros((c.T, 2))

c._mppi_loop()
print("Speed action:", c.u_sequence[0,0])
print("Steer action:", c.u_sequence[0,1])
rclpy.shutdown()
