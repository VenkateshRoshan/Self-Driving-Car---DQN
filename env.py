import pybullet as p
import pybullet_data
import numpy as np
import time

# Initialize pybullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Create a class for the environment
class CarEnv:
    def __init__(self):
        self.planeId = p.loadURDF("plane.urdf")
        self.startPos = [0, 0, 0.3]
        self.startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.carId = p.loadURDF("urdf/car.urdf", self.startPos, self.startOrientation)
        self.wheel_indices = [0, 1, 2, 3]  # Assuming these are the wheel joint indices
        p.setRealTimeSimulation(1)

    def reset(self):
        p.resetSimulation()
        self.planeId = p.loadURDF("plane.urdf")
        self.carId = p.loadURDF("urdf/car.urdf", self.startPos, self.startOrientation)
        return np.array(p.getBasePositionAndOrientation(self.carId)[0])

    def step(self, action):
        self.set_wheel_velocities(action)
        p.stepSimulation()
        return np.array(p.getBasePositionAndOrientation(self.carId)[0])

    def set_wheel_velocities(self, velocity):
        for wheel_index in self.wheel_indices:
            p.setJointMotorControl2(self.carId, wheel_index, p.VELOCITY_CONTROL, targetVelocity=velocity, force=1000)

    def render(self):
        pass

    def close(self):
        p.disconnect()

if __name__ == '__main__':
    # Initialize the environment
    env = CarEnv()
    # Reset the environment
    env.reset()
    # Loop for 1000 steps
    for i in range(1000):
        # Set a constant velocity for the wheels
        velocity = 10.0  # Set the desired wheel velocity
        # Take a step in the environment
        next_state = env.step(velocity)
        # Sleep for 0.01s
        time.sleep(0.01)
    # Close the environment
    env.close()
