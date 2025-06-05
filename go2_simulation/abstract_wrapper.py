from typing import Tuple, List

Configuration = List[float]
Velocities = List[float]
Accelerations = List[float]
Torques = List[float]
FeetForces = List[float]

class AbstractSimulatorWrapper:
    def step(tau: Torques) -> Tuple[Configuration, Velocities, Accelerations, FeetForces]:
        """
        Take as input torque commands for each joint and ouptut the robot new state.
        Note: the base velocity and acceleration (the first 6 values of the Velocities and Accelerations vector) are expressed in the local frame of the robot base.
        """
        raise NotImplementedError