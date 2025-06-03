from typing import Tuple, List

Configuration = List[float]
Velocities = List[float]
Accelerations = List[float]
Torques = List[float]
FeetForces = List[float]

class AbstractSimulatorWrapper:
    def step(tau: Torques) -> Tuple[Configuration, Velocities, Accelerations, FeetForces]:
        raise NotImplementedError