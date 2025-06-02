from typing import Tuple, List, Float

type Configuration = List[Float]
type Velocities = List[Float]
type Accelerations = List[Float]
type Torques = List[Float]
type FeetForces = List[Float]

class AbstractSimulatorWrapper:
    def step(tau: Torques) -> Tuple[Configuration, Velocities, Accelerations, FeetForces]:
        raise NotImplementedError