from dataclasses import dataclass
from typing import Union, Literal

import numpy as np


@dataclass
class ShooterSolution:
    """
    Represents a completed sleipnir solve.
    """
    velocity_mps: float
    pitch_rad: float
    time_secs: float
    X: np.ndarray

    def format(self) -> str:
        vel = self.velocity_mps
        pitch = self.pitch_rad
        time = self.time_secs
        return f"entry({vel:.05f}, new ShotMapResult({pitch:.05f}, {time:.05f}))"


@dataclass
class MinVelocityWithPitch:
    """ A solve mode for sleipnir that constrains the pitch to a certain value.  """
    pitch_rad: float


# Represents a possible solve mode for sleipnir.
SolveMode = Union[MinVelocityWithPitch, Literal["min_velocity", "max_velocity"]]
