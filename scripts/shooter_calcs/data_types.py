from dataclasses import dataclass
from sleipnir.autodiff import VariableMatrix


@dataclass
class ShooterSolution:
    """
    Represents a completed sleipnir solve.
    """
    velocity_mps: float
    pitch_rad: float
    time_secs: float
    X: VariableMatrix
