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

    def format(self) -> str:
        vel = round(self.velocity_mps, 8)
        pitch = round(self.pitch_rad, 8)
        time = round(self.time_secs, 8)
        return f"entry({vel}, new ShotMapResult({pitch}, {time}))"
