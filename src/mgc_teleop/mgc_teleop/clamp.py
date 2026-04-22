"""
clamp.py
===========

This module defines the Clamp class.
It checks whether the passed value is within it's defined limits.
If not it will either return the lower limit or the upper limit of the value.
"""

from dataclasses import dataclass

@dataclass(frozen=True)
class Clamp:
    """
    Immutable numeric clamp.
    Used to constrain values safely within limits.
    """
    min: float
    max: float

    # Post initialisation method which ensure min <= max even if user passes wrong order
    def __post_init__(self):
        if self.min > self.max:
            object.__setattr__(self, "min", self.max)
            object.__setattr__(self, "max", self.min)
            # object.__setattr__ bypasses immutability. Allowed only inside initialization.

    # A method which returns a clamped value
    def clamp(self, value: float) -> float:
        return max(self.min, min(self.max, value))
