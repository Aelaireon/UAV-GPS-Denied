#!/usr/bin/env python3
"""Base class for all frame processors."""
from abc import ABC, abstractmethod
import numpy as np


class FrameProcessor(ABC):
    """
    Inherit from this to create a new processing task.

    Each processor receives a frame, does its work, and returns
    an (optionally annotated) frame plus a results dict.
    """

    def __init__(self, name: str):
        self.name = name
        self.enabled = True

    @abstractmethod
    def process(self, frame: np.ndarray) -> tuple[np.ndarray, dict]:
        """
        Process a single frame.

        Args:
            frame: BGR numpy array from OpenCV

        Returns:
            (annotated_frame, results_dict)
            - annotated_frame: frame with drawings/overlays (can be same object)
            - results_dict: any data you want available to other processors
                           or the main loop (e.g. {"flow": ..., "markers": [...]})
        """

    def reset(self):
        """Optional: reset internal state (e.g. between runs)."""

    def __repr__(self):
        return f"{self.__class__.__name__}(name={self.name!r}, enabled={self.enabled})"