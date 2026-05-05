"""Temporal filtering utilities for stable gesture decisions."""

from collections import deque
from typing import Optional


class MajorityVoteFilter:
    """Return the dominant label in a recent history window."""

    def __init__(self, window_size: int = 5, min_count: int = 3) -> None:
        self.window_size = max(1, window_size)
        self.min_count = max(1, min_count)
        self._history: deque[str] = deque(maxlen=self.window_size)

    def reset(self) -> None:
        self._history.clear()

    def update(self, label: str) -> str:
        self._history.append(label)
        if not self._history:
            return label

        counts: dict[str, int] = {}
        for item in self._history:
            counts[item] = counts.get(item, 0) + 1

        best_label = max(counts, key=counts.get)
        if counts[best_label] >= self.min_count:
            return best_label
        return label

    def dominant_label(self) -> Optional[str]:
        if not self._history:
            return None
        counts: dict[str, int] = {}
        for item in self._history:
            counts[item] = counts.get(item, 0) + 1
        return max(counts, key=counts.get)
