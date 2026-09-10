from dataclasses import asdict
from collections import deque
import csv
from pathlib import Path
import threading

from .telemetry import TelemetrySample


class TraceBuffer:
    def __init__(self, maxlen: int = 5000):
        if maxlen <= 0:
            raise ValueError("maxlen must be positive")
        self._samples = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def append(self, sample: TelemetrySample) -> None:
        with self._lock:
            self._samples.append(sample)

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()

    def snapshot(self) -> list[TelemetrySample]:
        with self._lock:
            return list(self._samples)

    def write_csv(self, path: str | Path) -> None:
        rows = self.snapshot()
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = list(TelemetrySample.__dataclass_fields__)
        with destination.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for sample in rows:
                writer.writerow(asdict(sample))