from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import List
from .state import FlightLogEntry


class FlightLogger:
    def __init__(self):
        self.entries: List[FlightLogEntry] = []

    def append(self, entry: FlightLogEntry) -> None:
        self.entries.append(entry)

    def write_jsonl(self, path: str | Path) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", encoding="utf-8") as f:
            for e in self.entries:
                f.write(json.dumps(asdict(e)) + "\n")
