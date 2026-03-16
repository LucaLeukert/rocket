from __future__ import annotations

import json
from pathlib import Path
from typing import Iterator


def replay_jsonl(path: str | Path) -> Iterator[dict]:
    with Path(path).open("r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                yield json.loads(line)
