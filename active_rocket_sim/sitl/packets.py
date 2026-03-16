from __future__ import annotations

import json
from dataclasses import asdict
from active_rocket_sim.core.state import CommandVector, MeasurementPacket


def encode_measurement(pkt: MeasurementPacket) -> bytes:
    return json.dumps(asdict(pkt)).encode("utf-8")


def decode_command(raw: bytes) -> CommandVector:
    data = json.loads(raw.decode("utf-8"))
    return CommandVector(**data)
