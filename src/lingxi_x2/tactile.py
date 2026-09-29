"""Export raw sensor grids without claiming calibrated pressure or orientation."""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict
import json
from pathlib import Path

from PIL import Image, ImageDraw

from .models import Side, TactileFrame


def save_tactile_map(frames: Mapping[Side, TactileFrame], output: Path) -> dict[str, object]:
    image = Image.new("RGB", (1120, 420), "#101827")
    draw = ImageDraw.Draw(image)
    draw.text((16, 12), "OmniHand raw_uint8 | fixed scale 0..255 | array order, physical orientation unverified", fill="white")
    nonzero = 0
    peak = 0
    for row, side in enumerate(Side):
        frame = frames[side]
        y = 48 + row * 175
        draw.text((16, y), f"{side.value} | received monotonic_ns={frame.timestamp.monotonic_ns}", fill="white")
        surfaces = [frame.palm, frame.back_of_hand, *frame.fingertips.values()]
        for column, surface in enumerate(surfaces):
            height, width = surface.shape
            if len(surface.values) != width * height or any(not 0 <= value <= 255 for value in surface.values):
                raise ValueError(f"Invalid raw tactile surface: {side.value}/{surface.name}")
            x = 16 + column * 156
            draw.text((x, y + 20), surface.name, fill="white")
            for index, value in enumerate(surface.values):
                nonzero += int(value != 0)
                peak = max(peak, value)
                cell_x, cell_y = x + (index % width) * 18, y + 40 + (index // width) * 18
                color = (int(30 + value * .88), int(42 + value * .5), int(65 - value * .2))
                draw.rectangle((cell_x, cell_y, cell_x + 16, cell_y + 16), fill=color)
    draw.text((16, 400), f"nonzero cells: {nonzero} | peak: {peak}/255 | sensor response not established by a snapshot", fill="white")
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, format="PNG")
    metadata = {
        "units": "raw_uint8", "physical_orientation_verified": False,
        "nonzero_cells": nonzero, "peak": peak,
        "frames": {side.value: asdict(frame) for side, frame in frames.items()},
    }
    sidecar = output.with_suffix(".json")
    sidecar.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return {"image": str(output), "data": str(sidecar), "units": "raw_uint8", "nonzero_cells": nonzero, "peak": peak}
