"""Export raw sensor grids without claiming calibrated pressure or orientation."""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict
import json
from pathlib import Path

from PIL import Image, ImageDraw

from .models import Side, TactileFrame
from .tactile_quality import validate_frame, FINGERS


def save_tactile_map(frames: Mapping[Side, TactileFrame], output: Path) -> dict[str, object]:
    output = Path(output)
    sidecar = output.with_suffix('.json')
    if output == sidecar or output.exists() or sidecar.exists() or output.is_symlink() or sidecar.is_symlink():
        raise FileExistsError('Refusing to overwrite tactile evidence')
    if set(frames) != set(Side):
        raise ValueError('Both tactile sides required for PNG export; missing is not zero')
    for side, frame in frames.items(): validate_frame(asdict(frame), side.value)
    synthetic_sides = [s.value for s, f in frames.items() if f.source.startswith(('mock/', 'synthetic/'))]
    image = Image.new("RGB", (1120, 420), "#101827")
    draw = ImageDraw.Draw(image)
    draw.text((16, 12), "OmniHand raw_uint8 | fixed scale 0..255 | array order, physical orientation unverified", fill="white")
    if synthetic_sides: draw.text((16, 28), f"SYNTHETIC DATA: {', '.join(synthetic_sides)} | not physical contact evidence", fill="#ffd166")
    nonzero = 0
    peak = 0
    ceiling = 0
    for row, side in enumerate(Side):
        frame = frames[side]
        y = 48 + row * 175
        draw.text((16, y), f"{side.value} | received monotonic_ns={frame.timestamp.monotonic_ns}", fill="white")
        surfaces = [frame.palm, frame.back_of_hand, *[frame.fingertips[f] for f in FINGERS]]
        for column, surface in enumerate(surfaces):
            height, width = surface.shape
            if len(surface.values) != width * height or any(not 0 <= value <= 255 for value in surface.values):
                raise ValueError(f"Invalid raw tactile surface: {side.value}/{surface.name}")
            x = 16 + column * 156
            draw.text((x, y + 20), surface.name, fill="white")
            for index, value in enumerate(surface.values):
                nonzero += int(value != 0)
                ceiling += int(value == 255)
                peak = max(peak, value)
                cell_x, cell_y = x + (index % width) * 18, y + 40 + (index // width) * 18
                color = (int(30 + value * .88), int(42 + value * .5), int(65 - value * .2))
                draw.rectangle((cell_x, cell_y, cell_x + 16, cell_y + 16), fill=color)
    draw.text((16, 400), f"nonzero cells: {nonzero} | peak: {peak}/255 | sensor response not established by a snapshot", fill="white")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream: image.save(stream, format="PNG")
    metadata = {
        "units": "raw_uint8", "physical_orientation_verified": False,
        "nonzero_cells": nonzero, "peak": peak,
        "synthetic_sides": synthetic_sides, "raw_ceiling_cells": ceiling,
        "physical_saturation_verified": False, "tactile_contact_verified": False,
        "frames": {side.value: asdict(frame) for side, frame in frames.items()},
    }
    with sidecar.open('x', encoding='utf-8') as stream: json.dump(metadata, stream, indent=2)
    return {"image": str(output), "data": str(sidecar), "units": "raw_uint8", "nonzero_cells": nonzero, "peak": peak}
