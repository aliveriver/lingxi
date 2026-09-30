"""Reproduce the 2026-09-29 source audit, offline, against frozen evidence.

Reads text/XML/ZIP only. Does not load the vendor SDK, infer a calibrated
HAL mapping, generate a combined URDF, or initialize a robot backend.
The ignored logs directory must be preserved or relocated with --root.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET
import zipfile


REFRESH = "logs/official-model-refresh-20260929/"
FOLLOWUP = "logs/model-adaptation-followup-20260929/"
SOURCES = {
    "left": (REFRESH + "OmniHandleft3.urdf", "8c33ee9e9e9ce08cb9e379618d7e06f07791bf65a159e36f6481dd4e4d75e468"),
    "right": (REFRESH + "OmniHandright4.urdf", "5acb477166ae7b1211d2a8378fd1593e59e7742aa26f4f4578c0ba01bda9e952"),
    "sdk_header": (FOLLOWUP + "blob-omnihand_2025_solver.h", "dec33005af5e3f8143e89adb80ae04843b66527be989764512a810867754a979"),
    "sdk_api": (FOLLOWUP + "blob-API_CPP_O10.md", "c965e46a2a76f0aa2597f5dde8b6c0a60356fcb8a0411eafa653396c5a26e85c"),
    "robot_archive": (FOLLOWUP + "legacy-official-models.zip", "aeb0155a372111a7421397883f8570ca6b291908ee6e39214381b2a3652f48c2"),
    "trace": ("logs/20260929-gravity-live-bias-001-confirmed.json", "638a088ceee0b311bae28824dc3440b17104070d406fade36161e4164b8a1631"),
}
SUFFIXES = ("thumb_roll", "thumb_abad", "thumb_mcp", "index_abad", "index_pip",
            "middle_pip", "ring_abad", "ring_pip", "pinky_abad", "pinky_pip")


def audit(root: Path) -> dict:
    data = {}
    for key, (relative, expected) in SOURCES.items():
        raw = (root / relative).read_bytes()
        if hashlib.sha256(raw).hexdigest() != expected:
            raise ValueError(f"Source hash mismatch: {relative}; review a new version separately")
        data[key] = raw

    api = data["sdk_api"].decode("utf-8-sig")
    header = data["sdk_header"].decode("utf-8-sig")
    hands = json.loads(data["trace"])["result"]["final_hands"]
    sides = {}
    for side, api_prefix, urdf_prefix in (("left", "L", "l"), ("right", "R", "R")):
        model = ET.fromstring(data[side])
        api_rows = re.findall(
            rf"^\|\s*(\d+)\s*\|\s*({api_prefix}_\w+_joint)\s*\|\s*([-\d.]+)\s*\|\s*([-\d.]+)\s*\|",
            api, re.MULTILINE)
        if len(api_rows) != 10:
            raise ValueError(f"Expected exactly ten SDK table rows for {side}")
        rows = []
        for index, (table_index, name, low, high) in enumerate(api_rows):
            suffix = SUFFIXES[index] + "_joint"
            if int(table_index) != index + 1 or name != api_prefix + "_" + suffix:
                raise ValueError("Unexpected SDK table order")
            joint = model.find(f"joint[@name='{urdf_prefix}_{suffix}']")
            bounds = [float(joint.find("limit").get(key)) for key in ("lower", "upper")]
            sdk_bounds = [float(low), float(high)]
            hal = hands[side]["joints"][index]["position_rad"]
            rows.append({
                "hal_index_zero_based": index, "sdk_table_index_one_based": int(table_index),
                "urdf_joint": joint.get("name"), "sdk_documented_joint": name,
                "urdf_bounds_rad": bounds, "sdk_documented_bounds_rad": sdk_bounds,
                "historical_hal_rad": hal,
                "raw_hal_in_urdf_interval": bounds[0] <= hal <= bounds[1],
                "raw_hal_in_sdk_documented_interval": sdk_bounds[0] <= hal <= sdk_bounds[1],
                "sdk_and_urdf_bounds_numerically_equal": bounds == sdk_bounds,
            })
        mimics = [{"joint": j.get("name"), **j.find("mimic").attrib}
                  for j in model.findall("joint") if j.find("mimic") is not None]
        sides[side] = {
            "rows": rows, "urdf_mimics": mimics,
            "raw_hal_outside_urdf_count": sum(not r["raw_hal_in_urdf_interval"] for r in rows),
            "raw_hal_outside_sdk_documented_count": sum(not r["raw_hal_in_sdk_documented_interval"] for r in rows),
        }

    declarations = {}
    for name in ("kLeftDirection", "finger_pip2dip_poly_", "thumb_mcp2pip_poly_", "thumb_mcp2dip_poly_"):
        matches = re.findall(r"\b" + re.escape(name) + r"\s*=\s*\{([^}]+)\}", header)
        if len(matches) != 1:
            raise ValueError(f"Expected one declaration of {name}")
        # Numeric literals only, never evaluate C++ or import an SDK binary.
        declarations[name] = [float(v.strip()) for v in matches[0].split(",") if v.strip()]
    enum_match = re.search(r"enum OmnihandJoint\s*\{(.*?)\};", header, re.DOTALL)
    enum_order = re.findall(r"^\s*(Joint\w+)\s*(?:=\s*\d+)?\s*,", enum_match[1], re.MULTILINE)

    inventory = []
    with zipfile.ZipFile(io.BytesIO(data["robot_archive"])) as archive:
        for member in archive.namelist():
            if not member.endswith(".urdf"):
                continue
            raw = archive.read(member)
            model = ET.fromstring(raw)
            mounts = {}
            for side in ("left", "right"):
                mounts[side] = [{"joint": j.get("name"), "child": j.find("child").get("link"),
                                 "origin": j.find("origin").attrib}
                                for j in model.findall("joint")
                                if j.find("parent").get("link") == side + "_wrist_roll_link"]
            inventory.append({"member": member, "sha256": hashlib.sha256(raw).hexdigest(),
                              "wrist_children": mounts})

    return {
        "kind": "frozen_o10_source_contract_audit", "schema_version": 1,
        "robot_connected": False, "sdk_executed": False, "hardware_validated": False,
        "execution_authorized": False, "combined_model_ready": False,
        "sdk_commit": "c23801f03e396d8d667a307548a05b5d0aecf9aa",
        "sources": {k: {"path": p, "sha256": h} for k, (p, h) in SOURCES.items()},
        "historical_numeric_interval_comparison": sides,
        "sdk_header_declarations_only": declarations,
        "sdk_solver_enum_order_not_verified_api_return_order": enum_order,
        "robot_archive_models": inventory,
        "unknowns": [
            "X2 Ultra v1.3 wrist-to-palm transforms for both installed hands",
            "Per-axis HAL-to-standalone-URDF sign/zero mapping and validity range",
            "Installed hand revision and SDK/firmware relationship to Agi v1.1.4",
            "Actual passive-joint coupling and SDK API return ordering",
            "Wrist/adapter/hand inertial component accounting",
            "HAL calibration application and IMU matrix multiplication direction",
        ],
        "limitations": [
            "Interval membership is not a coordinate mapping or physical limit diagnosis.",
            "SDK table is indexed 1..10; HAL arrays are indexed 0..9.",
            "SDK header declarations do not establish runtime behavior or polynomial evaluation convention.",
            "No calibration, payload, command, combined URDF or motion acceptance is generated.",
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, required=True, help="New report path; refuses overwrite")
    args = parser.parse_args(argv)
    try:
        report = audit(args.root)
        serialized = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        with args.output.open("x", encoding="utf-8") as output:
            output.write(serialized)
    except (OSError, ValueError, KeyError, ET.ParseError, zipfile.BadZipFile) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"output": str(args.output), "combined_model_ready": False,
                      "unknowns": report["unknowns"]}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
