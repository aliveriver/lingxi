from __future__ import annotations

from contextlib import asynccontextmanager
from dataclasses import asdict
from pathlib import Path
import threading
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.responses import Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from ..client import X2Client
from ..models import ArmCommand, CameraName, HandCommand, Side


class ArmRequest(BaseModel):
    positions_rad: list[float] = Field(min_length=14, max_length=14)
    duration_s: float = Field(default=2.0, gt=0, le=30)
    confirmation: str


class HandRequest(BaseModel):
    side: Side
    positions_rad: list[float] = Field(min_length=10, max_length=10)
    duration_s: float = Field(default=1.0, gt=0, le=30)
    confirmation: str


def create_app(client: X2Client | None = None) -> FastAPI:
    hardware = client or X2Client()
    lock = threading.Lock()

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        hardware.start()
        yield
        hardware.close()

    app = FastAPI(title="Lingxi X2", version="0.1.0", lifespan=lifespan)

    @app.get("/api/status")
    def status() -> dict[str, Any]:
        return hardware.status().as_dict()

    @app.get("/api/state")
    def state() -> dict[str, Any]:
        arm = hardware.arm_state()
        hands = hardware.hand_states()
        tactile: dict[str, Any] = {}
        try:
            tactile = {side.value: asdict(value) for side, value in hardware.tactile_frames(0.2).items()}
        except Exception:
            pass
        return {
            "arm": asdict(arm),
            "hands": {side.value: asdict(value) for side, value in hands.items()},
            "tactile": tactile,
        }

    @app.get("/api/cameras/{camera}/frame")
    def frame(camera: CameraName) -> Response:
        sample = hardware.camera_frame(camera)
        if sample.compressed:
            media_type = "image/jpeg" if "jpeg" in sample.encoding.lower() or "jpg" in sample.encoding.lower() else "application/octet-stream"
            return Response(sample.data, media_type=media_type, headers={
                "X-Frame-Stamp": f"{sample.timestamp.sec}.{sample.timestamp.nanosec:09d}",
                "X-Frame-Size": f"{sample.width}x{sample.height}",
            })
        raise HTTPException(415, "The selected raw stream cannot be rendered directly")

    @app.post("/api/control/arm", status_code=202)
    def control_arm(request: ArmRequest) -> dict[str, str]:
        if request.confirmation != "MOVE X2":
            raise HTTPException(409, "confirmation must equal MOVE X2")
        if not lock.acquire(blocking=False):
            raise HTTPException(409, "another control request is active")
        try:
            hardware.move_arm(
                ArmCommand.from_positions(request.positions_rad, request.duration_s), confirm_hardware=True
            )
            return {"status": "completed"}
        finally:
            lock.release()

    @app.post("/api/control/hand", status_code=202)
    def control_hand(request: HandRequest) -> dict[str, str]:
        if request.confirmation != "MOVE X2":
            raise HTTPException(409, "confirmation must equal MOVE X2")
        if not lock.acquire(blocking=False):
            raise HTTPException(409, "another control request is active")
        try:
            hardware.move_hand(
                HandCommand.from_positions(request.side, request.positions_rad, request.duration_s),
                confirm_hardware=True,
            )
            return {"status": "completed"}
        finally:
            lock.release()

    static = Path(__file__).with_name("static")
    app.mount("/", StaticFiles(directory=static, html=True), name="static")
    return app

