from __future__ import annotations

from contextlib import asynccontextmanager
from dataclasses import asdict
from pathlib import Path
import threading
import time
from typing import Any, Literal

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.responses import Response, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from ..client import X2Client
from ..models import ArmCommand, CameraName, HandCommand, Side
from ..errors import X2Error
from .recordings import RecordingLibrary


class RecordingRequest(BaseModel):
    duration_s: float = Field(default=10, ge=0.1, le=600, allow_inf_nan=False)
    rate_hz: float = Field(default=10, ge=1, le=50, allow_inf_nan=False)
    cameras: list[CameraName] = Field(default_factory=list, max_length=3)
    include_tactile: bool = True
    include_image_data: bool = False


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
    library = RecordingLibrary(hardware, Path(hardware.config.web.recordings_dir))

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        hardware.start()
        try:
            yield
        finally:
            try:
                library.close()
            finally:
                hardware.close()

    app = FastAPI(title="Lingxi X2", version="0.1.0", lifespan=lifespan)

    app.state.recordings = library

    @app.exception_handler(X2Error)
    async def robot_error(_: Request, exc: X2Error):
        return JSONResponse(status_code=409, content={"detail": str(exc), "motion_verified": False})

    @app.get("/api/readiness")
    def readiness():
        return {"backend": hardware.status().backend, "control_enabled": hardware.config.control.enabled,
                "web_motion_allowed": hardware.status().backend == "mock" and hardware.config.control.enabled,
                "motion_verified": False, "tactile_contact_verified": False,
                "reason": "实机跟随、回位与重复性尚未通过；网页运动仅开放 mock"}

    @app.get("/api/preflight/{subsystem}")
    def preflight(subsystem: Literal["arm", "hand"]):
        return hardware.control_preflight(subsystem).as_dict()

    def library_call(method, *args, **kwargs):
        try:
            return method(*args, **kwargs)
        except FileNotFoundError as exc:
            raise HTTPException(404, "Recording not found") from exc
        except ValueError as exc:
            raise HTTPException(422, str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.get("/api/recordings")
    def recordings():
        return library_call(library.list)

    @app.post("/api/recordings", status_code=202)
    def record(request: RecordingRequest):
        return library_call(library.start, request.model_dump(mode="json"))

    @app.get("/api/recordings/{identifier}")
    def recording(identifier: str):
        return library_call(library.get, identifier)

    @app.post("/api/recordings/{identifier}/stop", status_code=202)
    def stop_recording(identifier: str):
        return library_call(library.stop, identifier)

    @app.get("/api/recordings/{identifier}/download")
    def download(identifier: str):
        return FileResponse(library_call(library.path, identifier), media_type="application/x-ndjson",
                            filename=f"{identifier}.jsonl")

    @app.get("/api/recordings/{identifier}/samples")
    def samples(identifier: str, offset: int = Query(0, ge=0, le=100000), limit: int = Query(20, ge=1, le=100)):
        return library_call(library.samples, identifier, offset, limit)

    def require_mock_motion():
        if hardware.status().backend != "mock":
            raise HTTPException(409, "实机跟随与回位尚未通过；使用现场确认的固定基线验收脚本")

    @app.get("/api/status")
    def status() -> dict[str, Any]:
        return hardware.status().as_dict()

    @app.get("/api/state")
    def state() -> dict[str, Any]:
        arm = hardware.arm_state()
        hands = hardware.hand_states()
        tactile: dict[str, Any] = {}
        tactile_error = None
        try:
            tactile = {side.value: asdict(value) for side, value in hardware.tactile_frames(0.2).items()}
        except X2Error as exc:
            tactile_error = str(exc)
        return {
            "arm": asdict(arm),
            "hands": {side.value: asdict(value) for side, value in hands.items()},
            "tactile": tactile,
            "tactile_error": tactile_error,
            "received_monotonic_ns": time.monotonic_ns(),
            "source": "live",
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
    def control_arm(request: ArmRequest) -> dict[str, Any]:
        require_mock_motion()
        if request.confirmation != "MOVE X2":
            raise HTTPException(409, "confirmation must equal MOVE X2")
        if not lock.acquire(blocking=False):
            raise HTTPException(409, "another control request is active")
        try:
            stats = hardware.move_arm(
                ArmCommand.from_positions(request.positions_rad, request.duration_s), confirm_hardware=True
            )
            return {"status": "published", "publication_stats": stats, "motion_verified": False}
        finally:
            lock.release()

    @app.post("/api/control/hand", status_code=202)
    def control_hand(request: HandRequest) -> dict[str, Any]:
        require_mock_motion()
        if request.confirmation != "MOVE X2":
            raise HTTPException(409, "confirmation must equal MOVE X2")
        if not lock.acquire(blocking=False):
            raise HTTPException(409, "another control request is active")
        try:
            stats = hardware.move_hand(
                HandCommand.from_positions(request.side, request.positions_rad, request.duration_s),
                confirm_hardware=True,
            )
            return {"status": "published", "publication_stats": stats, "motion_verified": False}
        finally:
            lock.release()

    static = Path(__file__).with_name("static")
    app.mount("/", StaticFiles(directory=static, html=True), name="static")
    return app

