from __future__ import annotations

from fastapi.testclient import TestClient

from lingxi_x2 import X2Client
from lingxi_x2.config import PlatformConfig
from lingxi_x2.web.app import create_app


def test_web_status_state_and_camera() -> None:
    app = create_app(X2Client(PlatformConfig(backend="mock")))
    with TestClient(app) as web:
        assert web.get("/api/status").json()["backend"] == "mock"
        assert len(web.get("/api/state").json()["arm"]["joints"]) == 14
        response = web.get("/api/cameras/rgbd_front_rgb/frame")
        assert response.status_code == 200
        assert response.headers["content-type"] == "image/jpeg"


def test_web_control_requires_confirmation_phrase() -> None:
    app = create_app(X2Client(PlatformConfig(backend="mock")))
    with TestClient(app) as web:
        response = web.post(
            "/api/control/hand",
            json={"side": "left", "positions_rad": [0.0] * 10, "duration_s": 0.01, "confirmation": "no"},
        )
        assert response.status_code == 409

