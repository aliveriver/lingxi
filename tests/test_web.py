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


def test_web_recording_http_roundtrip(tmp_path) -> None:
    import time
    config = PlatformConfig(backend="mock", web={"recordings_dir": str(tmp_path)})
    with TestClient(create_app(X2Client(config))) as web:
        assert web.get('/api/readiness').json()['motion_verified'] is False
        assert 'ready' in web.get('/api/preflight/arm').json()
        response = web.post('/api/recordings', json={'duration_s': .1, 'rate_hz': 20})
        assert response.status_code == 202
        identifier = response.json()['id']
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            job = web.get(f'/api/recordings/{identifier}').json()
            if job['status'] != 'recording': break
            time.sleep(.01)
        assert job['status'] == 'completed' and job['samples'] > 0
        assert web.get(f'/api/recordings/{identifier}/download').status_code == 200
        page = web.get(f'/api/recordings/{identifier}/samples?limit=1').json()
        assert len(page['samples']) == 1 and page['executed_actions'] == 0
        assert web.get(f'/api/recordings/{identifier}/samples?limit=1000').status_code == 422
        assert web.get('/api/recordings/not-an-id').status_code == 422
        assert web.post('/api/recordings', json={'duration_s': 601}).status_code == 422
        assert web.get('/api/recordings').json()[0]['id'] == identifier
