import threading
import time
from dataclasses import replace

import pytest
from fastapi import HTTPException

from lingxi_x2 import X2Client, PlatformConfig
from lingxi_x2.errors import DataTimeoutError, SafetyInterlockError
from lingxi_x2.experiments import ExperimentRunner
from lingxi_x2.recording import JsonlRecorder
from lingxi_x2.replay import RecordingReader
from lingxi_x2.web.app import create_app, HandRequest, RecordingRequest
from lingxi_x2.web.recordings import RecordingLibrary


def wait_finished(library, identifier):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        job = library.get(identifier)
        if job['status'] not in {'recording', 'stopping'}:
            return job
        time.sleep(.01)
    pytest.fail('Recording job did not finish')


def test_record_stop_replay_and_reopen(tmp_path):
    with X2Client(PlatformConfig(backend='mock')) as client:
        library = RecordingLibrary(client, tmp_path)
        try:
            job = library.start(RecordingRequest(duration_s=5, rate_hz=50).model_dump(mode='json'))
            with pytest.raises(RuntimeError): library.start({})
            with pytest.raises(RuntimeError): library.path(job['id'])
            # Wait for at least one flushed frame, then stop an ongoing episode.
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                path = tmp_path / (job['id'] + '.jsonl')
                if path.exists() and len(path.read_bytes().splitlines()) > 1: break
                time.sleep(.01)
            library.stop(job['id'])
            result = wait_finished(library, job['id'])
            assert result['status'] == 'stopped' and result['samples'] >= 1
            assert result['inspection']['sha256']
            page = library.samples(job['id'], limit=1)
            assert page['source'] == 'recording' and page['executed_actions'] == 0
            assert len(page['samples']) == 1
            assert page['metadata']['status']['backend'] == 'mock'
            assert not result['motion_verified']
            assert RecordingLibrary(client, tmp_path).get(job['id'])['status'] == 'stopped'
        finally:
            library.close()


def test_failure_and_interrupted_jobs_remain_distinct(tmp_path, monkeypatch):
    with X2Client(PlatformConfig(backend='mock')) as client:
        library = RecordingLibrary(client, tmp_path)
        def fail(*args, **kwargs): raise DataTimeoutError('tactile missing')
        monkeypatch.setattr(client, 'observe', fail)
        job = library.start(RecordingRequest(duration_s=.1).model_dump(mode='json'))
        result = wait_finished(library, job['id'])
        assert result['status'] == 'failed' and 'tactile missing' in result['error']
        library.close()
        result['status'] = 'recording'; library._save(result)
        assert RecordingLibrary(client, tmp_path).get(job['id'])['status'] == 'interrupted'


def test_paths_confined_and_shutdown_interrupts_wait(tmp_path):
    with X2Client(PlatformConfig(backend='mock')) as client:
        library = RecordingLibrary(client, tmp_path)
        with pytest.raises(ValueError): library.get('../secret')
        (tmp_path / ('a'*32 + '.json')).symlink_to(tmp_path.parent / 'outside')
        with pytest.raises(ValueError): library.get('a'*32)
        job = library.start(RecordingRequest(duration_s=60, rate_hz=1).model_dump(mode='json'))
        started = time.monotonic(); library.close()
        assert time.monotonic() - started < 2
        assert library.get(job['id'])['status'] == 'stopped'
        with pytest.raises(RuntimeError): library.start({})


def test_recorder_cancelled_before_sample(tmp_path):
    stop = threading.Event(); stop.set()
    with X2Client(PlatformConfig(backend='mock')) as client:
        path = tmp_path / 'cancelled.jsonl'
        assert JsonlRecorder(client,path).run(60,1,stop_event=stop) == 0
        assert list(RecordingReader(path)) == []


def endpoint(app, path):
    return next(route.endpoint for route in app.routes if route.path == path)


def test_routes_truthful_publication_tactile_failure_and_hardware_gate(tmp_path, monkeypatch):
    # Direct route tests supplement HTTP integration tests, which require an
    # environment where AnyIO's portal can wake its event loop.
    config = PlatformConfig(backend='mock', control={'enabled': True}, web={'recordings_dir': str(tmp_path)})
    with X2Client(config) as client:
        app = create_app(client)
        result = endpoint(app, '/api/control/hand')(HandRequest(side='left', positions_rad=[0.]*10,
                                                              duration_s=.01, confirmation='MOVE X2'))
        assert result['status'] == 'published' and not result['motion_verified']
        assert 'publication_stats' in result
        assert 'ready' in endpoint(app, '/api/preflight/{subsystem}')('arm')
        def fail(*args, **kwargs): raise DataTimeoutError('pressure expired')
        monkeypatch.setattr(client, 'tactile_frames', fail)
        state = endpoint(app, '/api/state')()
        assert state['tactile'] == {} and state['tactile_error'] == 'pressure expired'
        real_status = replace(client.status(), backend='ros2')
        monkeypatch.setattr(client, 'status', lambda: real_status)
        with pytest.raises(HTTPException) as error:
            endpoint(app, '/api/control/hand')(HandRequest(side='left',positions_rad=[0.]*10,confirmation='MOVE X2'))
        assert error.value.status_code == 409
        with pytest.raises(SafetyInterlockError, match='ShadowExperimentRunner'):
            ExperimentRunner(client, None).run(max_steps=1, allow_hardware_actions=True)
