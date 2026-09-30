from dataclasses import asdict
import json
from types import SimpleNamespace

import pytest

from lingxi_x2.models import Side
from lingxi_x2.recording import JsonlRecorder, observation_to_dict
from lingxi_x2.replay import RecordingReader, RecordingError, decode_observation, inspect_recording
from lingxi_x2.tactile import save_tactile_map
from lingxi_x2.tactile_fixture import synthetic_observations, write_synthetic_recording
from lingxi_x2.tactile_quality import review_tactile
from lingxi_x2.web.recordings import RecordingLibrary


def quality(observation):
    return review_tactile({s.value: asdict(f) for s, f in observation.tactile.items()}, observation.captured_monotonic_ns)


def test_synthetic_scenarios_distinguish_zero_missing_stale_and_ceiling():
    samples = list(synthetic_observations())
    q = [quality(s) for s in samples]
    assert all(r['cell_count'] == 141 and r['nonzero_cells'] == 0 for r in q[0]['sides'].values())
    assert q[1]['sides']['left']['nonzero_cells'] == 26
    assert q[1]['sides']['right']['peak_raw_uint8'] == 125
    assert q[2]['sides']['left']['raw_ceiling_cells'] == 1
    assert not q[2]['physical_saturation_verified'] and not q[2]['tactile_contact_verified']
    assert q[3]['sides']['left']['status'] == 'stale'
    assert q[4]['sides']['left']['age_s'] == 2.01
    assert q[5]['sides']['right']['status'] == 'missing'
    assert q[5]['sides']['right']['peak_raw_uint8'] is None
    assert all(s['status'] == 'missing' for s in q[6]['sides'].values())
    assert all(s['status'] == 'fresh' and s['nonzero_cells'] == 0 for s in q[7]['sides'].values())


@pytest.mark.parametrize('value', [True, 1.0, '1', -1, 256, None])
def test_raw_values_never_coerced_to_valid_integers(value):
    payload = observation_to_dict(next(synthetic_observations()))
    payload['tactile']['left']['palm']['values'] = [value]*25
    q = review_tactile(payload['tactile'], payload['captured_monotonic_ns'])
    assert q['sides']['left']['status'] == 'invalid'
    assert q['sides']['right']['status'] == 'fresh'
    with pytest.raises(RecordingError): decode_observation(payload, 0)


@pytest.mark.parametrize('damage', ['missing_tip','extra_tip','wrong_shape','wrong_name','wrong_side','wrong_unit','clock'])
def test_incomplete_or_mislabelled_grid_not_filled_with_zeros(damage):
    payload = observation_to_dict(next(synthetic_observations()))
    frame = payload['tactile']['left']
    if damage == 'missing_tip': del frame['fingertips']['little']
    if damage == 'extra_tip': frame['fingertips']['extra'] = frame['fingertips']['thumb']
    if damage == 'wrong_shape': frame['palm']['shape'] = [1, 25]
    if damage == 'wrong_name': frame['palm']['name'] = 'back_of_hand'
    if damage == 'wrong_side': frame['side'] = 'right'
    if damage == 'wrong_unit': frame['unit'] = 'N'
    if damage == 'clock': frame['timestamp']['monotonic_ns'] = True
    assert review_tactile(payload['tactile'], payload['captured_monotonic_ns'])['sides']['left']['status'] == 'invalid'
    with pytest.raises(RecordingError): decode_observation(payload)


def test_age_boundary_future_clock_and_replay_original_time():
    observation = next(synthetic_observations())
    frames = {s.value: asdict(f) for s, f in observation.tactile.items()}
    received = frames['left']['timestamp']['monotonic_ns']
    assert review_tactile(frames, received+500_000_000)['sides']['left']['status'] == 'fresh'
    assert review_tactile(frames, received+500_000_001)['sides']['left']['status'] == 'stale'
    assert review_tactile(frames, received-1)['sides']['left']['status'] == 'time_error'
    assert review_tactile({'left': None}, received)['sides']['left']['status'] == 'invalid'


def test_live_api_reports_partial_and_bad_frames_without_reusing_old_data(tmp_path, monkeypatch):
    from lingxi_x2 import X2Client, PlatformConfig
    from lingxi_x2.web.app import create_app
    from test_web_recordings import endpoint
    with X2Client(PlatformConfig(backend='mock', web={'recordings_dir': str(tmp_path)})) as client:
        app = create_app(client)
        frames = list(synthetic_observations())[5].tactile
        monkeypatch.setattr(client, 'tactile_frames', lambda *a: frames)
        state = endpoint(app, '/api/state')()
        assert set(state['tactile']) == {'left'}
        assert state['tactile_quality']['sides']['right']['status'] == 'missing'
        assert state['tactile_quality']['sides']['left']['status'] == 'stale'
        monkeypatch.setattr(client, 'tactile_frames', lambda *a: {})
        state = endpoint(app, '/api/state')()
        assert state['tactile'] == {}
        assert all(r['status'] == 'missing' for r in state['tactile_quality']['sides'].values())


def test_fixture_roundtrip_inspection_and_no_overwrite(tmp_path):
    path = tmp_path/'synthetic.jsonl'
    write_synthetic_recording(path)
    reader = RecordingReader(path)
    assert [s.observation for s in reader] == list(synthetic_observations())
    assert reader.metadata['synthetic'] and not reader.metadata['robot_connected']
    report = inspect_recording(path)
    assert report['tactile_raw_ceiling_cell_samples'] == 6  # repeated raw frame included
    assert report['tactile_repeated_receipt_samples'] == {'left': 2, 'right': 2}
    assert report['tactile_quality_counts']['right']['missing'] == 2
    assert report['tactile_quality_counts']['left']['stale'] == 2
    before = path.read_bytes()
    with pytest.raises(FileExistsError): write_synthetic_recording(path)
    assert before == path.read_bytes()


def test_actual_recorder_preserves_synthetic_raw_values_and_missing_sides(tmp_path, monkeypatch):
    observations = iter(synthetic_observations())
    fake = SimpleNamespace(status=lambda: SimpleNamespace(as_dict=lambda: {'backend': 'synthetic_tactile_fixture'}),
                           observe=lambda *a, **k: next(observations))
    class Stop:
        waits = 0
        def is_set(self): return self.waits >= 8
        def wait(self, seconds): self.waits += 1
    monkeypatch.setattr('lingxi_x2.recording.time.monotonic', lambda: 0.)
    path = tmp_path/'recorded.jsonl'
    assert JsonlRecorder(fake, path).run(10, 10, stop_event=Stop()) == 8
    assert [s.observation for s in RecordingReader(path)] == list(synthetic_observations())
    library = RecordingLibrary(None, tmp_path)
    identifier = 'a'*32
    target = tmp_path/f'{identifier}.jsonl'; target.write_bytes(path.read_bytes())
    (tmp_path/f'{identifier}.json').write_text(json.dumps({'id':identifier,'status':'completed'}))
    page = library.samples(identifier, offset=4, limit=3)
    assert page['samples'][0]['tactile_quality']['sides']['left']['age_s'] == 2.01
    assert page['samples'][1]['tactile_quality']['sides']['right']['status'] == 'missing'
    assert page['samples'][2]['observation']['tactile'] == {}
    assert page['executed_actions'] == 0


def test_png_label_metadata_and_existing_sidecar_protected(tmp_path):
    frames = list(synthetic_observations())[2].tactile
    save_tactile_map(frames, tmp_path/'synthetic.png')
    meta = json.loads((tmp_path/'synthetic.json').read_text())
    assert meta['synthetic_sides'] == ['left','right'] and meta['raw_ceiling_cells'] == 2
    assert not meta['physical_saturation_verified']
    with pytest.raises(FileExistsError): save_tactile_map(frames, tmp_path/'synthetic.png')
    (tmp_path/'old.json').write_text('historical evidence')
    with pytest.raises(FileExistsError): save_tactile_map(frames, tmp_path/'old.png')
    assert not (tmp_path/'old.png').exists()
    assert (tmp_path/'old.json').read_text() == 'historical evidence'
    with pytest.raises(ValueError): save_tactile_map({Side.LEFT: frames[Side.LEFT]}, tmp_path/'missing.png')
    assert not (tmp_path/'missing.png').exists()
