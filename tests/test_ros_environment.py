from pathlib import Path

from lingxi_x2.backends.ros_environment import environment_updates


def make_prefix(path):
    (path/'local/lib/python3.10/dist-packages').mkdir(parents=True)
    (path/'lib').mkdir()
    return path


def test_fresh_pc2_environment_adds_underlay_and_firmware_overlay_first(tmp_path, monkeypatch):
    ros=make_prefix(tmp_path/'humble');sdk=make_prefix(tmp_path/'common')
    for key in ('AMENT_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH'):monkeypatch.delenv(key,raising=False)
    result=environment_updates(str(sdk),str(ros))
    assert result['AMENT_PREFIX_PATH'].split(':')==[str(sdk),str(ros)]
    assert result['LD_LIBRARY_PATH'].split(':')==[str(sdk/'lib'),str(ros/'lib')]
    assert result['PYTHONPATH'].split(':')==[str(sdk/'local/lib/python3.10/dist-packages'),str(ros/'local/lib/python3.10/dist-packages')]


def test_existing_paths_preserved_and_reexec_does_not_duplicate(tmp_path, monkeypatch):
    ros=make_prefix(tmp_path/'humble');sdk=make_prefix(tmp_path/'common')
    for key in ('AMENT_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH'):monkeypatch.setenv(key,'/caller/path')
    first=environment_updates(str(sdk),str(ros))
    for key,value in first.items():
        assert value.endswith(':/caller/path');monkeypatch.setenv(key,value)
    assert environment_updates(str(sdk),str(ros))==first


def test_missing_sdk_leaves_development_host_untouched(tmp_path):
    ros=make_prefix(tmp_path/'humble')
    assert environment_updates(str(tmp_path/'absent'),str(ros))=={}


def test_only_existing_python_paths_added(tmp_path, monkeypatch):
    ros=tmp_path/'humble';(ros/'lib/python3.10/site-packages').mkdir(parents=True)
    sdk=tmp_path/'common';sdk.mkdir()
    monkeypatch.delenv('PYTHONPATH',raising=False)
    result=environment_updates(str(sdk),str(ros))
    assert result['PYTHONPATH']==str(ros/'lib/python3.10/site-packages')
    assert not any('dist-packages' in value for value in result.values())
