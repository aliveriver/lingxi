import io
import json
from pathlib import Path
import tarfile

import pytest

from lingxi_x2.deployment_review import digest, verify_bundle, inspect_target, rehearse


def test_builder_preserves_frozen_output_and_excludes_config(tmp_path):
    import importlib.util
    spec = importlib.util.spec_from_file_location('build_review_bundle', 'scripts/build_review_bundle.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    root = tmp_path/'source'; (root/'src').mkdir(parents=True)
    (root/'src/a.py').write_bytes(b'a')
    output = tmp_path/'frozen'
    selection = {'software_files': ['src/a.py'], 'evidence_files': []}
    meta = module.build(root, selection, output)
    assert verify_bundle(output/'update.tar.gz', output/'manifest.json', meta['sha256'])['data']['src/a.py'] == b'a'
    before = inventory(output)
    with pytest.raises(FileExistsError): module.build(root, selection, output)
    assert before == inventory(output)
    with pytest.raises(ValueError): module.build(root, {'software_files': ['config/x2.yaml'], 'evidence_files': []}, tmp_path/'forbidden')
    assert not (tmp_path/'forbidden').exists()


def bundle_fixture(tmp_path, *, extra=None, data=None):
    data = data or {'src/a.py': b'new a', 'src/b.py': b'new b', 'logs/evidence.json': b'{}'}
    hashes = {n: digest(raw) for n, raw in data.items()}
    archive = tmp_path/'update.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for name, raw in {**data, 'UPDATE-SHA256.json': json.dumps(hashes).encode()}.items():
            member = tarfile.TarInfo(name); member.size = len(raw)
            tar.addfile(member, io.BytesIO(raw))
        if extra:
            tar.addfile(extra, io.BytesIO(b'x'*extra.size) if extra.isfile() else None)
    sha = digest(archive.read_bytes())
    sidecar = tmp_path/'sidecar.json'
    sidecar.write_text(json.dumps({'sha256': sha, 'payload_files': len(data), 'hashes': hashes,
        'software_files': [n for n in data if not n.startswith('logs/')],
        'evidence_files': [n for n in data if n.startswith('logs/')]}))
    return archive, sidecar, sha


def target_fixture(tmp_path):
    root = tmp_path/'target'
    (root/'src').mkdir(parents=True)
    (root/'src/a.py').write_bytes(b'old a')
    (root/'SOFTWARE-SHA256.json').write_text(json.dumps({'src/a.py': digest(b'old a')}))
    (root/'config').mkdir()
    for name in ('x2.yaml', 'x2.motion-test.yaml'):
        (root/'config'/name).write_text('# preserve comment\ncontrol:\n  enabled: false\n')
    return root


def inventory(root):
    return {str(p.relative_to(root)): p.read_bytes() for p in root.rglob('*') if p.is_file()}


def test_frozen_bundle_normal_and_every_interruption_restores_bytes(tmp_path):
    bundle = verify_bundle(*bundle_fixture(tmp_path))
    result = rehearse(bundle)
    assert result['normal_apply_verified'] and result['backup_verified']
    assert result['rollback_verified_after_write_counts'] == [0, 1, 2, 3, 4]
    assert result['config_and_unrelated_evidence_preserved']
    assert result['synthetic_fixture'] and not result['pc2_snapshot']


@pytest.mark.parametrize('name,kind', [('../escape','file'), ('/absolute','file'),
    ('src/../escape','file'), ('src//a','file'), ('src/a.py','file'),
    ('src/link','symlink'), ('src/hard','hardlink'), ('src/dir','directory')])
def test_archive_member_attacks_rejected_without_extraction(tmp_path, name, kind):
    extra = tarfile.TarInfo(name)
    extra.type = {'file': tarfile.REGTYPE, 'symlink': tarfile.SYMTYPE,
                  'hardlink': tarfile.LNKTYPE, 'directory': tarfile.DIRTYPE}[kind]
    extra.linkname = '/tmp/elsewhere'
    with pytest.raises(ValueError): verify_bundle(*bundle_fixture(tmp_path, extra=extra))
    assert not (tmp_path/'src').exists()


@pytest.mark.parametrize('data', [
    {'config/x2.yaml': b'control: {enabled: true}'},
    {'src/a': b'x', 'src/a/b': b'y'},
    {'SOFTWARE-SHA256.json': b'{}'},
])
def test_protected_payload_or_parent_collision_rejected(tmp_path, data):
    with pytest.raises(ValueError): verify_bundle(*bundle_fixture(tmp_path, data=data))


def test_outer_and_inner_manifest_mismatch(tmp_path):
    args = bundle_fixture(tmp_path)
    with pytest.raises(ValueError, match='Archive SHA256'): verify_bundle(args[0], args[1], '0'*64)
    meta = json.loads(args[1].read_text()); meta['hashes']['src/a.py'] = '0'*64
    args[1].write_text(json.dumps(meta))
    with pytest.raises(ValueError, match='Inner manifest'): verify_bundle(*args)


def test_readonly_target_actions_and_unrelated_files_survive(tmp_path):
    bundle = verify_bundle(*bundle_fixture(tmp_path)); root = target_fixture(tmp_path)
    (root/'logs').mkdir(); (root/'logs/old.json').write_bytes(b'old evidence')
    before = inventory(root)
    r = inspect_target(bundle, root)
    assert r['actions'] == {'src/a.py': 'replace_tracked', 'src/b.py': 'create', 'logs/evidence.json': 'create'}
    assert before == inventory(root)


@pytest.mark.parametrize('damage', ['manifest_drift','enabled','missing_config','quoted_false',
                                  'evidence_collision','untracked_collision','symlink','parent_file'])
def test_target_conflicts_fail_without_changes(tmp_path, damage):
    bundle = verify_bundle(*bundle_fixture(tmp_path)); root = target_fixture(tmp_path)
    if damage == 'manifest_drift': (root/'src/a.py').write_bytes(b'changed')
    if damage == 'enabled': (root/'config/x2.yaml').write_text('control: {enabled: true}')
    if damage == 'quoted_false': (root/'config/x2.yaml').write_text('control: {enabled: "false"}')
    if damage == 'missing_config': (root/'config/x2.yaml').unlink()
    if damage == 'evidence_collision':
        (root/'logs').mkdir(); (root/'logs/evidence.json').write_bytes(b'historical')
    if damage == 'untracked_collision': (root/'src/b.py').write_bytes(b'user edits')
    if damage == 'symlink': (root/'logs').symlink_to(tmp_path, target_is_directory=True)
    if damage == 'parent_file': (root/'logs').write_bytes(b'file')
    before = inventory(root)
    with pytest.raises(ValueError): inspect_target(bundle, root)
    assert before == inventory(root)


def test_identical_existing_evidence_allowed(tmp_path):
    bundle = verify_bundle(*bundle_fixture(tmp_path)); root = target_fixture(tmp_path)
    (root/'logs').mkdir(); (root/'logs/evidence.json').write_bytes(b'{}')
    assert inspect_target(bundle, root)['actions']['logs/evidence.json'] == 'keep_identical'


@pytest.mark.parametrize('name', ['.python-version', 'config/mock.yaml', 'config/x2.example.yaml'])
def test_legacy_registered_files_are_verified_but_never_update_payload(tmp_path, name):
    bundle = verify_bundle(*bundle_fixture(tmp_path)); root = target_fixture(tmp_path)
    (root/name).write_bytes(b'legacy bytes')
    manifest = json.loads((root/'SOFTWARE-SHA256.json').read_text())
    manifest[name] = digest(b'legacy bytes')
    (root/'SOFTWARE-SHA256.json').write_text(json.dumps(manifest))
    before = inventory(root)
    assert inspect_target(bundle, root)['installed_manifest_files'] == 2
    assert inventory(root) == before
    (root/name).write_bytes(b'changed')
    with pytest.raises(ValueError, match='Installed manifest mismatch'):
        inspect_target(bundle, root)
    with pytest.raises(ValueError):
        verify_bundle(*bundle_fixture(tmp_path, data={name: b'new bytes'}))


@pytest.mark.parametrize('name', ['config/x2.yaml', 'config/x2.motion-test.yaml',
                                  'config/unknown.yaml', 'logs/old.json'])
def test_other_protected_manifest_entries_still_rejected(tmp_path, name):
    bundle = verify_bundle(*bundle_fixture(tmp_path)); root = target_fixture(tmp_path)
    (root/name).parent.mkdir(exist_ok=True)
    if not (root/name).exists(): (root/name).write_bytes(b'protected')
    manifest = json.loads((root/'SOFTWARE-SHA256.json').read_text())
    manifest[name] = digest((root/name).read_bytes())
    (root/'SOFTWARE-SHA256.json').write_text(json.dumps(manifest))
    before = inventory(root)
    with pytest.raises(ValueError, match='protected path'): inspect_target(bundle, root)
    assert inventory(root) == before
