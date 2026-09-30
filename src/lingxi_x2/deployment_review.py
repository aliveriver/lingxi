"""Validate update archives and rehearse writes ONLY in owned temporary fixtures.

No deploy API: a supplied target is inspected read-only. Synthetic site configs
exist only inside TemporaryDirectory, never in the user's workspace.
"""
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import tarfile
import tempfile

import yaml

from .replay import strict_json

MANIFEST = 'SOFTWARE-SHA256.json'
CONFIGS = ('config/x2.yaml', 'config/x2.motion-test.yaml')
# Legacy PC2 manifests register these files. They may be verified and retained,
# but software_name() deliberately excludes them from every update payload.
LEGACY_PRESERVED_FILES = frozenset(('.python-version', 'config/mock.yaml', 'config/x2.example.yaml'))


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def safe_name(name):
    if (not isinstance(name, str) or not name or '\\' in name or '\x00' in name
            or PurePosixPath(name).is_absolute() or str(PurePosixPath(name)) != name
            or any(p in ('..', '.') for p in name.split('/'))):
        raise ValueError(f'Unsafe member path: {name!r}')
    return name


def software_name(name):
    safe_name(name)
    return name in ('README.md', 'pyproject.toml', 'uv.lock') or (
        '/' in name and name.split('/')[0] in {'src', 'scripts', 'tests', 'docs', 'examples'})


def hash_map(value):
    if not isinstance(value, dict) or not value:
        raise ValueError('Expected nonempty hash manifest')
    for name, sha in value.items():
        safe_name(name)
        if not isinstance(sha, str) or not re.fullmatch('[0-9a-f]{64}', sha):
            raise ValueError('Invalid SHA256')
    return value


def read_regular(root, name, *, optional=False):
    safe_name(name)
    path = root
    for part in name.split('/'):
        path = path / part
        if path.is_symlink():
            raise ValueError(f'Symlink forbidden: {name}')
    if not path.exists() and optional:
        return None
    if not path.is_file():
        raise ValueError(f'Missing/nonregular target: {name}')
    return path.read_bytes()


def verify_bundle(archive, sidecar, expected_sha256):
    archive, sidecar = Path(archive), Path(sidecar)
    if archive.stat().st_size > 128*1024**2:
        raise ValueError('Archive exceeds review size limit')
    raw = archive.read_bytes()
    if not re.fullmatch('[0-9a-f]{64}', expected_sha256) or digest(raw) != expected_sha256:
        raise ValueError('Archive SHA256 mismatch')
    meta = strict_json(sidecar.read_text())
    hashes = hash_map(meta['hashes'])
    software, evidence = meta['software_files'], meta['evidence_files']
    if (not isinstance(software, list) or not isinstance(evidence, list)
            or len(set(software+evidence)) != len(software+evidence)
            or set(software+evidence) != set(hashes)
            or meta['payload_files'] != len(hashes) or meta['sha256'] != expected_sha256):
        raise ValueError('Sidecar classification/count/hash mismatch')
    if not all(software_name(n) for n in software):
        raise ValueError('Software path not allowed; configs and evidence cannot be software')
    if not all(safe_name(n).split('/')[0] in ('logs', 'snapshots') for n in evidence):
        raise ValueError('Evidence path not allowed')
    data = {}
    with tarfile.open(fileobj=io.BytesIO(raw), mode='r:gz') as tar:
        total = 0
        for member in tar:
            name = safe_name(member.name)
            if (not member.isfile() or member.issparse() or name in data
                    or name not in set(hashes) | {'UPDATE-SHA256.json'}):
                raise ValueError('Unexpected, duplicate or nonregular archive member')
            total += member.size
            if member.size > 64*1024**2 or total > 512*1024**2 or len(data) >= 10000:
                raise ValueError('Archive expansion exceeds review size limit')
            data[name] = tar.extractfile(member).read()
    if set(data) != set(hashes) | {'UPDATE-SHA256.json'}:
        raise ValueError('Missing archive members')
    if strict_json(data.pop('UPDATE-SHA256.json').decode()) != hashes:
        raise ValueError('Inner manifest mismatch')
    if any(digest(data[n]) != sha for n, sha in hashes.items()):
        raise ValueError('Payload SHA256 mismatch')
    # Reject file/parent collisions without extracting anything.
    for name in data:
        if any(str(p) in data for p in PurePosixPath(name).parents if str(p) != '.'):
            raise ValueError('Archive file/parent collision')
    return {'archive_sha256': expected_sha256, 'sidecar_sha256': digest(sidecar.read_bytes()),
            'software_files': software, 'evidence_files': evidence, 'hashes': hashes, 'data': data}


def inspect_target(bundle, target):
    """Read-only preflight; requires actual configs and manifest, never fabricates them."""
    root = Path(target)
    if root.is_symlink() or not root.is_dir() or root.absolute() != root.resolve():
        raise ValueError('Target must be a real directory without symlink ancestors')
    prior = hash_map(strict_json(read_regular(root, MANIFEST).decode()))
    if not all(software_name(n) or n in LEGACY_PRESERVED_FILES for n in prior):
        raise ValueError('Installed software manifest contains a protected path')
    for name, sha in prior.items():
        if digest(read_regular(root, name)) != sha:
            raise ValueError(f'Installed manifest mismatch: {name}')
    configs = {}
    for name in CONFIGS:
        raw = read_regular(root, name)
        config = yaml.safe_load(raw)
        if not isinstance(config, dict) or not isinstance(config.get('control'), dict) or config['control'].get('enabled') is not False:
            raise ValueError(f'Control must be explicitly false: {name}')
        configs[name] = digest(raw)
    actions = {}
    for name, sha in bundle['hashes'].items():
        raw = read_regular(root, name, optional=True)
        # Missing parents may be created in a later installer; file parents cannot.
        for parent in (root/name).parents:
            if parent == root:
                break
            if parent.exists() and not parent.is_dir():
                raise ValueError(f'Parent is not directory: {name}')
        if raw is None:
            actions[name] = 'create'
        elif digest(raw) == sha:
            actions[name] = 'keep_identical'
        elif name in bundle['evidence_files'] or name not in prior:
            raise ValueError(f'Protected evidence or untracked collision: {name}')
        else:
            actions[name] = 'replace_tracked'
    return {'installed_manifest_files': len(prior), 'config_sha256': configs,
            'actions': actions, 'target_read_only': True}


def _inventory(root):
    return {str(p.relative_to(root)): digest(p.read_bytes()) for p in root.rglob('*') if p.is_file()}


def rehearse(bundle):
    """Synthetic normal apply plus failure/rollback at every write boundary.

    Tests transaction design, not production crash durability or a PC2 snapshot.
    All writes are constrained to an internally allocated TemporaryDirectory.
    """
    with tempfile.TemporaryDirectory(prefix='lingxi-deploy-rehearsal-') as directory:
        root = Path(directory)/'synthetic-target'
        root.mkdir()
        old = {}
        for i, name in enumerate(bundle['software_files']):
            if i % 2 == 0:
                p = root/name
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_bytes(b'synthetic previous software\n')
                old[name] = digest(p.read_bytes())
        for name in CONFIGS:
            p = root/name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(b'# SYNTHETIC REHEARSAL ONLY\ncontrol:\n  enabled: false\n')
        for name in ('logs/preexisting-evidence.bin', 'snapshots/preexisting-evidence.bin'):
            p = root/name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(b'synthetic evidence must survive\x00')
        (root/MANIFEST).write_text(json.dumps(old))
        preflight = inspect_target(bundle, root)
        before = _inventory(root)
        writes = dict(bundle['data'])
        new_manifest = {**old, **{n: bundle['hashes'][n] for n in bundle['software_files']}}
        writes[MANIFEST] = (json.dumps(new_manifest, indent=2)+'\n').encode()
        # Backup all replaced bytes, config bytes, manifest, and list of new paths.
        originals = {name: read_regular(root, name, optional=True) for name in writes}
        backup = Path(directory)/'backup.tar.gz'
        with tarfile.open(backup, 'x:gz') as tar:
            for name in sorted(set(writes) | set(CONFIGS)):
                raw = read_regular(root, name, optional=True)
                if raw is not None:
                    m = tarfile.TarInfo(name); m.size = len(raw)
                    tar.addfile(m, io.BytesIO(raw))
        with tarfile.open(backup, 'r:gz') as tar:
            saved = {m.name: tar.extractfile(m).read() for m in tar}
        if any(saved[n] != raw for n, raw in originals.items() if raw is not None):
            raise ValueError('Backup validation failed')
        def restore():
            for name, raw in originals.items():
                p = root/name
                if raw is None:
                    if p.exists(): p.unlink()
                else:
                    p.write_bytes(saved[name])
            for p in sorted(root.rglob('*'), key=lambda p: len(p.parts), reverse=True):
                if p.is_dir() and not any(p.iterdir()): p.rmdir()
        def apply(fail_after=None):
            count = 0
            if fail_after == 0: raise RuntimeError('Injected interruption')
            for name, raw in writes.items():
                p = root/name
                p.parent.mkdir(parents=True, exist_ok=True)
                staging = p.with_name(p.name+'.rehearsal-staging')
                with staging.open('xb') as f: f.write(raw)
                os.chmod(staging, (p.stat().st_mode & 0o777) if p.exists() else 0o644)
                os.replace(staging, p)
                count += 1
                if count == fail_after: raise RuntimeError('Injected interruption')
        failure_checks = []
        for boundary in range(len(writes)+1):
            try:
                apply(boundary)
            except RuntimeError:
                restore()
            else:
                raise ValueError('Failure injection did not fire')
            if _inventory(root) != before:
                raise ValueError('Rollback failed to restore original file bytes')
            failure_checks.append(boundary)
        apply()
        postflight = inspect_target(bundle, root)
        if any(action != 'keep_identical' for action in postflight['actions'].values()):
            raise ValueError('Postflight payload mismatch')
        if any(read_regular(root, n) != saved[n] for n in CONFIGS):
            raise ValueError('Config bytes changed')
        after = _inventory(root)
        if any(after[n] != sha for n, sha in before.items() if n not in writes):
            raise ValueError('Unrelated evidence changed')
        return {'synthetic_fixture': True, 'pc2_snapshot': False, 'preflight': preflight,
                'backup_sha256': digest(backup.read_bytes()), 'backup_verified': True,
                'new_paths_for_recovery': [n for n, raw in originals.items() if raw is None],
                'normal_apply_verified': True, 'config_and_unrelated_evidence_preserved': True,
                'rollback_verified_after_write_counts': failure_checks,
                'limits': 'Injected Python exceptions after file replacement; not power-loss, disk-full, concurrent-writer or PC2 validation.'}
