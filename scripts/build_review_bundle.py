"""Freeze an explicit software/evidence file list into a NEW local review bundle.

No installer, configuration discovery, network, or robot initialization.
"""
import argparse
import io
import json
from pathlib import Path
import tarfile

from lingxi_x2.deployment_review import digest, read_regular, safe_name, software_name, verify_bundle
from lingxi_x2.replay import strict_json


def build(root, selection, output_dir):
    software, evidence = selection['software_files'], selection['evidence_files']
    names = software+evidence
    if not software or len(set(names)) != len(names):
        raise ValueError('Need software and unique explicit paths')
    if not all(software_name(n) for n in software):
        raise ValueError('Protected software path')
    if not all(safe_name(n).split('/')[0] in ('logs', 'snapshots') for n in evidence):
        raise ValueError('Invalid evidence path')
    data = {name: read_regular(Path(root), name) for name in sorted(names)}
    hashes = {name: digest(raw) for name, raw in data.items()}
    output_dir = Path(output_dir)
    output_dir.mkdir()  # Never reuse or overwrite a frozen output directory.
    archive = output_dir/'update.tar.gz'
    with tarfile.open(archive, 'x:gz') as tar:
        for name, raw in {**data, 'UPDATE-SHA256.json': json.dumps(hashes, indent=2).encode()}.items():
            member = tarfile.TarInfo(name); member.size = len(raw); member.mode = 0o644
            tar.addfile(member, io.BytesIO(raw))
    sha = digest(archive.read_bytes())
    meta = {'kind': 'offline_review_bundle_v1', 'archive': str(archive), 'sha256': sha,
            'payload_files': len(data), 'software_files': software, 'evidence_files': evidence,
            'hashes': hashes, 'deployed_to_pc2': False, 'execution_authorized': False,
            'site_configs_included': False,
            'scope': 'Explicit snapshot only; existing site config/dependencies/model evidence may still be required'}
    sidecar = output_dir/'manifest.json'
    with sidecar.open('x') as stream: json.dump(meta, stream, indent=2)
    verify_bundle(archive, sidecar, sha)
    with (output_dir/'update.sha256').open('x') as stream: stream.write(f'{sha}  update.tar.gz\n')
    return meta


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--selection', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    a = p.parse_args()
    meta = build(a.root, strict_json(a.selection.read_text()), a.output_dir)
    print(json.dumps({'archive': meta['archive'], 'sha256': meta['sha256'], 'payload_files': meta['payload_files']}))


if __name__ == '__main__': main()
