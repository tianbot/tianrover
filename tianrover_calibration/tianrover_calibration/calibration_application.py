"""Explicit hardware adapter contract; preview never calls a write operation."""
import json
import math
from pathlib import Path
import subprocess
import yaml
from tianrover_calibration.calibration import CODES, digest, validate
from tianrover_calibration.calibration_cli import checked_dataset


FIELDS = ('effective_radius_m', 'effective_diameter_m', 'steering_offset_rad')


def canonical(value):
    wheels = value['wheels']
    if set(wheels) != set(CODES):
        raise ValueError('adapter must expose all six wheels')
    result = {'schema': 1, 'wheels': {}}
    for code in CODES:
        p = {key: float(wheels[code][key]) for key in FIELDS}
        if (not all(math.isfinite(v) for v in p.values()) or p['effective_radius_m'] <= 0
                or abs(p['effective_diameter_m']-2*p['effective_radius_m']) > 1e-8):
            raise ValueError('invalid hardware parameters: '+code)
        result['wheels'][code] = p
    return result


def read(adapter):
    # The adapter owns unit conversion and hardware acknowledgement; no shell.
    result = subprocess.run([str(adapter), 'read'], check=True, capture_output=True,
                            text=True, timeout=30)
    return canonical(json.loads(result.stdout))


def write(adapter, parameters):
    subprocess.run([str(adapter), 'write'], input=json.dumps(canonical(parameters)),
                   check=True, capture_output=True, text=True, timeout=30)
    actual = read(adapter)
    expected = canonical(parameters)
    if any(abs(actual['wheels'][c][k]-expected['wheels'][c][k]) > 1e-7 for c in CODES for k in FIELDS):
        raise ValueError('hardware readback does not match requested parameters')


def approval(candidate, report, profile_path, validation_dataset):
    p = yaml.safe_load(Path(candidate).read_text())
    r = json.loads(Path(report).read_text())
    if (r.get('status') != 'validated' or r.get('hardware_application_eligible') is not True
            or r.get('candidate_sha256') != digest(candidate)):
        raise ValueError('hardware application requires a matching validated hardware candidate')
    data, manifest = checked_dataset(validation_dataset, profile_path)
    train = p['training']
    if (manifest != r['validation'] or manifest['dataset_kind'] != 'hardware'
            or train['dataset_kind'] != 'hardware' or manifest['recording_id'] == train['recording_id']
            or manifest['source_sha256'] == train['source_sha256']
            or manifest['samples_sha256'] == train['samples_sha256']
            or p['profile_sha256'] != digest(profile_path)):
        raise ValueError('independent hardware validation provenance mismatch')
    validate(data, yaml.safe_load(Path(profile_path).read_text()), p)
    return canonical(p)


def apply(candidate, report, profile, dataset, adapter, backup, execute=False):
    desired = approval(candidate, report, profile, dataset)
    if not execute:
        return {'status': 'preview', 'parameters': desired}
    adapter = Path(adapter).expanduser().resolve(strict=True)
    backup = Path(backup)
    if backup.exists():
        raise ValueError('backup path already exists')
    previous = read(adapter)
    backup.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive create + fsync ensures rollback material exists before any write.
    import os
    with backup.open('x') as stream:
        json.dump(dict(adapter=str(adapter), adapter_sha256=digest(adapter), parameters=previous), stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    try:
        write(adapter, desired)
    except Exception as original:
        try:
            write(adapter, previous)
        except Exception as restoration:
            raise RuntimeError('application failed and rollback failed; backup: '+str(backup)) from restoration
        raise ValueError('application failed; original parameters restored') from original
    return {'status': 'applied', 'backup': str(backup.resolve())}


def rollback(backup, adapter, execute=False):
    saved = json.loads(Path(backup).read_text())
    previous = canonical(saved['parameters'])
    if not execute:
        return {'status': 'rollback_preview', 'parameters': previous}
    adapter = Path(adapter).expanduser().resolve(strict=True)
    if str(adapter) != saved['adapter'] or digest(adapter) != saved['adapter_sha256']:
        raise ValueError('rollback adapter differs from saved hardware adapter')
    write(adapter, previous)
    return {'status': 'rolled_back'}


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('apply')
    for key in ('candidate', 'report', 'profile', 'dataset'):
        p.add_argument('--'+key, required=True)
    for action in (p, sub.add_parser('rollback')):
        action.add_argument('--adapter', default='')
        action.add_argument('--backup', required=True)
        action.add_argument('--execute', action='store_true', help='Explicitly write hardware; default preview')
    o = parser.parse_args(args)
    try:
        if o.execute and not o.adapter:
            raise ValueError('hardware adapter is required for execution')
        result = (apply(o.candidate, o.report, o.profile, o.dataset, o.adapter, o.backup, o.execute)
                  if o.command == 'apply' else rollback(o.backup, o.adapter, o.execute))
        print(json.dumps(result, indent=2, allow_nan=False))
    except (ValueError, KeyError, OSError, TypeError, RuntimeError, subprocess.SubprocessError) as error:
        parser.exit(2, 'Calibration application refused: '+str(error)+'\n')
