"""Offline solver with separate recordings and immutable result directories."""
import argparse
import json
from pathlib import Path
import uuid
import yaml
from tianrover.calibration import digest, load_dataset, solve, validate


def checked_dataset(path, profile):
    data, manifest = load_dataset(path)
    if manifest.get('reference') != 'kiss_icp_only':
        raise ValueError('dataset reference must be unfused KISS-ICP')
    if manifest.get('dataset_kind') not in ('hardware', 'synthetic'):
        raise ValueError('recording provenance missing')
    if not manifest.get('recording_id') or not manifest.get('source_sha256'):
        raise ValueError('recording ID and original source checksum required')
    if manifest.get('profile_sha256') != digest(profile):
        raise ValueError('dataset geometry/clock profile does not match')
    return data, manifest


def write_result(path, files):
    # Never leave stale validation approval beside freshly solved parameters.
    path = Path(path)
    path.mkdir(parents=True, exist_ok=False)
    for name, content in files.items():
        (path/name).write_text(content)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ('solve', 'validate'):
        p = sub.add_parser(command)
        p.add_argument('--dataset', required=True)
        p.add_argument('--profile', required=True)
        p.add_argument('--output', required=True)
        if command == 'validate':
            p.add_argument('--candidate', required=True)
    opts = parser.parse_args(args)
    try:
        profile = yaml.safe_load(Path(opts.profile).read_text())
        data, manifest = checked_dataset(opts.dataset, opts.profile)
        if opts.command == 'solve':
            params, report = solve(data, profile)
            params['candidate_id'] = str(uuid.uuid4())
            params['training'] = manifest
            params['profile_sha256'] = digest(opts.profile)
            files = {'parameters.yaml': yaml.safe_dump(params, sort_keys=False)}
        else:
            candidate_path = Path(opts.candidate)
            params = yaml.safe_load(candidate_path.read_text())
            training = params['training']
            if (manifest['recording_id'] == training['recording_id']
                    or manifest['source_sha256'] == training['source_sha256']
                    or manifest['samples_sha256'] == training['samples_sha256']):
                raise ValueError('independent validation requires a different original recording')
            if params['profile_sha256'] != digest(opts.profile):
                raise ValueError('candidate profile mismatch')
            if manifest['dataset_kind'] != training['dataset_kind']:
                raise ValueError('training/validation provenance mismatch')
            report = validate(data, profile, params)
            report.update(candidate_sha256=digest(candidate_path), candidate_id=params['candidate_id'],
                          training=training, validation=manifest,
                          hardware_application_eligible=manifest['dataset_kind'] == 'hardware')
            files = {}
        report['dataset'] = manifest
        files['report.json'] = json.dumps(report, indent=2, allow_nan=False)+'\n'
        write_result(opts.output, files)
        print(json.dumps({'output': str(Path(opts.output).resolve()), 'status': report['status']}))
    except (ValueError, KeyError, OSError, TypeError) as error:
        parser.exit(2, 'Calibration refused: '+str(error)+'\n')
