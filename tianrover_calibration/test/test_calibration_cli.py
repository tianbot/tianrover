import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from test_calibration import fixture
from tianrover_calibration.calibration import digest
from tianrover_calibration.calibration_cli import main


def recording(root, name, profile, seed):
    data, _, _, _ = fixture(seed)
    path = root/name
    path.mkdir()
    np.savez(path/'samples.npz', **data)
    manifest = dict(reference='kiss_icp_only', dataset_kind='synthetic', recording_id=name,
                    source_sha256=name, samples_sha256=digest(path/'samples.npz'),
                    profile_sha256=digest(profile))
    (path/'manifest.json').write_text(json.dumps(manifest))
    return path


def test_independent_validation_and_synthetic_application_rejection(tmp_path):
    _, profile, _, _ = fixture()
    profile_path = tmp_path/'profile.yaml'
    profile_path.write_text(yaml.safe_dump(profile))
    a = recording(tmp_path, 'train', profile_path, 1)
    b = recording(tmp_path, 'independent', profile_path, 2)
    out = tmp_path/'candidate'
    main(['solve', '--dataset', str(a), '--profile', str(profile_path), '--output', str(out)])
    candidate = out/'parameters.yaml'
    with pytest.raises(SystemExit) as error:
        main(['validate', '--dataset', str(a), '--profile', str(profile_path),
              '--candidate', str(candidate), '--output', str(tmp_path/'reuse')])
    assert error.value.code == 2
    assert not (tmp_path/'reuse').exists()
    approval = tmp_path/'validation'
    main(['validate', '--dataset', str(b), '--profile', str(profile_path),
          '--candidate', str(candidate), '--output', str(approval)])
    report = json.loads((approval/'report.json').read_text())
    assert report['status'] == 'validated'
    assert report['candidate_sha256'] == digest(candidate)
    assert report['hardware_application_eligible'] is False
    # A second solve cannot overwrite the candidate associated with approval.
    with pytest.raises(SystemExit):
        main(['solve', '--dataset', str(b), '--profile', str(profile_path), '--output', str(out)])
    assert digest(candidate) == report['candidate_sha256']


def test_tampered_samples_rejected(tmp_path):
    _, profile, _, _ = fixture()
    p = tmp_path/'profile.yaml'
    p.write_text(yaml.safe_dump(profile))
    a = recording(tmp_path, 'train', p, 1)
    with open(a/'samples.npz', 'ab') as stream:
        stream.write(b'tampered')
    with pytest.raises(SystemExit):
        main(['solve', '--dataset', str(a), '--profile', str(p), '--output', str(tmp_path/'output')])
    assert not (tmp_path/'output').exists()
