import copy
import json
import sys
import pytest
import yaml
from test_calibration import fixture
from test_calibration_cli import recording
from tianrover.calibration import digest
from tianrover.calibration_cli import main as solve_cli
from tianrover import calibration_application as app


def setup_candidate(root):
    _, profile, _, _ = fixture()
    p=root/'profile.yaml'
    p.write_text(yaml.safe_dump(profile))
    a=recording(root,'train',p,1)
    b=recording(root,'validation',p,2)
    # This test labels fixture input as hardware to exercise the application
    # transaction, not to claim any actual hardware acceptance.
    for d in (a,b):
        m=json.loads((d/'manifest.json').read_text())
        m['dataset_kind']='hardware'
        (d/'manifest.json').write_text(json.dumps(m))
    candidate=root/'candidate'
    validation=root/'approval'
    solve_cli(['solve','--dataset',str(a),'--profile',str(p),'--output',str(candidate)])
    c=candidate/'parameters.yaml'
    solve_cli(['validate','--dataset',str(b),'--profile',str(p),'--candidate',str(c),'--output',str(validation)])
    return c,validation/'report.json',p,b


def test_preview_does_not_access_hardware(tmp_path,monkeypatch):
    args=setup_candidate(tmp_path)
    monkeypatch.setattr(app,'read',lambda _:pytest.fail('preview touched hardware'))
    assert app.apply(*args,'missing',tmp_path/'backup')['status']=='preview'
    assert not (tmp_path/'backup').exists()


def test_apply_backup_readback_rollback_and_failure_restoration(tmp_path,monkeypatch):
    args=setup_candidate(tmp_path)
    adapter=tmp_path/'adapter'
    adapter.write_text('test adapter identity')
    original=app.canonical(yaml.safe_load(args[0].read_text()))
    original['wheels']['LF']['effective_radius_m']=.125
    original['wheels']['LF']['effective_diameter_m']=.25
    current=copy.deepcopy(original)
    backup=tmp_path/'backup.json'
    monkeypatch.setattr(app,'read',lambda _:copy.deepcopy(current))
    def write(_,value):
        assert backup.exists()
        current.clear()
        current.update(copy.deepcopy(value))
    monkeypatch.setattr(app,'write',write)
    assert app.apply(*args,adapter,backup,True)['status']=='applied'
    assert json.loads(backup.read_text())['parameters']==original
    assert current!=original
    assert app.rollback(backup,adapter,True)['status']=='rolled_back'
    assert current==original
    with pytest.raises(ValueError,match='already exists'):
        app.apply(*args,adapter,backup,True)
    backup=tmp_path/'failure-backup.json'
    calls=[]
    def fail_then_restore(_,value):
        calls.append(value)
        if len(calls)==1:
            raise ValueError('readback failure')
        current.clear()
        current.update(copy.deepcopy(value))
    monkeypatch.setattr(app,'write',fail_then_restore)
    with pytest.raises(ValueError,match='original parameters restored'):
        app.apply(*args,adapter,backup,True)
    assert len(calls)==2 and current==original


def test_candidate_changed_after_validation_refused(tmp_path):
    args=setup_candidate(tmp_path)
    args[0].write_text(args[0].read_text()+'\n# changed\n')
    with pytest.raises(ValueError,match='matching validated'):
        app.apply(*args,'missing',tmp_path/'backup')


def test_executable_adapter_must_read_back_written_values(tmp_path):
    """Exercise the subprocess contract, including an acknowledged but ignored write."""
    _, profile, _, _ = fixture()
    parameters = {'wheels': {code: dict(effective_radius_m=.125,
                                       effective_diameter_m=.25, steering_offset_rad=0.)
                             for code in profile['wheels']}}
    state = tmp_path/'state.json'
    state.write_text(json.dumps(parameters))
    adapter = tmp_path/'adapter'
    adapter.write_text('#!'+sys.executable+'\n'+
        'import json,sys\nfrom pathlib import Path\n'+
        'state=Path('+repr(str(state))+')\n'+
        "if sys.argv[1]=='read': print(state.read_text())\n"+
        "else:\n data=json.load(sys.stdin)\n"+
        " if not state.with_suffix('.ignore').exists(): state.write_text(json.dumps(data))\n")
    adapter.chmod(0o700)
    desired = copy.deepcopy(parameters)
    desired['wheels']['LF'].update(effective_radius_m=.13, effective_diameter_m=.26)
    app.write(adapter, desired)
    assert app.read(adapter) == app.canonical(desired)
    state.with_suffix('.ignore').touch()
    with pytest.raises(ValueError, match='readback does not match'):
        app.write(adapter, parameters)
    assert app.read(adapter) == app.canonical(desired)
