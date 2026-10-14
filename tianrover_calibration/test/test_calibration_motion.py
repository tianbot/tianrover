import sys
from tianrover_calibration.calibration_motion import main,plan


def test_preview_never_imports_ros_or_creates_publishers(capsys,monkeypatch):
    monkeypatch.setitem(sys.modules,'rclpy',None)
    main([])
    assert 'preview' in capsys.readouterr().out
    p=plan()
    assert sum(s['duration_s'] for s in p)==120
    assert {s['vx']>0 for s in p}=={True,False}
    assert {s['wz'] for s in p}=={0,.08,-.08}
    assert all(abs(s['vx'])<=.1 and abs(s['wz'])<=.08 for s in p)
