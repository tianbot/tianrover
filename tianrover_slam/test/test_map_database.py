import sqlite3
import pytest
from tianrover_slam.map_database import inspect_database,snapshot


def test_snapshot_preserves_graph_and_refuses_overwrite(tmp_path):
    source=tmp_path/'source.db';saved=tmp_path/'saved.db'
    with sqlite3.connect(source) as db:
        db.executescript('CREATE TABLE Node(id integer); CREATE TABLE Data(id integer); CREATE TABLE Link(id integer); INSERT INTO Node VALUES(1),(2);')
    assert snapshot(source,saved)==2
    with sqlite3.connect(source) as db:
        db.execute('DELETE FROM Node')
    assert inspect_database(saved)==2
    with pytest.raises(ValueError,match='already exists'):
        snapshot(saved,saved)
    with pytest.raises(ValueError,match='fewer than two'):
        inspect_database(source)


def test_wrong_database_and_corrupt_file_refused(tmp_path):
    path=tmp_path/'wrong.db'
    with sqlite3.connect(path) as db:
        db.execute('CREATE TABLE unrelated(id integer)')
    with pytest.raises(ValueError,match='not an RTAB'):
        inspect_database(path)
    path.write_bytes(b'corrupt database')
    with pytest.raises(sqlite3.DatabaseError):
        snapshot(path,tmp_path/'copy.db')
    assert not (tmp_path/'copy.db').exists()
