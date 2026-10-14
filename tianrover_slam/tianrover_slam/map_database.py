"""Read-only validation and durable snapshot of an RTAB-Map database."""
import os
from pathlib import Path
import sqlite3
import tempfile


def inspect_database(path):
    path=Path(path).expanduser().resolve(strict=True)
    with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
        if db.execute('PRAGMA quick_check').fetchone()[0]!='ok':
            raise ValueError('RTAB-Map database integrity check failed')
        tables={r[0].lower() for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        if not {'node','data','link'}.issubset(tables):
            raise ValueError('not an RTAB-Map database')
        count=db.execute('SELECT count(*) FROM Node').fetchone()[0]
        if count<2:
            raise ValueError('saved map has fewer than two graph nodes')
    return count


def snapshot(source,destination):
    source=Path(source).expanduser().resolve(strict=True)
    destination=Path(destination).expanduser().resolve()
    if destination.exists() or source==destination:
        raise ValueError('snapshot destination already exists')
    inspect_database(source)
    destination.parent.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(prefix='.rtabmap-',suffix='.db',dir=destination.parent)
    os.close(fd)
    temp=Path(name)
    try:
        with sqlite3.connect(source.as_uri()+'?mode=ro',uri=True) as src,sqlite3.connect(temp) as dst:
            src.backup(dst)
        count=inspect_database(temp)
        with temp.open('rb') as stream:
            os.fsync(stream.fileno())
        # Exclusive creation: never replace an existing user's saved map.
        os.link(temp,destination)
        return count
    finally:
        temp.unlink(missing_ok=True)
