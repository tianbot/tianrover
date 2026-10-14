"""Save a closed RTAB-Map database without resetting the live SLAM backend."""
import argparse
from pathlib import Path
import sqlite3
import sys
from tianrover_common.config import Config
from tianrover_slam.map_database import snapshot


def save(database, output, offline=False):
    if not offline:
        raise ValueError(
            'live backup is disabled after the NX backup/reload lost graph nodes; '
            'stop mapping_3d with Ctrl+C, wait for RTAB-Map to exit, then use --offline '
            '(launch: offline:=true). Keep bringup_3d running.')
    source = Path(database).expanduser().resolve(strict=True)
    destination = Path(output).expanduser().resolve()
    if not source.is_file():
        raise ValueError('database is not a regular file: ' + str(source))
    if source == destination or destination.exists():
        raise ValueError('snapshot destination already exists or equals the source')
    identity = source.stat()
    # Reject a database still held by a local process, before opening SQLite.
    # This checks the source inode, including alternate paths/symlinks.
    for directory in Path('/proc').glob('[0-9]*/fd'):
        try:
            descriptors = list(directory.iterdir())
        except (PermissionError, FileNotFoundError):
            continue
        for descriptor in descriptors:
            try:
                opened = descriptor.stat()
            except (PermissionError, FileNotFoundError):
                continue
            if (opened.st_dev, opened.st_ino) == (identity.st_dev, identity.st_ino):
                raise ValueError('database is still open by PID ' + directory.parent.name +
                                 '; wait for the mapping process to exit before saving')
    nodes = snapshot(source, destination)
    return dict(database=str(destination), graph_nodes=nodes, mode='offline')


def main(args=None):
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__)
    cfg = Config()
    parser.add_argument('--database', default=str(cfg.maps_dir / 'rtabmap.db'))
    parser.add_argument('--output', required=True)
    parser.add_argument('--offline', action='store_true',
                        help='Save after RTAB-Map has exited normally; no ROS services are called.')
    argv = sys.argv if args is None else ['map_save_3d', *args]
    options = parser.parse_args(remove_ros_args(argv)[1:])
    try:
        print(save(options.database, options.output, offline=options.offline))
    except (ValueError, OSError, RuntimeError, sqlite3.DatabaseError) as error:
        parser.exit(2, '3D map save refused: ' + str(error) + '\n')
