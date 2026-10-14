"""Save Cartographer pbstream and a Nav2-compatible PGM/YAML map without finishing mapping."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import uuid
from tianrover_common.config import Config


def save(map_name):
    import rclpy
    from rclpy.node import Node
    from cartographer_ros_msgs.srv import WriteState
    cfg = Config()
    base = Path(map_name).expanduser().resolve()
    state = Path(str(base)+'.pbstream')
    if any(Path(str(base)+suffix).exists() for suffix in ('.pbstream', '.yaml', '.pgm')):
        raise ValueError('map output already exists; choose a new map_name')
    base.parent.mkdir(parents=True, exist_ok=True)
    temporary = base.parent/('.'+base.name+'-'+uuid.uuid4().hex+'.pbstream')
    rclpy.init()
    node = Node('cartographer_map_save', namespace=cfg.namespace)
    client = node.create_client(WriteState, cfg.topic('write_state'))
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise ValueError('Cartographer write_state unavailable; keep mapping running')
        request = WriteState.Request(filename=str(temporary), include_unfinished_submaps=True)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=60)
        if not future.done() or future.exception():
            raise ValueError('Cartographer state save failed or timed out')
        response = future.result()
        if response.status.code != 0:
            raise ValueError('Cartographer state save refused: '+response.status.message)
        if not temporary.is_file() or temporary.stat().st_size == 0:
            raise ValueError('Cartographer did not produce a nonempty pbstream')
        os.link(temporary, state)  # exclusive destination; never replace an existing state
        try:
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                            '-t', cfg.topic('map'), '-f', str(base), '--occ', '0.51', '--free', '0.49',
                            '--ros-args', '-p', 'save_map_timeout:=10.0',
                            '-p', 'map_subscribe_transient_local:=true'], check=True, timeout=30)
        except subprocess.SubprocessError as error:
            raise ValueError('occupancy map save failed; pbstream retained at '+str(state)) from error
        if not Path(str(base)+'.yaml').is_file() or not Path(str(base)+'.pgm').is_file():
            raise ValueError('occupancy map was not saved; pbstream retained at '+str(state))
        return dict(state=str(state), map=str(base)+'.yaml')
    finally:
        if temporary.exists():
            temporary.unlink()
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-name', required=True)
    argv = sys.argv if args is None else ['cartographer_map_save', *args]
    options = parser.parse_args(remove_ros_args(argv)[1:])
    try:
        print(save(options.map_name))
    except (ValueError, OSError, RuntimeError, subprocess.SubprocessError) as error:
        parser.exit(2, 'Cartographer map save refused: '+str(error)+'\n')
