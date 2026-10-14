"""Launch helpers shared by the MarsRover functional packages."""
import tempfile
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def package_share(package):
    return get_package_share_directory(package)


def generated_yaml(data):
    handle = tempfile.NamedTemporaryFile(mode='w', prefix='tianrover_', suffix='.yaml', delete=False)
    with handle:
        yaml.safe_dump(data, handle)
    def cleanup(context, *args, **kwargs):
        Path(handle.name).unlink(missing_ok=True)
        return []
    return handle.name, RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))

def include(path, **arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments={k: str(v).lower() if isinstance(v, bool) else str(v)
                                                      for k, v in arguments.items()}.items())
