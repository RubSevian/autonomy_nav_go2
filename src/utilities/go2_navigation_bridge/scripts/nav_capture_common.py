import json
import math
from pathlib import Path

def default_recording_profile():
    # Source and installed scripts share the workspace ancestor. Read the same
    # editable source YAML as the normal NAV launch, without rebuilding for edits.
    for parent in Path(__file__).resolve().parents:
        candidate=parent/'repos/autonomy_nav_go2/src/utilities/go2_navigation_bridge/config/real_navigation.yaml'
        if candidate.is_file():return candidate
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('go2_navigation_bridge'))/'config/real_navigation.yaml'

def recording_config(path):
    import yaml
    config=yaml.safe_load(Path(path).read_text())['recording']
    for key in ('duration_s','max_duration_s','pose_capacity'):
        if type(config[key]) is not int or config[key]<=0:raise ValueError(key)
    if not 10<=config['duration_s']<=config['max_duration_s']<=540 or config['pose_capacity']>300000:
        raise ValueError('Recording duration/capacity outside existing bounds')
    for key in ('spin_timeout_s','stationary_window_s','stationary_max_radius_m','stationary_max_yaw_range_deg','minimum_stationary_s','closure_average_s','shutdown_timeout_s'):
        value=config[key]
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:raise ValueError(key)
    if config['stationary_window_s']<config['minimum_stationary_s'] or config['closure_average_s']>config['stationary_window_s']:
        raise ValueError('Inconsistent stationary windows')
    for key in ('enabled','interactive_marks','auto_bookends','analyze_on_finish'):
        if type(config[key]) is not bool:raise ValueError(key)
    if not config['output_prefix'] or Path(config['output_prefix']).name!=config['output_prefix']:
        raise ValueError('Output prefix must be one filename component')
    if not isinstance(config['point_lio_timing_topic'],str) or not config['point_lio_timing_topic'].startswith('/'):raise ValueError('point_lio_timing_topic')
    return config

def atomic(path, value):
    path=Path(path);temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n');temp.replace(path)


def diagnostic_statuses(message):
    """ROS byte fields are one-byte bytes objects in Jazzy, not JSON ints."""
    rows=[]
    for status in message.status:
        level=status.level
        if isinstance(level,bytes):
            if len(level)!=1:
                raise ValueError('DiagnosticStatus.level must contain one byte')
            level=level[0]
        if type(level) is not int or not 0<=level<=255:
            raise ValueError('Invalid DiagnosticStatus.level')
        rows.append(dict(name=status.name,level=level,message=status.message,
                         values={value.key:value.value for value in status.values}))
    return rows
