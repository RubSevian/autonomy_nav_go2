"""Navigation wire contracts; full point processing belongs to Point-LIO."""
import yaml
import math

# Same safe YAML constructors/resolvers, backed by libyaml when available.
# The full diagnostic payload is parsed at 10Hz; pure-Python SafeLoader was
# a measured hot path even though readiness uses only these five fields.
_SAFE_LOADER = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)


def locomotion_ready(payload):
    """Decode the existing R3 wire contract without changing readiness gates."""
    try:
        status = yaml.load(payload, Loader=_SAFE_LOADER)
        return (status['state'] == 'ACTIVE' and
                status['phase'] in ('RL_ZERO', 'RL_ACTIVE') and
                status['output_enabled'] and not status['fault_latched'] and
                status['command_source'] == 'NAVIGATION')
    except (KeyError, TypeError, yaml.YAMLError):
        return False


def valid_cloud_layout(msg):
    sizes = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    fields = {field.name: field for field in msg.fields}
    if len(fields) != len(msg.fields):
        return False
    # The configured Unitree (lidar_type=5) handler reads float32 `time`.
    # Accepting an Ouster-style `t` would admit a cloud Point-LIO cannot deskew.
    if not {'x', 'y', 'z', 'intensity', 'time'}.issubset(fields):
        return False
    if msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0:
        return False
    if msg.row_step < msg.width * msg.point_step or len(msg.data) != msg.row_step * msg.height:
        return False
    for field in fields.values():
        size = sizes.get(field.datatype, 0)
        if not size or field.count <= 0 or field.offset < 0 or field.offset + size * field.count > msg.point_step:
            return False
    return all(fields[name].datatype == 7 and fields[name].count == 1 for name in ('x', 'y', 'z', 'intensity', 'time'))


def locomotion_stopped(payload, now_ns, timeout_s):
    """Measured-pose settling additionally requires a fresh R3 zero-command report."""
    try:
        status=yaml.load(payload,Loader=_SAFE_LOADER)
        stamp=status['observed_ns'];command=status['motion_command']
        if (type(stamp) is int and 0<=now_ns-stamp<=timeout_s*1e9 and
            status['state']=='ACTIVE' and status['phase']=='RL_ZERO' and
            status['command_source']=='NAVIGATION' and status['output_enabled'] is True and
            status['fault_latched'] is False and len(command)==3 and
            all(type(v) in (int,float) and math.isfinite(v) and abs(v)<=1e-6 for v in command)):
            return stamp/1e9
    except (KeyError,TypeError,yaml.YAMLError):pass
    return None
