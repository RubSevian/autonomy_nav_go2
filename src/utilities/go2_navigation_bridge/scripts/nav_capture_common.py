import json
import math
import os
from pathlib import Path
import time

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
    period=config.get('resource_period_s')
    if type(period) not in (int,float) or not math.isfinite(period) or not 1 <= period <= 10:
        raise ValueError('resource_period_s must be 1..10 seconds')
    return config


class LaunchResourceSampler:
    """Read-only /proc accounting for the recorder's own ROS launch tree."""
    def __init__(self, root_pid=None, proc=Path('/proc')):
        self.root_pid=os.getppid() if root_pid is None else root_pid
        self.proc=Path(proc)
        self.previous={}
        self.ticks=os.sysconf('SC_CLK_TCK')
        self.page_kib=os.sysconf('SC_PAGE_SIZE')/1024

    def sample(self):
        now=time.monotonic_ns()
        records={};children={}
        try:
            entries=self.proc.iterdir()
        except OSError:
            entries=()
        for base in entries:
            if not base.name.isdigit():continue
            try:
                stat=(base/'stat').read_text()
                fields=stat[stat.rindex(')')+2:].split()
                pid=int(base.name);ppid=int(fields[1])
                records[pid]=(stat,fields)
                children.setdefault(ppid,[]).append(pid)
            except (OSError,ValueError,IndexError):
                continue
        pending=[self.root_pid];seen=set();processes=[];current={}
        while pending:
            pid=pending.pop()
            if pid in seen:continue
            seen.add(pid);pending.extend(children.get(pid,()))
            if pid not in records:continue
            try:
                stat,fields=records[pid]
                name=stat[stat.index('(')+1:stat.rindex(')')]
                try:
                    argv=(self.proc/str(pid)/'cmdline').read_bytes().split(b'\0')
                    args=[part.decode('utf-8','replace') for part in argv if part]
                    executable=Path(args[0]).name if args else name
                    label=Path(next((arg for arg in args[1:] if arg.endswith('.py')),args[0])).name if executable.startswith('python') and args else executable
                except OSError:
                    label=name
                key=(pid,int(fields[19])) # start ticks prevent PID reuse errors
                cpu_ticks=int(fields[11])+int(fields[12])
                prior=self.previous.get(key)
                cpu_percent=None if prior is None or now<=prior[0] else max(0.,100*(cpu_ticks-prior[1])*1e9/(self.ticks*(now-prior[0])))
                current[key]=(now,cpu_ticks)
                processes.append(dict(pid=pid,ppid=int(fields[1]),name=name,label=label,
                    cpu_percent=cpu_percent,rss_kib=int(fields[21])*self.page_kib,threads=int(fields[17])))
            except (ValueError,IndexError):
                continue
        self.previous=current
        memory={}
        try:
            for line in (self.proc/'meminfo').read_text().splitlines():
                key=line.partition(':')[0]
                if key in ('MemAvailable','SwapFree','SwapTotal'):
                    memory[key+'_kib']=int(line.split()[1])
        except (OSError,ValueError,IndexError):pass
        paging={}
        try:
            for line in (self.proc/'vmstat').read_text().splitlines():
                key,value=line.split()
                if key in ('pswpin','pswpout','pgmajfault'):paging[key]=int(value)
        except (OSError,ValueError):pass
        return dict(mono_ns=now,root_pid=self.root_pid,root_seen=self.root_pid in records,
                    processes=sorted(processes,key=lambda row:row['cpu_percent'] or 0,reverse=True),memory=memory,paging_counters=paging)

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
