import io,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from status_capture import StatusCapture,event_signature


def test_short_failure_and_recovery_are_kept_between_periodic_samples():
    output=io.StringIO();capture=StatusCapture(output,.5)
    def send(t,state,reason='',age=0):
        data=dict(state=state,reason=reason,observed_ns=t,age=age,
                  goal=dict(state='SUCCEEDED',revision=1,error_xy_m=age))
        capture.record(dict(topic='/nav',mono_ns=t,data=data),event_signature(data))
    send(1,'READY')
    for i in range(1,10):send(i*10_000_000,'READY',age=i)
    send(100_000_000,'BLOCKED','TF_MISSING')
    send(110_000_000,'READY')
    rows=[json.loads(line) for line in output.getvalue().splitlines()]
    assert [r['data']['state'] for r in rows] == ['READY','BLOCKED','READY']
    assert all(r['recording_kind']=='event' for r in rows)
    assert len(capture.pending)<=1


def test_periodic_retains_latest_original_time_and_shutdown_flushes_tail():
    output=io.StringIO();capture=StatusCapture(output,.5)
    for i in range(100):
        row=dict(topic='/nav',mono_ns=1+i*10_000_000,data={'age':i})
        capture.record(row,event_signature(row['data']))
        capture.flush(row['mono_ns'])
    assert capture.written==2
    capture.flush(1_000_000_000,force=True)
    rows=[json.loads(line) for line in output.getvalue().splitlines()]
    assert len(rows)==3 and rows[-1]['data']['age']==99
    assert rows[-1]['mono_ns']==990_000_001
    assert capture.summary()['pending']==0


def test_nested_reasons_readiness_and_blockers_bypass_limit_not_ages():
    old=dict(goal=dict(state='SUCCEEDED',revision=1),tf_pair=dict(reason=''),
             input_timing=dict(gap_s=.01),blockers=[])
    new=dict(old,input_timing=dict(gap_s=.9))
    assert event_signature(old)==event_signature(new)
    for change in (dict(tf_pair=dict(reason='TF_MISSING')),dict(blockers=['perception']),
                   dict(goal=dict(state='FAILED',revision=1)),dict(scene_ready=False)):
        assert event_signature(old)!=event_signature(dict(old,**change))
    assert event_signature([dict(level=0,message='OK')]) != event_signature([dict(level=2,message='FAILED')])
