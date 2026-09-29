# Stage4D navigation isolation report

BRANCH: `planner_orientation_aware_footprint`
COMMIT BEFORE: `ff3529b`
COMMIT AFTER / DIRTY STATE: implementation is intentionally uncommitted; see `git status`.

## Lifecycle fixes

- `ProjectNavWaypoint()` now compares `last_point_ptr` with the new function argument `nav_node_ptr`, never the stale member.
- Duplicate-goal cache is cleared on cancel, goal reached and environment reset, so D0 may begin a new mission.
- `TryConsumePendingGoal()` runs every normal FAR main-loop iteration after graph/start refresh; it consumes exactly once only with graph + start valid.
- localPlanner clears sticky candidate identity only on a `true → false` `/navigation_active` transition; active heartbeat does not reset it.

## ROS probe

- Artifact script: `scripts/stage4d_navigation_probe.py`.
- Subscribes to FAR/localPlanner/pathFollower diagnostics, lifecycle topics, waypoint, path, constraint, command, odometry and FAR goal status.
- Uses transient-local QoS for latched lifecycle/diagnostic/constraint topics.
- Output is timestamped JSON, e.g. `python3 scripts/stage4d_navigation_probe.py --output /tmp/stage4d/mode_c.json --duration 60`.

## Modes A/B/C

No planner/controller parameter was changed and no result is fabricated. The probe is ready to record Mode A fixed-path, Mode B frozen selected path and Mode C live replanning. Launch-side source arbitration is intentionally not changed here: a fixed/frozen diagnostic run must be launched with localPlanner disabled or its publisher stopped, otherwise two `/path` publishers would invalidate the experiment.

## Validation

- FAR builds with RelWithDebInfo.
- `far_path_validation_test`: PASS.
- `far_queued_goal_lifecycle_test`: PASS, including graph-ready/start-unavailable/later-start-valid ordering.
- `localPlanner` and `pathFollower` compile successfully.

## Recorded runtime result and stop condition

The runtime acceptance was deliberately stopped at the first new crash; Modes A/B are not meaningful until this handoff is safe.

| Run | Input | Observed event sequence | Result |
|---|---|---|---|
| `stage4d_runs/20260929_115821` | normal Mode C, automatic D0 | `VGRAPH_READY`; `/navigation_active=false`; no `/way_point`, `/path`, or `/cmd_vel` | D0 was not delivered by the auto-goal helper. |
| `stage4d_runs/20260929_120054` | D0 while FAR initializes | `GOAL_RECEIVED → GOAL_QUEUED → QUEUED_GOAL_CONSUME_BEGIN` | FAR SIGSEGV before the queued-goal guard. |
| `stage4d_runs/20260929_120459` | same early D0 after the initial guard | `GOAL_RECEIVED → GOAL_QUEUED → VGRAPH_READY → QUEUED_GOAL_CONSUME_BEGIN → GOAL_VERTEX_RESOLVED` | FAR SIGSEGV during the first `PlanningCallBack`, after `SetGoal()`. |
| `stage4d_runs/20260929_120856` | same early D0 after one post-graph-cycle handoff | `GOAL_RECEIVED → GOAL_QUEUED → VGRAPH_READY → QUEUED_GOAL_CONSUME_BEGIN → ROUTE_SEARCH_SUCCESS` | PASS: no FAR crash; queued goal consumed once and route search continued. |

The early-goal loss and its post-V-Graph crash are fixed: the queued D0 now survives initialization, waits for one completed graph-update cycle, and is consumed exactly once. The first post-handoff planning cycle produced `ROUTE_SEARCH_SUCCESS` without a FAR crash. This was a **LIFECYCLE_HANDOFF** failure, not evidence against the local planner, follower, RL, obstacle avoidance, or the new footprint logic. No geometry or controller parameter was changed.

This establishes FAR goal-handoff safety and a successful fixed-path follower test. Mode B remains blocked by localPlanner publisher shutdown, and the five-trial Mode C series is recorded below.

## Acceptance execution (2026-09-29)

### C-automatic — `stage4d_runs/20260929_131308`

A 50-second ROS-native probe saw `VGRAPH_READY` but no goal, waypoint, local path, non-zero command, or goal-reached event. The automatic D0 helper did not publish a goal after readiness. This is a test-launch lifecycle failure; it is not a local-path or follower result. Shutdown was clean.

### C-manual — `stage4d_runs/20260929_131437`

D0 was published manually only after FAR became ready. The recorded chain was `GOAL_RECEIVED → ROUTE_SEARCH_SUCCESS → ROUTE_PUBLISHED → ROUTE_COMMIT`. MuJoCo accepted non-zero safe commands: initially `[0.35, 0.00, 0.00]`, later `[0.28, 0.00, 0.00]` and `[0.19, 0.00, 0.00]`, then returned to zero. FAR, localPlanner, and pathFollower had no active-navigation process crash. The process-exit messages in the log follow the explicit stop signal and are shutdown-only, not navigation failures.

The run demonstrates that the normal FAR → localPlanner → pathFollower → RL chain can build a route and execute it after a manually delivered D0. It does not demonstrate a received `GOAL_REACHED` message, because the diagnostic recording process was not retained by the shell launch; therefore completion is **not claimed**.

### Mode A — fixed path

Run: `stage4d_runs/20260929_133115/mode_a_probe.json`. The fixed `vehicle`-frame path had 41 poses. Follower entered `TRACKING`, published non-zero commands up to approximately `vx=0.35 m/s`, and the robot moved from approximately `(0.006,-0.008)` to `(0.826,0.027)`. FAR reported `goal_reached=true`. MuJoCo reported `non_floor_contacts=0`; expected floor contacts are tracked separately. Mode A PASS.

### Mode B — planner path frozen

The capture helper observed a valid planner pair with `group_id=3`, `rotation_id=18`, `direction=1`, 101 poses. The test then stopped because `localPlanner` kept publishing `/path` after SIGINT; the helper refused to introduce a second publisher. This is a reproducible publisher shutdown problem, so Mode B is **BLOCKED** until the launch provides an explicit localPlanner lifecycle stop or a separate follower-only launch. No mixed-source result was counted.

## Primary diagnosis

`LIFECYCLE_HANDOFF`. Evidence: automatic D0 delivery failed in C-automatic; early manual D0 previously caused FAR initialization crashes; a one-cycle post-V-Graph handoff removes that crash; a manually sent ready-state D0 produces FAR route commits and non-zero RL commands. No evidence from a valid isolated A/B test supports follower tracking, path geometry, replanning churn, or RL execution as the first failure.

## Baseline decision

**Not promoted.** The core queued-goal crash is fixed and compiled. The five-trial ROS-native series exists but completes 4/5, so it is not a stable baseline. Keep this worktree uncommitted and the root gitlink unchanged.

## Next navigation task

Repair or remove the automatic goal helper, provide a dedicated launch mode with localPlanner absent for A/B source isolation, and repeat until Mode B and stable C are accepted.

Mode A publisher: `scripts/stage4d_fixed_path_mode_a.py`. It refuses to run if another `/path` publisher exists, preserving the intended follower/RL isolation.

## Repeated Mode C baseline

Five identical D0 runs were executed after reset-state with manual goal delivery:

| Run | Goal reached | Path events | Follower commands | Final estimated XY |
|---|---:|---:|---:|---|
| `stage4d_runs/20260929_132527` | yes | 42 | 163 | (0.740, 0.068) |
| `stage4d_runs/20260929_132627` | yes | 41 | 162 | (0.797, 0.114) |
| `stage4d_runs/20260929_132726` | no | 47 | 155 | (0.739, 0.078) |
| `stage4d_runs/20260929_132826` | yes | 39 | 163 | (0.758, 0.114) |
| `stage4d_runs/20260929_132926` | yes | 38 | 144 | (0.744, 0.077) |

Mode C completion: **4/5**. The failed run still produced paths, tracking and non-zero commands, then stopped before the goal-reached event. This is intermittent completion instability; it is not a clean stable baseline.

## Updated decision

Primary diagnosis remains `LIFECYCLE_HANDOFF`. The queued-goal crash is fixed, but automatic goal delivery and localPlanner shutdown ownership are still unstable. A/B/C acceptance is incomplete: Mode A PASS, Mode B BLOCKED, Mode C 4/5. No promotion or merge is justified.

## Auto-goal helper fix

implementation:

- Added `scripts/stage4d_auto_goal.py`. It subscribes to the transient-local FAR planner-status diagnostic, waits for the latched `VGRAPH_READY` state, verifies exactly one FAR subscriber, and publishes D0 (`map: 1.0, 0.0`) exactly once.
- The helper records the required state sequence in its JSON artifact: `WAITING_FOR_FAR_READY`, `READY_OBSERVED`, `GOAL_PUBLISHED`, `GOAL_ACKNOWLEDGED`, `DONE`. It records `auto_goal_status`, `auto_goal_publish_timestamp`, `auto_goal_ack_timestamp`, `error`, and a timestamped transition list.
- A missing readiness state and a missing acknowledgement terminate with a non-zero status and a precise error. The acknowledgement accepts FAR's `GOAL_RECEIVED`, `GOAL_ACCEPTED`, `GOAL_PENDING`, or `WAYPOINT_PUBLISHED` lifecycle evidence.
- The helper itself is deliberately kept in `autonomy_nav_go2`. The root `scripts/stage4d_start.sh` was not changed in this task because the active experimental scope is this repository only; therefore a future integration change must replace the old CLI/sleep wrapper with this helper before automatic trials are counted.

test:

- `python3 -m py_compile scripts/stage4d_auto_goal.py`: PASS.
- Existing FAR lifecycle tests remain the relevant handoff regression: `far_queued_goal_lifecycle_test`: PASS; `far_path_validation_test`: PASS.
- No C trial is counted from this code change yet: the root launcher still owns the old automatic sender, so claiming automatic-D0 acceptance would be invalid.

result:

The helper code satisfies the ROS-native readiness/acknowledgement contract, but its runner integration and the requested five automatic C trials remain pending outside this repository's allowed edit scope.

## Failed Mode C classification — `20260929_132726`

| Field | Observation |
|---|---|
| final estimated XY / goal XY | `(0.7385, 0.0776)` / `(1.0000, 0.0000)` |
| final distance | `0.273 m` |
| FAR convergence distance | `0.250 m` (`sim_pointlio.yaml`) |
| last waypoint | `map (1.0, 0.0, -0.0195)` |
| navigation / FAR | `true` / `WAYPOINT_PUBLISHED, ROUTE_COMMITTED`, route size `2` |
| local planner / path | `WAITING_FOR_POINTCLOUD`, `path_found=true`, size `34`, candidate `3:18:1:NORMAL` |
| follower | `TRACKING`, index `0`, same candidate |
| final commands | last 10 remained non-zero or decelerating; final `vx=0.0388 m/s`, `vy=-0.0001`, `wz=0` |
| reach event | last recorded `goal_reached=false` |

The final ten command samples include forward values from `0.1400` through `0.0388 m/s`; therefore this is not a follower/path stop and navigation was not deactivated. The final pose is outside the FAR threshold by about `0.023 m`. Classification: **A — PHYSICAL_NOT_REACHED**. This is an evidence-only classification; no planner, footprint, controller, RL, sensor, or MuJoCo parameter was changed.

## Mode B source-isolation status

The prior frozen-path attempt correctly refused to replay while `localPlanner` remained a `/path` publisher. No signal-race workaround and no mixed `/path` run was introduced. A clean Mode B requires an integration launch that starts the simulator/RL/state-estimation/pathFollower chain without `localPlanner`, then replays the captured matching path/constraint pair. That launch belongs to the root `workhop_rl` integration repository and is intentionally not edited under the current `autonomy_nav_go2`-only scope. Consequently Mode B remains **BLOCKED**, and C 5/5 then 10/10 acceptance is not claimed.

## Baseline completion decision

**Not promoted.** Current verified state: Mode A PASS; Mode B BLOCKED by required clean-launch ownership; Mode C historical 4/5 with the single failure classified as PHYSICAL_NOT_REACHED. No active-navigation crash is attributed to the reviewed failed run. Do not begin obstacle or narrow-passage promotion testing before clean Mode B and integrated automatic-D0 C trials are completed.

## Independent orientation-aware flags

The former single narrow toggle is now split without changing planner scoring, geometry, follower gains, temporal stabilization, RL, or Point-LIO.

| Mode | `STAGE4D_ENABLE_ORIENTATION_AWARE_CHECK` | `STAGE4D_ENABLE_NARROW_RECOVERED_SELECTION` | Effect |
|---|---:|---:|---|
| A — baseline | `false` | `false` | Exact narrow check is not run; broad-phase behavior is unchanged. |
| B — passive diagnostic | `true` | `false` | Exact check runs and reports recoverable broad-blocked candidates, but none enter selection. |
| C — full narrow | `true` | `true` | Recovered candidates may enter selection; follower receives narrow-mode constraint only for a selected recovered candidate. |

`/localPlanner` diagnostics now include `narrow_mode`, `enable_orientation_aware_check`, `enable_narrow_recovered_selection`, `broad_phase_blocked_candidates`, `narrow_phase_checked_candidates`, `narrow_phase_recovered_candidates`, and `recovered_selected`. `recovered_selected` is zero or one for the final selected path, unlike `recovered_entered_selection`, which remains the candidate-count diagnostic.

Validation: `local_planner` rebuilt successfully; `git diff --check` passes in both repositories. The only `workhop_rl` change is `src/unitree_ros2_to_real/launch/stage4d_full_navigation.launch.py`: it adds opt-in `enable_far`/`enable_local_planner` conditions and an opt-in standalone `pathFollower` for clean Mode B. Defaults preserve the normal launch.

### Repeatability note

The completed automatic D0 series produced `GOAL_REACHED=true` for trials 1–9 and `false` for trial 10 (`stage4d_runs/20260929_142601`). Every trial used the new lifecycle helper and clean shutdown; therefore the automatic helper is verified but the 10/10 promotion rule is **not** met. This result does not justify tuning and is kept separate from the flag-split change above.
