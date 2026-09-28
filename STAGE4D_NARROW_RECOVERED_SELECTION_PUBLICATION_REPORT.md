# STAGE4D — NARROW RECOVERED CANDIDATE SELECTION

## BRANCH

`planner_orientation_aware_footprint` in `RubSevian/autonomy_nav_go2`.

## WORKTREE BEFORE / AFTER

The worktree already contained the experimental narrow-passage, follower, temporal-selection, message, launch and test changes. They were preserved. This task added final-pool tracing in `localPlanner.cpp`, made final selection status publish after selection rather than exposing a stale pre-selection `PLANNING` snapshot, strengthened the existing narrow ROS smoke test, and retained the earlier short-goal geometry correction.

No reset, checkout-overwrite, parent gitlink update, commit, merge or push was performed. FAR, Point-LIO, extrinsics, RL, MuJoCo geometry, path library, footprint dimensions, margins and curvature thresholds were not changed.

## ROOT CAUSE

Two independent observations had been conflated.

1. The previous full D0 snapshot was a **pre-selection** `PLANNING` status. It could show counters from a prior or in-progress cycle while `selected_group_id=-1` and `path_found=false`, so it did not prove that recovered candidates disappeared downstream.
2. `checkNarrowGroup()` rejected all remaining paths shorter than 0.30 m as `NARROW_GEOMETRY_UNAVAILABLE`, even when two valid samples were available. For a D0 remaining distance near 0.29 m this was a false geometry rejection. The checker now requires at least two points and lets the actual tangent/curvature/swept-footprint checks decide.

The recovered candidate does clear the broad-phase rejection for scoring: the existing condition `!broadBlocked || narrowRecovered` is the final eligibility gate. The synthetic acceptance confirms that a recovered group enters the exact score array used by final selection and is selected/published.

## RECOVERED PIPELINE — DETERMINISTIC NARROW SMOKE

Gap: 0.66 m, obstacle rows, narrow mode on, same immutable path library and footprint.

| Stage | Result |
|---|---:|
| Candidate total | 12,348 |
| Broad blocked | 4,263 |
| Narrow checked | 4,263 |
| Narrow recovered | 49 |
| Recovered after curvature filter | 49 |
| Recovered after direction filter | 49 |
| Recovered after other/scoring filters | at least 1 group |
| Recovered entered final selection | 1 group |
| Final selectable groups | 33 |
| Best candidate | `group=3, rotation=18` |
| Best recovered candidate | `group=3, rotation=18` |
| Selection failure | `NONE` |
| Selected group | 3 |
| `path_found` | true |
| Published path size | 101 |
| Follower state | `NARROW_APPROACH → NARROW_TRAVERSE` |
| Command | `vx=0.14 m/s`, `vy=0` |

This directly proves `RECOVERED → selectable → selected → /path` for the straight group 3. The new status fields identify both `group:rotation` candidates and expose whether recovery reaches final selection.

## CURVATURE

For the successful group-3 recovered candidate, the final reason is `NARROW_RECOVERED`; it is not rejected by curvature. Other candidate groups/rotations can legitimately report `NARROW_CURVATURE_TOO_HIGH`; that is recorded separately and does not overwrite the reason for the selected/best recovered candidate. No curvature limit was loosened.

## REJECTION CASE

At 0.30 m gap, narrow recovery is zero and the narrow candidate is rejected. The planner still selects an ordinary safe group 4 path (`path_found=true`, 101 poses), demonstrating that a failed narrow candidate does not leave stale broad-phase state in the final pool.

## HYSTERESIS

The existing selection code starts from `bestCandidateIndex`. When `activeCandidateValid=false`, no hysteresis branch can replace it, so a valid recovered candidate is selected immediately. When the current candidate is unsafe/missing, `SAFETY_CURRENT_BLOCKED` leaves the best valid candidate selected. The safe-to-safe hold/rate limit applies only when `currentSafe=true`.

## PUBLICATION GUARD

Final selection diagnostics are now published only after final selection. A selected non-stop candidate constructs `path` from `startPaths`; the smoke test confirms 101 poses. The existing stop-path remains a one-pose explicit revocation. The status now reports `final_selectable_count`, best normal/recovered IDs, `selection_failure_reason`, `path_found` and `published_path_size` together.

## TESTS

- `colcon build --symlink-install --packages-select local_planner`: PASS.
- `test_narrow_passage`: 6/6 PASS.
- `test_unicycle_follower`: 5/5 PASS.
- `follower_refresh_regression.py`: PASS in `holonomic` and `unicycle`.
- `narrow_ros_smoke.py --enable --gap 0.66`: PASS; asserts recovered entered final selection, selected group valid, `path_found=true`, path size >1, narrow traverse and `vx>0.1`.
- `narrow_ros_smoke.py --enable --gap 0.30 --expect-reject`: PASS; no narrow recovery, ordinary safe path selected.

## FULL MUJOCO D0

**PARTIAL.** Full Stage4D was started twice and stopped through `scripts/stage4d_stop.sh` (`CLEAN SHUTDOWN: PASS`). In the final run, FAR accepted the queued goal after V-Graph initialization then exited with code `-11`. This occurs before local planner candidate handoff, so it is outside the scope of this task and prevents the requested full-stack first-level acceptance. The synthetic ROS test is the acceptance evidence for recovered selection/publication; the full D0 result cannot yet be claimed.

## RESULT

**PARTIAL PASS.** The local planner recovered-candidate pipeline is proven in a deterministic ROS integration test. Full MuJoCo D0 is blocked by an existing FAR process crash after queued-goal acceptance.

## NEXT

Diagnose the FAR `-11` on queued goal acceptance in a separate task, then rerun the unchanged D0 full-stack acceptance and inspect the final local-planner status fields.
