# CODEX TASK — Stage4D Navigation Debug and Stable Baseline

## Goal

Finish debugging the current navigation stack with the minimum number of changes and freeze a stable diploma baseline.

Repository:
https://github.com/RubSevian/autonomy_nav_go2.git
branch: planner_orientation_aware_footprint
reviewed head before this documentation change: ff3529b

Primary chain:
goal → FAR → /navigation_active + /way_point → localPlanner → /path + /local_path_constraint → pathFollower → /cmd_vel → RL locomotion → Go2

Do not redesign the planner. Identify whether the remaining instability comes from:
- lifecycle / FAR → localPlanner handoff;
- follower / path tracking;
- geometry of selected local path;
- continuous replanning / candidate churn;
- RL execution.

## 1. Preserve the current planner

Do not initially change:
- 343-path offline library;
- path_generator.m;
- FAR global planning algorithm;
- localPlanner scoring formula;
- Point-LIO / extrinsics;
- RL policy / MuJoCo dynamics;
- robot dimensions / collision margins;
- narrow yaw thresholds;
- Pure Pursuit gains.

The two-post / constrained-gap scene is a stress test of the general planner, not a semantic gate mode.

## 2. Small lifecycle fixes

### 2.1 ProjectNavWaypoint momentum comparison
Current call uses ProjectNavWaypoint(candidate_nav, nav_node_ptr_).
Inside the function compare the previous point against the NEW candidate argument nav_node_ptr, not the member nav_node_ptr_.
Expected near comparison:
last_point_ptr->position - nav_node_ptr->position

### 2.2 Duplicate goal must be mission-scoped
Repeated publication of the same goal may be ignored only during the same active mission.
After navigation cancel, goal reached, or graph/environment reset, the same coordinates must be accepted as a new mission.
Regression: D0 → cancel → D0 again and D0 → GOAL_REACHED → D0 again.

### 2.3 Pending goal must be retry-safe
Do not consume queued goals only on the one-time V-Graph false→true transition.
Add a helper such as TryConsumePendingGoal() and call it from the normal FAR loop after graph/start state is refreshed.
Consume exactly once only when pending goal exists, graph is initialized/non-empty, and start/odom vertex is valid.

### 2.4 Reset localPlanner sticky candidate on navigation revoke
On /navigation_active=false reset activeCandidateValid, group, rotation, direction, narrow flag and score.
Do not reset candidate identity on repeated /navigation_active=true heartbeats during the same mission.

## 3. ROS-native recorder

Keep the existing baseline script for launch/timeout/shutdown/artifact collection, but do not infer planner/follower success mainly from stdout regex.
Subscribe directly to:
- /far/planner_status
- /navigation_active
- /way_point
- /local_planner/status
- /path
- /local_path_constraint
- /path_follower/status
- /cmd_vel
- /state_estimation
- /far_reach_goal_status

Record timestamps and key values: GOAL_ACCEPTED, ROUTE_COMMITTED, navigation_active, waypoint, local path_found, published_path_size, selected group/rotation/direction, path switches/refreshes, follower state, tracking index, cmd_vel, robot pose/yaw and goal reached.
stdout remains authoritative for crashes, SIGSEGV, launch errors and shutdown issues.

## 4. Mode A — FIXED_STRAIGHT_PATH

Purpose: isolate pathFollower + RL execution.
Publish a fixed straight local /path through the center of the existing obstacle-pair test and prevent localPlanner from replacing it.
Test approximately 0°, 5° and 10° initial yaw.
Record robot yaw, path tangent, heading error, cross-track error if available, vx/vy/wz, displacement, contacts and pass/fail.

Interpretation:
- FAIL → follower / RL execution first suspect.
- PASS → continue to Mode B.

## 5. Mode B — PLANNER_PATH_FROZEN

Purpose: isolate geometry/frame contract of the path selected by localPlanner.
Run normal perception and localPlanner, capture the first valid /path + matching constraint, freeze that pair and let pathFollower execute it without continuous replacement.
Record selected candidate identity, path size/tangent, robot yaw, heading/cross-track errors, vx/vy/wz, contacts and pass/fail.

Interpretation:
- Mode A PASS + Mode B FAIL → selected path geometry / frame / planner-follower contract.
- Mode A PASS + Mode B PASS → continue to Mode C.

## 6. Mode C — LIVE_REPLANNING

Restore normal continuous local planning.
Record selected group/rotation/direction, best/current score, path_switch_count, path_refresh_count, switch_reason, path tangent, robot yaw, heading error, vx/vy/wz, robot XY and contacts.
Explicitly detect oscillations such as rotation 18→19→18→19 or group 3→2→3→2.

Interpretation:
- A PASS + B PASS + C FAIL → continuous replanning / candidate churn / temporal stabilization.

## 7. Primary diagnosis

After A/B/C output one primary diagnosis:
- LIFECYCLE_HANDOFF
- FOLLOWER_TRACKING
- PLANNER_PATH_GEOMETRY
- LIVE_REPLANNING_CHURN
- RL_EXECUTION
- NO_FAILURE_REPRODUCED
- MULTIPLE_INDEPENDENT_FAILURES

Do not tune gains before this classification exists.

## 8. Apply only one minimal fix

If follower tracking fails, inspect path tangent vs robot yaw, cross-track error, lateral command and yaw command.
If path geometry fails, inspect vehicle/path transform, selected group/rotation, centerline and collision envelope.
If live replanning fails, inspect candidate score deltas, candidate identity, refresh-vs-switch, pathSwitchScoreMargin and safePathSwitchMinIntervalSec.
If RL execution fails, prove /cmd_vel is correct while simulated robot motion does not match it.

## 9. Stable baseline validation

After the minimal fix return to normal FAR + normal localPlanner + holonomic follower + normal RL.
Run 5 identical D0 trials; if stable, run 10.
Verify per run: goal accepted, route committed, navigation_active=true, waypoint delivered, local path published, follower tracking, non-zero cmd_vel, measured robot motion, goal reached, no active-navigation crash.
Keep shutdown-only failures separate from navigation failures.

## 10. Test order after D0

1. Ordinary obstacle avoidance with no semantic special mode.
2. Only then the two-post / constrained-gap stress test.
3. Orientation-aware footprint may remain a general refinement of the original FALCO-style broad collision phase.
4. Do not shrink the physical robot just to pass the stress test.

## 11. Existing tests must remain green

- far_path_validation_test
- far_queued_goal_lifecycle_test
- test_narrow_passage
- test_unicycle_follower
- follower_refresh_regression.py
- narrow_ros_smoke.py

## 12. Required report

Create STAGE4D_NAVIGATION_ISOLATION_REPORT.md with:
- branch / commit before / after / dirty state;
- lifecycle fixes;
- ROS recorder topics and QoS;
- Mode A result and metrics;
- Mode B result and selected candidate;
- Mode C result and candidate sequence / switches / refreshes;
- primary diagnosis;
- minimal fix;
- post-fix D0 successes;
- ordinary obstacle result;
- two-post stress-test result or NOT RUN;
- current baseline status;
- one justified next navigation task.

Also save machine-readable metrics for Modes A/B/C.

## 13. Repository safety

Before and after work capture git status, git diff, git diff --stat and git log --oneline --decorate -n 20.
Do not reset, discard unrelated work, merge, push automatically, or update the parent sim2sim gitlink.

## Definition of success

The task is complete when there is measured evidence explaining why the robot behaves incorrectly near the obstacle pair and the basic navigation baseline is repeatable.