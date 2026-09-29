# CODEX TASK — Stage4D Baseline Completion: Auto-Goal, Mode B, and 5/10 Repeatability

## Context

Repository:

```text
https://github.com/RubSevian/autonomy_nav_go2.git
branch: planner_orientation_aware_footprint
current branch head before this task file: ed822d2
```

Use the existing report as the factual basis:

```text
STAGE4D_NAVIGATION_ISOLATION_REPORT.md
```

The current measured state is:

```text
Mode A: PASS
Mode B: BLOCKED by localPlanner still publishing /path after SIGINT
Mode C: 4/5 goal reached
Primary diagnosis: LIFECYCLE_HANDOFF
```

Important measured facts from the latest report:

```text
- fixed-path follower test works;
- follower publishes valid non-zero commands;
- MuJoCo/Go2 moves;
- FAR can route successfully after a manually delivered ready-state D0;
- queued-goal initialization crash was fixed;
- automatic D0 helper can still fail to deliver the goal;
- one repeated Mode C run had paths + follower commands but no GOAL_REACHED;
- no evidence currently justifies tuning planner scoring, footprint, RL, or narrow-mode parameters.
```

The purpose of this task is to finish the navigation baseline acceptance, not to redesign navigation.

---

# 1. Do NOT change planner/controller math

Do NOT change in this task:

```text
343-path library
path_generator.m
FAR route scoring
localPlanner scoring
robot footprint dimensions
narrow passage thresholds
narrow state machine
holonomic follower equations
unicycle/Pure Pursuit gains
RL policy
Point-LIO
extrinsics
MuJoCo dynamics
```

Do not add new planning algorithms.

Only change infrastructure/lifecycle logic if evidence below proves it is necessary.

---

# 2. Fix the automatic D0 goal helper

Current failure:

```text
C-automatic:
VGRAPH_READY observed
but no goal was delivered
therefore no waypoint/path/cmd chain started
```

The helper must not depend on arbitrary fixed sleeps.

Required behavior:

```text
launch stack
    ↓
wait for actual FAR readiness
    ↓
publish D0 exactly once
    ↓
confirm GOAL_RECEIVED / GOAL_ACCEPTED
```

Use the ROS-native state already available from FAR diagnostics/lifecycle.

Requirements:

```text
- no fixed timing guess as the primary trigger;
- exactly one D0 publication per test run;
- no duplicate mission;
- if readiness is already latched when helper starts, still publish once;
- log helper state transitions;
- fail clearly if readiness is never observed;
- fail clearly if goal publication is not acknowledged.
```

Suggested helper states:

```text
WAITING_FOR_FAR_READY
READY_OBSERVED
GOAL_PUBLISHED
GOAL_ACKNOWLEDGED
DONE
```

Add an artifact/log field:

```text
auto_goal_status
auto_goal_publish_timestamp
auto_goal_ack_timestamp
```

---

# 3. Analyze the one failed Mode C run before tuning anything

Failed run:

```text
stage4d_runs/20260929_132726
```

Successful comparison runs:

```text
stage4d_runs/20260929_132527
stage4d_runs/20260929_132627
stage4d_runs/20260929_132826
stage4d_runs/20260929_132926
```

The failed run still had:

```text
47 path events
155 follower commands
```

but no `GOAL_REACHED`.

Compare the failed run against the four successful runs.

Extract at minimum:

```text
final estimated robot XY
goal XY
final Euclidean distance to goal

last FAR waypoint
last navigation_active value
last localPlanner status
last /path size
last path timestamp

last follower state
last tracking point index

last 10 cmd_vel samples
last non-zero cmd_vel timestamp

robot linear displacement during final seconds
robot velocity during final seconds

FAR goal tolerance / reach threshold
last far_reach_goal_status
last FAR diagnostic event
```

Determine which class applies:

```text
A. PHYSICAL_NOT_REACHED
   robot stopped outside goal tolerance

B. REACHED_BUT_EVENT_MISSING
   robot entered goal tolerance but GOAL_REACHED/status was not emitted

C. PATH_OR_FOLLOWER_STOP_EARLY
   navigation remained active but path/follower stopped commanding before tolerance

D. NAVIGATION_DEACTIVATED_EARLY
   navigation_active went false before valid goal completion

E. DATA_INSUFFICIENT
```

Do NOT tune anything until this classification is written into the report.

---

# 4. Build a clean follower-only launch for Mode B

Current Mode B is invalid because captured planner `/path` cannot be replayed while localPlanner is still a publisher.

Do not solve this by racing SIGINT or allowing two `/path` publishers.

Create a dedicated diagnostic launch/mode where:

```text
pathFollower runs
RL / sim command chain runs
state estimation runs as needed
localPlanner is NOT launched
```

Workflow:

```text
STEP 1:
normal full stack
→ capture one valid planner /path
→ capture matching /local_path_constraint
→ save both to an artifact

STEP 2:
stop full stack cleanly

STEP 3:
launch follower-only diagnostic stack
with NO localPlanner publisher

STEP 4:
replay the captured path + matching constraint exactly once

STEP 5:
observe follower + cmd_vel + robot motion
```

The captured pair must preserve:

```text
frame_id
timestamps or a controlled replay stamp
path contents
constraint candidate identity
narrow/direction flags if present
```

Do not modify path geometry between capture and replay.

---

# 5. Mode B acceptance

Record:

```text
selected_group_id
selected_rotation_id
selected_direction
path pose count
path tangent / heading
robot yaw
heading error
cross-track error if available
vx
vy
wz
robot displacement
contacts
```

Mode B passes if:

```text
follower enters TRACKING
non-zero command is produced
robot follows the frozen planner-selected path
no mixed /path source exists
no active-navigation crash occurs
```

Interpretation:

```text
Mode A PASS + Mode B PASS
→ planner-selected path is executable without live replanning

Mode A PASS + Mode B FAIL
→ inspect planner path geometry / frame contract before Mode C tuning
```

---

# 6. Re-run Mode C with the repaired automatic goal helper

After auto-goal and Mode B infrastructure are correct:

Run:

```text
5 identical Mode C D0 trials
```

Use identical:

```text
world/reset state
goal
parameters
launch files
ROS_DOMAIN_ID policy
timeout
```

Do not manually send the goal.

For each run record:

```text
GOAL_RECEIVED
ROUTE_SEARCH_SUCCESS
ROUTE_COMMIT
navigation_active=true
waypoint
path events
follower tracking
non-zero commands
robot motion
GOAL_REACHED
shutdown classification
```

If result is:

```text
5/5
```

continue immediately to:

```text
10 identical Mode C D0 trials
```

If result is less than 5/5:

```text
STOP
analyze failed run(s)
do not tune planner
```

---

# 7. Baseline promotion rule

The baseline is considered stable only if:

```text
Mode A = PASS
Mode B = PASS
Mode C = 10/10 GOAL_REACHED
no active-navigation crash
automatic goal helper is used
no manual intervention during runs
```

Shutdown-only process exit messages after the explicit stop signal do not count as navigation failures.

Do NOT require the two-post narrow stress test for baseline promotion.

---

# 8. After 10/10 only: ordinary obstacle test

Only after the baseline is accepted:

```text
normal FAR
normal localPlanner
holonomic follower
normal RL
```

Run an ordinary obstacle-avoidance scene that is not the narrow two-post case.

Record:

```text
candidate group/rotation
path switches
path refreshes
robot trajectory
contacts
goal reached
```

The objective is only to confirm standard obstacle avoidance.

Do not tune narrow-passage logic here.

---

# 9. Do NOT return to the two-post stress test yet unless baseline passes

The two-post test remains useful, but only after the basic stack is repeatable.

Do not change:

```text
footprint
yaw limits
candidate library
scoring
```

to make that test pass during this task.

---

# 10. Tests to run

Keep existing tests green:

```text
far_path_validation_test
far_queued_goal_lifecycle_test
test_narrow_passage
test_unicycle_follower
follower_refresh_regression.py
narrow_ros_smoke.py
```

Add focused tests for the new infrastructure if practical:

```text
auto-goal publishes exactly once after FAR ready
auto-goal works when readiness is already latched
follower-only launch contains no localPlanner /path publisher
frozen path replay produces a single known source
```

---

# 11. Update the existing report

Update:

```text
STAGE4D_NAVIGATION_ISOLATION_REPORT.md
```

Do not create a conflicting second conclusion.

Append:

```text
## Auto-goal helper fix
implementation:
test:
result:

## Failed run 132726 classification
final distance:
goal tolerance:
last waypoint:
last follower state:
last cmd:
classification:

## Mode B follower-only replay
captured candidate:
path size:
result:
interpretation:

## Repeated Mode C with automatic goal
5-run result:
10-run result if executed:

## Final baseline decision
PROMOTED / NOT PROMOTED

Reason:
```

Also save machine-readable artifacts.

---

# 12. Repository safety

Before work:

```bash
git status
git diff
git log --oneline --decorate -n 20
```

Do not discard existing uncommitted implementation from the previous isolation task.

Do NOT:

```text
git reset
git clean
discard unrelated changes
merge
push automatically
update a parent repository gitlink
```

If the worktree contains the previous lifecycle/probe changes, build on them.

---

# 13. Final deliverable to the user

Report:

```text
1. Was automatic goal delivery fixed?
2. Why did run 20260929_132726 fail?
3. Did Mode B pass in a true single-/path-publisher setup?
4. What was the new Mode C 5-run result?
5. If 5/5, what was the 10-run result?
6. Is the navigation baseline now promoted?
7. If not, what ONE failure remains?
```

Do not make planner/controller changes unless the measurements in this task directly justify them.
