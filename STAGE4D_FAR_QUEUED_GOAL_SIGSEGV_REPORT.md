# Stage4D FAR queued-goal SIGSEGV report

## Scope

Only `far_planner` was changed. `localPlanner`, `pathFollower`, FAR geometry, Point-LIO, MuJoCo and the parent sim2sim gitlink were not edited.

## Reproduction evidence

- Historical failing full-stack run: `stage4d_runs/20260929_000204`.
- FAR logged `FAR Planner accepted the goal queued before graph initialization`, then the launch reported FAR exit code `-11` (SIGSEGV).
- FAR uses `rclcpp::spin(...)` in `src/far_planner.cpp`; callbacks and timers use the default single-threaded executor. The failure is not a concurrent goal/map callback race.

## Root cause

The first valid route is processed before `nav_node_ptr_` has been committed. `ProjectNavWaypoint(candidate_nav, nav_node_ptr_)` correctly receives a null previous node on that first route, but the old implementation then dereferenced the member `nav_node_ptr_` in two places:

1. momentum comparison: `nav_node_ptr_->position`;
2. viewpoint extension / visualization: `ExtendViewpointOnObsCloud(nav_node_ptr_, ...)` and `VizViewpointExtend(nav_node_ptr_, ...)`.

Thus queued-goal acceptance was only the timing trigger: the first planning cycle reached the null member pointer. The corrected code uses the validated function parameter `nav_node_ptr` for candidate projection and adds an explicit null-candidate failure path.

## Lifecycle safety changes

- Added a small ROS-independent `QueuedGoalLifecycle` gate.
- A pre-ready goal is retained and is consumed exactly once only when V-Graph, graph container, and odom/start vertex are valid.
- Pending ownership is cleared before `SetGoal`; reset/cancel clear the gate.
- `SetGoal` also safely queues any goal received while graph/start are unavailable.
- Added lifecycle diagnostics: `GOAL_RECEIVED`, `GOAL_QUEUED`, `VGRAPH_READY`, `QUEUED_GOAL_CONSUME_BEGIN`, `START_VERTEX_RESOLVED`, `GOAL_VERTEX_RESOLVED`, `ROUTE_SEARCH_BEGIN`, `ROUTE_SEARCH_SUCCESS`, `ROUTE_PUBLISHED`.
- `/far/planner_status` now exposes lifecycle state, queued/consumed revisions, graph node/edge counts, and start/goal vertex validity and IDs.

## Tests

Build mode: `RelWithDebInfo` (CMake no longer overwrites an explicitly requested build type).

- `far_path_validation_test`: PASS.
- `far_queued_goal_lifecycle_test`: PASS. It covers pre-ready queueing, incomplete graph/start rejection, exactly-once consumption, two updates (latest payload remains in FAR), and cancellation/clear.
- Full stack: `STAGE4D_RVIZ=false STAGE4D_AUTO_GOAL_CASE=D0 ./scripts/stage4d_start.sh`.
- Run directory: `/home/ruben/go2_diploma_sim2sim/stage4d_runs/20260929_004109`.
- FAR stayed alive, accepted D0, produced `ROUTE_SEARCH_SUCCESS` and `ROUTE_PUBLISHED`; first `ROUTE_COMMIT` was logged at `1790631696.117635197` with waypoint `(1.000, 0.000)`.
- No FAR `process has died`, `exit code -11`, or segmentation-fault line appeared before controlled shutdown. `./scripts/stage4d_stop.sh` reported `CLEAN SHUTDOWN: PASS`.

## Acceptance limitation

This full-stack D0 run delivered its goal after V-Graph readiness, so it validates the fixed first-route null-pointer path and FAR-to-localPlanner handoff but not the queued timing in the complete launch. The new deterministic lifecycle test covers that ordering. The launch shutdown log includes later crashes of unrelated Point-LIO/local-planner processes after stack teardown; they are outside this FAR-only task and were not modified.

## No promotion

No merge, push, or parent-repository gitlink update was performed.
