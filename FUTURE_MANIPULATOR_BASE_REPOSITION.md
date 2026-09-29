# Future manipulator integration — base reposition after unreachable grasp

Status: design note only. Do not implement until the Stage4D navigation baseline is stable.

## Intended sequence

1. Manipulator remains folded on the Go2 back during normal navigation.
2. Go2 receives a normal navigation target and reaches the search area with normal tolerance.
3. Robot stops.
4. Manipulator moves to a predefined SCAN pose.
5. RGB-D camera detects the required object on the ground and estimates its 3D pose.
6. Existing GraspNet pipeline generates top-K grasp candidates.
7. IK / FK / Cartesian waypoint / collision feasibility is checked.

If a safe reachable grasp exists:
IK SUCCESS → pregrasp → grasp → retreat.

If the object is detected but no valid grasp is reachable:
IK UNREACHABLE → preserve object pose → estimate object relative to current base → compute a new safe base pre-grasp pose → move arm to a safe/home locomotion pose → send the new base goal to the normal navigation stack → Go2 moves closer → stop → SCAN again → re-localize object → retry grasp planning.

## Important design rule

The detected object position must NOT be used directly as the Go2 base navigation goal.

Compute a separate base target:
BASE_PREGRASP_POSE = [x_base, y_base, yaw_base]

The target base pose should place the object inside the usable RARS01 workspace while satisfying:
- safe Go2 body clearance;
- suitable base orientation;
- IK feasibility;
- joint limits;
- manipulator ↔ Go2 collision constraints;
- manipulator ↔ environment collision constraints.

## Object coordinates

Do not preserve the object only in the instantaneous camera frame.
After first detection transform it into a navigation-fixed/world frame so that the robot base can move while retaining an estimate of the object position.
The exact fixed frame (Point-LIO/world/map equivalent used by the final navigation stack) must be verified during integration.

Use the preserved object pose to generate the reposition navigation target.
Before final grasp, re-detect/re-localize the object with RGB-D because depth noise, camera extrinsic error, navigation error and localization drift can accumulate.

## Proposed state machine

NAV_TO_SEARCH_AREA
→ STOP
→ ARM_SCAN
→ OBJECT_DETECTED
→ GRASP_PLANNING

Branch A:
IK OK → GRASP

Branch B:
IK UNREACHABLE
→ BASE_REPOSITION
→ ARM_SAFE / HOME
→ NAVIGATE
→ STOP
→ ARM_SCAN
→ RELOCALIZE
→ GRASP_PLANNING

Add guards against infinite loops:
- max_reposition_attempts;
- object_lost;
- no_safe_base_pose;
- IK_fail_after_reposition;
- collision_reject.

## Relation to current GraspNet / IK pipeline

Keep the current inner grasp pipeline:
RGB-D → YOLO/GraspNet → top-K candidates → camera→base_link → IK → FK validation → Cartesian waypoint IK → candidate scoring → pregrasp→grasp→retreat.

The reposition logic is an outer coordinator:
no reachable safe candidate → reposition coordinator → new Go2 navigation goal → retry the same grasp pipeline.

Navigation and manipulation remain separate subsystems; the coordinator connects them.

## Questions for later integration

1. Which fixed frame stores the object pose?
2. How is BASE_PREGRASP_POSE computed?
3. Single target distance or sampled reachable workspace?
4. How should base yaw be selected?
5. Should several candidate base poses be evaluated?
6. How many reposition attempts are allowed?
7. When should detector / GraspNet run again?
8. How is Go2 + RARS01 collision geometry represented during locomotion and final approach?
9. How is base reposition coordinated with the full robot collision checker?

This file is intentionally documentation only and must not alter the current navigation debugging scope.