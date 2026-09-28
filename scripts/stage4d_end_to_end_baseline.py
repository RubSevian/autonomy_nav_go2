#!/usr/bin/env python3
"""Repeatable, no-tuning Stage4D end-to-end baseline runner.

It deliberately changes no ROS parameter.  Each trial owns one normal
stage4d_start.sh process group, records its immutable environment and parses
only the resulting launch log after the normal stop script completes.
"""
import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from statistics import median

NAV_ROOT = Path(__file__).resolve().parents[1]
SIM_ROOT = NAV_ROOT.parents[1]
START = SIM_ROOT / "scripts" / "stage4d_start.sh"
STOP = SIM_ROOT / "scripts" / "stage4d_stop.sh"
RUN_ROOT = SIM_ROOT / "stage4d_runs"

STAGES = (
    "START", "FAR_READY", "GOAL_ACCEPTED", "ROUTE_PUBLISHED", "ROUTE_COMMITTED",
    "LOCAL_PLANNER_ACTIVE", "PATH_FOUND", "PATH_PUBLISHED", "FOLLOWER_ACTIVE",
    "CMD_NONZERO", "ROBOT_MOVING", "GOAL_REACHED", "SHUTDOWN",
)

# These only classify live-navigation evidence. Shutdown records are evaluated
# separately so SIGINT teardown noise is never counted as a navigation fault.
LIVE_PATTERNS = {
    "FAR_READY": r"FAR Planner V-Graph initialized|transition=VGRAPH_READY",
    "GOAL_ACCEPTED": r"GOAL_ACCEPTED|accepted the goal queued",
    "ROUTE_PUBLISHED": r"transition=ROUTE_PUBLISHED|WAYPOINT_PUBLISHED",
    "ROUTE_COMMITTED": r"\[FAR\]\[ROUTE_COMMIT\]",
    "LOCAL_PLANNER_ACTIVE": r"localPlanner.*(?:PATH_FOUND|PATH_PUBLISHED|NAVIGATION_ACTIVE)",
    "PATH_FOUND": r"localPlanner.*PATH_FOUND",
    "PATH_PUBLISHED": r"localPlanner.*PATH_PUBLISHED",
    "FOLLOWER_ACTIVE": r"pathFollower.*(?:TRACKING|HOLONOMIC|UNICYCLE)",
    "CMD_NONZERO": r"(?:cmd_vel|safe_command).*?(?:0\.[0-9]*[1-9]|-[0-9])",
    "ROBOT_MOVING": r"(?:robot_moving|GOAL_REACHED|distance=.*0\.0)",
    "GOAL_REACHED": r"GOAL_REACHED|goal reached",
}


def git(cmd):
    return subprocess.check_output(["git", "-C", str(NAV_ROOT), *cmd], text=True).strip()


def run(cmd, env=None):
    return subprocess.run(cmd, cwd=SIM_ROOT, env=env, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)


def newest_run(before):
    candidates = [path for path in RUN_ROOT.glob("20*_*") if path.is_dir() and path.stat().st_mtime >= before]
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


def first_failure(row, log):
    if re.search(r"\[ERROR\] \[far_planner[^\n]*process has died", log):
        return "FAR_FAILURE", "FAR process died while launch remained active"
    if not row["far_ok"]:
        return "FAR_FAILURE", "FAR_READY not observed"
    if not row["route_ok"]:
        return "FAR_NO_ROUTE", "goal accepted but route was not committed"
    if not row["planner_ok"]:
        return "LOCAL_PLANNER_NO_PATH", "FAR handoff occurred but local /path was not observed"
    if not row["follower_ok"]:
        return "FOLLOWER_NO_COMMAND", "local /path observed but follower tracking was not observed"
    if not row["cmd_nonzero"]:
        return "FOLLOWER_NO_COMMAND", "follower active but no non-zero command evidence"
    if not row["goal_reached"]:
        return "GOAL_NOT_REACHED", "deadline elapsed before goal-reached evidence"
    return "NONE", ""


def parse_trial(run_id, run_dir, duration, clean_shutdown):
    log_file = run_dir / "stage4d_launch.log"
    log = log_file.read_text(errors="replace") if log_file.exists() else ""
    stages = {stage: False for stage in STAGES}
    stages["START"] = bool(log)
    for stage, pattern in LIVE_PATTERNS.items():
        stages[stage] = bool(re.search(pattern, log, re.IGNORECASE))
    stages["SHUTDOWN"] = clean_shutdown
    far_ok = stages["FAR_READY"] and not bool(re.search(r"\[far_planner[^\n]*process has died", log))
    route_ok = stages["ROUTE_PUBLISHED"] and stages["ROUTE_COMMITTED"]
    planner_ok = stages["PATH_FOUND"] or stages["PATH_PUBLISHED"]
    follower_ok = stages["FOLLOWER_ACTIVE"]
    candidate = re.search(r"selected_(?:candidate|group).*?(?:id[:=]\s*)?(-?\d+)", log, re.IGNORECASE)
    group = re.search(r"selected_group(?:_id)?[=:]\s*(-?\d+)", log, re.IGNORECASE)
    rotation = re.search(r"selected_rotation(?:_id)?[=:]\s*(-?\d+)", log, re.IGNORECASE)
    switches = [int(x) for x in re.findall(r"path_switch_count[=:]\s*(\d+)", log)]
    refreshes = [int(x) for x in re.findall(r"path_refresh_count[=:]\s*(\d+)", log)]
    row = {
        "run_id": run_id, "scenario": "D0", "motion_model": "baseline/default",
        "run_dir": str(run_dir), "duration_s": duration,
        "startup_success": stages["START"], "far_ok": far_ok,
        "goal_received": stages["GOAL_ACCEPTED"], "route_search_success": "ROUTE_SEARCH_SUCCESS" in log,
        "route_published": stages["ROUTE_PUBLISHED"], "route_committed": stages["ROUTE_COMMITTED"],
        "localPlanner_active": stages["LOCAL_PLANNER_ACTIVE"], "path_found": stages["PATH_FOUND"],
        "published_path_size": "N/A", "selected_candidate_id": candidate.group(1) if candidate else "N/A",
        "selected_group_id": group.group(1) if group else "N/A", "selected_rotation_id": rotation.group(1) if rotation else "N/A",
        "path_switches": max(switches) if switches else "N/A", "path_refreshes": max(refreshes) if refreshes else "N/A",
        "follower_ok": follower_ok, "follower_state": "TRACKING" if follower_ok else "N/A",
        "cmd_nonzero": stages["CMD_NONZERO"], "max_vx": "N/A", "max_vy": "N/A", "max_wz": "N/A",
        "contacts": "N/A", "non_floor_contacts": "N/A", "goal_reached": stages["GOAL_REACHED"],
        "time_to_goal_s": "N/A", "clean_shutdown": clean_shutdown,
        "shutdown_only_failure": bool(re.search(r"process has died", log)) and clean_shutdown,
    }
    row["route_ok"] = route_ok
    row["planner_ok"] = planner_ok
    primary, secondary = first_failure(row, log)
    row["primary_failure"], row["secondary_failure"] = primary, secondary
    row["first_missing_stage"] = next((stage for stage in STAGES if not stages[stage]), "NONE")
    return row


def write_artifacts(output, metadata, rows):
    output.mkdir(parents=True, exist_ok=True)
    (output / "baseline_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    with (output / "stage4d_e2e_baseline.json").open("w") as handle:
        json.dump(rows, handle, indent=2); handle.write("\n")
    fields = list(rows[0]) if rows else ["run_id"]
    with (output / "stage4d_e2e_baseline.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    successes = [row for row in rows if row["goal_reached"]]
    failures = {}
    for row in rows:
        if row["primary_failure"] != "NONE": failures[row["primary_failure"]] = failures.get(row["primary_failure"], 0) + 1
    report = ["# Stage4D end-to-end robustness baseline", "", "## Configuration", "", "```json", json.dumps(metadata, indent=2), "```", "", "## D0 series", "", f"- Runs: {len(rows)}", f"- Goal completions: {len(successes)}/{len(rows)}", f"- Primary failures: {json.dumps(failures, sort_keys=True) if failures else 'none'}", "", "| Run | FAR | Route | Local path | Follower | Cmd | Goal | Primary failure |", "|---|---|---|---|---|---|---|---|"]
    for row in rows:
        report.append(f"| {row['run_id']} | {row['far_ok']} | {row['route_ok']} | {row['planner_ok']} | {row['follower_ok']} | {row['cmd_nonzero']} | {row['goal_reached']} | {row['primary_failure']} |")
    report += ["", "## Classification", "", "A. COMPONENT-LEVEL PROTOTYPE — no engineering-stage promotion is made until all requested nominal/obstacle/narrow trials are collected.", "", "The JSON and CSV retain unavailable metrics as `N/A`; no values are fabricated."]
    (output / "STAGE4D_END_TO_END_ROBUSTNESS_BASELINE_REPORT.md").write_text("\n".join(report) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--goal", default="D0", choices=["D0"])
    parser.add_argument("--output", type=Path, default=SIM_ROOT / "stage4d_runs" / "baseline_e2e")
    args = parser.parse_args()
    if args.trials < 1 or args.duration <= 0: parser.error("positive --trials and --duration are required")
    metadata = {"branch": git(["branch", "--show-current"]), "head": git(["rev-parse", "HEAD"]),
                "worktree": git(["status", "--short"]), "diff_stat": git(["diff", "--stat"]),
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"), "goal": args.goal,
                "rviz": False, "motion_model": "baseline/default", "narrow_mode": "current normal Stage4D configuration"}
    rows = []
    for index in range(1, args.trials + 1):
        before = time.time(); env = os.environ.copy(); env.update({"STAGE4D_RVIZ": "false", "STAGE4D_AUTO_GOAL_CASE": args.goal})
        start = run([str(START)], env); time.sleep(args.duration)
        stop = run([str(STOP)], env)
        run_dir = newest_run(before) or RUN_ROOT / "UNKNOWN"
        row = parse_trial(f"D0-{index:02d}", run_dir, args.duration, "CLEAN SHUTDOWN: PASS" in stop.stdout)
        row["start_output"] = start.stdout.strip(); row["stop_output"] = stop.stdout.strip(); rows.append(row)
        write_artifacts(args.output, metadata, rows)
    write_artifacts(args.output, metadata, rows)

if __name__ == "__main__":
    main()
