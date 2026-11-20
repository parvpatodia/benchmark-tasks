# robocurve-kit fastrun.py 03cdad8c15fc
# Parts of this file (_no_cameras, _put_back, the step-limit and result-line logic) are adapted from demokit.py in
# robocurve/benchmark-tasks, tasks/chalk-line (by K-XZY, merged as b23660a, 2026-11-14), which that task's README
# publishes under the MIT License. The source file carries no licence header of its own; the MIT terms follow.
# Changes here: it drives the stock yam_scripted policy (a task's demo() waypoints) instead of a demo_script
# generator, takes a flock and waits for free GPU slots before the simulator starts, reuses one black image per
# camera, writes a per-step trace.csv, and checks each mode's score against the task's EXPECT.
#
#   MIT License
#
#   Copyright (c) 2026 the authors of tasks/chalk-line (K-XZY)
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
#   documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
#   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all copies or substantial portions of
#   the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#   WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#   OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""fastrun.py: every mode of a task's yam_scripted demo in ONE simulator session, cameras off. Box only.

    source /data/IsaacLab-Arena/yam.env
    /data/IsaacLab-Arena/.venv/bin/python fastrun.py task.py OUT --modes demo,nothing,one_arm [--fast] [--trace]

Default: GPU physics (cuda:0), no camera sensors, no renderer. --fast: physics on the CPU as well. --cameras keeps
the harness's cameras (for a like-for-like timing check). Each mode reaches the task as env TASK_DEMO (and as the
module's VARIANT, if it has one, for older tasks), runs from a reset that also puts free objects and mechanisms back,
and writes OUT/<mode>/result.json (task_score, steps, steps/s, worst IK miss per arm) and OUT/<mode>/trace.csv (per
step: score, each arm's label, observed and commanded TCP, the gap between them, grip, droop trim, every free object's
pose; with --trace also the task's trace_row(env) columns). With several --seeds, folders are OUT/<mode>_s<seed>.

Before the simulator starts it takes an exclusive flock on $LOCKS/sim.lock[.b|.c] (--lock, default b; LOCKS default
~/parv/locks) and then waits while nvidia-smi lists >= SIMCAP (default 3) compute apps. Exit code: the number of modes
whose score differs from the task's EXPECT (0 when it has none); 2 for a setup error.

Fidelity (inspect_robots 1.0.0, the box's version): each trial is inspect_robots.rollout.rollout() itself, called as
eval._run_eval calls it for `inspect-robots run --instruction ... --seed N --max-steps M`: policy.bind(info), the
embodiment's and policy's bind_task(envelope), eval._before_trial (health, prepare_trial, home, when offered),
policy.on_trial_start (when offered), seed derive_seed(N, None, 0), DefaultActionManager(policy.config.replan_interval),
and the CLI's default guard chain (cli._build_guardrails, plus any guards in the inspect-robots defaults env). The
score is yam_arena's task_score scorer on the returned Record. trace.csv comes from a log sink (rollout's log_step).
UNTESTED at the kit's first version: no fastrun run has finished on the box yet (the first try failed on a 0.60-only
import, fixed here); whether CPU physics (--fast) gives the GPU's scores, and whether a camera-free Kit still takes GPU
memory, are unmeasured. Report scores from the film.
"""
from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import os
import pathlib
import re
import shutil
import subprocess
import sys
import time
import zlib

import numpy as np

ARMS = ("left", "right")
CONTROL_HZ = 20
_BLIND = {"on": False}


# -- guards -------------------------------------------------------------------------------------------------------------

def lock_path(lock: str, locks_dir: str | None = None) -> pathlib.Path:
    base = pathlib.Path(locks_dir or os.environ.get("LOCKS") or "~/parv/locks").expanduser()
    name = {"a": "sim.lock", "b": "sim.lock.b", "c": "sim.lock.c"}.get(lock)
    if name is None:
        raise SystemExit(f"--lock must be a, b or c, got {lock!r}")
    base.mkdir(parents=True, exist_ok=True)
    return base / name


def take_lock(path: pathlib.Path):
    """Block until this process holds an exclusive flock on `path`; the lock lives as long as the returned file."""
    fh = open(path, "a")
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(f"[fastrun] waiting for {path}", flush=True)
        fcntl.flock(fh, fcntl.LOCK_EX)
    print(f"[fastrun] holding {path}", flush=True)
    return fh


def gpu_apps(smi: str = "nvidia-smi") -> int:
    out = subprocess.run([smi, "--query-compute-apps=pid", "--format=csv,noheader"], capture_output=True, text=True,
                         check=True).stdout
    return sum(1 for line in out.splitlines() if line.strip())


def wait_for_slot(simcap: int, poll_s: float = 20.0, smi: str = "nvidia-smi") -> None:
    """Wait while nvidia-smi lists >= simcap compute apps. SIMCAP 0 skips the check (a machine with no GPU)."""
    if simcap <= 0:
        return
    if shutil.which(smi) is None:
        raise SystemExit(f"{smi} not found: set SIMCAP=0 only on a machine with no shared GPU")
    n = gpu_apps(smi)
    while n >= simcap:
        print(f"[fastrun] {n} compute apps on the GPU (SIMCAP {simcap}): waiting", flush=True)
        time.sleep(poll_s)
        n = gpu_apps(smi)


# -- the simulator, cameras off (from demokit) --------------------------------------------------------------------------

def _no_cameras() -> None:
    """Start Kit with no camera sensors and no renderer: observations carry black images (one array per camera, made
    once). A task or scorer that reads camera images cannot run this way. Nothing on disk changes."""
    _BLIND["on"] = True
    from isaaclab.app import AppLauncher
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    start_app = AppLauncher.__init__

    def app_without_cameras(self, *a, **k):
        k["enable_cameras"] = False
        start_app(self, *a, **k)
        import yam_arena.embodiment as embodiment  # needs the app

        build = embodiment.BimanualYamEmbodiment.__init__

        def without_cameras(body, *a2, **k2):
            k2["enable_cameras"] = False
            build(body, *a2, **k2)

        embodiment.BimanualYamEmbodiment.__init__ = without_cameras

    AppLauncher.__init__ = app_without_cameras
    black: dict = {}

    def blind_observation(self):
        from inspect_robots.types import Observation
        from inspect_robots_yam.config import DEFAULT_CAMERAS
        from inspect_robots_yam.packing import STATE_KEY, pack

        key = (self.cam_height, self.cam_width)
        if key not in black:
            black[key] = {n: np.zeros((*key, 3), dtype=np.uint8) for n in DEFAULT_CAMERAS}
        images = black[key]
        per_arm = []
        for name in ("left_robot", "right_robot"):
            arm_ids, finger_ids = self._ids[name]
            q = self._base.scene[name].data.joint_pos.torch[0]
            grip = float(np.clip(q[finger_ids].mean().item() / FINGER_OPEN, 0.0, 1.0))
            per_arm.append(np.append(q[arm_ids].cpu().numpy().astype(np.float64), grip))
        return Observation(images=dict(images), state={STATE_KEY: pack(*per_arm)}, instruction=self._instruction,
                           image_times=dict.fromkeys(images, self._t), state_time=self._t)

    YamArenaEmbodiment._observation = blind_observation


def _put_back(emb) -> None:
    """The harness's reset returns the arms only; free objects and mechanisms stay as the last trial left them. Put
    them back so that several modes share one session (mechanism.py then rests mechanisms and runs on_place)."""
    base = emb._base
    for obj in base.scene.rigid_objects.values():
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] += base.scene.env_origins
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=obj.data.default_root_vel.torch.clone() * 0.0)
    for name, art in base.scene.articulations.items():
        if name.endswith("_robot"):
            continue
        art.write_joint_position_to_sim_index(position=art.data.default_joint_pos.torch.clone())
        art.write_joint_velocity_to_sim_index(velocity=art.data.default_joint_vel.torch.clone() * 0.0)


# -- the harness's choices, read from the installed packages --------------------------------------------------------------

def trial_seed(cli_seed: int) -> int:
    """The seed `inspect-robots run --seed N` passes to the embodiment for an ad-hoc scene's first epoch."""
    try:
        from inspect_robots.rollout import derive_seed
        return int(derive_seed(cli_seed, None, 0))
    except ImportError:
        return zlib.crc32(f"{cli_seed}:None:0".encode()) & 0xFFFFFFFF


def guardrails(emb):
    """(guard, name): the CLI's default chain for the embodiment's action space (1.0.0: cli._build_guardrails, with
    the guards the inspect-robots defaults env adds, as _cmd_run passes them). No fallback: a missing helper is a
    version change that must be looked at, not papered over."""
    from inspect_robots.cli import _build_guardrails
    from inspect_robots.defaults import load_defaults
    extra = list(load_defaults(os.environ).guards)
    guard, active, warns = _build_guardrails(emb.info.action_space, None, emb, extra, defaults=True)
    for w in warns:
        print(f"[fastrun] guardrails warning: {w}", flush=True)
    return guard, " + ".join(active) if active else "none active"


def step_limit(module, task: pathlib.Path) -> int:
    if hasattr(module, "RUN_LIMIT_STEPS"):
        return int(module.RUN_LIMIT_STEPS)
    path = task.parent / "task.json"
    if path.exists():
        found = re.search(r"[\d,]+", str(json.loads(path.read_text()).get("run_limit", "")))
        if found:
            return int(found.group().replace(",", ""))
    return 2400


def instruction_for(task: pathlib.Path, module) -> str:
    path = task.parent / "task.json"
    if path.exists():
        return json.loads(path.read_text()).get("instruction") or "demo"
    return getattr(module, "INSTRUCTION", "demo")


# -- one trial ----------------------------------------------------------------------------------------------------------

def _bases(module) -> dict:
    from yam_arena import station
    from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS
    return dict(station.BASES) if getattr(module, "SCENE", "station") == "station" else \
        {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}


def _set_mode(module, mode: str) -> None:
    os.environ["TASK_DEMO"] = mode
    if hasattr(module, "VARIANT"):   # WHY: our pre-kit tasks pick their stem from a module constant
        module.VARIANT = mode


def _yaw(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def default_states(emb, names):
    from yam_arena.scene import object_states
    return object_states(emb._env, names)


class TraceSink:
    """A log sink for rollout(): one trace.csv row per executed step (the action after the guard)."""

    def __init__(self, fh, emb, module, policy, states, trace: bool):
        from yam_arena import ik
        self.ik, self.emb, self.module, self.policy, self.states, self.trace = ik, emb, module, policy, states, trace
        self.writer, self.fh = csv.writer(fh), fh
        self.bases = _bases(module)
        task = getattr(emb, "task", None)
        self.tracked = [o.name for o in (task.objects if task is not None else ()) if getattr(o, "free", False)
                        or hasattr(o, "all_links")]
        kit = sys.modules.get("motionkit")
        self.corr = kit.active_corrector() if kit is not None and hasattr(kit, "active_corrector") else None
        self.cols = ["step", "t", "score", "success"]
        for a in ARMS:
            self.cols += [f"{a}_label", *(f"{a}_obs_{c}" for c in "xyz"), *(f"{a}_cmd_{c}" for c in "xyz"),
                          f"{a}_gap_mm", f"{a}_grip_obs", f"{a}_grip_cmd", f"{a}_trim_mm"]
        for n in self.tracked:
            self.cols += [f"{n}_{c}" for c in ("x", "y", "z", "yaw")]
        self.extra_cols = None
        self.first_step_at = None
        self.last_info: dict = {}

    def log_step(self, t, observation, action, result) -> None:
        from inspect_robots_yam.packing import STATE_KEY, split
        if self.first_step_at is None:
            self.first_step_at = time.perf_counter()
        n = t + 1
        info = dict(result.info or {})
        self.last_info = info
        cmd = split(np.asarray(action.data, float))
        seen = split(np.asarray(result.observation.state[STATE_KEY], float))
        row = [n, round(n / CONTROL_HZ, 3), info.get("score"), int(bool(info.get("success")))]
        plans = getattr(self.policy, "_plans", {})
        for i, a in enumerate(ARMS):
            label = plans[a].target(n / CONTROL_HZ)[3] if a in plans else "ready"
            po = self.ik.tcp_pose(seen[i][:6], self.bases[a])[0]
            pc = self.ik.tcp_pose(cmd[i][:6], self.bases[a])[0]
            trim = self.corr.tcp_shift_mm(a, cmd[i][:6]) if self.corr is not None and a in plans else 0.0
            row += [label, *np.round(po * 1000, 2), *np.round(pc * 1000, 2),
                    round(float(np.linalg.norm(pc - po)) * 1000, 2), round(float(seen[i][6]), 4),
                    round(float(cmd[i][6]), 4), round(trim, 2)]
        if self.tracked:
            st = self.states(self.emb, self.tracked)
            for name in self.tracked:
                row += [*np.round(np.asarray(st[name].pos, float) * 1000, 2), round(_yaw(st[name].quat_xyzw), 3)]
        if self.trace and hasattr(self.module, "trace_row"):
            extra = self.module.trace_row(self.emb._env)
            if self.extra_cols is None:
                self.extra_cols = list(extra)
            row += [extra.get(k) for k in self.extra_cols]
        if n == 1:
            self.writer.writerow(self.cols + (self.extra_cols or []))
        self.writer.writerow(row)
        if n % 100 == 0:
            rate = n / max(time.perf_counter() - self.first_step_at, 1e-9)
            print(f"[fastrun] step {n}, {rate:.1f} steps/s, score {info.get('score')}", flush=True)


def score_record(record) -> float:
    """yam_arena's task_score scorer on the Record (1.0 for success, else the last step's info score, else 0)."""
    try:
        from yam_arena.scoring import task_score
        return float(task_score()(record, None).value)
    except ImportError:   # WHY: same rule, for a yam_arena without scoring.py
        last = record.steps[-1] if record.steps else None
        if last is None:
            return 0.0
        if last.result.terminated and last.result.termination_reason == "success":
            return 1.0
        return float(last.result.info.get("score") or 0.0)


def play(emb, module, mode: str, cli_seed: int, out: pathlib.Path, *, make_policy, max_steps: int, instruction: str,
         trace: bool = False, put_back=_put_back, states=default_states, make_guard=guardrails) -> dict:
    """One trial of `mode` in the session `emb` holds: inspect_robots' own rollout(), set up as eval._run_eval does."""
    from inspect_robots import Scene
    from inspect_robots.action_manager import DefaultActionManager
    from inspect_robots.rollout import rollout
    from inspect_robots.task import TaskEnvelope

    out.mkdir(parents=True, exist_ok=True)
    _set_mode(module, mode)
    if emb._env is not None:
        put_back(emb)
    policy = make_policy()
    if callable(getattr(policy, "bind", None)):
        policy.bind(emb.info)
    envelope = TaskEnvelope(name="adhoc", max_steps=int(max_steps))
    for obj in (emb, policy):
        if callable(getattr(obj, "bind_task", None)):
            obj.bind_task(envelope)
    scene = Scene(id="scene-0", instruction=instruction)
    try:
        from inspect_robots.eval import _before_trial
        not_ready, halt = _before_trial(emb, scene, 0)
    except ImportError:
        not_ready, halt = None, None
    if not_ready is not None:
        raise SystemExit(f"[fastrun] embodiment not ready: {not_ready}") from halt
    if callable(getattr(policy, "on_trial_start", None)):
        policy.on_trial_start(scene.id, 0, str(out), "fastrun")
    manager = DefaultActionManager(getattr(getattr(policy, "config", None), "replan_interval", None))
    guard, guard_name = make_guard(emb)
    seed = trial_seed(cli_seed)
    reset0 = emb.reset
    timing = {}

    def timed_reset(*a, **k):   # WHY: an instance attribute, so rollout's embodiment.reset() is timed, then removed
        b = time.perf_counter()
        try:
            return reset0(*a, **k)
        finally:
            timing["reset_s"] = time.perf_counter() - b

    emb.reset = timed_reset
    begin = time.perf_counter()
    with open(out / "trace.csv", "w", newline="") as fh:
        sink = TraceSink(fh, emb, module, policy, states, trace)
        try:
            record = rollout(policy, emb, scene, max_steps=int(max_steps), seed=seed, epoch=0, action_manager=manager,
                             guard=guard, sink=sink)
        finally:
            try:
                del emb.reset
            except AttributeError:
                emb.reset = reset0
    total = time.perf_counter() - begin
    steps = len(record.steps)
    wall = total - timing.get("reset_s", 0.0)
    success = bool(record.terminated and record.termination_reason == "success")
    plans = getattr(policy, "_plans", {})
    info = sink.last_info
    res = {"mode": mode, "seed_cli": cli_seed, "seed_trial": seed, "task_score": score_record(record),
           "success": success, "termination": record.termination_reason, "status": record.status,
           "error": record.error, "steps": steps, "seconds": round(steps / CONTROL_HZ, 2),
           "plan_s": {a: round(p.duration, 2) for a, p in plans.items()}, "wall_s": round(wall, 2),
           "reset_s": round(timing.get("reset_s", 0.0), 2), "steps_per_s": round(steps / wall, 2) if wall > 0 else None,
           "ik_worst": {a: [round(p.worst_ik_mm, 2), round(p.worst_ik_deg, 2)] for a, p in plans.items()},
           "guardrails": guard_name, "cameras": not _BLIND["on"],
           "final_info": {k: v for k, v in info.items() if isinstance(v, (int, float, bool))}}
    (out / "result.json").write_text(json.dumps(res, indent=1))
    print(f"[fastrun] RESULT {mode} s{cli_seed}: task_score {res['task_score']}, {res['termination']}, steps {steps} "
          f"({res['seconds']} s), {res['steps_per_s']} steps/s, reset {res['reset_s']} s, IK worst {res['ik_worst']}"
          + (f", status {record.status}: {record.error}" if record.status != "success" else ""), flush=True)
    return res


def run_modes(emb, module, modes, seeds, out: pathlib.Path, **kw) -> list[dict]:
    results = []
    for mode in modes:
        for s in seeds:
            folder = out / (mode if len(seeds) == 1 else f"{mode}_s{s}")
            results.append(play(emb, module, mode, s, folder, **kw))
    return results


def expected_misses(module, results) -> list[str]:
    want = getattr(module, "EXPECT", {})
    return [f"{r['mode']} s{r['seed_cli']}: {r['task_score']} (EXPECT {want[r['mode']]})" for r in results
            if r["mode"] in want and abs(r["task_score"] - float(want[r["mode"]])) > 1e-6]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Run every mode of a task's yam_scripted demo in one session. See the docstring.")
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--modes", help="comma list (default: the task's MODES, else demo)")
    ap.add_argument("--seeds", default="0", help="CLI seeds, comma list (default 0), as inspect-robots run --seed")
    ap.add_argument("--fast", action="store_true", help="CPU physics as well as no cameras")
    ap.add_argument("--cameras", action="store_true", help="keep the harness's cameras (render as the CLI does)")
    ap.add_argument("--device", help="physics device, overrides --fast (cuda:0 or cpu)")
    ap.add_argument("--max-steps", type=int, help="default: RUN_LIMIT_STEPS, else task.json run_limit, else 2400")
    ap.add_argument("--hold", type=float, default=3.0, help="yam_scripted's hold after the plan (CLI default 3)")
    ap.add_argument("--trace", action="store_true", help="add the task's trace_row(env) columns to trace.csv")
    ap.add_argument("--lock", default="b", help="a, b or c: $LOCKS/sim.lock[.b|.c] (default b)")
    ap.add_argument("--simcap", type=int, default=int(os.environ.get("SIMCAP", "3")))
    a = ap.parse_args(argv)
    if a.fast and a.cameras:
        ap.error("--fast has no cameras")
    task = pathlib.Path(a.task).resolve()
    out = pathlib.Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    lock = take_lock(lock_path(a.lock))   # noqa: F841  WHY: held until the process exits
    wait_for_slot(a.simcap)
    os.environ.pop("TOP_CAM_VIDEO", None)  # WHY: mechanism.py would record the top camera, which needs cameras
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    device = a.device or ("cpu" if a.fast else "cuda:0")
    begin = time.perf_counter()
    sys.path.insert(0, str(task.parent))
    if not a.cameras:
        _no_cameras()
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.scripted import yam_scripted_policy

    emb = yam_arena_embodiment(task=str(task), object_jitter=0.0, control_hz=CONTROL_HZ, cam_width=320, cam_height=176,
                               device=device)
    module = emb.task.module
    modes = a.modes.split(",") if a.modes else list(getattr(module, "MODES", None) or ["demo"])
    seeds = [int(s) for s in a.seeds.split(",")]
    results = run_modes(emb, module, modes, seeds, out, make_policy=lambda: yam_scripted_policy(hold_s=a.hold),
                        max_steps=a.max_steps or step_limit(module, task), instruction=instruction_for(task, module),
                        trace=a.trace)
    misses = expected_misses(module, results)
    summary = {"task": str(task), "device": device, "cameras": a.cameras, "total_wall_s": round(time.perf_counter() - begin, 1),
               "results": results, "expect_misses": misses}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(f"[fastrun] DONE {len(results)} runs in {summary['total_wall_s']} s on {device}, cameras {'on' if a.cameras else 'off'}"
          + (f"; EXPECT misses: {misses}" if misses else ""), flush=True)
    sys.stdout.flush()
    os._exit(len(misses))   # WHY: Kit can hang in close() at exit (inspect_embodiment caps it at 30 s); results are written


if __name__ == "__main__":
    main()
