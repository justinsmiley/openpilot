#!/usr/bin/env python3
"""Offline fit of lateral torque params (latAccelFactor, friction) from pulled routes,
replicating openpilot/selfdrive/locationd/torqued.py, plus a steering-activity baseline.

  python fit_torque.py <routes_dir> <repo_root>

Needs numpy, pycapnp, zstandard in a Python 3.12 venv.
"""
import collections
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import capnp
import numpy as np
import zstandard

G = 9.81
DT_MDL = 0.05
MIN_VEL = 15.0
STEER_MIN = 0.02
LAT_ACC_MAX = 1.0
MIN_ENGAGE = 2.0
BUCKETS = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0), (0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]
PER_BUCKET = 1500
FRICTION_FACTOR = 1.5
HIGHWAY_V = 25.0  # m/s, about 56 mph
_log = None


def schema(root):
  global _log
  if _log is None:
    capnp.remove_import_hook()
    _log = capnp.load(f"{root}/openpilot/cereal/log.capnp", imports=[f"{root}/openpilot/cereal", f"{root}/opendbc_repo/opendbc/car"])
  return _log


def rot_from_euler(rpy):
  r, p, y = rpy
  rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
  ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
  rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
  return rz @ ry @ rx


def count_reversals(angle, min_swing):
  """Direction changes of the steering angle whose swing is >= min_swing degrees (hysteresis)."""
  n, ref, direction = 0, angle[0], 0
  for a in angle[1:]:
    if direction >= 0 and a < ref - min_swing:
      n += direction == 1
      direction, ref = -1, a
    elif direction <= 0 and a > ref + min_swing:
      n += direction == -1
      direction, ref = 1, a
    elif (direction >= 0 and a > ref) or (direction <= 0 and a < ref):
      ref = a
    if direction == 0:
      direction = 1 if a > angle[0] else -1
  return n


def segment(args):
  path, root = args
  log = schema(root)
  data = zstandard.ZstdDecompressor().stream_reader(open(path, "rb")).read()
  cc_t, cc_a, co_t, co_v, cs_t, cs_v, cs_p, cs_ang = [], [], [], [], [], [], [], []
  dm_t, dm_w, dm_roll, lag, calib = [], [], [], [], np.eye(3)
  for ev in log.Event.read_multiple_bytes(data):
    w, t = ev.which(), ev.logMonoTime * 1e-9
    if w == "carControl":
      cc_t.append(t); cc_a.append(ev.carControl.latActive)
    elif w == "carOutput":
      co_t.append(t); co_v.append(-ev.carOutput.actuatorsOutput.torque)
    elif w == "carState":
      c = ev.carState
      cs_t.append(t); cs_v.append(c.vEgo); cs_p.append(c.steeringPressed); cs_ang.append(c.steeringAngleDeg)
    elif w == "deviceMotion":
      d = ev.deviceMotion
      if d.angularVelocityDevice.valid and d.orientationNED.valid and d.inputsOK and d.sensorsOK and d.posenetOK:
        dm_t.append(t); dm_roll.append(d.orientationNED.x)
        dm_w.append([d.angularVelocityDevice.x, d.angularVelocityDevice.y, d.angularVelocityDevice.z])
    elif w == "extrinsicsCalibration":
      calib = rot_from_euler(np.array(ev.extrinsicsCalibration.rpyCalib)).T
    elif w == "lateralDelay":
      lag.append(ev.lateralDelay.lateralDelay)

  out = {"points": np.empty((0, 2)), "hw_time": 0.0, "hw_rev": 0, "hw_dtq": 0.0, "hw_arate": 0.0}
  if min(len(cc_t), len(co_t), len(cs_t), len(dm_t)) < 50:
    return path.parent.name, out
  lag_s = float(np.median(lag)) if lag else 0.2
  cc_t, co_t, cs_t, dm_t = (np.asarray(x) + lag_s for x in (cc_t, co_t, cs_t, dm_t))  # torqued shifts every stream by lag
  dm_t_raw = dm_t - lag_s
  cc_a, co_v, cs_v, cs_p = (np.asarray(x, dtype=float) for x in (cc_a, co_v, cs_v, cs_p))
  w_cal = (calib @ np.asarray(dm_w).T).T
  vego = np.interp(dm_t_raw, cs_t, cs_v)
  steer = np.interp(dm_t_raw, co_t, co_v)
  lat_acc = vego * w_cal[:, 2] - np.sin(np.asarray(dm_roll)) * G
  pts = []
  for i, t in enumerate(dm_t_raw):
    if not (vego[i] > MIN_VEL and abs(steer[i]) > STEER_MIN and abs(lat_acc[i]) <= LAT_ACC_MAX):
      continue
    grid = np.arange(t - MIN_ENGAGE, t + lag_s, DT_MDL)
    if grid[0] < cc_t[0] or not np.interp(grid, cc_t, cc_a).astype(bool).all():
      continue
    if np.interp(grid, cs_t, cs_p).astype(bool).any():
      continue
    pts.append((steer[i], lat_acc[i]))
  out["points"] = np.asarray(pts).reshape(-1, 2)

  # steering activity while lateral is engaged on the highway, hands off
  cs_t_raw = cs_t - lag_s
  ang = np.asarray(cs_ang)
  mask = (np.interp(cs_t, cc_t, cc_a) > 0.5) & (cs_v > HIGHWAY_V) & (cs_p < 0.5)
  if mask.sum() > 500:
    dt = np.diff(cs_t_raw, prepend=cs_t_raw[0])
    dt = np.clip(dt, 0, 0.1)
    out["hw_time"] = float(dt[mask].sum())
    # reversals counted over contiguous masked runs
    idx = np.flatnonzero(mask)
    runs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
    for run in runs:
      if len(run) > 200:
        out["hw_rev"] += count_reversals(ang[run], 0.5)
    out["hw_arate"] = float((np.abs(np.gradient(ang, cs_t_raw))[mask] * dt[mask]).sum())
  return path.parent.name, out


def fit(points):
  """Same total-least-squares fit as TorqueEstimator.estimate_params, with per-bucket caps."""
  rng = np.random.default_rng(0)
  chosen = []
  for lo, hi in BUCKETS:
    b = points[(points[:, 0] >= lo) & (points[:, 0] < hi)]
    if len(b) > PER_BUCKET:
      b = b[rng.choice(len(b), PER_BUCKET, replace=False)]
    chosen.append(b)
  p = np.vstack(chosen)
  m = np.column_stack([p[:, 0], np.ones(len(p)), p[:, 1]])
  _, _, v = np.linalg.svd(m, full_matrices=False)
  slope, offset = -v.T[0:2, 2] / v.T[2, 2]
  sin, cos = np.sqrt(slope**2 / (slope**2 + 1)), np.sqrt(1 / (slope**2 + 1))
  spread = (m[:, [0, 2]] @ np.array([[cos, -sin], [sin, cos]])).T[1]
  return slope, offset, np.std(spread) * FRICTION_FACTOR, [len(c) for c in chosen]


def main():
  routes, root = Path(sys.argv[1]), sys.argv[2]
  files = sorted(routes.glob("*/rlog.zst"))
  by_route = collections.defaultdict(list)
  hw = collections.Counter()
  with ProcessPoolExecutor() as pool:
    for i, (seg, out) in enumerate(pool.map(segment, [(f, root) for f in files], chunksize=4), 1):
      route = seg.rsplit("--", 1)[0]
      by_route[route].append(out["points"])
      for k in ("hw_time", "hw_rev", "hw_arate"):
        hw[k] += out[k]
      print(f"\r{i}/{len(files)}", end="", flush=True)
  print("\n")
  allp = np.vstack([p for ps in by_route.values() for p in ps])
  print(f"total usable points: {len(allp)} (~{len(allp) * DT_MDL / 60:.0f} min at 20 Hz)")
  slope, offset, fric, counts = fit(allp)
  print(f"\nFIT (all routes): latAccelFactor={slope:.3f}  latAccelOffset={offset:.3f}  friction={fric:.3f}")
  print(f"bucket point counts (after cap): {counts}")
  print("current (Sonata substitute):      latAccelFactor=2.964  friction=0.078")

  print("\nper-route fits (stability check):")
  for route, ps in sorted(by_route.items()):
    pts = np.vstack(ps)
    if len(pts) < 2000:
      continue
    s, o, f, _ = fit(pts)
    print(f"  {route}  n={len(pts):6d}  factor={s:.3f} offset={o:+.3f} friction={f:.3f}")

  rng = np.random.default_rng(1)
  boot = []
  for _ in range(30):
    s, _, f, _ = fit(allp[rng.choice(len(allp), len(allp))])
    boot.append((s, f))
  b = np.array(boot)
  print(f"\nbootstrap (30x): factor {b[:,0].mean():.3f} +/- {b[:,0].std():.3f}, friction {b[:,1].mean():.3f} +/- {b[:,1].std():.3f}")

  if hw["hw_time"] > 0:
    mins = hw["hw_time"] / 60
    print(f"\nBASELINE highway steering activity ({mins:.0f} min engaged, hands off, > 56 mph):")
    print(f"  steering direction reversals (>=0.5 deg swing): {hw['hw_rev'] / mins:.1f} per minute")
    print(f"  mean steering-wheel speed: {hw['hw_arate'] / hw['hw_time']:.2f} deg/s")


if __name__ == "__main__":
  main()
