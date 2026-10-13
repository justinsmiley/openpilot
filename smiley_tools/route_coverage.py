#!/usr/bin/env python3
"""Summarize how much usable tuning data a folder of pulled routes contains.

Needs pycapnp + zstandard (use a Python 3.12 venv; pycapnp has no wheels for 3.14).
Reads rlog.zst from every segment, in parallel.

  python route_coverage.py <routes_dir> <repo_root>
"""
import collections
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import capnp
import zstandard

MIN_SPEED = 15.0  # m/s, torqued ignores slower data
MPS_TO_MPH = 2.23694
# same buckets torqued uses for steering torque
TORQUE_BUCKETS = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0), (0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]
_log = None


def load_schema(root: str):
  global _log
  if _log is None:
    capnp.remove_import_hook()
    _log = capnp.load(f"{root}/openpilot/cereal/log.capnp", imports=[f"{root}/openpilot/cereal", f"{root}/opendbc_repo/opendbc/car"])
  return _log


def analyze_segment(args):
  path, root = args
  log = load_schema(root)
  data = zstandard.ZstdDecompressor().stream_reader(open(path, "rb")).read()
  r = collections.Counter()
  buckets = collections.Counter()
  lat_active = long_active = False
  torque = 0.0
  cp = None
  last_t = None
  for ev in log.Event.read_multiple_bytes(data):
    w = ev.which()
    if w == "carParams" and cp is None:
      cp = (ev.carParams.carFingerprint, bool(ev.carParams.openpilotLongitudinalControl))
    elif w == "carControl":
      lat_active, long_active = ev.carControl.latActive, ev.carControl.longActive
    elif w == "carOutput":
      torque = ev.carOutput.actuatorsOutput.torque
    elif w == "carState":
      t = ev.logMonoTime
      dt = min((t - last_t) / 1e9, 0.1) if last_t else 0.0
      last_t = t
      cs = ev.carState
      r["time"] += dt
      r["dist_m"] += cs.vEgo * dt
      r["max_v"] = max(r["max_v"], cs.vEgo)
      if lat_active:
        r["lat_active"] += dt
        if cs.vEgo >= MIN_SPEED and not cs.steeringPressed:
          r["usable"] += dt
          for lo, hi in TORQUE_BUCKETS:
            if lo <= torque < hi:
              buckets[(lo, hi)] += dt
              break
      if long_active:
        r["long_active"] += dt
      if cs.brakePressed:
        r["brake_pressed"] += dt
  return path.parent.name, cp, r, buckets


def main():
  routes_dir, root = Path(sys.argv[1]), sys.argv[2]
  files = sorted(routes_dir.glob("*/rlog.zst"))
  per_route = collections.defaultdict(collections.Counter)
  all_buckets = collections.Counter()
  cps = set()
  with ProcessPoolExecutor() as pool:
    for i, (seg, cp, r, b) in enumerate(pool.map(analyze_segment, [(f, root) for f in files], chunksize=4), 1):
      route = seg.rsplit("--", 1)[0]
      for k, v in r.items():
        per_route[route][k] = max(per_route[route][k], v) if k == "max_v" else per_route[route][k] + v
      all_buckets.update(b)
      if cp:
        cps.add(cp)
      print(f"\r{i}/{len(files)} segments", end="", flush=True)
  print("\n")

  print("car / openpilot-long:", sorted(cps))
  print(f"\n{'route':24} {'min':>5} {'mi':>6} {'lat-on':>7} {'usable':>7} {'maxmph':>7}")
  tot = collections.Counter()
  for route, r in sorted(per_route.items()):
    print(f"{route:24} {r['time']/60:5.0f} {r['dist_m']/1609.34:6.1f} {r['lat_active']/60:6.0f}m {r['usable']/60:6.0f}m {r['max_v']*MPS_TO_MPH:7.0f}")
    for k in ("time", "dist_m", "lat_active", "usable", "long_active", "brake_pressed"):
      tot[k] += r[k]
  print(f"\nTOTAL: {tot['time']/3600:.1f} h, {tot['dist_m']/1609.34:.0f} mi, lateral active {tot['lat_active']/3600:.1f} h, "
        f"usable for torque fit (>= {MIN_SPEED*MPS_TO_MPH:.0f} mph, lat active, hands off) {tot['usable']/3600:.1f} h")
  print(f"openpilot longitudinal active: {tot['long_active']/3600:.1f} h")

  print("\nsteer-torque bucket coverage (seconds; torqued wants roughly 100-500 samples/bucket at its rate):")
  for b in TORQUE_BUCKETS:
    print(f"  {b[0]:+.1f}..{b[1]:+.1f}  {all_buckets[b]:8.0f} s")


if __name__ == "__main__":
  main()
