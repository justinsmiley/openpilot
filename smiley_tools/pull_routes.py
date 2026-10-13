#!/usr/bin/env python3
"""Pull drive logs (rlog/qlog only, no video) from a comma device over SSH.

Uses the Windows built-in OpenSSH client and a single tar-over-ssh stream, so it
needs no third-party packages. Already-downloaded files (same size) are skipped,
so it is safe to re-run after every drive. Videos and other bulky files are never
requested.

Examples:
  python pull_routes.py 192.168.1.50 --list
  python pull_routes.py 192.168.1.50
  python pull_routes.py 192.168.1.50 --route 00000012--abc123def4
  python pull_routes.py 192.168.1.50 --include-latest --key C:\\Users\\me\\.ssh\\id_ed25519

Requires SSH enabled on the device (Settings -> Developer) with your public key authorized.
"""
import argparse
import re
import shutil
import subprocess
import sys
import tarfile
import threading
from pathlib import Path

REMOTE_DIR = "/data/media/0/realdata"
DEFAULT_DEST = Path(__file__).resolve().parents[2] / "smileypilot-data" / "routes"
# only these files are pulled: full log + reduced log
FILE_PATTERNS = ("rlog*", "qlog*")
SEGMENT_RE = re.compile(r"^(?P<route>.+)--(?P<seg>\d+)$")


def ssh_base(args) -> list[str]:
  cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-p", str(args.port)]
  if args.key:
    cmd += ["-i", args.key]
  return cmd + [f"{args.user}@{args.host}"]


def list_remote_files(args) -> dict[str, int]:
  """Return {relative_path: size} for every rlog/qlog on the device."""
  name_expr = " -o ".join(f"-name '{p}'" for p in FILE_PATTERNS)
  remote = f"cd {REMOTE_DIR} && find . -mindepth 2 -maxdepth 2 -type f \\( {name_expr} \\) -printf '%P\\t%s\\n'"
  res = subprocess.run(ssh_base(args) + [remote], capture_output=True, text=True)
  if res.returncode != 0:
    sys.exit(f"ssh failed ({res.returncode}). Is the device on the network and SSH enabled?\n{res.stderr.strip()}")
  files = {}
  for line in res.stdout.splitlines():
    path, _, size = line.partition("\t")
    if path and size.isdigit():
      files[path] = int(size)
  return files


def segment_of(path: str) -> str:
  return path.split("/", 1)[0]


def route_of(path: str) -> str:
  m = SEGMENT_RE.match(segment_of(path))
  return m.group("route") if m else segment_of(path)


def newest_segment(files: dict[str, int]) -> str | None:
  """The segment most likely still being written: highest route name, then highest segment number."""
  best = None
  for path in files:
    m = SEGMENT_RE.match(segment_of(path))
    if not m:
      continue
    key = (m.group("route"), int(m.group("seg")))
    if best is None or key > best[0]:
      best = (key, segment_of(path))
  return best[1] if best else None


def safe_extract(tar: tarfile.TarFile, member: tarfile.TarInfo, dest: Path) -> None:
  target = (dest / member.name).resolve()
  if not member.isfile() or dest.resolve() not in target.parents:
    return  # ignore anything that isn't a plain file inside dest
  target.parent.mkdir(parents=True, exist_ok=True)
  src = tar.extractfile(member)
  if src is None:
    return
  tmp = target.with_suffix(target.suffix + ".part")
  with open(tmp, "wb") as out:
    shutil.copyfileobj(src, out)
  tmp.replace(target)  # only appears under its real name once complete


def fetch(args, wanted: list[str], dest: Path) -> int:
  remote = f"cd {REMOTE_DIR} && tar cf - -T -"
  proc = subprocess.Popen(ssh_base(args) + [remote], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

  def feed():
    proc.stdin.write(("\n".join(wanted) + "\n").encode())
    proc.stdin.close()

  threading.Thread(target=feed, daemon=True).start()
  count = 0
  with tarfile.open(fileobj=proc.stdout, mode="r|") as tar:
    for member in tar:
      safe_extract(tar, member, dest)
      count += 1
      print(f"\r  {count}/{len(wanted)} files", end="", flush=True)
  print()
  err = proc.stderr.read().decode(errors="replace").strip()
  if proc.wait() != 0:
    print(f"warning: ssh/tar exited {proc.returncode}: {err}", file=sys.stderr)
  return count


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("host", help="device IP or hostname")
  ap.add_argument("--user", default="comma")
  ap.add_argument("--port", type=int, default=22)
  ap.add_argument("--key", help="path to private key, if not your default one")
  ap.add_argument("--dest", type=Path, default=DEFAULT_DEST, help=f"local folder (default: {DEFAULT_DEST})")
  ap.add_argument("--route", action="append", help="only this route (repeatable), e.g. 00000012--abc123def4")
  ap.add_argument("--include-latest", action="store_true", help="also pull the newest segment (may still be recording)")
  ap.add_argument("--list", action="store_true", help="show what is on the device and exit")
  args = ap.parse_args()

  files = list_remote_files(args)
  if not files:
    sys.exit("No rlog/qlog files found on the device.")

  if args.route:
    files = {p: s for p, s in files.items() if route_of(p) in args.route}
    if not files:
      sys.exit("None of the requested routes were found.")

  if not args.include_latest and not args.route:
    latest = newest_segment(files)
    if latest:
      files = {p: s for p, s in files.items() if segment_of(p) != latest}

  routes: dict[str, list[str]] = {}
  for p in files:
    routes.setdefault(route_of(p), []).append(p)

  if args.list:
    for r in sorted(routes):
      segs = {segment_of(p) for p in routes[r]}
      mb = sum(files[p] for p in routes[r]) / 1e6
      print(f"{r}  {len(segs)} segments (~{len(segs)} min)  {mb:.0f} MB")
    return

  wanted = [p for p, size in files.items() if not ((args.dest / p).exists() and (args.dest / p).stat().st_size == size)]
  total_mb = sum(files[p] for p in wanted) / 1e6
  print(f"{len(files)} log files on device, {len(wanted)} new ({total_mb:.0f} MB) -> {args.dest}")
  if not wanted:
    print("Nothing new to download.")
    return

  args.dest.mkdir(parents=True, exist_ok=True)
  got = fetch(args, wanted, args.dest)
  print(f"Downloaded {got} files. Routes now in {args.dest}:")
  for r in sorted({route_of(p) for p in wanted}):
    n = len(list(args.dest.glob(f"{r}--*")))
    print(f"  {r}  ({n} segments)")


if __name__ == "__main__":
  main()
