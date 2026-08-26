#!/usr/bin/env bash
# Float Cargo firmware build wrapper — idempotent, re-run freely.
#
#   fc/scripts/build.sh [sitl|cubeorange|cubeorangeplus|all] [--define X=Y ...]
#
# Ensures a build virtual environment with the pinned dependencies exists, then
# runs waf configure plus `waf plane` for each requested board and prints the
# artifact paths. `all` finishes by deriving manifest.json from the artifacts.
#
# Why the dependency pin: ArduPilot's message generator runs on empy, and empy
# 4.x breaks it, so 3.3.4 is law here exactly as it is in v0-sim.
#
# Why the toolchain pin: upstream builds ChibiOS targets with
# gcc-arm-none-eabi-10-2020-q4-major and continuous integration uses that same
# compiler inside upstream's container image. A newer compiler changes flash
# size and warning behavior, so a locally built binary would no longer be
# comparable to the one the release pipeline produces. Set ARM_TOOLCHAIN_DIR to
# point at an unpacked copy; the usual locations are searched first.
#
# Concurrency: waf reads JOBS from the environment. BUILD_JOBS defaults to 8
# rather than the core count because this workstation runs several simulation
# stacks at once and the house cap on concurrent heavy jobs is four.
set -euo pipefail
cd "$(dirname "$0")/../.."
ROOT="$PWD"

export JOBS="${BUILD_JOBS:-8}"
VENV="${FC_BUILD_VENV:-$ROOT/.venv-build}"

targets_arg="${1:-all}"
shift || true
EXTRA=("$@")

case "$targets_arg" in
  sitl)            BOARDS=(sitl) ;;
  cubeorange)      BOARDS=(CubeOrange) ;;
  cubeorangeplus)  BOARDS=(CubeOrangePlus) ;;
  all)             BOARDS=(sitl CubeOrangePlus CubeOrange) ;;
  *) echo "usage: $0 [sitl|cubeorange|cubeorangeplus|all] [--define X=Y ...]" >&2; exit 2 ;;
esac

echo "==> build virtual environment ($VENV)"
if [ ! -x "$VENV/bin/python" ]; then
  # uv-managed CPython 3.14 when uv is available, because that is the
  # interpreter v0-sim pins on every machine; system python3 otherwise, which
  # is what the continuous-integration container has.
  UV="${UV:-$(command -v uv || echo "$HOME/.local/bin/uv")}"
  if command -v "$UV" >/dev/null 2>&1; then
    "$UV" python install 3.14
    "$UV" venv --seed --python 3.14 "$VENV"
  else
    python3 -m venv "$VENV"
  fi
  "$VENV/bin/pip" install --quiet --upgrade pip wheel
  "$VENV/bin/pip" install --quiet "empy==3.3.4" pexpect future pymavlink MAVProxy dronecan
fi
PY="$VENV/bin/python"
echo "    python: $("$PY" --version)"

need_arm=0
for b in "${BOARDS[@]}"; do [ "$b" = "sitl" ] || need_arm=1; done
if [ "$need_arm" = 1 ]; then
  echo "==> ARM toolchain (gcc-arm-none-eabi-10-2020-q4-major)"
  if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    for d in "${ARM_TOOLCHAIN_DIR:-}" \
             "$HOME/opt/gcc-arm-none-eabi-10-2020-q4-major" \
             "/opt/gcc-arm-none-eabi-10-2020-q4-major"; do
      if [ -n "$d" ] && [ -x "$d/bin/arm-none-eabi-gcc" ]; then
        export PATH="$d/bin:$PATH"; break
      fi
    done
  fi
  if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    echo "arm-none-eabi-gcc not found. Unpack" >&2
    echo "  https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2" >&2
    echo "and set ARM_TOOLCHAIN_DIR to it, or run Tools/environment_install/install-prereqs-ubuntu.sh." >&2
    exit 3
  fi
  echo "    $(arm-none-eabi-gcc --version | head -1)"
fi

for board in "${BOARDS[@]}"; do
  echo "==> configure --board $board ${EXTRA[*]:-}"
  "$PY" ./waf configure --board "$board" ${EXTRA[@]+"${EXTRA[@]}"}
  echo "==> plane (JOBS=$JOBS)"
  "$PY" ./waf plane
  echo "    artifacts:"
  ls -1 "build/$board/bin/" | sed 's/^/      /'
done

if [ "$targets_arg" = "all" ]; then
  for board in "${BOARDS[@]}"; do
    "$PY" "$ROOT/fc/scripts/make_manifest.py" "build/$board/bin"
  done
fi
echo "==> Done: ${BOARDS[*]}"
