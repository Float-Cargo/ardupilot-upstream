# FLOAT_CHANGES — the manifest of the Float diff on this fork

This file is the answer to "what exactly do we change?". Every commit on
`fc/main` that touches a file inherited from upstream ArduPilot adds or updates
a row here in the same commit, and carries an `upstream-touch:` trailer naming
those files, so `git log --grep=upstream-touch` enumerates the rebase risk
exactly. New files that upstream will never own — everything under `fc/`, new
`ArduPlane/*.cpp` modes, new `libraries/` classes — are listed too, but they
cost nothing at rebase time because the build globs them and upstream has no
competing version.

The branch layout this manifest describes is the one in
`v0-sim/docs/firmware/build-system.md` Section 1: `fc/sitl-base-4.7.0` is the
frozen pointer to the stock base the ship flies (upstream tag `Plane-4.7.0`,
commit `1511f27194`), and `fc/main` is the Float line rebased onto stable
upstream tags as a readable patch stack rather than merged.

## Kinds

`new` is a file upstream does not have. `edited` is an upstream file we modify,
which is the only kind that can conflict during a rebase. `deleted` is an
upstream file we remove, which we avoid because it turns every rebase into a
modify-delete conflict.

## Manifest

| File | Kind | Reason | Feature macro | Rebase notes |
|---|---|---|---|---|
| `fc/FLOAT_CHANGES.md` | new | This manifest | — | Ours alone |
| `fc/RELEASES.md` | new | Tag to approval and ship-install record | — | Ours alone |
| `fc/ci/v0sim-ref` | new | Which `v0-sim` ref the simulation gate flies against | — | Must read `main` on `fc/main` outside a release window |
| `fc/scripts/build.sh` | new | Three-target local build wrapper | — | Ours alone |
| `fc/scripts/make_manifest.py` | new | Derives `manifest.json` from the built artifacts | — | Ours alone |
| `fc/scripts/disable_upstream_workflows.sh` | new | Turns off the inherited upstream workflows server-side | — | Re-run after every rebase; new upstream workflows arrive enabled |
| `.github/workflows/fc-build.yml` | new | The fork's own continuous integration | — | Ours alone; upstream never adds a file by this name |
| `ArduPlane/float_config.h` | new | Vehicle-side spelling of the feature macros; now a one-line include of the library header | — | Ours alone |
| `libraries/AP_FloatConfig/float_config.h` | new | The `AP_FLOAT_*` feature macros with defaults, including `AP_FLOAT_PATCHES_ENABLED`, reachable from `libraries/` as well as from the vehicle | — | Ours alone |
| `libraries/AP_Motors/AP_MotorsMulticopter.h` | edited | The hover-throttle floor becomes the `Q_M_THST_HVR_MIN` parameter on the read clamp, plus the member that holds it | `AP_FLOAT_PATCHES_ENABLED` | One line of `get_throttle_hover()` and one member; upstream rewrites of this accessor are the rebase risk |
| `libraries/AP_Motors/AP_MotorsMulticopter.cpp` | edited | The same floor on the learn clamp, and the parameter table entry (group index 46) | `AP_FLOAT_PATCHES_ENABLED` | One line of `update_throttle_hover()` and one `AP_GROUPINFO`; index 46 was free at 4.7.0 and upstream may claim it |
| `ArduPlane/tiltrotor.h` | edited | Declares `Q_TILT_MAX_EXT`, `Q_TILT_YAW_MAX`, `Q_TILT_PIT_GAIN` and the two helpers that read them | `AP_FLOAT_PATCHES_ENABLED` | Member and method declarations only |
| `ArduPlane/tiltrotor.cpp` | edited | Commanded tilt may pass 90 deg to `Q_TILT_MAX_EXT`; the vectored-yaw wedge gets its own limit `Q_TILT_YAW_MAX`, decoupled from `Q_TILT_YAW_ANGLE`'s role in the tilt-to-servo mapping; `vectoring()` gains a fore/aft `pitch*sin(tilt)` term on `Q_TILT_PIT_GAIN`; parameter table entries 11-13 | `AP_FLOAT_PATCHES_ENABLED` | Four call sites of `get_forward_flight_tilt()`, `tilt_over_max_angle()`, and the output block of `vectoring()`; upstream reworks `vectoring()` occasionally, so that block is the real rebase risk |
| `libraries/AP_Motors/AP_Motors_Airship_Alloc.{h,cpp}` | new | The wrench allocator: effectiveness matrix, (Tx,Tz) reparametrisation, sector supervisor, primal active-set QP | `AP_MOTORS_AIRSHIP_ENABLED` | Ours alone; no ArduPilot includes, compiles standalone against `fc/tools/alloc_replay.cpp` |
| `libraries/AP_Motors/AP_Motors_Airship.{h,cpp}` | new | Frame class 20: `output_armed_stabilizing()` as the wrench allocation, direct tilt servo output, unclamped live hover estimate, FLAW/FLAM/FLAC allocation log | `AP_MOTORS_AIRSHIP_ENABLED` | Ours alone |
| `fc/tools/alloc_replay.cpp` | new | Standalone driver that replays (wrench, tilt state) records through the firmware's allocator translation unit for the parity gate | — | Ours alone |
| `libraries/AP_Motors/AP_Motors_config.h` | edited | `AP_MOTORS_AIRSHIP_ENABLED`, following `AP_FLOAT_MOTORS_ENABLED` when that is defined | `AP_MOTORS_AIRSHIP_ENABLED` | Seven lines appended at the end of the file; trivial to re-apply |
| `libraries/AP_Motors/AP_Motors_Class.h` | edited | `MOTOR_FRAME_AIRSHIP = 20` in `motor_frame_class` | — | One enumerator, numbered clear of upstream's run (17 today); if upstream reaches 20, renumber here and in `sim/v0.parm` |
| `ArduPlane/quadplane.h` | edited | `#include` of the airship class and `float_config.h`; `motors_airship` pointer beside `motors` | `AP_FLOAT_MOTORS_ENABLED` | Two hunks; the pointer sits under the same guard as everything that uses it |
| `ArduPlane/quadplane.cpp` | edited | Frame-class case in `setup()` (defaults and motors object), the per-cycle surge and vertical-state push in `motors_output()`, `Q_FRAME_CLASS` value list | `AP_FLOAT_MOTORS_ENABLED` | Three hunks, each a `case` or a guarded block adjacent to an upstream `switch`; the parameter doc string edit is cosmetic |
| `Tools/autotest/run_in_terminal_window.sh` | edited | Headless SITL console log goes to a per-user, per-invocation path; upstream's fixed `/tmp/$name.log` is unwritable by a second UID on a shared box, and the failure is silent | — | One line in the final `else` branch; upstream touches this file rarely (Zellij support, tmux prefix) |

## Upstream edits as of G5

Four upstream files carry `edited` rows: the motors configuration header, the frame-class
enumeration, and the two quadplane files. All four edits are additive — a new enumerator, a new
`case`, a guarded block, an appended `#ifndef` — and every one is under `AP_FLOAT_MOTORS_ENABLED`
except the enumerator and the configuration default, which are inert when the macro is off
because nothing then instantiates the class. Building with `--define AP_FLOAT_MOTORS_ENABLED=0`
therefore reproduces the `fc-4.7.0-v0.0` binary's behaviour with `Q_FRAME_CLASS 20` rejected at
boot as an unsupported frame, which is the A/B this manifest exists to make cheap.

`Tools/autotest/run_in_terminal_window.sh` is `edited` too, but it is test-harness
scaffolding rather than a compiled source: it enters no binary, sits under no feature macro,
and leaves the comparison above untouched. The counts in this section and in "The zero-diff
state" are about compiled sources only.

## Zero upstream edits at v0.0

## The zero-diff state, and what replaced it

`fc-4.7.0-v0.0` was the state in which nothing in the table was `edited`: the
compiled behavior of the fork was bit-for-bit stock `Plane-4.7.0`, because the
only additions were documentation, build and continuous-integration
scaffolding outside the compiled sources plus one header that no translation
unit included. That is what made the first genuinely custom build a change of
one variable rather than two, and it is still the comparison baseline: every
Float claim about how the vehicle flies is measured against a binary built
from this same tree with the feature macro off.

The four `edited` rows above are the first upstream touches, and they are all
one feature: the surgical patches to the stock quadplane path
(`test-flights/docs/Stabilized_forward_mode_design.md` section 3, Atlas
`projects/controls-test.md` G4). Every hunk in all four files sits inside an
`#if AP_FLOAT_PATCHES_ENABLED` guard, and each guard that replaces a stock line
carries that line unchanged in its `#else`, so with the macro off those files
are byte-identical to `fc/sitl-base-4.7.0`. `gates/g4_patches.py` in
`controls-test` asserts exactly that, file by file, by resolving the guard
itself and diffing the result against the stock blob — a stronger statement
than reading the patch, and the reason a zero-diff build remains available for
every future A/B.

Two of the four also carry a new parameter each, which is the other rebase
risk: `Q_M_THST_HVR_MIN` takes `AP_MotorsMulticopter` group index 46 and
`Q_TILT_MAX_EXT` / `Q_TILT_YAW_MAX` / `Q_TILT_PIT_GAIN` take `Tiltrotor`
indices 11 to 13. All were free at 4.7.0. If upstream claims one, the
parameter moves and the ship's parameter file moves with it.

The upstream touches the planned control work will need are named in advance in
Section 2.2 of `v0-sim/docs/firmware/build-system.md` — mode registration in
`ArduPlane/mode.h`, `ArduPlane/Plane.h` and `ArduPlane/control_modes.cpp`, one
frame-class case in `ArduPlane/quadplane.cpp`, one enumerator in
`libraries/AP_Motors/AP_Motors_Class.h`, parameter group entries in
`ArduPlane/Parameters.cpp`, and the `THISFIRMWARE` string in
`ArduPlane/version.h` on release commits. Each lands with its row here.
