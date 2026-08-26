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
| `ArduPlane/float_config.h` | new | `AP_FLOAT_*` feature macros with defaults | — | Ours alone; header only, no upstream file includes it yet |

## Zero upstream edits at this point

Nothing in the table is `edited`. That is the whole point of the
`fc-4.7.0-v0.0` state: the compiled behavior of this fork is bit-for-bit stock
`Plane-4.7.0`, because the only additions are documentation, build and
continuous-integration scaffolding outside the compiled sources, plus one
header that no translation unit includes. The build system, the release
pipeline and the simulation gate are therefore proven on a firmware that cannot
have changed how the vehicle flies, which is what makes the first genuinely
custom build a change of one variable rather than two.

The upstream touches the planned control work will need are named in advance in
Section 2.2 of `v0-sim/docs/firmware/build-system.md` — mode registration in
`ArduPlane/mode.h`, `ArduPlane/Plane.h` and `ArduPlane/control_modes.cpp`, one
frame-class case in `ArduPlane/quadplane.cpp`, one enumerator in
`libraries/AP_Motors/AP_Motors_Class.h`, parameter group entries in
`ArduPlane/Parameters.cpp`, and the `THISFIRMWARE` string in
`ArduPlane/version.h` on release commits. Each lands with its row here.
