# RELEASES — Float Cargo firmware tags on this fork

Tag format is `fc-<upstream-base>-v<major.minor>`, where the base is the
upstream ArduPilot tag underneath the patch stack and the Float version counts
Float changesets independently of the base, so the same Float code rebased onto
a newer base keeps its Float version. Tags are annotated and their message is
the changelog since the previous Float tag.

A tag is *built* by continuous integration. It becomes *flight approved* only
after the three-target build is green, the `v0-sim` regression suite is green at
that commit, and a bench verification on a cube has been done by a human. This
table is the record of that status; it is not a plan. Nothing in this repository
writes to a cube — flashing is a human bench action carried out from a release
asset, following Section 5 of `v0-sim/docs/firmware/build-system.md`.

Firmware identity in the field is `AUTOPILOT_VERSION.flight_custom_version`,
which carries the first eight hexadecimal digits of the git commit. That is the
ground truth a bench check compares against the tag's commit; from `v0.1`
onward the release commit also sets `THISFIRMWARE` in `ArduPlane/version.h` so
the Float identity appears in the boot status text.

| Tag | Base | Commit | Boards built | Approval status | Ship install |
|---|---|---|---|---|---|
| `fc-4.7.0-v0.0` | `Plane-4.7.0` (`1511f27194`) | see tag | `sitl`, `CubeOrangePlus` (`CubeOrange` on request) | Pipeline dry run only — zero behavioral diff from stock, deliberately not proposed for the ship | none |

The `v0.0` row exists so that the release path is exercised while the firmware
is still stock behavior. It carries no Float code, so there is nothing about it
worth flying that the ship does not already have.

## fc-4.7.0-v0.0 — zero-diff dry run (2026-08-26)

- **Commit**: `5fa07397` — last bit-stock commit on `fc/main` (base
  `Plane-4.7.0` = `1511f271` + fork CI/manifest files only; predates
  the AP_FLOAT_PATCHES set in `70e23383`).
- **Pipeline**: run 33020896414 — three board builds, armed sitl-suite
  gate (v0-sim @ main), release job. All green.
- **Purpose**: prove the rail before any behavioral diff ships
  (build-system.md P2). No approval for flight intended or implied.
- **Ship install**: NOT for the ship. Bench-flash a SPARE cube from the
  release assets (`uploader.py`, board-id checked), verify
  AUTOPILOT_VERSION reports `5fa07397`, param snapshot before/after —
  pending bench access (2026-08-27 RFS trip if a spare travels).

