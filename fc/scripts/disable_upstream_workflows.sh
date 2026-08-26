#!/usr/bin/env bash
# Disable the ~37 workflows this fork inherits from upstream ArduPilot.
#
# Their `on: push` triggers are not branch filtered, so every push to fc/*
# would fire all of them. The files stay in the tree — deleting them would turn
# every upstream rebase into a modify-delete conflict — and the disablement is
# applied server-side, which survives pushes and rebases. New upstream
# workflows arrive enabled, so re-running this is a fixed item on the rebase
# checklist in Section 1.2 of v0-sim/docs/firmware/build-system.md.
set -euo pipefail
REPO="${FC_FORK_REPO:-Float-Cargo/ardupilot}"
gh workflow list --repo "$REPO" --limit 200 --all \
  | grep -v 'fc-build' \
  | awk -F'\t' '{print $NF}' \
  | while read -r wf; do
      [ -n "$wf" ] || continue
      echo "disabling $wf"
      gh workflow disable "$wf" --repo "$REPO" || true
    done
