#!/usr/bin/env bash
#
# Apply the TMR integration to a LABS checkout.
#
#   ./apply_to_labs.sh ~/Documents/code/labs
#
# We cannot ship these changes through the LABS repo itself: frankarobotics/labs is the
# vendor's, and we only have read access. So the delta lives here and is applied on top of
# a clean LABS checkout.
#
# What it does:
#   1. checks the LABS checkout is at the commit this patch was built against
#   2. applies labs-tmr-integration.patch  (3 source fixes + DDS/sysctl bits)
#   3. installs deployments/tmr_station/   (the station config)
#
# Re-running is safe: an already-applied patch is detected and skipped.

set -euo pipefail

BASE_COMMIT="6a62eb5ef07cc864d19bf03e70d6351bae9ee4da"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH="$HERE/labs-tmr-integration.patch"
STATION="$HERE/tmr_station"

LABS_DIR="${1:-}"
if [[ -z "$LABS_DIR" ]]; then
  echo "usage: $0 /path/to/labs" >&2
  exit 2
fi
LABS_DIR="$(cd "$LABS_DIR" && pwd)"

if [[ ! -d "$LABS_DIR/services/pipeline/data-collection" ]]; then
  echo "ERROR: $LABS_DIR does not look like a LABS checkout." >&2
  exit 1
fi

cd "$LABS_DIR"

# ---- 1. base commit -------------------------------------------------------------
current="$(git rev-parse HEAD)"
if [[ "$current" != "$BASE_COMMIT" ]]; then
  cat >&2 <<EOF
WARNING: LABS is at $current
         this patch was built against $BASE_COMMIT

The patch may still apply. If it does not, the three source changes are small and
documented in ../docs/LABS_INTEGRATION.md - reapply them by hand and regenerate this
patch with:  cd $LABS_DIR && git diff -- services .gitignore > $PATCH

EOF
  read -r -p "Continue anyway? [y/N] " reply
  [[ "$reply" == "y" || "$reply" == "Y" ]] || exit 1
fi

# ---- 2. the patch ---------------------------------------------------------------
if git apply --reverse --check "$PATCH" >/dev/null 2>&1; then
  echo "==> patch already applied, skipping"
elif git apply --check "$PATCH" >/dev/null 2>&1; then
  git apply "$PATCH"
  echo "==> applied labs-tmr-integration.patch"
else
  echo "ERROR: patch does not apply cleanly to $LABS_DIR" >&2
  echo "       run 'git apply --3way $PATCH' and resolve, or apply by hand" >&2
  echo "       (the changes are listed in docs/LABS_INTEGRATION.md)" >&2
  exit 1
fi

# ---- 3. the station config ------------------------------------------------------
DEST="$LABS_DIR/deployments/tmr_station"
if [[ -d "$DEST" ]]; then
  backup="$DEST.bak.$(date +%s)"
  mv "$DEST" "$backup"
  echo "==> existing tmr_station moved to $backup"
fi
cp -R "$STATION" "$DEST"
echo "==> installed deployments/tmr_station"

# ---- 4. the one thing that cannot be scripted -----------------------------------
ip_line="$(grep -nE '^[[:space:]]*<address>' "$DEST/fastdds_labs.xml" | head -1 || true)"
cat <<EOF

Done.

NEXT, and nothing will discover anything until you do it:

  edit $DEST/fastdds_labs.xml
       ${ip_line:-  <address>...}

  Set it to THIS HOST's address on the robot subnet - not the robot's:

      ip -4 addr show | grep 172.16.16

Then:

  cd $DEST
  task build      # first time only, ~20 min
  task start

EOF
