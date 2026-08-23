#!/usr/bin/env bash
# Sync the robot-side scripts between this repo and the TMR companion.
#
# Copying is a workaround for the robot not having a clone of this repo; see README.md in
# this directory for the symlink approach that removes the sync problem entirely.
#
#   ./robot/deploy.bash           # repo -> robot (backs up anything that differs)
#   ./robot/deploy.bash --check   # report differences only, change nothing
#   ./robot/deploy.bash --pull    # robot -> repo, to capture edits made on the robot
set -Eeuo pipefail

HOST="${TMR_HOST:-companion}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(dirname "$here")"

# Deployed to the robot's home directory. teleop_home_pose.yaml comes from configs/, which
# owns it, so the robot never has a second editable copy.
FILES=(start_robot.bash activate_arms.py home_arms.py recover_arms.py base_health.sh base_nudge.py fastdds_wifi.xml
       start_base.bash start_upper.bash start_zed.bash)
EXTRA_SRC="$repo/configs/teleop_home_pose.yaml"
EXTRA_DST="teleop_home_pose.yaml"

mode=push
case "${1:-}" in
  --check) mode=check ;;
  --pull)  mode=pull ;;
  "")      ;;
  *) echo "Usage: $0 [--check|--pull]" >&2; exit 2 ;;
esac

stamp="$(date +%Y%m%d-%H%M%S)"
differs=0

compare() {  # <local_path> <remote_name>
  local lp="$1" rn="$2" rc=0
  ssh "$HOST" "cat ~/$rn" 2>/dev/null | diff -q - "$lp" >/dev/null 2>&1 || rc=1
  return $rc
}

for f in "${FILES[@]}"; do
  lp="$here/$f"
  [ -f "$lp" ] || { echo "  missing locally: $f" >&2; continue; }
  if compare "$lp" "$f"; then
    [ "$mode" = check ] && echo "  same: $f"
    continue
  fi
  differs=1
  case "$mode" in
    check) echo "  DIFFERS: $f" ;;
    push)
      # Back up first: the robot copy may contain an edit that was never pulled.
      ssh "$HOST" "[ -f ~/$f ] && cp -p ~/$f ~/$f.pre-deploy-$stamp || true"
      ssh "$HOST" "cat > ~/$f" < "$lp"
      ssh "$HOST" "chmod +x ~/$f" 2>/dev/null || true
      echo "  pushed: $f  (robot backup: ~/$f.pre-deploy-$stamp)"
      ;;
    pull)
      cp -p "$lp" "$lp.pre-pull-$stamp"
      ssh "$HOST" "cat ~/$f" > "$lp"
      echo "  pulled: $f  (repo backup: $f.pre-pull-$stamp)"
      ;;
  esac
done

# configs/teleop_home_pose.yaml is push-only: the repo owns it.
if [ "$mode" != pull ] && [ -f "$EXTRA_SRC" ]; then
  if compare "$EXTRA_SRC" "$EXTRA_DST"; then
    [ "$mode" = check ] && echo "  same: $EXTRA_DST (from configs/)"
  else
    differs=1
    if [ "$mode" = check ]; then
      echo "  DIFFERS: $EXTRA_DST (from configs/)"
    else
      ssh "$HOST" "[ -f ~/$EXTRA_DST ] && cp -p ~/$EXTRA_DST ~/$EXTRA_DST.pre-deploy-$stamp || true"
      ssh "$HOST" "cat > ~/$EXTRA_DST" < "$EXTRA_SRC"
      echo "  pushed: $EXTRA_DST (from configs/)"
    fi
  fi
fi

if [ "$mode" = check ]; then
  (( differs )) && { echo; echo "Repo and robot differ. ./robot/deploy.bash to push, --pull to capture."; exit 1; }
  echo "Repo and robot are in sync."
fi
