#!/usr/bin/env bash
# Diagnose (and optionally repair) the realtime conditions the arm FCI loops need.
#
# WHY THIS EXISTS
# ---------------
# The FR3's FCI expects a command packet every 1 ms. When enough of them arrive late the
# robot aborts the motion ITSELF with:
#
#   libfranka: Move command aborted: motion aborted by reflex! ["communication_constraints_violation"]
#
# That is a robot-side reflex, not a controller bug, and nothing in the ROS log says which
# of the several possible causes fired. This script checks all of them in one pass.
#
# WHAT IT CHECKS, AND WHY EACH ONE MATTERS
# ----------------------------------------
#   1. FIFO scheduling.  controller_manager calls realtime_tools::configure_sched_fifo().
#      If the rtprio limit is 0 it logs "Could not enable FIFO RT scheduling policy" and
#      the 1 kHz loop then competes with every normal process on the box. The companion
#      routinely sits at load 12-14 on 12 cores with the full stack up (see the Robotiq
#      500 -> 100 Hz commit), so losing FIFO is fatal on its own.
#
#   2. Memory locking.  ros2_control_node also calls lock_memory(), and the robot has been
#      observed logging "Unable to lock the memory: 'No proper privileges to lock the
#      memory!'". This is NOT benign for a 1 kHz loop. FIFO priority stops other processes
#      preempting the thread; it does nothing about a page fault inside it, which can stall
#      the loop for milliseconds regardless of priority. Missing memlock is the one failure
#      that survives a correctly prioritised, otherwise idle system.
#
#   3. PREEMPT_RT.  Franka requires it for 1 kHz control. On a generic kernel the tail of
#      the scheduling-latency distribution reaches into the milliseconds, so the reflex
#      fires occasionally no matter how the limits are set.
#
#   4. The FCI network interface.  Both arms (172.16.16.11 / .12), the spine (.10) and the
#      teleop laptop's DDS traffic all share one flat /24. Franka wants FCI on a dedicated
#      link precisely because discovery multicast and joint-state bursts queue ahead of the
#      control stream. Drops or overruns on the interface confirm it.
#
#   5. Live load, and the actual scheduling class of the running ros2_control_nodes.
#
# USAGE
#   ./configs/check_realtime.sh              # from the LAPTOP, over ssh to the companion
#   ./configs/check_realtime.sh --local      # on the companion itself
#   ./configs/check_realtime.sh --fix        # additionally install the rtprio/memlock limits
#   ./configs/check_realtime.sh --host other
#
# --fix writes /etc/security/limits.d/99-tmr-realtime.conf and nothing else. It does NOT
# touch the NIC: changing coalescing on a live interface bounces the link, which is not
# something to do underneath a running robot. The ethtool command is printed instead.
#
# Limits apply per LOGIN SESSION. After --fix every terminal running a robot stack must be
# closed and reopened - re-sourcing or `exec bash` inherits the old limits.
set -uo pipefail

HOST="${TMR_HOST:-companion}"
LOCAL=0
FIX=0
ARM_IPS=(172.16.16.12 172.16.16.11)   # LEFT, RIGHT - see tmr_duo_config.yaml

while [ $# -gt 0 ]; do
    case "$1" in
        --local) LOCAL=1; shift ;;
        --fix)   FIX=1; shift ;;
        --host)  HOST="$2"; shift 2 ;;
        -h|--help) sed -n '2,45p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

# The whole probe is one blob of shell so it costs a single ssh round-trip (and a single
# sudo prompt when --fix is on). $FIX and the arm IPs are interpolated in by the caller.
probe() {
cat <<PROBE
set -u
fix=$FIX
arm_ips="${ARM_IPS[*]}"
PROBE
cat <<'PROBE'
fail=0
warn=0
say_ok()   { printf '  ok       %s\n' "$*"; }
say_bad()  { printf '  FAIL     %s\n' "$*"; fail=$((fail+1)); }
say_warn() { printf '  warn     %s\n' "$*"; warn=$((warn+1)); }

echo "host: $(hostname)   user: $(id -un)"
echo

echo "1. FIFO scheduling limit (rtprio)"
rtprio="$(ulimit -Hr 2>/dev/null || echo 0)"
if [ "${rtprio:-0}" = "unlimited" ] || [ "${rtprio:-0}" -ge 50 ] 2>/dev/null; then
    say_ok "hard rtprio limit is $rtprio (controller_manager asks for 50 by default)"
else
    say_bad "hard rtprio limit is ${rtprio:-0} - controller_manager cannot get SCHED_FIFO at all"
fi

echo
echo "2. Memory locking limit (memlock)"
memlock="$(ulimit -Hl 2>/dev/null || echo 0)"
if [ "${memlock:-0}" = "unlimited" ]; then
    say_ok "hard memlock limit is unlimited"
else
    say_bad "hard memlock limit is ${memlock:-0} kB - lock_memory() fails, page faults can stall the 1 kHz loop"
fi

echo
echo "3. Kernel"
kver="$(uname -r)"; kver_full="$(uname -v)"
case "$kver$kver_full" in
    *PREEMPT_RT*|*preempt_rt*) say_ok "PREEMPT_RT kernel ($kver)" ;;
    *) say_warn "$kver is not PREEMPT_RT - Franka requires it for 1 kHz FCI control" ;;
esac

echo
echo "4. FCI network interface"
seen_dev=""
for ip in $arm_ips; do
    line="$(ip route get "$ip" 2>/dev/null | head -1)"
    dev="$(printf '%s\n' "$line" | sed -n 's/.* dev \([^ ]*\).*/\1/p')"
    if [ -z "$dev" ]; then
        say_warn "no route to $ip"
        continue
    fi
    echo "  arm $ip via $dev"
    seen_dev="$seen_dev $dev"
    # Column order is fixed by iproute2:
    #   RX: bytes packets errors dropped overrun mcast
    #   TX: bytes packets errors dropped carrier collsns
    stats="$(ip -s link show "$dev" 2>/dev/null | awk '
        /RX:/{getline; rx_err=$3; rx_drop=$4; rx_over=$5}
        /TX:/{getline; tx_err=$3; tx_drop=$4}
        END{printf "%s %s %s %s %s", rx_err+0, rx_drop+0, rx_over+0, tx_err+0, tx_drop+0}')"
    set -- $stats
    if [ "${1:-0}" -gt 0 ] || [ "${2:-0}" -gt 0 ] || [ "${3:-0}" -gt 0 ] \
       || [ "${4:-0}" -gt 0 ] || [ "${5:-0}" -gt 0 ]; then
        say_bad "$dev: rx err=$1 dropped=$2 overrun=$3 / tx err=$4 dropped=$5 - the link is losing packets"
    else
        say_ok "$dev: no errors, drops or overruns since boot"
    fi
    coal="$(ethtool -c "$dev" 2>/dev/null | awk '/^rx-usecs:/{print $2; exit}')"
    if [ -n "$coal" ] && [ "$coal" != "0" ]; then
        say_warn "$dev: rx-usecs=$coal - interrupt coalescing delays FCI replies; Franka wants 0"
        echo "           sudo ethtool -C $dev rx-usecs 0 rx-frames 1   (bounces the link - robot idle only)"
    fi
done
# One interface carrying both arms means it also carries the laptop's DDS traffic: the
# arms, the spine and the laptop are all on the same 172.16.16.0/24.
uniq_devs="$(printf '%s\n' $seen_dev | sort -u | tr '\n' ' ' | sed 's/ *$//')"
if [ -n "$uniq_devs" ] && [ "$(printf '%s\n' $uniq_devs | wc -l)" = "1" ]; then
    say_warn "both arms and all ROS 2 DDS traffic share $uniq_devs - FCI has no dedicated link"
fi

echo
echo "5. Live state"
echo "  load: $(cat /proc/loadavg)   cores: $(nproc)"
pids="$(pgrep -f ros2_control_node 2>/dev/null)"
if [ -z "$pids" ]; then
    say_warn "no ros2_control_node running - start the robot stack to check the loops themselves"
else
    for p in $pids; do
        cmd="$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null | grep -o '__ns:=[^ ]*' | head -1)"
        [ -z "$cmd" ] && cmd="(no namespace)"
        # The control loop is a thread of the process, not the main thread, so check every
        # thread and report the highest scheduling class found.
        best_pol=""; best_prio=0
        for t in /proc/$p/task/*; do
            tid="${t##*/}"
            info="$(chrt -p "$tid" 2>/dev/null)"
            pol="$(printf '%s' "$info" | sed -n 's/.*scheduling policy: //p')"
            prio="$(printf '%s' "$info" | sed -n 's/.*priority: //p')"
            case "$pol" in SCHED_FIFO|SCHED_RR)
                if [ "${prio:-0}" -ge "$best_prio" ] 2>/dev/null; then
                    best_pol="$pol"; best_prio="$prio"
                fi ;;
            esac
        done
        lck="$(awk '/VmLck/{print $2}' /proc/$p/status 2>/dev/null)"
        if [ -n "$best_pol" ]; then
            say_ok "pid $p $cmd: $best_pol prio $best_prio"
        else
            say_bad "pid $p $cmd: no RT thread - the control loop runs SCHED_OTHER"
        fi
        if [ "${lck:-0}" -gt 0 ] 2>/dev/null; then
            say_ok "pid $p $cmd: VmLck ${lck} kB (memory locked)"
        else
            say_bad "pid $p $cmd: VmLck 0 kB - memory is NOT locked"
        fi
    done
fi

if [ "$fix" = "1" ]; then
    echo
    echo "6. Applying limits (--fix)"
    u="$(id -un)"
    # Authenticate first so the password prompt is unambiguous rather than appearing in
    # the middle of a pipeline. ssh -t supplies the tty sudo reads it from.
    if ! sudo -v; then
        echo "  sudo authentication failed; limits not written" >&2
        echo
        echo "summary: $fail failure(s), $warn warning(s)"
        exit 1
    fi
    sudo tee /etc/security/limits.d/99-tmr-realtime.conf >/dev/null <<LIMITS
# Installed by teleoperation/configs/check_realtime.sh.
# Both lines are required by libfranka's 1 kHz FCI loop: rtprio so controller_manager can
# take SCHED_FIFO, memlock so lock_memory() succeeds and page faults cannot stall it.
$u  -  rtprio   99
$u  -  memlock  unlimited
LIMITS
    if [ $? -eq 0 ]; then
        echo "  wrote /etc/security/limits.d/99-tmr-realtime.conf:"
        sed 's/^/    /' /etc/security/limits.d/99-tmr-realtime.conf
        echo
        echo "  Limits are granted at LOGIN. Close every terminal running a robot stack and"
        echo "  open new ones - re-sourcing or 'exec bash' keeps the old limits. Then rerun"
        echo "  this script without --fix to confirm."
    else
        echo "  could not write the limits file" >&2
    fi
fi

echo
echo "summary: $fail failure(s), $warn warning(s)"
PROBE
}

if [ "$LOCAL" = "1" ]; then
    probe | bash
    exit $?
fi

echo "Realtime check for the arm FCI loops  (host: $HOST)"
echo
if [ "$FIX" = "1" ]; then
    # -t so the single sudo prompt has a tty, exactly as sync_robot_clock.sh does.
    probe | ssh -t -o ConnectTimeout=10 "$HOST" 'bash -s'
else
    probe | ssh -o ConnectTimeout=10 "$HOST" 'bash -s'
fi
