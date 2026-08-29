#!/usr/bin/env bash
# Render the ebimHP DDS whitelist profile from the LIVE wired address, and echo its path.
#
#   profile="$(bash ~/teleoperation/render_dds_profile.sh)"   # empty if no wired link
#
# WHY RENDERED, NOT HARDCODED
# A whitelist with a stale address silently partitions the graph: participants announce
# only on an interface the host no longer has, every topic simply vanishes, and nothing
# logs an error. That exact failure (fastdds.xml pinning 172.16.16.140 while the host had
# no such address) cost a debugging session and is documented in record_bag.bash. This
# script derives the address at each startup, so DHCP churn cannot strand the profile.
#
# WHY A WHITELIST AT ALL
# With default discovery, FastDDS announces and transmits on EVERY interface. Measured
# during recording: 25-46 MB/s of image traffic on WiFi, causing drops, retransmits and
# publisher back-pressure. All ROS peers (laptop pedals, Jetson, this host) are on the
# wired 172.16.16.0/24 now, so DDS has no business on WiFi. WiFi stays up for internet -
# DDS just ignores it.
#
# 127.0.0.1 is whitelisted alongside the wired address so recorder <-> camera traffic on
# this host keeps flowing even if the wired link drops mid-session.
#
# UDP only, no SHM - shared memory between containers with separate /dev/shm delivers
# discovery but no data (diagnosed 2026-08-24; do not re-add SHM without re-testing).
# maxMessageSize must accept the ZED publisher's ~64 KiB UDP fragments. At 1400 bytes the
# recorder discovered and subscribed to the image topic but received zero payloads, while
# the same recorder at 65500 captured the expected 30 Hz. The interface whitelist still
# keeps DDS off WiFi; this limit only controls accepted RTPS datagram size.
set -u

addr="$(ip -4 -o addr show up scope global 2>/dev/null \
        | awk '$4 ~ /^172\.16\.16\./ {sub(/\/.*/, "", $4); print $4; exit}')"
[ -n "$addr" ] || exit 0   # no wired link: caller falls back to all-interface discovery

# The dedicated 2.5GbE direct link to the Jetson (its eno1 / PC-Direct, 172.16.1.9).
# The Jetson's DDS is whitelisted to THAT subnet only, so without this address here the
# two hosts stop exchanging DDS entirely - ZED, odometry, cmd_vel, all of it.
direct="$(ip -4 -o addr show up scope global 2>/dev/null \
        | awk '$4 ~ /^172\.16\.1\./ {sub(/\/.*/, "", $4); print $4; exit}')"

profile="$HOME/.tmr_dds_whitelist.xml"
cat > "$profile" <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<!-- RENDERED by render_dds_profile.sh - do not edit; edits are overwritten. -->
<dds>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>wired_udp</transport_id>
        <type>UDPv4</type>
        <maxMessageSize>65500</maxMessageSize>
        <interfaceWhiteList>
          <address>${addr}</address>${direct:+
          <address>${direct}</address>}
          <address>127.0.0.1</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="wired_only" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>wired_udp</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
XML

# Keep the camera container's mounted profile in step. Takes effect on its next
# recreate (docker compose up -d --force-recreate in ~/labs-camera).
cam="$HOME/labs-camera/fastdds.xml"
if [ -f "$cam" ]; then cp "$profile" "$cam"; fi

echo "$profile"
