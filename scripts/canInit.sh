#!/bin/bash
# canInit.sh — Jetson CAN bring-up script for Micronix BMS

IFACE=can0
BITRATE=500000   # Change to 250000 if BMS uses 250K

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set $IFACE down 2>/dev/null || true
sudo ip link set $IFACE type can bitrate $BITRATE dbitrate $BITRATE fd off sjw 1 sample-point 0.875
sudo ip link set $IFACE up

echo "[OK] $IFACE up @ ${BITRATE} bps"
ip -details link show $IFACE | grep -A3 can
