#!/bin/bash
# setup_vcan.sh — Create virtual CAN interface for testing
set -e

echo "=== Setting up vcan0 for cpiper testing ==="

# Load vcan module
if ! lsmod | grep -q vcan; then
    echo "Loading vcan module..."
    sudo modprobe vcan
fi

# Create vcan0 if it doesn't exist
if ! ip link show vcan0 &>/dev/null; then
    echo "Creating vcan0..."
    sudo ip link add vcan0 type vcan
fi

# Bring it up
echo "Bringing up vcan0..."
sudo ip link set up vcan0

echo "vcan0 is ready."
echo ""
echo "Verify with: ip link show vcan0"
echo "Monitor with: candump vcan0"
