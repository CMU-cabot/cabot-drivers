#!/bin/bash

# Root privileges are required to run this script
if [ "$EUID" -ne 0 ]; then
  echo "This script requires root privileges to execute."
  echo "Please rerun it using sudo."
  exit 1
fi

# Load the vcan module
echo "Loading the vcan module..."
modprobe vcan
if [ $? -ne 0 ]; then
  echo "Failed to load the vcan module. Please check the kernel module."
  exit 1
fi

# Check if 'can0' already exists
if ip link show can0 &>/dev/null; then
  echo "The virtual CAN interface 'can0' already exists. Skipping creation."
else
  # Create the virtual CAN interface
  echo "Creating the virtual CAN interface 'can0'..."
  ip link add dev can0 type vcan
  if [ $? -ne 0 ]; then
    echo "Failed to create the virtual CAN interface 'can0'."
    exit 1
  fi
fi

# Enable the virtual CAN interface
echo "Enabling the virtual CAN interface 'can0'..."
ip link set up can0
if [ $? -ne 0 ]; then
  echo "Failed to enable the virtual CAN interface 'can0'."
  exit 1
fi

echo "The virtual CAN interface 'can0' is now set up and configured."
echo "Verification command: ip link show can0"
