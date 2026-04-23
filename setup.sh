#!/bin/bash

#

# Setup script for MASt3R Camera Client on Raspberry Pi

#

# Usage: sudo bash setup.sh

set -e  # Exit on error

# Colors for output

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running as root

if [ "$EUID" -ne 0 ]; then
echo -e "${RED}Error: This script must be run as root${NC}"
echo "Usage: sudo bash setup.sh"
exit 1
fi

# Get the actual user (not root)

ACTUAL_USER=${SUDO_USER:-$USER}
USER_HOME=$(eval echo ~$ACTUAL_USER)

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}MASt3R Camera Client Setup${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo "Installing for user: $ACTUAL_USER"
echo "Home directory: $USER_HOME"
echo ""

# Update package list

echo -e "${YELLOW}[1/4] Updating package list...${NC}"
apt update

# Install system dependencies

echo -e "${YELLOW}[2/4] Installing system dependencies...${NC}"
apt install -y \
python3 \
python3-pip \
python3-picamera2 \
libopenblas-dev

# Install camera apps (new or legacy)

if apt-cache show rpicam-apps >/dev/null 2>&1; then
echo "Installing rpicam-apps (newer)..."
apt install -y rpicam-apps
else
echo "Installing libcamera-apps (legacy)..."
apt install -y libcamera-apps
fi

echo -e "${YELLOW}[4/4] Installing Python dependencies...${NC}"
pip3 install --break-system-packages \ 
requests \
adafruit-circuitpython-lsm6ds \
fastapi \
uvicorn 

sudo -u "$ACTUAL_USER" chmod +x "$PWD/camera_client.py"

echo ""
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Setup Complete!${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo ""
