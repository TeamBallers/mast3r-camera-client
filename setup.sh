#!/bin/bash
#
# Setup script for MASt3R Camera Client on Raspberry Pi
#
# This script will:
# 1. Install required system packages
# 2. Install Python dependencies
# 3. Configure the systemd service
# 4. Enable the service to run at startup
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
    python3-picamera2

# Install camera apps (try newer rpicam-apps first, fall back to libcamera-apps)
if apt-cache show rpicam-apps >/dev/null 2>&1; then
    echo "Installing rpicam-apps (newer)..."
    apt install -y rpicam-apps
else
    echo "Installing libcamera-apps (legacy)..."
    apt install -y libcamera-apps
fi

echo -e "${YELLOW}[3/4] Creating virtual environment...${NC}"
python3 -m venv ~/.venv
source ~/.venv/bin/activate
echo 'source ~/.venv/bin/activate' >> ~/.bashrc


# Install Python dependencies
echo -e "${YELLOW}[4/4] Installing Python dependencies...${NC}"
pip3 install requests
pip3 install adafruit-circuitpython-lsm6ds

# Make script executable
chmod +x "./camera_client.py"

echo ""
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Setup Complete!${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo "The camera client has been installed and is ready to be run."
echo "A venv was created at ~/.venv with the necessary Python dependencies."
echo "The venv will be activated automatically when you open a new terminal session."
echo ""
