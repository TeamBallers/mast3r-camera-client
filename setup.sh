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
VENV_PATH="$USER_HOME/.venv"

echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}MASt3R Camera Client Setup${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo "Installing for user: $ACTUAL_USER"
echo "Home directory: $USER_HOME"
echo "Virtualenv path: $VENV_PATH"
echo ""

# Update package list

echo -e "${YELLOW}[1/4] Updating package list...${NC}"
apt update

# Install system dependencies

echo -e "${YELLOW}[2/4] Installing system dependencies...${NC}"
apt install -y \
python3 \
python3-pip \
python3-venv \
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

# Create virtual environment as the actual user

echo -e "${YELLOW}[3/4] Creating virtual environment...${NC}"
sudo -u "$ACTUAL_USER" python3 -m venv --system-site-packages "$VENV_PATH"

# Upgrade pip inside venv

sudo -u "$ACTUAL_USER" "$VENV_PATH/bin/pip" install --upgrade pip

# Install Python dependencies inside venv

echo -e "${YELLOW}[4/4] Installing Python dependencies...${NC}"
sudo -u "$ACTUAL_USER" "$VENV_PATH/bin/pip" install \
requests \
adafruit-circuitpython-lsm6ds \
fastapi \
uvicorn \
numpy \
RPi.GPIO

# Ensure ownership of venv

chown -R "$ACTUAL_USER":"$ACTUAL_USER" "$VENV_PATH"

# Add auto-activation to user's bashrc (if not already present)

BASHRC_FILE="$USER_HOME/.bashrc"
ACTIVATE_LINE='source ~/.venv/bin/activate'

if ! grep -Fxq "$ACTIVATE_LINE" "$BASHRC_FILE"; then
echo "$ACTIVATE_LINE" >> "$BASHRC_FILE"
chown "$ACTUAL_USER":"$ACTUAL_USER" "$BASHRC_FILE"
fi

# Make script executable (as user)

sudo -u "$ACTUAL_USER" chmod +x "$PWD/camera_client.py"

echo ""
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}Setup Complete!${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo "Virtual environment created at:"
echo "  $VENV_PATH"
echo ""
echo "To activate manually:"
echo "  source ~/.venv/bin/activate"
echo ""
echo "Dependencies installed inside the virtual environment."
echo "New terminals will auto-activate the venv."
echo ""
