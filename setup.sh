#!/usr/bin/env bash
#
# setup_repo.sh
# Lives inside the PYNQ Bootcamp repo. Contains all of the Kria's one-time
# setup logic. Invoked by local_setup.sh after the repo is cloned/pulled.

set -euo pipefail

# The directory this script lives in (i.e. the repo root), so steps below can
# reference files shipped in the repo regardless of where it was cloned.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


STATE_DIR="/var/lib/pynq-setup"
AWAITING_FLAG="$STATE_DIR/FW_UPDATE_AWAITING_POWER_CYCLE"
DONE_FLAG="$STATE_DIR/FW_UPDATED"

# Firmware image (already present on the board image)
FIRMWARE_BIN="/home/ubuntu/Downloads/BOOT_xilinx-k26-starterkit-v2022.1-09152304_update3.BIN"

mkdir -p "$STATE_DIR"

# =============================================================================
if [[ ! -f "$AWAITING_FLAG" ]]; then
    echo "First boot: running one-time setup from $SCRIPT_DIR ..."

    echo "Installing pynqsim (editable)..."
    pip3 install -e "$SCRIPT_DIR/genesis/client/pynq-sim"

    echo "Installing pynq-p2p..."
    pip3 install "$SCRIPT_DIR/bootcamp_sessions/PYNQ 504 - Board to Board Communication/pynq-p2p"

    # Writes to the inactive boot partition; takes effect on next power cycle.
    echo "Current boot firmware status:"
    xmutil bootfw_status

    echo "Updating boot firmware from $FIRMWARE_BIN ..."
    xmutil bootfw_update -i "$FIRMWARE_BIN"

    touch "$AWAITING_FLAG"
    echo "First-boot setup complete. Power cycle the board to verify firmware."

    # Exit non-zero on first boot: firmware is flashed but not yet verified, so
    # the run is not "successfully complete". Leaving a non-zero exit lets the
    # caller/service treat this boot as incomplete and run again after the power
    # cycle. Service enable/disable is managed by the caller, not here.
    exit 1

else
    # second boot
    echo "Second boot: verifying firmware..."
    xmutil bootfw_update -v

    # Clear the pending flag and record completion with a timestamp.
    rm -f "$AWAITING_FLAG"
    date -u +"%Y-%m-%dT%H:%M:%SZ" > "$DONE_FLAG"

    echo "Firmware verified and committed; setup complete."
fi
