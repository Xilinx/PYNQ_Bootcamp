#!/usr/bin/env bash
#
# install_strix_halo.sh
#
# One-shot installer that provisions a fresh AMD Strix Halo machine
# (Ryzen AI Max / Max+, "gfx1151", Radeon 8060S iGPU) to run the Genesis
# simulation server on the "amdgpu" backend.
#
# It performs, in order:
#   1. System package install (build tools, Python, OpenCV/render/video libs)
#   2. ROCm 7.x install via amdgpu-install (kernel driver + userspace)
#   3. A dedicated Python virtual environment for the server
#   4. PyTorch for gfx1151 (AMD "TheRock" wheels by default -- these are the
#      only ones that reliably run on Strix Halo; the pytorch.org rocm7.2
#      nightly is kept as a documented fallback)
#   5. The Genesis server package + its Python dependencies (genesis-world,
#      numpy, flask, opencv-python, pillow)
#   6. A ~/.genesis_server_config env file (GENESIS_BACKEND=amdgpu, ports, ...)
#   7. Verification of ROCm + PyTorch + Genesis backends
#
# Target OS:  Ubuntu 24.04 LTS (noble), x86_64.
# Run as a normal user WITH sudo privileges (NOT as root). The script calls
# sudo where it needs to and keeps pip installs inside the venv.
#
# Usage:
#   cd genesis/server
#   ./install_strix_halo.sh
#
# Environment overrides (all optional):
#   ROCM_VERSION           ROCm release to install          (default: 7.2)
#   AMDGPU_INSTALL_DEB     Full .deb filename for that ROCm release
#   PYTORCH_CHANNEL        therock | pytorch-rocm72         (default: therock)
#   SKIP_ROCM=1            Skip the ROCm system install (already installed)
#   SKIP_APT=1             Skip the apt package install
#   VENV_DIR               venv location                    (default: <server>/venv)

set -euo pipefail

# -----------------------------------------------------------------------------
# Paths (resolved relative to THIS script, so the clone location doesn't matter)
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_DIR="$SCRIPT_DIR"                       # genesis/server
VENV_DIR="${VENV_DIR:-$SERVER_DIR/venv}"
CONFIG_FILE="$HOME/.genesis_server_config"

# -----------------------------------------------------------------------------
# Tunables
# -----------------------------------------------------------------------------
ROCM_VERSION="${ROCM_VERSION:-7.2}"
# amdgpu-install package for the chosen ROCm release (Ubuntu 24.04 / noble).
# Update the default here if you bump ROCM_VERSION.
AMDGPU_INSTALL_DEB="${AMDGPU_INSTALL_DEB:-amdgpu-install_7.2.70200-1_all.deb}"
UBUNTU_CODENAME_EXPECTED="noble"
PYTORCH_CHANNEL="${PYTORCH_CHANNEL:-therock}"

# gfx1151 (Strix Halo) TheRock nightly index -- the known-good source.
THEROCK_INDEX="https://rocm.nightlies.amd.com/v2/gfx1151/"
# Fallback: the pytorch.org ROCm 7.2 nightly referenced in INSTALL.md. Known to
# SEGFAULT on gfx1151 GPU tensor access for many builds -- use only if TheRock
# is unavailable and you accept the risk.
PYTORCH_ROCM72_INDEX="https://download.pytorch.org/whl/nightly/rocm7.2"

# -----------------------------------------------------------------------------
# Logging helpers
# -----------------------------------------------------------------------------
log()  { printf '\n\033[1;34m==>\033[0m %s\n' "$*"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; exit 1; }

# -----------------------------------------------------------------------------
# 0. Preflight guards
# -----------------------------------------------------------------------------
log "Preflight checks"

if [[ "${EUID}" -eq 0 ]]; then
    die "Do not run this script as root. Run as a normal user with sudo access."
fi

if ! command -v sudo >/dev/null 2>&1; then
    die "sudo is required but not found."
fi

ARCH="$(uname -m)"
[[ "$ARCH" == "x86_64" ]] || warn "Expected x86_64, found '$ARCH'. Continuing anyway."

if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    info "Detected OS: ${PRETTY_NAME:-unknown}"
    if [[ "${ID:-}" != "ubuntu" ]]; then
        warn "This installer targets Ubuntu. Detected ID='${ID:-}'. Continuing anyway."
    fi
    if [[ "${VERSION_CODENAME:-}" != "$UBUNTU_CODENAME_EXPECTED" ]]; then
        warn "Expected Ubuntu 24.04 ($UBUNTU_CODENAME_EXPECTED). Detected codename='${VERSION_CODENAME:-}'."
        warn "If you are on a different release, override AMDGPU_INSTALL_DEB to match."
    fi
else
    warn "/etc/os-release not found; cannot verify the OS."
fi

# Best-effort check that an AMD GPU is present.
if command -v lspci >/dev/null 2>&1; then
    if lspci | grep -Ei 'vga|display|3d' | grep -qi amd; then
        info "AMD GPU detected via lspci."
    else
        warn "No AMD GPU found in lspci output. ROCm/amdgpu backend needs an AMD GPU."
    fi
else
    warn "lspci not available; skipping AMD GPU detection (installing pciutils below)."
fi

info "Server dir : $SERVER_DIR"
info "venv       : $VENV_DIR"
info "ROCm       : $ROCM_VERSION (deb: $AMDGPU_INSTALL_DEB)"
info "PyTorch    : channel=$PYTORCH_CHANNEL"

# -----------------------------------------------------------------------------
# 1. System packages
# -----------------------------------------------------------------------------
if [[ "${SKIP_APT:-0}" == "1" ]]; then
    log "SKIP_APT=1 -> skipping apt package install"
else
    log "Installing system packages (apt)"
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        python3 python3-venv python3-pip python3-dev \
        git wget curl ca-certificates gnupg \
        pciutils \
        ffmpeg \
        libgl1 libglu1-mesa libegl1 libgles2 libglvnd0 \
        libglib2.0-0 \
        libjpeg-dev libpng-dev \
        libopencv-dev python3-opencv \
        "linux-headers-$(uname -r)" "linux-modules-extra-$(uname -r)"
fi

# -----------------------------------------------------------------------------
# 2. ROCm (amdgpu-install)
# -----------------------------------------------------------------------------
if [[ "${SKIP_ROCM:-0}" == "1" ]]; then
    log "SKIP_ROCM=1 -> skipping ROCm system install"
elif command -v rocminfo >/dev/null 2>&1 && [[ -e /dev/kfd ]]; then
    log "ROCm already present (rocminfo found, /dev/kfd exists) -> skipping install"
    info "Re-run with SKIP_ROCM=0 removed and delete this guard to force reinstall."
else
    log "Installing ROCm $ROCM_VERSION via amdgpu-install"
    DEB_URL="https://repo.radeon.com/amdgpu-install/${ROCM_VERSION}/ubuntu/${UBUNTU_CODENAME_EXPECTED}/${AMDGPU_INSTALL_DEB}"
    DEB_PATH="/tmp/${AMDGPU_INSTALL_DEB}"

    if [[ ! -f "$DEB_PATH" ]]; then
        info "Downloading $DEB_URL"
        wget -O "$DEB_PATH" "$DEB_URL" \
            || die "Failed to download amdgpu-install deb. Check ROCM_VERSION/AMDGPU_INSTALL_DEB."
    else
        info "Reusing cached $DEB_PATH"
    fi

    info "Installing the amdgpu-install package manager"
    sudo apt-get install -y "$DEB_PATH"
    sudo apt-get update

    # graphics,rocm: kernel driver + full ROCm userspace (rocminfo, rocm-smi, HIP).
    info "Running amdgpu-install (this takes 10-20 minutes and a few GB)"
    sudo amdgpu-install -y --usecase=graphics,rocm

    info "Adding $USER to the render and video groups"
    sudo usermod -a -G render,video "$USER"
fi

# -----------------------------------------------------------------------------
# 3. Python virtual environment
# -----------------------------------------------------------------------------
log "Creating Python virtual environment"
if [[ ! -d "$VENV_DIR" ]]; then
    python3 -m venv "$VENV_DIR"
    info "Created venv at $VENV_DIR"
else
    info "Reusing existing venv at $VENV_DIR"
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip setuptools wheel

# -----------------------------------------------------------------------------
# 4. PyTorch for gfx1151
# -----------------------------------------------------------------------------
log "Installing PyTorch (channel: $PYTORCH_CHANNEL)"
case "$PYTORCH_CHANNEL" in
    therock)
        # AMD "TheRock" gfx1151 nightly wheels: native Strix Halo support.
        pip install --pre --index-url "$THEROCK_INDEX" torch torchvision torchaudio \
            || die "TheRock gfx1151 torch install failed. Try PYTORCH_CHANNEL=pytorch-rocm72."
        ;;
    pytorch-rocm72)
        # Fallback: pytorch.org ROCm 7.2 nightly (per INSTALL.md). WARNING: known to
        # segfault on gfx1151 GPU tensor access for many builds.
        warn "pytorch.org rocm7.2 nightly can SEGFAULT on gfx1151. TheRock is recommended."
        pip uninstall -y torch torchvision torchaudio || true
        pip install --pre torch torchvision torchaudio --index-url "$PYTORCH_ROCM72_INDEX"
        ;;
    *)
        die "Unknown PYTORCH_CHANNEL='$PYTORCH_CHANNEL' (use 'therock' or 'pytorch-rocm72')."
        ;;
esac

# -----------------------------------------------------------------------------
# 5. Genesis server + Python dependencies
# -----------------------------------------------------------------------------
log "Installing the Genesis server package (editable)"
# Pulls genesis-world, numpy, flask, opencv-python, pillow from setup.py.
pip install -e "$SERVER_DIR"
# imageio-ffmpeg gives Genesis a bundled ffmpeg for video re-encoding fallback.
pip install imageio-ffmpeg

# -----------------------------------------------------------------------------
# 6. Server environment config
# -----------------------------------------------------------------------------
log "Writing server config to $CONFIG_FILE"
cat > "$CONFIG_FILE" <<'EOF'
# Genesis server configuration (source this before starting the server).

# --- Ports ---
export GENESIS_PORT=9002              # Main HTTP JSON API (students connect here)
export GENESIS_STREAM_PORT=9005       # Flask MJPEG video stream

# --- Backend ---
export GENESIS_BACKEND=amdgpu         # amdgpu | gpu | cuda | cpu
export GENESIS_SHOW_VIEWER=false      # headless server: no desktop viewer

# --- Session limits ---
export GENESIS_MAX_SESSIONS=30
export GENESIS_SESSION_TIMEOUT=7200   # 2 hours
export GENESIS_ADMIN_PASSWORD=bootcamp2024

# --- Strix Halo (gfx1151) stability tweaks ---
# Disable SDMA to avoid artifacts/hangs on some Strix Halo + kernel combos.
export HSA_ENABLE_SDMA=0
# If using pytorch.org rocm wheels (NOT TheRock) and the GPU is not detected,
# try uncommenting an ISA override matching gfx1151:
# export HSA_OVERRIDE_GFX_VERSION=11.5.1
EOF
info "Config written. It sets GENESIS_BACKEND=amdgpu."

# -----------------------------------------------------------------------------
# 7. Verification
# -----------------------------------------------------------------------------
log "Verifying installation"

if command -v rocminfo >/dev/null 2>&1; then
    if rocminfo 2>/dev/null | grep -q gfx1151; then
        info "rocminfo reports gfx1151 (Strix Halo)."
    else
        warn "rocminfo did not report gfx1151. If ROCm was just installed, REBOOT and re-run verification."
    fi
else
    warn "rocminfo not on PATH yet (expected right after a fresh ROCm install; reboot first)."
fi

command -v rocm-smi >/dev/null 2>&1 && rocm-smi || warn "rocm-smi unavailable (reboot may be required)."

info "Checking PyTorch sees the GPU..."
python - <<'PY' || warn "PyTorch GPU check failed. This is expected before a reboot; re-run after rebooting."
import torch
print(f"    torch      : {torch.__version__}")
print(f"    hip/rocm   : {getattr(torch.version, 'hip', None)}")
print(f"    gpu avail  : {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"    device     : {torch.cuda.get_device_name(0)}")
    x = torch.ones(3, device="cuda") * 2
    print(f"    gpu tensor : {x.tolist()}")
PY

info "Running the Genesis backend probe (scripts/check_backends.py)..."
python "$SERVER_DIR/scripts/check_backends.py" \
    || warn "Backend probe failed/incomplete. Reboot, 'source $CONFIG_FILE', then re-run it."

# -----------------------------------------------------------------------------
# Done
# -----------------------------------------------------------------------------
log "Installation complete"
cat <<EOF

Next steps
----------
1. REBOOT (required after a fresh ROCm install and for render/video group
   membership to take effect):

       sudo reboot

2. After reboot, verify the GPU stack:

       source "$VENV_DIR/bin/activate"
       source "$CONFIG_FILE"
       rocminfo | grep gfx
       python "$SERVER_DIR/scripts/check_backends.py"

3. Start the Genesis server (amdgpu backend):

       source "$VENV_DIR/bin/activate"
       source "$CONFIG_FILE"
       python "$SERVER_DIR/scripts/run_server.py"

   If the amdgpu backend misbehaves, fall back to CPU:

       GENESIS_BACKEND=cpu python "$SERVER_DIR/scripts/run_server.py"

Notes
-----
* PyTorch channel used: $PYTORCH_CHANNEL
  - 'therock' wheels ($THEROCK_INDEX) are recommended for gfx1151.
  - Re-run with PYTORCH_CHANNEL=pytorch-rocm72 only if you must use the
    pytorch.org rocm7.2 nightly (may segfault on gfx1151).
EOF
