#!/usr/bin/env python3
"""
ROCm Installation Helper for Genesis Server

Checks if amdgpu backend is available, and if not, provides
guided installation of ROCm on Ubuntu systems.

Usage:
    python scripts/install_rocm.py [--auto-install]
"""

import subprocess
import sys
import os
import platform
import argparse


def run_command(cmd, check=True, capture=True):
    """Run a shell command and return output."""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            check=check,
            capture_output=capture,
            text=True
        )
        return result.stdout.strip() if capture else None
    except subprocess.CalledProcessError as e:
        if capture:
            return e.stdout.strip()
        return None


def check_amdgpu_backend():
    """Check if amdgpu backend is available."""
    print("Checking Genesis backend availability...")
    print("-" * 60)

    try:
        import genesis as gs

        # Try to initialize amdgpu backend
        try:
            gs.init(backend=gs.amdgpu, logging_level="error")
            gs.destroy()
            print("✓ amdgpu backend is AVAILABLE")
            return True
        except Exception as e:
            print(f"✗ amdgpu backend NOT AVAILABLE: {e}")
            return False

    except ImportError:
        print("✗ Genesis is not installed!")
        print("  Install with: pip install genesis-world")
        sys.exit(1)


def check_gpu_backend():
    """Check if generic gpu backend is available."""
    try:
        import genesis as gs
        try:
            gs.init(backend=gs.gpu, logging_level="error")
            gs.destroy()
            print("✓ gpu backend is AVAILABLE (can use as fallback)")
            return True
        except Exception:
            print("✗ gpu backend NOT AVAILABLE")
            return False
    except ImportError:
        return False


def detect_os():
    """Detect OS and version."""
    if platform.system() != "Linux":
        print(f"✗ Unsupported OS: {platform.system()}")
        print("  ROCm only supports Linux")
        return None, None

    try:
        with open("/etc/os-release") as f:
            os_info = {}
            for line in f:
                if "=" in line:
                    key, value = line.strip().split("=", 1)
                    os_info[key] = value.strip('"')

        os_id = os_info.get("ID", "").lower()
        version_id = os_info.get("VERSION_ID", "")
        codename = os_info.get("VERSION_CODENAME", "")

        print(f"\nDetected OS: {os_info.get('PRETTY_NAME', 'Unknown')}")

        return os_id, codename

    except Exception as e:
        print(f"✗ Could not detect OS: {e}")
        return None, None


def check_amd_gpu():
    """Check if AMD GPU is detected."""
    print("\nChecking for AMD GPU...")

    # Check lspci
    lspci_out = run_command("lspci | grep -i 'vga\\|display\\|3d' | grep -i amd")
    if lspci_out:
        print(f"✓ AMD GPU detected: {lspci_out.split(':', 1)[1].strip() if ':' in lspci_out else lspci_out}")
        return True
    else:
        print("✗ No AMD GPU detected in lspci output")
        print("  ROCm requires an AMD GPU")
        return False


def check_existing_rocm():
    """Check if ROCm is already partially installed."""
    print("\nChecking existing ROCm installation...")

    # Check for rocm-smi
    rocm_smi = run_command("which rocm-smi", check=False)
    if rocm_smi:
        print(f"✓ rocm-smi found: {rocm_smi}")
    else:
        print("✗ rocm-smi not found")

    # Check for HIP libraries
    hip_libs = run_command("ls /usr/lib/x86_64-linux-gnu/libamdhip* 2>/dev/null", check=False)
    if hip_libs:
        print(f"✓ HIP libraries found")
        return "full"
    else:
        print("✗ HIP libraries NOT found (needed for amdgpu backend)")
        if rocm_smi:
            return "partial"
        return "none"


def get_rocm_install_commands(os_id, codename):
    """Get installation commands for specific OS."""

    if os_id != "ubuntu":
        return None

    # Map Ubuntu codenames to ROCm installer
    ubuntu_versions = {
        "noble": ("24.04", "noble"),      # Ubuntu 24.04
        "jammy": ("22.04", "jammy"),      # Ubuntu 22.04
        "focal": ("20.04", "focal"),      # Ubuntu 20.04
    }

    if codename not in ubuntu_versions:
        print(f"✗ Unsupported Ubuntu version: {codename}")
        print(f"  Supported versions: {', '.join(ubuntu_versions.keys())}")
        return None

    version, installer_codename = ubuntu_versions[codename]
    rocm_version = "6.2.4"
    installer_version = "6.2.60204-1"

    return f"""
# ROCm Installation Commands for Ubuntu {version}

# 1. Download amdgpu-install package
wget https://repo.radeon.com/amdgpu-install/{rocm_version}/ubuntu/{installer_codename}/amdgpu-install_{installer_version}_all.deb

# 2. Install the package manager
sudo dpkg -i amdgpu-install_{installer_version}_all.deb

# 3. Update apt cache
sudo apt update

# 4. Install ROCm (this will take 10-20 minutes and ~2-3GB)
sudo amdgpu-install --usecase=rocm --no-dkms

# 5. Add your user to render and video groups
sudo usermod -a -G render,video $USER

# 6. Reboot to apply changes
echo "Installation complete! Please reboot your system."
echo "After reboot, run: python scripts/check_backends.py"
"""


def install_rocm_interactive(os_id, codename):
    """Guide user through ROCm installation."""
    commands = get_rocm_install_commands(os_id, codename)

    if not commands:
        return False

    print("\n" + "=" * 60)
    print("ROCm Installation Guide")
    print("=" * 60)
    print(commands)
    print("=" * 60)

    print("\nOptions:")
    print("  1. Copy commands above and run them manually")
    print("  2. Save commands to install_rocm.sh script")
    print("  3. Exit")

    choice = input("\nYour choice (1/2/3): ").strip()

    if choice == "2":
        script_path = "install_rocm.sh"
        with open(script_path, "w") as f:
            f.write("#!/bin/bash\n")
            f.write("# ROCm Installation Script\n")
            f.write("# Generated by install_rocm.py\n\n")
            f.write("set -e  # Exit on error\n\n")

            # Extract just the commands
            for line in commands.split("\n"):
                line = line.strip()
                if line and not line.startswith("#") and line != "":
                    f.write(line + "\n")

        os.chmod(script_path, 0o755)
        print(f"\n✓ Installation script saved to: {script_path}")
        print(f"  Run with: sudo bash {script_path}")
        return True

    elif choice == "1":
        print("\n✓ Commands are ready to copy and paste")
        return True

    return False


def main():
    parser = argparse.ArgumentParser(description="ROCm installation helper for Genesis")
    parser.add_argument(
        "--auto-install",
        action="store_true",
        help="Automatically generate install script without prompts"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("Genesis ROCm Installation Helper")
    print("=" * 60)

    # Check if amdgpu backend is available
    amdgpu_available = check_amdgpu_backend()
    gpu_available = check_gpu_backend()

    if amdgpu_available:
        print("\n✓ amdgpu backend is working! No installation needed.")
        print("\nYou can run the server with:")
        print("  GENESIS_BACKEND=amdgpu python scripts/run_server.py")
        return 0

    print("\n" + "=" * 60)
    print("amdgpu backend is NOT available")
    print("=" * 60)

    if gpu_available:
        print("\n⚠ NOTE: The 'gpu' backend IS working on your system.")
        print("  You can use it without installing ROCm:")
        print("  GENESIS_BACKEND=gpu python scripts/run_server.py")
        print("\n  Continue only if you specifically need the 'amdgpu' backend.")

        if not args.auto_install:
            proceed = input("\nInstall ROCm for amdgpu support? (y/N): ").strip().lower()
            if proceed != 'y':
                print("\nExiting. Use GENESIS_BACKEND=gpu to run the server.")
                return 0

    # Detect OS
    os_id, codename = detect_os()
    if not os_id:
        return 1

    # Check for AMD GPU
    if not check_amd_gpu():
        print("\n⚠ Warning: No AMD GPU detected!")
        print("  ROCm installation will not provide GPU acceleration.")
        if not args.auto_install:
            proceed = input("\nContinue anyway? (y/N): ").strip().lower()
            if proceed != 'y':
                return 1

    # Check existing ROCm
    rocm_status = check_existing_rocm()

    if rocm_status == "full":
        print("\n⚠ ROCm appears to be fully installed but amdgpu backend is not working.")
        print("  This might be a configuration issue.")
        print("  Try running: python scripts/check_backends.py")
        return 1

    elif rocm_status == "partial":
        print("\n⚠ ROCm is partially installed (rocm-smi found, but no HIP libraries)")
        print("  You need to install the full ROCm stack.")

    # Generate installation commands
    if args.auto_install:
        install_rocm_interactive(os_id, codename)
    else:
        install_rocm_interactive(os_id, codename)

    print("\nAfter installation and reboot, verify with:")
    print("  python scripts/check_backends.py")

    return 0


if __name__ == "__main__":
    sys.exit(main())
