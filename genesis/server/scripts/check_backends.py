#!/usr/bin/env python3
"""
Check which Genesis backends are available on this machine.
"""

import genesis as gs

print("Genesis version:", gs.__version__)
print("\nChecking backend availability...\n")

backends = {
    "cpu": gs.cpu,
    "gpu": gs.gpu,
    "cuda": gs.cuda,
    "amdgpu": gs.amdgpu,
    "metal": gs.metal,
}

for name, backend in backends.items():
    try:
        gs.init(backend=backend, logging_level="error")
        print(f"✓ {name:10s} - AVAILABLE")
        gs.destroy()
    except Exception as e:
        print(f"✗ {name:10s} - NOT AVAILABLE: {e}")

print("\nRecommendation:")
print("Use the first available backend from: gpu, cuda, amdgpu, cpu")
