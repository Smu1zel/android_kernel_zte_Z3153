#!/bin/bash
# Z3153V Kernel Build Script
# Designed to run in a Linux environment (Ubuntu, Debian, WSL, etc.)

set -e

# --- Toolchain Configuration ---
# Android 10 (4.9) kernel is typically compiled using the arm-linux-androideabi-4.9 toolchain.
# You can download it from AOSP prebuilts or use an Android NDK (e.g., NDK r20b).
# Example: export CROSS_COMPILE=/path/to/android-ndk/toolchains/arm-linux-androideabi-4.9/prebuilt/linux-x86_64/bin/arm-linux-androideabi-

export ARCH=arm
export SUBARCH=arm

if [ -z "$CROSS_COMPILE" ]; then
    echo "Warning: CROSS_COMPILE is not set. Assuming the toolchain binary prefix is in your PATH."
    export CROSS_COMPILE=arm-linux-androideabi-
fi

KERNEL_DIR="."

echo "=== Changing directory to $KERNEL_DIR ==="
cd "$KERNEL_DIR"

if [ "$1" = "clean" ]; then
    echo "Cleaning kernel build..."
    make mrproper
    exit 0
fi

# 1. Apply our custom Z3153V defconfig
echo "Applying Z3153V configuration..."
make z3153v_defconfig

# 2. Compile kernel and device tree blobs
echo "Starting kernel build with $(nproc) jobs..."
make -j$(nproc)

echo "=== Build Complete ==="
echo "Output files:"
echo "- Kernel image with appended DTB: arch/arm/boot/zImage-dtb"
echo "- Device Tree Blob (DTB):       arch/arm/boot/dts/alps/mt6761.dtb"
echo "- Device Tree Overlay (DTBO):   arch/arm/boot/dts/alps/k61v1_32_bsp_hdp.dtbo"
