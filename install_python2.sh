#!/bin/bash
# Script to compile and install Python 2.7.18 from source
# Useful for newer WSL/Ubuntu distributions where Python 2 is no longer in apt repositories.

set -e

echo "=== Installing dependencies for Python 2 compilation ==="
sudo apt-get update
sudo apt-get install -y build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev wget curl libsqlite3-dev

cd ~
echo "=== Downloading Python 2.7.18 source ==="
wget https://www.python.org/ftp/python/2.7.18/Python-2.7.18.tgz
tar -xf Python-2.7.18.tgz
cd Python-2.7.18

echo "=== Configuring Python 2 ==="
# Configure to install locally in ~/.local to avoid touching system paths
./configure --prefix=$HOME/.local

echo "=== Compiling Python 2 ==="
make -j$(nproc)

echo "=== Installing Python 2 ==="
make install

echo "=== Cleaning up ==="
cd ~
rm -rf Python-2.7.18 Python-2.7.18.tgz

# Make sure ~/.local/bin is in PATH and python2 exists as a symlink
mkdir -p ~/.local/bin
if [ -f ~/.local/bin/python ] && [ ! -f ~/.local/bin/python2 ]; then
    ln -s ~/.local/bin/python ~/.local/bin/python2
fi

# Add ~/.local/bin to PATH in .bashrc if not already present
if ! grep -q '\.local/bin' ~/.bashrc; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    echo "Added ~/.local/bin to PATH in ~/.bashrc"
fi

echo ""
echo "=== Python 2 Installation Complete! ==="
echo "Please run: source ~/.bashrc"
echo "Then verify by running: python2 --version"
