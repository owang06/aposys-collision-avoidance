#!/usr/bin/env bash

set -euo pipefail

echo "🔍 Checking Python 3 and pip..."

if ! command -v python3 >/dev/null 2>&1; then
    echo "❌ Python 3 is not installed. Please install it first."
    exit 1
fi

if ! command -v pip3 >/dev/null 2>&1; then
    echo "📦 pip3 not found. Attempting to install via apt..."
    sudo apt update
    sudo apt install -y python3-pip
fi

echo "✅ Python 3 and pip3 are available."

# Install dependencies only if not already installed
for pkg in pyserial pynmea2 pybind11; do
    if ! python3 -c "import $pkg" 2>/dev/null; then
        echo "📦 Installing $pkg..."
        pip3 install "$pkg"
    else
        echo "✔️ $pkg is already installed."
    fi
done

echo "✅ All GPS Python dependencies are installed."
