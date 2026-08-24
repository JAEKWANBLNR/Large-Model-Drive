#!/usr/bin/env bash
# Install non-ROS runtime dependencies on Ubuntu.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/../.." && pwd)"

sudo apt-get update
sudo apt-get install -y \
  ffmpeg \
  libportaudio2 \
  mpv \
  portaudio19-dev \
  python3-opencv \
  python3-pip \
  python3-venv

requirements_file="${repository_root}/requirements.txt"
if [[ "${1:-}" == "--with-whisper" ]]; then
  requirements_file="${repository_root}/requirements-local-whisper.txt"
fi

python3 -m pip install --user --upgrade pip
python3 -m pip install --user -r "${requirements_file}"

echo "Python dependencies installed from ${requirements_file}."
echo "Install ROS dependencies with:"
echo "  rosdep install --from-paths src --ignore-src -r -y"
