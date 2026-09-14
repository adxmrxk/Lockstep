#!/usr/bin/env bash
# Build Cyclone DDS into $HOME, for the comparison benchmark.
#
# No root. The apt package needs it, building from source does not, which is
# what makes the comparison possible on a machine you only borrow.
set -euo pipefail

prefix="${1:-$HOME/cdds}"
src="${CDDS_SRC:-$HOME/cdds-src}"
branch="${CDDS_BRANCH:-releases/0.10.x}"

if [ ! -d "$src" ]; then
  echo "[cyclone] cloning $branch"
  git clone --depth 1 --branch "$branch" \
    https://github.com/eclipse-cyclonedds/cyclonedds.git "$src"
fi

echo "[cyclone] building into $prefix"
cmake -S "$src" -B "$src/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_IDLC=ON \
  -DCMAKE_INSTALL_PREFIX="$prefix" >/dev/null
cmake --build "$src/build" -j"$(nproc)" >/dev/null
cmake --install "$src/build" >/dev/null

echo "[cyclone] installed. Re-configure lockstep with:"
echo "    cmake -S . -B build -DCMAKE_PREFIX_PATH=$prefix"
