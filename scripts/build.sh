#!/usr/bin/env bash
# (assumes the helper functions `log`, `warn`, `die` and the earlier udev
#  sections are already present in this script)
# log()  { printf '\e[1;34m[i] %s\e[0m\n'  "$*"; }
# warn() { printf '\e[1;33m[!] %s\e[0m\n' "$*"; }
# die()  { printf '\e[1;31m[✗] %s\e[0m\n' "$*" >&2; exit 1; }

# # ─── Workspace root & “build‑as‑user” wrapper ─────────────────────────
# WS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
# ROS_SETUP="/opt/ros/humble/setup.bash"
# [[ -r "$ROS_SETUP" ]] || die "ROS 2 Humble not found at $ROS_SETUP"

# # figure out who should own the artefacts
# if [[ -n "${SUDO_USER:-}" ]]; then
#   BUILD_USER="$SUDO_USER"
# else
#   BUILD_USER="$(logname 2>/dev/null || true)"
#   [[ -n "$BUILD_USER" && "$BUILD_USER" != "root" ]] \
#     || die "Run this script with sudo so we know which user should own the build."
# fi

# USER_HOME="$(getent passwd "$BUILD_USER" | cut -d: -f6)"
# SU=(sudo -u "$BUILD_USER" -E env HOME="$USER_HOME")

# # local prefix that is always writable
# LOCAL_PREFIX="$WS_ROOT/install/local"
# mkdir -p "$LOCAL_PREFIX"

# # derive python X.Y and a writable site‑packages path
# PY_VER="$("${SU[@]}" python3 - <<'PY' -q
# import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")
# PY
# )"
# PY_SITE="$LOCAL_PREFIX/lib/python${PY_VER}/site-packages"

# # common CMake arguments – used in both manual builds and colcon
# CMAKE_COMMON_ARGS=(
#   -DCMAKE_INSTALL_PREFIX="$LOCAL_PREFIX"
#   -DPYTHON_INSTALL_DIR="$PY_SITE"
# )

# # helper: configure + make (no install)
# build_pkg () {
#   local SRC_DIR="$1"  NAME="$2"  DO_INSTALL="$3"
#   [[ -d "$SRC_DIR" ]] || { warn "$NAME source not found — skipping."; return; }

#   log "--> Building $NAME at $SRC_DIR"
#   "${SU[@]}" bash -c '
#     set -e
#     source "'"$ROS_SETUP"'"
#     mkdir -p "'"$SRC_DIR"'/build"
#     cd "'"$SRC_DIR"'/build"
#     rm -rf CMakeCache.txt CMakeFiles
#     cmake .. '"${CMAKE_COMMON_ARGS[*]}"'
#     make -j"$(nproc)"
#     '"$( [[ $DO_INSTALL == yes ]] && echo make install )"'
#   '
#   log "--> $NAME $( [[ $DO_INSTALL == yes ]] && echo installed || echo compiled )."
# }

# # # ─── SECTION 1 — OpenZen (compile only) ───────────────────────────────
# build_pkg "$WS_ROOT/src/OpenZenRos/openzen" "OpenZen" no

# # # ─── SECTION 2 — Collision (compile + install) ───────────────────────
# build_pkg "$WS_ROOT/src/collision" "Collision" yes

# # ─── SECTION 3 — Build the ROS 2 workspace (user‑owned) ─────────────
# log 'Building ROS 2 workspace with colcon…'
# HOOK_DIR="$LOCAL_PREFIX/share/collision_geom"
# mkdir -p "$HOOK_DIR"
# printf 'export PYTHONPATH="%s:$PYTHONPATH"\n' "$PY_SITE" \
#   | "${SU[@]}" tee "$HOOK_DIR/pythonpath.sh" >/dev/null



# "${SU[@]}" bash -c "
#   source '$ROS_SETUP'
#   source '$WS_ROOT/install/setup.bash'
#   cd '$WS_ROOT'

#   colcon build \
#       --symlink-install \
#       --cmake-args ${CMAKE_COMMON_ARGS[*]}
# "

# log '--> Build complete.  Source the overlay with:'
# log "     source \"$WS_ROOT/install/setup.bash\""

# # Optionally append to the user's ~/.bashrc so it sources automatically
# read -r -p "Add workspace overlay to $BUILD_USER's ~/.bashrc? [y/N] " RESP
# if [[ "$RESP" =~ ^[Yy]$ ]]; then
#   printf '\n# ASV_localization workspace overlay\nsource "%s/install/setup.bash"\n' "$WS_ROOT" \
#     | sudo -u "$BUILD_USER" tee -a "$USER_HOME/.bashrc" >/dev/null
#   log "--> Overlay added to $USER_HOME/.bashrc (takes effect in new shells)"
# fi
log()  { printf '\e[1;34m[i] %s\e[0m\n'  "$*"; }
warn() { printf '\e[1;33m[!] %s\e[0m\n' "$*"; }
die()  { printf '\e[1;31m[✗] %s\e[0m\n' "$*" >&2; exit 1; }

# ─── Workspace root & “build‑as‑user” wrapper ─────────────────────────
WS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
ROS_SETUP="/opt/ros/humble/setup.bash"
[[ -r "$ROS_SETUP" ]] || die "ROS 2 Humble not found at $ROS_SETUP"

# figure out who should own the artefacts
if [[ -n "${SUDO_USER:-}" ]]; then
  BUILD_USER="$SUDO_USER"
else
  BUILD_USER="$(logname 2>/dev/null || true)"
  [[ -n "$BUILD_USER" && "$BUILD_USER" != "root" ]] \
    || die "Run this script with sudo so we know which user should own the build."
fi

USER_HOME="$(getent passwd "$BUILD_USER" | cut -d: -f6)"
SU=(sudo -u "$BUILD_USER" -E env HOME="$USER_HOME")

# ─── Ensure install tree exists under the build user ───────────────────
"${SU[@]}" mkdir -p "$WS_ROOT/install"
# local prefix that is always writable by $BUILD_USER
LOCAL_PREFIX="$WS_ROOT/install/local"
"${SU[@]}" mkdir -p "$LOCAL_PREFIX"

# derive python X.Y and a writable site‑packages path
PY_VER="$("${SU[@]}" python3 - <<'PY' -q
import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
PY_SITE="$LOCAL_PREFIX/lib/python${PY_VER}/site-packages"

# common CMake arguments – used in both manual builds and colcon
CMAKE_COMMON_ARGS=(
  -DCMAKE_INSTALL_PREFIX="$LOCAL_PREFIX"
  -DPYTHON_INSTALL_DIR="$PY_SITE"
)

# helper: configure + make (no install)
build_pkg () {
  local SRC_DIR="$1"  NAME="$2"  DO_INSTALL="$3"
  [[ -d "$SRC_DIR" ]] || { warn "$NAME source not found — skipping."; return; }

  log "--> Building $NAME at $SRC_DIR"
  "${SU[@]}" bash -c '
    set -e
    source "'"$ROS_SETUP"'"
    mkdir -p "'"$SRC_DIR"'/build"
    cd "'"$SRC_DIR"'/build"
    rm -rf CMakeCache.txt CMakeFiles
    cmake .. '"${CMAKE_COMMON_ARGS[*]}"'
    make -j"$(nproc)"
    '"$( [[ $DO_INSTALL == yes ]] && echo make install )"'
  '
  log "--> $NAME $( [[ $DO_INSTALL == yes ]] && echo installed || echo compiled )."
}

# # ─── SECTION 1 — OpenZen (compile only) ───────────────────────────────
build_pkg "$WS_ROOT/src/OpenZenRos/openzen" "OpenZen" no

# # ─── SECTION 2 — Collision (compile + install) ───────────────────────
build_pkg "$WS_ROOT/src/collision" "Collision" yes

# ─── SECTION 3 — Build the ROS 2 workspace (user‑owned) ─────────────
log 'Building ROS 2 workspace with colcon…'
HOOK_DIR="$LOCAL_PREFIX/share/collision_geom"
"${SU[@]}" mkdir -p "$HOOK_DIR"
printf 'export PYTHONPATH="%s:$PYTHONPATH"\n' "$PY_SITE" \
  | "${SU[@]}" tee "$HOOK_DIR/pythonpath.sh" >/dev/null

"${SU[@]}" bash -c "
  source '$ROS_SETUP'
  source '$WS_ROOT/install/setup.bash'
  cd '$WS_ROOT'

  colcon build \
      --symlink-install \
      --cmake-args ${CMAKE_COMMON_ARGS[*]}
"

log '--> Build complete.  Source the overlay with:'
log "     source \"$WS_ROOT/install/setup.bash\""

# Optionally append to the user's ~/.bashrc so it sources automatically
read -r -p "Add workspace overlay to $BUILD_USER's ~/.bashrc? [y/N] " RESP
if [[ "$RESP" =~ ^[Yy]$ ]]; then
  printf '\n# ASV_localization workspace overlay\nsource "%s/install/setup.bash"\n' "$WS_ROOT" \
    | sudo -u "$BUILD_USER" tee -a "$USER_HOME/.bashrc" >/dev/null
  log "--> Overlay added to $USER_HOME/.bashrc (takes effect in new shells)"
fi
