#!/bin/bash
# =============================================================================
# UAV-X: Spawn 10 PX4 SITL instances in Gazebo
# =============================================================================
# Usage:
#   ./spawn_fleet.sh [PX4_DIR] [WORLD]
#
# Arguments:
#   PX4_DIR  - Path to the PX4-Autopilot source directory
#              (default: $HOME/PX4-Autopilot)
#   WORLD    - Gazebo world name (SDF file stem, default: disaster_zone)
#
# Requirements:
#   - PX4-Autopilot built from source  (https://github.com/PX4/PX4-Autopilot)
#   - Gazebo Garden or Ignition Fortress
#   - ROS 2 Humble (sourced)
#   - bc, jq (apt install bc jq)
#
# Ports assigned per UAV instance (ID = 0 … 9):
#   MAVSDK / MAVLink UDP : 14540 + ID
#   QGC GCS UDP          : 14550 + ID
#   PX4 sim UDP          : 10001 + ID (internal)
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PX4_DIR="${1:-$HOME/PX4-Autopilot}"
WORLD="${2:-disaster_zone}"
NUM_UAVS=10

# Home position for the simulated GPS origin (Zurich Kloten area)
BASE_LAT=47.397742
BASE_LON=8.545594
BASE_ALT=488.0

# Spacing between spawned UAVs (degrees latitude per index step)
LAT_STEP=0.0001   # ~11 m per step

# Gazebo model to use for each vehicle (iris quadrotor)
VEHICLE_MODEL="iris"

# How long to wait for Gazebo to fully start before spawning UAVs (seconds)
GAZEBO_BOOT_WAIT=8

# ---------------------------------------------------------------------------
# ANSI colours
# ---------------------------------------------------------------------------
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'   # No colour / reset

# ---------------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------------
log_info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_err()   { echo -e "${RED}[ERR]${NC}   $*"; }

# ---------------------------------------------------------------------------
# Dependency checks
# ---------------------------------------------------------------------------
check_deps() {
    local errors=0

    log_info "Checking dependencies …"

    # 1. PX4-Autopilot directory
    if [[ ! -d "${PX4_DIR}" ]]; then
        log_err "PX4 directory not found: ${PX4_DIR}"
        log_err "Clone it with:  git clone --recursive https://github.com/PX4/PX4-Autopilot.git ${PX4_DIR}"
        errors=$((errors + 1))
    else
        log_ok "PX4 directory found: ${PX4_DIR}"
    fi

    # 2. PX4 binary must have been built
    if [[ ! -f "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]]; then
        log_warn "PX4 binary not found – building now may take 10-20 minutes."
        log_warn "To pre-build:  cd ${PX4_DIR} && make px4_sitl_default"
        # Not fatal – we still attempt to run
    else
        log_ok "PX4 binary present."
    fi

    # 3. Gazebo (either 'gz' CLI for Garden or 'gazebo' for Classic)
    if command -v gz &>/dev/null; then
        log_ok "Gazebo Garden detected: $(gz --version 2>&1 | head -1)"
        GZ_CMD="gz sim"
    elif command -v gazebo &>/dev/null; then
        log_ok "Gazebo Classic detected: $(gazebo --version 2>&1 | head -1)"
        GZ_CMD="gazebo"
    else
        log_err "Neither 'gz' nor 'gazebo' found in PATH."
        log_err "Install Gazebo Garden:  https://gazebosim.org/docs/garden/install"
        errors=$((errors + 1))
    fi
    export GZ_CMD

    # 4. ROS 2 environment
    if [[ -z "${ROS_DISTRO:-}" ]]; then
        log_warn "ROS_DISTRO not set – ROS 2 may not be sourced."
        log_warn "Run:  source /opt/ros/humble/setup.bash"
    else
        log_ok "ROS 2 distro: ${ROS_DISTRO}"
    fi

    # 5. Required system utilities
    for tool in bc python3 tmux; do
        if ! command -v "${tool}" &>/dev/null; then
            log_warn "'${tool}' not found – install with:  sudo apt install ${tool}"
        fi
    done

    # 6. World SDF file
    local world_file="${PX4_DIR}/Tools/simulation/gz/worlds/${WORLD}.sdf"
    local alt_world_file="$(dirname "$0")/../worlds/${WORLD}.sdf"
    if [[ -f "${world_file}" ]]; then
        log_ok "World SDF: ${world_file}"
        export WORLD_SDF="${world_file}"
    elif [[ -f "${alt_world_file}" ]]; then
        log_ok "World SDF (local): ${alt_world_file}"
        export WORLD_SDF="$(realpath "${alt_world_file}")"
    else
        log_warn "World SDF not found at ${world_file}"
        log_warn "Falling back to default Gazebo 'empty' world."
        export WORLD_SDF=""
    fi

    if [[ "${errors}" -gt 0 ]]; then
        log_err "${errors} critical dependency check(s) failed. Aborting."
        exit 1
    fi

    log_ok "All critical dependencies satisfied."
}

# ---------------------------------------------------------------------------
# Cleanup handler (called on EXIT / SIGINT / SIGTERM)
# ---------------------------------------------------------------------------
cleanup() {
    echo ""
    log_info "Killing all SITL instances and Gazebo …"

    # Kill all background jobs started by this script
    # 'jobs -p' returns PIDs of background processes in this shell
    local job_pids
    job_pids=$(jobs -p 2>/dev/null) || true
    if [[ -n "${job_pids}" ]]; then
        # shellcheck disable=SC2086
        kill ${job_pids} 2>/dev/null || true
    fi

    # Also sweep for any lingering PX4 / Gazebo processes
    pkill -f 'bin/px4 '    2>/dev/null || true
    pkill -f 'px4_sitl'    2>/dev/null || true
    pkill -f 'gz sim'      2>/dev/null || true
    pkill -f 'gzserver'    2>/dev/null || true
    pkill -f 'gzclient'    2>/dev/null || true
    pkill -f 'gazebo'      2>/dev/null || true
    pkill -f 'ruby.*gazebo' 2>/dev/null || true

    log_info "Cleanup complete."
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# Spawn a single PX4 SITL instance
# ---------------------------------------------------------------------------
# Arguments:
#   $1  UAV index (0-based)
#
# Each instance gets its own:
#   - Working directory under /tmp/px4_sitl_<ID>/
#   - Unique MAVLink UDP ports (no collision between instances)
#   - GPS home position offset so UAVs don't all start on top of each other
# ---------------------------------------------------------------------------
spawn_uav() {
    local ID=$1

    # ---- Port assignments ----
    local MAVSDK_PORT=$((14540 + ID))     # MAVSDK / external MAVLink
    local GCS_PORT=$((14550 + ID))         # QGroundControl / GCS
    local SIM_PORT=$((10001 + ID))         # PX4-internal simulator port
    local BROADCAST_PORT=$((17556 + ID))   # MAVLink broadcast

    # ---- GPS position (spread UAVs out slightly) ----
    # bc is used for floating-point arithmetic in bash
    local LAT
    LAT=$(echo "scale=8; ${BASE_LAT} + ${ID} * ${LAT_STEP}" | bc)
    local LON="${BASE_LON}"
    local ALT="${BASE_ALT}"

    # ---- Per-instance working directory ----
    local INST_DIR="/tmp/px4_sitl_${ID}"
    mkdir -p "${INST_DIR}"

    log_info "Spawning UAV ${ID}: MAVSDK=:${MAVSDK_PORT}  GCS=:${GCS_PORT}  lat=${LAT}"

    # ---- Environment variables consumed by PX4 SITL ----
    # PX4_SIM_MODEL      : which airframe / model to simulate
    # PX4_HOME_LAT/LON/ALT : GPS home position
    # PX4_INSTANCE       : instance index (affects default port offsets)
    # HEADLESS           : disable rendering (set to 1 for no GUI per-vehicle)
    # PX4_SIM_HOSTNAME   : simulator host (localhost for SITL)
    # PX4_SIM_PORT       : UDP port the simulator listens on
    env \
        PX4_SIM_MODEL="${VEHICLE_MODEL}" \
        PX4_HOME_LAT="${LAT}" \
        PX4_HOME_LON="${LON}" \
        PX4_HOME_ALT="${ALT}" \
        PX4_INSTANCE="${ID}" \
        PX4_SIM_HOSTNAME="localhost" \
        PX4_SIM_PORT="${SIM_PORT}" \
        HEADLESS=1 \
        "${PX4_DIR}/build/px4_sitl_default/bin/px4" \
            -d \
            -w "${INST_DIR}" \
            "${PX4_DIR}/build/px4_sitl_default/etc" \
            -s "etc/init.d-posix/rcS" \
        >"${INST_DIR}/px4_stdout.log" 2>"${INST_DIR}/px4_stderr.log" &

    local PX4_PID=$!
    log_ok "  UAV ${ID} PX4 PID=${PX4_PID}  logs: ${INST_DIR}/"

    # ---- Print MAVLink endpoint info for the user ----
    echo -e "  ${CYAN}UAV ${ID}${NC}: MAVSDK → udp://:${MAVSDK_PORT}  |  QGC → udp://:${GCS_PORT}"
}

# ---------------------------------------------------------------------------
# Launch the Gazebo simulation world
# ---------------------------------------------------------------------------
launch_gazebo() {
    log_info "Launching Gazebo world: ${WORLD_SDF:-empty} …"

    if [[ -n "${WORLD_SDF}" ]]; then
        # Gazebo Garden / Ignition
        if [[ "${GZ_CMD}" == "gz sim" ]]; then
            gz sim "${WORLD_SDF}" --headless-rendering &
        else
            # Gazebo Classic
            gazebo --verbose "${WORLD_SDF}" &
        fi
    else
        # Fall back to Gazebo Garden empty world
        if [[ "${GZ_CMD}" == "gz sim" ]]; then
            gz sim --headless-rendering &
        else
            gazebo --verbose &
        fi
    fi

    log_info "Waiting ${GAZEBO_BOOT_WAIT}s for Gazebo to initialise …"
    sleep "${GAZEBO_BOOT_WAIT}"
    log_ok "Gazebo should be up."
}

# ---------------------------------------------------------------------------
# Print a summary table of all spawned UAVs
# ---------------------------------------------------------------------------
print_summary() {
    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║              UAV-X Fleet Spawned Successfully                ║${NC}"
    echo -e "${GREEN}╠══════════════════════════════════════════════════════════════╣${NC}"
    printf "${GREEN}║${NC} %-6s %-20s %-14s %-14s ${GREEN}║${NC}\n" \
        "ID" "GPS Lat" "MAVSDK Port" "GCS Port"
    echo -e "${GREEN}╠══════════════════════════════════════════════════════════════╣${NC}"
    for i in $(seq 0 $((NUM_UAVS - 1))); do
        local lat
        lat=$(echo "scale=6; ${BASE_LAT} + ${i} * ${LAT_STEP}" | bc)
        printf "${GREEN}║${NC} %-6d %-20s %-14s %-14s ${GREEN}║${NC}\n" \
            "${i}" \
            "${lat}" \
            "udp://:$((14540 + i))" \
            "udp://:$((14550 + i))"
    done
    echo -e "${GREEN}╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${GREEN}║${NC}  Logs: /tmp/px4_sitl_<ID>/px4_stdout.log                    ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  Press Ctrl+C to stop all instances.                        ${GREEN}║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
main() {
    echo ""
    echo -e "${CYAN}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║        UAV-X Fleet Launcher v1.0             ║${NC}"
    echo -e "${CYAN}║  Spawning ${NUM_UAVS} PX4 SITL instances           ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════╝${NC}"
    echo ""

    # 1. Dependency checks
    check_deps

    # 2. Source ROS 2 if available (non-fatal if already sourced)
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
        log_ok "ROS 2 Humble sourced."
    elif [[ -n "${ROS_DISTRO:-}" ]]; then
        log_ok "ROS 2 already sourced: ${ROS_DISTRO}"
    else
        log_warn "Could not source ROS 2 – proceeding without it."
    fi

    # 3. Start Gazebo
    launch_gazebo

    # 4. Spawn all UAV SITL instances in parallel
    log_info "Spawning ${NUM_UAVS} UAV SITL instances …"
    for i in $(seq 0 $((NUM_UAVS - 1))); do
        spawn_uav "${i}"
        # Small stagger to avoid race conditions on startup
        sleep 0.5
    done

    # 5. Print connection summary
    print_summary

    log_ok "All ${NUM_UAVS} UAVs spawned. Waiting for processes …"
    log_info "Connect MAVSDK:  MissionController().connect_all({'uav_0':'udp://:14540', ...})"
    log_info "Connect QGC:     Add UDP link to port 14550"

    # 6. Block until Ctrl+C (cleanup trap will fire on exit)
    wait
}

main "$@"
