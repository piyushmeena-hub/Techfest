#!/bin/bash
# ==============================================================================
# UAV-X One-Click Setup Script
# ==============================================================================
# Detects macOS vs Linux, installs all Python dependencies, clones vendor
# repositories, creates a virtual environment, and prints next steps.
#
# Usage:
#   chmod +x setup.sh && ./setup.sh
#
# Supported platforms:
#   - macOS 12+ (Homebrew required for system deps)
#   - Ubuntu 20.04 / 22.04 (apt-get)
#   - Other Debian-based Linux distributions
#
# What this script does:
#   1. Detects OS and package manager
#   2. Checks Python 3.8+ and pip are available
#   3. Creates a virtual environment at ./venv
#   4. Installs Python dependencies (flask, matplotlib, numpy, pytest, etc.)
#   5. Creates ./vendor/ and clones required third-party repositories
#   6. Creates a .env file with sensible defaults
#   7. Prints formatted next-steps instructions
# ==============================================================================

set -euo pipefail

# ==============================================================================
# Colour helpers
# ==============================================================================
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'   # No colour / reset

log_info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_err()   { echo -e "${RED}[ERR]${NC}   $*"; exit 1; }
log_step()  { echo -e "\n${BOLD}${CYAN}══ $* ══${NC}"; }

# ==============================================================================
# OS Detection
# ==============================================================================
detect_os() {
    log_step "Detecting Operating System"

    OS_TYPE=""
    PKG_MGR=""

    case "$(uname -s)" in
        Darwin)
            OS_TYPE="macos"
            log_ok "macOS detected: $(sw_vers -productVersion 2>/dev/null || uname -r)"
            if command -v brew &>/dev/null; then
                PKG_MGR="brew"
                log_ok "Homebrew found: $(brew --version | head -1)"
            else
                log_warn "Homebrew not found.  Install it for system dependencies:"
                log_warn '  /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"'
                PKG_MGR="none"
            fi
            ;;
        Linux)
            OS_TYPE="linux"
            log_ok "Linux detected: $(uname -r)"
            if command -v apt-get &>/dev/null; then
                PKG_MGR="apt"
                log_ok "Package manager: apt-get (Debian/Ubuntu)"
            elif command -v dnf &>/dev/null; then
                PKG_MGR="dnf"
                log_ok "Package manager: dnf (Fedora/RHEL)"
            elif command -v pacman &>/dev/null; then
                PKG_MGR="pacman"
                log_ok "Package manager: pacman (Arch Linux)"
            else
                PKG_MGR="none"
                log_warn "No supported package manager found."
            fi
            ;;
        *)
            log_warn "Unknown OS: $(uname -s) – proceeding with generic setup."
            OS_TYPE="unknown"
            PKG_MGR="none"
            ;;
    esac

    export OS_TYPE PKG_MGR
}

# ==============================================================================
# Install system-level dependencies
# ==============================================================================
install_system_deps() {
    log_step "Installing System Dependencies"

    case "${PKG_MGR}" in
        apt)
            log_info "Updating apt cache …"
            sudo apt-get update -qq
            log_info "Installing system packages …"
            sudo apt-get install -y --no-install-recommends \
                python3 python3-pip python3-venv python3-dev \
                git curl wget bc jq \
                build-essential libssl-dev libffi-dev \
                2>/dev/null
            log_ok "apt packages installed."
            ;;
        brew)
            log_info "Installing Homebrew packages …"
            brew install python3 git curl bc jq 2>/dev/null || true
            log_ok "Homebrew packages installed."
            ;;
        dnf)
            log_info "Installing dnf packages …"
            sudo dnf install -y python3 python3-pip git curl wget bc jq \
                gcc python3-devel openssl-devel libffi-devel 2>/dev/null
            log_ok "dnf packages installed."
            ;;
        pacman)
            log_info "Installing pacman packages …"
            sudo pacman -Sy --noconfirm python python-pip git curl wget bc jq \
                base-devel openssl libffi 2>/dev/null
            log_ok "pacman packages installed."
            ;;
        none)
            log_warn "Skipping system package installation (no supported package manager)."
            ;;
    esac
}

# ==============================================================================
# Python 3.8+ check
# ==============================================================================
check_python() {
    log_step "Checking Python Version"

    PYTHON_CMD=""
    for cmd in python3.12 python3.11 python3.10 python3.9 python3.8 python3 python; do
        if command -v "${cmd}" &>/dev/null; then
            VER=$("${cmd}" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>/dev/null || echo "0.0")
            MAJOR=$(echo "${VER}" | cut -d. -f1)
            MINOR=$(echo "${VER}" | cut -d. -f2)
            if [[ "${MAJOR}" -ge 3 && "${MINOR}" -ge 8 ]]; then
                PYTHON_CMD="${cmd}"
                log_ok "Python ${VER} found at: $(command -v ${cmd})"
                break
            fi
        fi
    done

    if [[ -z "${PYTHON_CMD}" ]]; then
        log_err "Python 3.8+ not found. Please install Python 3.8 or newer and re-run."
    fi

    export PYTHON_CMD
}

# ==============================================================================
# pip check
# ==============================================================================
check_pip() {
    log_step "Checking pip"

    if "${PYTHON_CMD}" -m pip --version &>/dev/null; then
        PIP_VERSION=$("${PYTHON_CMD}" -m pip --version | awk '{print $2}')
        log_ok "pip ${PIP_VERSION} available."
    else
        log_warn "pip not found – attempting to install …"
        if [[ "${OS_TYPE}" == "linux" ]] && command -v apt-get &>/dev/null; then
            sudo apt-get install -y python3-pip
        else
            "${PYTHON_CMD}" -m ensurepip --upgrade 2>/dev/null || true
        fi

        if "${PYTHON_CMD}" -m pip --version &>/dev/null; then
            log_ok "pip installed successfully."
        else
            log_err "Could not install pip. Please install it manually."
        fi
    fi
}

# ==============================================================================
# Create virtual environment
# ==============================================================================
create_venv() {
    log_step "Creating Virtual Environment"

    VENV_DIR="$(pwd)/venv"

    if [[ -d "${VENV_DIR}" ]]; then
        log_warn "Virtual environment already exists at ${VENV_DIR}"
        log_warn "Delete it and re-run to recreate:  rm -rf venv"
    else
        log_info "Creating venv at ${VENV_DIR} …"
        "${PYTHON_CMD}" -m venv "${VENV_DIR}"
        log_ok "Virtual environment created."
    fi

    # Activate for the rest of this script
    # shellcheck disable=SC1090,SC1091
    source "${VENV_DIR}/bin/activate"
    log_ok "Virtual environment activated."

    # Upgrade pip inside venv
    pip install --quiet --upgrade pip setuptools wheel
    log_ok "pip/setuptools/wheel upgraded."

    export VENV_DIR
}

# ==============================================================================
# Install Python dependencies
# ==============================================================================
install_python_deps() {
    log_step "Installing Python Dependencies"

    # Core Phase-0 dependencies
    CORE_DEPS=(
        "flask>=2.0.0"
        "matplotlib>=3.5.0"
        "numpy>=1.21.0"
        "pytest>=7.0.0"
        "pytest-cov>=3.0.0"
        "scipy>=1.7.0"
        "requests>=2.27.0"
    )

    # Optional Phase-1 MAVSDK (not installed by default – needs hardware)
    OPTIONAL_DEPS=(
        # "mavsdk>=1.4.0"
    )

    log_info "Installing core dependencies: ${CORE_DEPS[*]} …"
    pip install --quiet "${CORE_DEPS[@]}" || {
        log_warn "Batch install failed; retrying one-by-one …"
        for dep in "${CORE_DEPS[@]}"; do
            pip install --quiet "${dep}" || log_warn "  Failed to install: ${dep}"
        done
    }

    log_ok "Core Python dependencies installed."

    # Print optional deps hint
    log_warn "Optional (install manually when hardware is ready):"
    log_warn "  MAVSDK:  pip install mavsdk>=1.4.0"
    log_warn "  rclpy:   source /opt/ros/humble/setup.bash"
}

# ==============================================================================
# Clone vendor repositories
# ==============================================================================
clone_vendor_repos() {
    log_step "Cloning Vendor Repositories"

    VENDOR_DIR="$(pwd)/vendor"
    mkdir -p "${VENDOR_DIR}"
    log_ok "Vendor directory: ${VENDOR_DIR}"

    # ---- Helper: clone if not already present ----------------------------
    clone_if_missing() {
        local name="$1"
        local url="$2"
        local dest="${VENDOR_DIR}/${name}"

        if [[ -d "${dest}/.git" ]]; then
            log_warn "Already cloned: ${dest} – skipping."
        else
            log_info "Cloning ${name} …"
            if git clone --depth 1 --quiet "${url}" "${dest}"; then
                log_ok "Cloned: ${name}"
            else
                log_warn "Failed to clone ${name} (no network? git not installed?)."
                log_warn "Manual clone:  git clone --depth 1 ${url} ${dest}"
            fi
        fi
    }

    # ---- MAVSDK Python (all platforms) -----------------------------------
    clone_if_missing \
        "mavsdk-python" \
        "https://github.com/mavlink/MAVSDK-Python.git"

    # ---- Fast-Planner (Linux only – requires ROS) ------------------------
    if [[ "${OS_TYPE}" == "linux" ]]; then
        clone_if_missing \
            "Fast-Planner" \
            "https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git"

        # ---- EGO-Planner-v2 (Linux only) ---------------------------------
        clone_if_missing \
            "EGO-Planner" \
            "https://github.com/ZJU-FAST-Lab/EGO-Planner-v2.git"
    else
        log_warn "Skipping Fast-Planner and EGO-Planner clones on ${OS_TYPE}."
        log_warn "These require ROS 2 (Linux).  To clone manually:"
        log_warn "  git clone --depth 1 https://github.com/HKUST-Aerial-Robotics/Fast-Planner.git vendor/Fast-Planner"
        log_warn "  git clone --depth 1 https://github.com/ZJU-FAST-Lab/EGO-Planner-v2.git vendor/EGO-Planner"
    fi
}

# ==============================================================================
# Create directory structure
# ==============================================================================
create_directories() {
    log_step "Creating Project Directories"

    dirs=(
        "logs"
        "vendor"
        "dashboard"
        "phase0_prototype/core"
        "phase0_prototype/configs"
        "phase0_prototype/tests"
        "phase1_sitl/mavsdk_scripts"
        "phase1_sitl/launch"
        "phase1_sitl/worlds"
        "phase2_comms/ns3_models"
        "phase3_planning/ego_swarm_bridge"
        "phase3_planning/fast_planner_bridge"
    )

    for dir in "${dirs[@]}"; do
        mkdir -p "${dir}"
    done

    log_ok "Project directories created."
}

# ==============================================================================
# Create .env file
# ==============================================================================
create_env_file() {
    log_step "Creating .env File"

    ENV_FILE="$(pwd)/.env"

    if [[ -f "${ENV_FILE}" ]]; then
        log_warn ".env already exists – not overwriting."
        return
    fi

    cat > "${ENV_FILE}" << 'EOF'
# ==============================================================================
# UAV-X Environment Configuration
# Copy to .env and adjust values as needed.
# Do NOT commit this file to version control.
# ==============================================================================

# --- Simulation ---------------------------------------------------------------
SIM_TICKS=300
SIM_SEED=42

# --- GCS Dashboard ------------------------------------------------------------
GCS_HOST=0.0.0.0
GCS_PORT=5000
API_SECRET=changeme_in_production

# --- PX4 SITL -----------------------------------------------------------------
PX4_HOME_LAT=47.397742
PX4_HOME_LON=8.545594
PX4_HOME_ALT=488.0
PX4_DIR=/home/$USER/PX4-Autopilot

# --- MAVSDK -------------------------------------------------------------------
# Ports for 10 simulated UAVs: 14540-14549 (MAVSDK), 14550-14559 (QGC)
MAVSDK_BASE_PORT=14540
GCS_BASE_PORT=14550

# --- ROS 2 --------------------------------------------------------------------
ROS_DISTRO=humble

# --- Docker -------------------------------------------------------------------
COMPOSE_PROJECT_NAME=uavx
EOF

    log_ok ".env file created at ${ENV_FILE}"
}

# ==============================================================================
# Create Dockerfiles for sub-projects (dashboard & simulation)
# ==============================================================================
create_dockerfiles() {
    log_step "Creating Dockerfiles"

    # ---- dashboard/Dockerfile ------------------------------------------------
    DASH_DIR="$(pwd)/dashboard"
    mkdir -p "${DASH_DIR}"

    if [[ ! -f "${DASH_DIR}/Dockerfile" ]]; then
        cat > "${DASH_DIR}/Dockerfile" << 'DOCKEREOF'
# UAV-X GCS Dashboard
FROM python:3.10-slim

LABEL maintainer="UAV-X Team"
LABEL description="Ground Control Station web dashboard"

WORKDIR /app

# System dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl && \
    rm -rf /var/lib/apt/lists/*

# Python dependencies
COPY requirements.txt* ./
RUN pip install --no-cache-dir flask>=2.0.0 requests>=2.27.0 gunicorn>=20.0.0

# Application source
COPY . .

# Create data directory for SQLite
RUN mkdir -p /app/data

EXPOSE 5000

# Health check
HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
    CMD curl -f http://localhost:5000/health || exit 1

CMD ["python", "app.py"]
DOCKEREOF
        log_ok "dashboard/Dockerfile created."
    else
        log_warn "dashboard/Dockerfile already exists – skipping."
    fi

    # ---- dashboard/app.py placeholder (if not present) ----------------------
    if [[ ! -f "${DASH_DIR}/app.py" ]]; then
        cat > "${DASH_DIR}/app.py" << 'APPEOF'
"""UAV-X GCS Dashboard – minimal Flask application stub."""
import os
from flask import Flask, jsonify

app = Flask(__name__)

@app.route("/health")
def health():
    return jsonify({"status": "ok", "service": "gcs-dashboard"})

@app.route("/api/telemetry", methods=["GET", "POST"])
def telemetry():
    return jsonify({"status": "received"})

@app.route("/api/link-states", methods=["GET", "POST"])
def link_states():
    return jsonify({"status": "received"})

@app.route("/")
def index():
    return "<h1>UAV-X GCS Dashboard</h1><p>Dashboard under construction.</p>"

if __name__ == "__main__":
    host = os.environ.get("GCS_HOST", "0.0.0.0")
    port = int(os.environ.get("GCS_PORT", 5000))
    app.run(host=host, port=port, debug=False)
APPEOF
        log_ok "dashboard/app.py stub created."
    fi

    # ---- phase0_prototype/Dockerfile -----------------------------------------
    P0_DIR="$(pwd)/phase0_prototype"
    mkdir -p "${P0_DIR}"

    if [[ ! -f "${P0_DIR}/Dockerfile" ]]; then
        cat > "${P0_DIR}/Dockerfile" << 'DOCKEREOF'
# UAV-X Phase-0 Simulation
FROM python:3.10-slim

LABEL maintainer="UAV-X Team"
LABEL description="Phase-0 pure-Python swarm simulation"

WORKDIR /app

# Python dependencies
COPY requirements.txt* ./
RUN pip install --no-cache-dir \
        matplotlib>=3.5.0 \
        numpy>=1.21.0 \
        requests>=2.27.0 \
    2>/dev/null || true

# Application source
COPY . .

# Log output directory
RUN mkdir -p /app/logs

CMD ["python", "main.py", "--no-viz", "--ticks", "300"]
DOCKEREOF
        log_ok "phase0_prototype/Dockerfile created."
    else
        log_warn "phase0_prototype/Dockerfile already exists – skipping."
    fi
}

# ==============================================================================
# Run basic smoke tests
# ==============================================================================
run_smoke_tests() {
    log_step "Running Smoke Tests"

    # Activate venv if not already active
    if [[ -z "${VIRTUAL_ENV:-}" ]] && [[ -f "$(pwd)/venv/bin/activate" ]]; then
        # shellcheck disable=SC1091
        source "$(pwd)/venv/bin/activate"
    fi

    # Test that key imports work
    log_info "Testing Python imports …"

    IMPORT_TESTS=(
        "import flask; print('flask', flask.__version__)"
        "import numpy; print('numpy', numpy.__version__)"
        "import matplotlib; print('matplotlib', matplotlib.__version__)"
        "import pytest; print('pytest', pytest.__version__)"
    )

    all_ok=true
    for test_cmd in "${IMPORT_TESTS[@]}"; do
        pkg=$(echo "${test_cmd}" | awk '{print $2}' | tr -d ';')
        if python -c "${test_cmd}" 2>/dev/null | grep -q "${pkg}"; then
            log_ok "  import ${pkg} … OK"
        else
            log_warn "  import ${pkg} … FAILED (may not be needed for Phase 0)"
            all_ok=false
        fi
    done

    if [[ "${all_ok}" == true ]]; then
        log_ok "All smoke tests passed."
    else
        log_warn "Some optional imports failed – see warnings above."
    fi
}

# ==============================================================================
# Print next steps
# ==============================================================================
print_next_steps() {
    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║              UAV-X Setup Complete!                               ║${NC}"
    echo -e "${GREEN}╠══════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  ${BOLD}Next Steps:${NC}                                                     ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  1. Activate the virtual environment:                            ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}       ${CYAN}source venv/bin/activate${NC}                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  2. Run Phase-0 simulation:                                      ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}       ${CYAN}cd phase0_prototype && python main.py${NC}                     ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  3. Run test suite:                                              ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}       ${CYAN}cd phase0_prototype && pytest tests/ -v${NC}                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  4. Start GCS Dashboard:                                         ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}       ${CYAN}docker compose up gcs-dashboard${NC}                          ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}       ${CYAN}open http://localhost:5000${NC}                                ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  5. Full Docker Compose stack:                                   ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}       ${CYAN}docker compose up${NC}                                         ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  6. SITL stack (requires PX4 + Gazebo):                         ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}       ${CYAN}docker compose --profile sitl up${NC}                         ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  7. Edit ${CYAN}.env${NC} to customise ports, seeds, and API keys.         ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  ${YELLOW}Vendor repos cloned to:${NC} ./vendor/                            ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  ${YELLOW}Virtual environment:${NC}     ./venv/                             ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}  ${YELLOW}Environment config:${NC}      ./.env                              ${GREEN}║${NC}"
    echo -e "${GREEN}║${NC}                                                                  ${GREEN}║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# ==============================================================================
# Main entry point
# ==============================================================================
main() {
    echo ""
    echo -e "${CYAN}╔══════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║        UAV-X Setup Script v1.0           ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════╝${NC}"
    echo ""

    # Ensure we are in the script's directory
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    cd "${SCRIPT_DIR}"
    log_info "Working directory: $(pwd)"

    # Run setup stages
    detect_os
    install_system_deps
    check_python
    check_pip
    create_directories
    create_venv
    install_python_deps
    clone_vendor_repos
    create_env_file
    create_dockerfiles
    run_smoke_tests
    print_next_steps
}

main "$@"
