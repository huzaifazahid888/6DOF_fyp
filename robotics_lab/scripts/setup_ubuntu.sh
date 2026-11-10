#!/usr/bin/env bash
# Prepare Ubuntu 24.04 for the PIEAS ROS 2 Jazzy / Gazebo Harmonic lab.
set -Eeuo pipefail

usage() {
    cat <<'HELP'
Usage: bash scripts/setup_ubuntu.sh [--check] [--yes] [--build] [--with-moveit]

Default           Install missing prerequisites and resolve workspace dependencies.
--check           Inspect prerequisites locally without sudo, downloads, or changes.
--yes             Accept apt and rosdep installation prompts.
--build           Also build the workspace after installation.
--with-moveit     Include MoveIt and its Setup Assistant for later milestones.
--help            Show this help.

Run as your normal user. sudo is requested only for system changes.
HELP
}

check_only=false
assume_yes=false
build_workspace=false
with_moveit=false
for argument in "$@"; do
    case "$argument" in
        --check) check_only=true ;;
        --yes) assume_yes=true ;;
        --build) build_workspace=true ;;
        --with-moveit) with_moveit=true ;;
        --help|-h) usage; exit 0 ;;
        *) printf 'Unknown option: %s\n' "$argument" >&2; usage >&2; exit 2 ;;
    esac
done
if "$check_only" && "$build_workspace"; then
    printf '%s\n' '--check and --build cannot be combined.' >&2
    exit 2
fi

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
trap 'printf "Setup failed at line %s. Resolve the error above, then rerun the same command.\n" "$LINENO" >&2' ERR
[[ -r /etc/os-release ]] || fail 'Cannot identify this operating system.'
# shellcheck source=/dev/null
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] ||
    fail "This project targets Ubuntu 24.04. Detected ${PRETTY_NAME:-unknown}."
architecture=$(dpkg --print-architecture)
[[ "$architecture" == amd64 || "$architecture" == arm64 ]] ||
    fail "Supported architectures are amd64 and arm64. Detected $architecture."
[[ -z "${ROS_DISTRO:-}" || "$ROS_DISTRO" == jazzy ]] ||
    fail "This shell has ROS $ROS_DISTRO loaded. Open a terminal with that environment disabled."
[[ -z "${CONDA_PREFIX:-}" && -z "${VIRTUAL_ENV:-}" ]] ||
    fail 'Deactivate the Python virtual environment or Conda before setup.'

script_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd -- "$script_directory/.." && pwd)
[[ -f "$workspace/src/pieas_arm_description/package.xml" ]] ||
    fail 'Keep this script inside the cloned repository under robotics_lab/scripts.'
export LANG=C.UTF-8 LC_ALL=C.UTF-8

packages=(
    ca-certificates curl git python3 build-essential cmake
    python3-colcon-common-extensions python3-rosdep python3-yaml
    ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
    ros-jazzy-ros2-controllers ros-jazzy-controller-manager ros-jazzy-ros2controlcli
    ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros
    ros-jazzy-ament-cmake ros-jazzy-ament-index-python
    ros-jazzy-launch ros-jazzy-launch-ros
)
if "$with_moveit"; then
    packages+=(ros-jazzy-moveit ros-jazzy-moveit-setup-assistant)
fi

installed() {
    [[ "$(dpkg-query -W -f='${Status}' "$1" 2>/dev/null || true)" == 'install ok installed' ]]
}

missing=()
find_missing() {
    missing=()
    local package
    for package in "${packages[@]}"; do
        if ! installed "$package"; then missing+=("$package"); fi
    done
}
find_missing
printf 'Platform: %s (%s)\nWorkspace: %s\n' "$PRETTY_NAME" "$architecture" "$workspace"
printf 'Missing apt packages: %s\n' "${#missing[@]}"
if (("${#missing[@]}")); then printf '  %s\n' "${missing[@]}"; fi

verify_runtime() {
    [[ -r /opt/ros/jazzy/setup.bash ]] || return 1
    # ROS setup scripts can reference unset optional variables.
    set +u
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    set -u
    local package
    for package in ros_gz_sim ros_gz_bridge gz_ros2_control robot_state_publisher joint_trajectory_controller; do
        ros2 pkg prefix "$package" >/dev/null || return 1
    done
    ros2 control --help >/dev/null || return 1
    command -v colcon >/dev/null || return 1
    command -v rosdep >/dev/null || return 1
    command -v gz >/dev/null || return 1
    local versions
    versions=$(gz sim --versions) || return 1
    printf 'Available Gazebo Sim versions: %s\n' "$versions"
    [[ "$versions" =~ (^|[^0-9])8[.] ]] || return 1
}

if "$check_only"; then
    (("${#missing[@]}" == 0)) || exit 1
    verify_runtime || fail 'ROS package discovery, ros2 control CLI, or Gazebo Harmonic version check failed.'
    [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] ||
        fail 'rosdep has not been initialized. Run setup without --check.'
    rosdep check --from-paths "$workspace/src" --ignore-src --rosdistro jazzy ||
        fail 'Workspace dependencies or the local rosdep cache need setup.'
    printf '%s\n' 'Local prerequisite checks passed. GUI and simulation behavior need a launch test.'
    exit 0
fi

((EUID != 0)) || fail 'Run as your normal user, without sudo before this script.'
command -v sudo >/dev/null || fail 'sudo is required for system package installation.'
command -v flock >/dev/null || fail 'flock from util-linux is required.'
# Serialize concurrent runs for this user. apt also maintains its own system lock.
lock_directory="${XDG_RUNTIME_DIR:-/tmp}"
exec 9>"$lock_directory/pieas-setup-$UID.lock"
flock -n 9 || fail 'Another setup run is active for this user.'
temporary_directory=''
trap '[[ -z "$temporary_directory" ]] || rm -rf -- "$temporary_directory"' EXIT
apt_options=(-o DPkg::Lock::Timeout=120)
install_options=(--no-remove)
if "$assume_yes"; then install_options+=(-y); fi

install_missing() {
    local needed=() package
    for package in "$@"; do
        if ! installed "$package"; then needed+=("$package"); fi
    done
    if (("${#needed[@]}")); then
        sudo apt-get "${apt_options[@]}" install "${install_options[@]}" "${needed[@]}"
    fi
}

if (("${#missing[@]}")); then
    sudo -v
    sudo apt-get "${apt_options[@]}" update
    install_missing ca-certificates curl python3 software-properties-common
    sudo add-apt-repository -y universe
    # Reuse a configured ROS repository where Jazzy packages already have a candidate.
    candidate=$(apt-cache policy ros-jazzy-ros-base | awk '/Candidate:/ {print $2}')
    if [[ -z "$candidate" || "$candidate" == '(none)' ]]; then
        if ! installed ros2-apt-source; then
            temporary_directory=$(mktemp -d)
            curl --fail --location --retry 3 --connect-timeout 20 --max-time 180 \
                https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
                -o "$temporary_directory/release.json"
            version=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$temporary_directory/release.json")
            [[ "$version" =~ ^[0-9]+[.][0-9]+[.][0-9]+$ ]] ||
                fail "Unexpected ROS apt source release version: $version"
            curl --fail --location --retry 3 --connect-timeout 20 --max-time 180 \
                "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$version/ros2-apt-source_$version.noble_all.deb" \
                -o "$temporary_directory/ros2-apt-source.deb"
            [[ "$(dpkg-deb -f "$temporary_directory/ros2-apt-source.deb" Package)" == ros2-apt-source ]] ||
                fail 'Downloaded package is not ros2-apt-source.'
            sudo dpkg -i "$temporary_directory/ros2-apt-source.deb"
        fi
    fi
    sudo apt-get "${apt_options[@]}" update
    install_missing "${packages[@]}"
fi

verify_runtime || fail 'ROS package discovery, ros2 control CLI, or Gazebo Harmonic version check failed.'
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update --rosdistro jazzy
rosdep_options=()
if "$assume_yes"; then rosdep_options+=(-y); fi
rosdep install --from-paths "$workspace/src" --ignore-src --rosdistro jazzy "${rosdep_options[@]}"
rosdep check --from-paths "$workspace/src" --ignore-src --rosdistro jazzy

if "$build_workspace"; then
    (cd -- "$workspace" && colcon build --symlink-install)
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    printf '%s\n' 'No graphical display was detected. Launch the Gazebo GUI from a desktop session.'
fi
printf '\n%s\n' 'Prerequisite setup completed. Run these in your terminal:'
printf 'source /opt/ros/jazzy/setup.bash\ncd %q\n' "$workspace"
if ! "$build_workspace"; then printf '%s\n' 'colcon build --symlink-install'; fi
printf '%s\n' 'source install/setup.bash' 'ros2 launch pieas_arm_description lab.launch.py'
