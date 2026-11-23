#!/usr/bin/env bash
set -Eeuo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world="$root/worlds/farm_5ha_project.sdf"
px4="${PX4_DIR:-$HOME/PX4-Autopilot}"
mode="${1:-view}"
pad="${2:-main}"

if [[ "$mode" == view ]]; then
    exec gz sim -r "$world"
fi
case "$mode" in
    x500) target=gz_x500 ;;
    lidar) target=gz_x500_lidar_2d ;;
    *) printf 'Dùng: bash scripts/run_world.sh [view|x500|lidar] [main|east]\n' >&2; exit 2 ;;
esac
case "$pad" in
    main) pose="-119,-151,0.35,0,0,0" ;;
    east) pose="54,-165,0.35,0,0,0" ;;
    *) printf 'Bãi đáp phải là main hoặc east.\n' >&2; exit 2 ;;
esac

if pgrep -ax px4; then
    printf 'Dừng phiên PX4 đang chạy trước khi mở world mới.\n' >&2
    exit 1
fi
destination="$px4/Tools/simulation/gz/worlds/farm_5ha_project.sdf"
[[ -d "$(dirname "$destination")" ]] || {
    printf 'Không tìm thấy thư mục Gazebo worlds trong %s\n' "$px4" >&2
    exit 1
}
linked=0
cleanup() {
    if (( linked )) && [[ -L "$destination" ]] && [[ "$(readlink "$destination")" == "$world" ]]; then
        unlink "$destination"
    fi
}
trap cleanup EXIT

if [[ -e "$destination" || -L "$destination" ]]; then
    cmp -s "$world" "$destination" || {
        printf 'World đích đã tồn tại và khác nội dung: %s\n' "$destination" >&2
        exit 1
    }
else
    ln -sT "$world" "$destination"
    linked=1
fi

cd "$px4"
unset PX4_GZ_MODEL_NAME PX4_GZ_STANDALONE PX4_SIM_MODEL PX4_SYS_AUTOSTART
export PX4_GZ_WORLD=farm_5ha_project PX4_GZ_MODEL_POSE="$pose"
make px4_sitl "$target"
