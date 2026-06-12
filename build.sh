#!/usr/bin/env bash
# ============================================================================
# Build hikari_loclite .deb locally with Docker buildx.
#
# Defaults target the Jetson-friendly ROS 2 Humble / Ubuntu Jammy package:
#   ./build.sh
#
# Useful overrides:
#   ROS_DISTRO=humble  UBUNTU_CODENAME=jammy     ./build.sh
#   ROS_DISTRO=jazzy   UBUNTU_CODENAME=noble     ./build.sh
#   TARGETARCH=amd64  ./build.sh
#   TARGETARCH=arm64  ./build.sh
#   BUILD_JOBS=2 OUTPUT_DIR=/tmp/hikari-loclite-debs ./build.sh
# ============================================================================
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
UBUNTU_CODENAME="${UBUNTU_CODENAME:-jammy}"
LIVOX_DEB_REPO="${LIVOX_DEB_REPO:-Aecric/livox_ros_driver2}"
LIVOX_DEB_TAG="${LIVOX_DEB_TAG:-1.2.6.1}"
LIVOX_DEB_VERSION="${LIVOX_DEB_VERSION:-auto}"

HOST_ARCH="$(uname -m)"
case "${HOST_ARCH}" in
    x86_64|amd64) DEFAULT_TARGETARCH="amd64" ;;
    aarch64|arm64) DEFAULT_TARGETARCH="arm64" ;;
    *) DEFAULT_TARGETARCH="${HOST_ARCH}" ;;
esac

TARGETARCH="${TARGETARCH:-${DEFAULT_TARGETARCH}}"
case "${TARGETARCH}" in
    amd64) PLATFORM="linux/amd64" ;;
    arm64) PLATFORM="linux/arm64" ;;
    *) echo "ERROR: unsupported TARGETARCH='${TARGETARCH}' (expected amd64 or arm64)" >&2; exit 1 ;;
esac

DOCKERFILE="docker2/Dockerfile"
OUTPUT_DIR="${OUTPUT_DIR:-./debs/${ROS_DISTRO}-${TARGETARCH}}"
BUILD_JOBS="${BUILD_JOBS:-6}"
BUILDER_NAME="${BUILDER_NAME:-hikari-loclite-${TARGETARCH}-builder}"

echo "==> [1/5] Check target architecture"
if [ "${TARGETARCH}" = "arm64" ] && [ "${HOST_ARCH}" != "aarch64" ] && [ "${HOST_ARCH}" != "arm64" ]; then
    if [ ! -f /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
        echo "    qemu-aarch64 not detected; installing binfmt support..."
        docker run --privileged --rm tonistiigi/binfmt --install arm64
    else
        echo "    qemu-aarch64 is ready"
    fi
else
    echo "    ${TARGETARCH} target does not require QEMU"
fi

echo "==> [2/5] Prepare buildx builder"
if ! docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
    docker buildx create \
        --name "${BUILDER_NAME}" \
        --driver docker-container \
        --driver-opt network=host \
        --use \
        --bootstrap
else
    docker buildx use "${BUILDER_NAME}"
fi
docker buildx inspect --bootstrap | grep -E "Name|Driver|Platforms"

echo "==> [3/5] Show build resources"
HOST_CORES="$(nproc)"
HOST_MEM_GB="$(free -g | awk '/^Mem:/ {print $2}')"
HOST_MEM_AVAIL_GB="$(free -g | awk '/^Mem:/ {print $7}')"
echo "    host arch:     ${HOST_ARCH}"
echo "    host cores:    ${HOST_CORES}"
echo "    host memory:   ${HOST_MEM_GB} GB"
echo "    available:     ${HOST_MEM_AVAIL_GB} GB"
echo "    target arch:   ${TARGETARCH}"
echo "    platform:      ${PLATFORM}"
echo "    build jobs:    ${BUILD_JOBS}"

RECOMMENDED_JOBS=$(( HOST_MEM_AVAIL_GB / 4 ))
[ "${RECOMMENDED_JOBS}" -lt 1 ] && RECOMMENDED_JOBS=1
[ "${RECOMMENDED_JOBS}" -gt 8 ] && RECOMMENDED_JOBS=8
if [ "${BUILD_JOBS}" -gt "${RECOMMENDED_JOBS}" ]; then
    echo ""
    echo "    WARNING: BUILD_JOBS=${BUILD_JOBS} exceeds recommended ${RECOMMENDED_JOBS}; build may OOM."
    echo "    Press Ctrl+C to abort, or wait 5 seconds to continue."
    sleep 5
fi

echo "==> [4/5] Build package"
mkdir -p "${OUTPUT_DIR}"
BUILD_START="$(date +%s)"
echo "    target:        ros-${ROS_DISTRO} on ubuntu-${UBUNTU_CODENAME} (${TARGETARCH})"
echo "    livox deb:     repo=${LIVOX_DEB_REPO} tag=${LIVOX_DEB_TAG} version=${LIVOX_DEB_VERSION}"
echo "    output:        ${OUTPUT_DIR}"

docker buildx build \
    --platform "${PLATFORM}" \
    --file "${DOCKERFILE}" \
    --target export \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --build-arg UBUNTU_CODENAME="${UBUNTU_CODENAME}" \
    --build-arg BUILD_JOBS="${BUILD_JOBS}" \
    --build-arg LIVOX_DEB_REPO="${LIVOX_DEB_REPO}" \
    --build-arg LIVOX_DEB_TAG="${LIVOX_DEB_TAG}" \
    --build-arg LIVOX_DEB_VERSION="${LIVOX_DEB_VERSION}" \
    --output "type=local,dest=${OUTPUT_DIR}" \
    --progress plain \
    .

echo ""
echo "==> [5/5] Generated packages"
BUILD_END="$(date +%s)"
BUILD_ELAPSED=$(( BUILD_END - BUILD_START ))
printf "    elapsed: %02d:%02d:%02d\n" \
    $((BUILD_ELAPSED / 3600)) \
    $(((BUILD_ELAPSED % 3600) / 60)) \
    $((BUILD_ELAPSED % 60))
ls -lh "${OUTPUT_DIR}"/*.deb

echo ""
echo "Install on target:"
echo "    scp ${OUTPUT_DIR}/*.deb <user>@<host>:~/"
echo "    ssh <user>@<host> 'sudo dpkg -i ~/ros-${ROS_DISTRO}-livox-ros-driver2_*.deb ~/ros-${ROS_DISTRO}-hikari-loclite_*.deb && sudo apt -f install'"
