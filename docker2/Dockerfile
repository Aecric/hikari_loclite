# syntax=docker/dockerfile:1.6
# ============================================================================
# hikari_loclite ROS2 .deb packaging Dockerfile.
#
# Build via build.sh or GitHub Actions. The default target is Humble / Jammy;
# other ROS/Ubuntu pairs can be selected with ROS_DISTRO and UBUNTU_CODENAME.
# ============================================================================

ARG ROS_DISTRO=humble
ARG UBUNTU_CODENAME=jammy

FROM ros:${ROS_DISTRO}-perception AS builder

ARG ROS_DISTRO
ARG UBUNTU_CODENAME
ARG TARGETARCH
ARG BUILD_JOBS=6

ENV ROS_WS=/root/slam_ws \
    DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

RUN if [ "$TARGETARCH" = "arm64" ]; then \
        REPO="mirrors.ustc.edu.cn/ubuntu-ports"; \
    else \
        REPO="mirrors.ustc.edu.cn/ubuntu"; \
    fi && \
    printf "deb https://$REPO/ ${UBUNTU_CODENAME} main restricted universe multiverse\n\
deb-src https://$REPO/ ${UBUNTU_CODENAME} main restricted universe multiverse\n\
deb https://$REPO/ ${UBUNTU_CODENAME}-updates main restricted universe multiverse\n\
deb-src https://$REPO/ ${UBUNTU_CODENAME}-updates main restricted universe multiverse\n\
deb https://$REPO/ ${UBUNTU_CODENAME}-security main restricted universe multiverse\n\
deb-src https://$REPO/ ${UBUNTU_CODENAME}-security main restricted universe multiverse\n" > /etc/apt/sources.list

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache-${TARGETARCH}-${UBUNTU_CODENAME} \
    --mount=type=cache,target=/var/lib/apt,sharing=locked,id=apt-lib-${TARGETARCH}-${UBUNTU_CODENAME} \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        cmake make g++ gcc libssl-dev libboost-all-dev libeigen3-dev \
        libpcl-dev libyaml-cpp-dev libflann-dev liblz4-dev libtbb-dev \
        python3-bloom python3-rosdep \
        fakeroot debhelper dpkg-dev devscripts equivs lsb-release \
        ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-visualization-msgs \
        zip ca-certificates curl libcap2-bin && \
    rm -rf /var/lib/apt/lists/*

ARG LIVOX_DEB_REPO=Aecric/livox_ros_driver2
ARG LIVOX_DEB_TAG=1.2.6.1
ARG LIVOX_DEB_VERSION=auto

RUN if [ "$TARGETARCH" = "arm64" ]; then LIVOX_ARCH="arm64"; else LIVOX_ARCH="amd64"; fi && \
    if [ "${LIVOX_DEB_VERSION}" = "auto" ]; then \
        if [ "${LIVOX_DEB_TAG}" = "latest" ]; then \
            RELEASE_API="https://api.github.com/repos/${LIVOX_DEB_REPO}/releases/latest"; \
        else \
            RELEASE_API="https://api.github.com/repos/${LIVOX_DEB_REPO}/releases/tags/${LIVOX_DEB_TAG}"; \
        fi && \
        echo "Resolving livox deb from ${RELEASE_API}" && \
        curl --retry 5 --retry-delay 2 --retry-all-errors -fSL "${RELEASE_API}" -o /tmp/livox-release.json && \
        python3 docker2/resolve_livox_deb.py \
            --release-json /tmp/livox-release.json \
            --ros-distro "${ROS_DISTRO}" \
            --ubuntu-codename "${UBUNTU_CODENAME}" \
            --arch "${LIVOX_ARCH}" \
            --output /tmp/livox-url.tsv && \
        LIVOX_DEB_NAME="$(cut -f1 /tmp/livox-url.tsv)" && \
        LIVOX_DEB_URL="$(cut -f2 /tmp/livox-url.tsv)" && \
        echo "Matched livox deb asset: ${LIVOX_DEB_NAME}" && \
        rm /tmp/livox-release.json; \
    else \
        LIVOX_DEB_URL="https://github.com/${LIVOX_DEB_REPO}/releases/download/${LIVOX_DEB_TAG}/ros-${ROS_DISTRO}-livox-ros-driver2_${LIVOX_DEB_VERSION}_${LIVOX_ARCH}.deb"; \
    fi && \
    echo "Fetching livox deb: ${LIVOX_DEB_URL}" && \
    curl --retry 5 --retry-delay 2 --retry-all-errors -fSL "${LIVOX_DEB_URL}" -o /tmp/livox.deb && \
    dpkg -i /tmp/livox.deb && \
    rm -f /tmp/livox.deb /tmp/livox-url.tsv

RUN rosdep init 2>/dev/null || true && \
    for i in 1 2 3; do \
        rosdep update --rosdistro=${ROS_DISTRO} && break; \
        if [ "$i" = "3" ]; then \
            echo "WARNING: rosdep update failed after 3 attempts; continuing because build deps are installed explicitly"; \
        else \
            echo "WARNING: rosdep update failed, retrying ($i/3)"; \
            sleep 5; \
        fi; \
    done

WORKDIR ${ROS_WS}/src/hikari_loclite
COPY . .

RUN /bin/bash -c "set -eo pipefail && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    set -u && \
    \
    python3 -c \"from pathlib import Path; import re; path=Path('package.xml'); text=re.sub(r'  <depend>(livox_ros_driver2|rosbag2_cpp)</depend>\\n', '', path.read_text()); path.write_text(text)\" && \
    \
    bloom-generate rosdebian \
        --os-name ubuntu \
        --os-version ${UBUNTU_CODENAME} \
        --ros-distro ${ROS_DISTRO} && \
    \
    sed -i 's|dh_auto_configure --|dh_auto_configure -- -DCMAKE_BUILD_TYPE=Release|' debian/rules && \
    sed -i \"s|^\\([[:space:]]*\\)dh_auto_build$|\\1dh_auto_build -- -j${BUILD_JOBS}|\" debian/rules && \
    python3 docker2/inject_control_dep.py debian/control \"ros-${ROS_DISTRO}-livox-ros-driver2\" && \
    printf '\noverride_dh_usrlocal:\n\ttrue\n' >> debian/rules && \
    \
    cp docker2/postinst.in debian/postinst && \
    cp docker2/postrm.in debian/postrm && \
    sed -i \"s|__ROS_DISTRO__|${ROS_DISTRO}|g\" debian/postinst && \
    sed -i \"s|__ROS_DISTRO__|${ROS_DISTRO}|g\" debian/postrm && \
    chmod 0755 debian/postinst debian/postrm && \
    \
    echo '===== package.xml used for bloom =====' && grep -n '<depend>\\|<version>' package.xml && \
    echo '===== debian/control Depends =====' && grep -A20 '^Depends:' debian/control && \
    echo '===== debian/rules key sections =====' && grep -A3 'override_dh_auto_configure\|override_dh_auto_build\|override_dh_usrlocal' debian/rules && \
    \
    export DEB_BUILD_OPTIONS=parallel=${BUILD_JOBS} && \
    fakeroot debian/rules binary"

RUN mkdir -p /output && \
    cp ${ROS_WS}/src/*.deb /output/ && \
    ls -lh /output/

FROM scratch AS export
COPY --from=builder /output/*.deb /
