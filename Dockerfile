FROM debian:bullseye-slim

ARG DEBIAN_FRONTEND=noninteractive

RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt \
 apt update && apt-get install --no-install-recommends -y \
    build-essential \
    libopencv-dev \
    udev usbutils autoconf \
    libtool \
    curl git libusb-dev libusb-1.0-0-dev software-properties-common \
    automake \
    cmake
    
SHELL ["/bin/bash", "-c"]
ENV PYTHONUNBUFFERED 1

RUN git clone -v https://github.com/libusb/libusb.git /repos/libusb || true && \
    cd /repos/libusb && git fetch && \
    git reset --hard master && \
    cd /repos/libusb && ./autogen.sh --disable-udev && make install


RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules
ARG REPO_URL_depthai_core=https://github.com/luxonis/depthai-core.git
ARG REPO_BRANCH_depthai_core=tof_rgb_mono

ADD https://api.github.com/repos/luxonis/depthai-core/branches/${REPO_BRANCH_depthai_core} cache-check
RUN git clone -b ${REPO_BRANCH_depthai_core} https://github.com/luxonis/depthai-core.git --recursive /repos/depthai_core && \
    mkdir -p /build/depthai-core/${REPO_BRANCH_depthai_core} && \
    cd /build/depthai-core/${REPO_BRANCH_depthai_core} && cmake -DDEPTHAI_BUILD_EXAMPLES=OFF -DDEPTHAI_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_SHARED_LIBS=On /repos/depthai_core && make -j4 install

ADD . /
RUN mkdir -p /build/depthai_tof_demo && \
    cd /build/depthai_tof_demo && \
    cmake / && \
    make -j 4

RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt \
 apt update && apt-get install --no-install-recommends -y \
    build-essential \
    udev
    

CMD /build/depthai_tof_demo/depthai-tof-demo
