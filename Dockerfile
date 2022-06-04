FROM debian:bullseye-slim

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
ENV PYTHONUNBUFFERED 1

RUN apt update && apt-get install --no-install-recommends -y \
    build-essential \
    udev usbutils autoconf \
    libtool \
    curl git libusb-dev libusb-1.0-0-dev software-properties-common \
    automake

RUN git clone -v https://github.com/libusb/libusb.git /repos/libusb && \
    cd /repos/libusb && git fetch && \
    git reset --hard master && \
    cd /repos/libusb && ./autogen.sh --disable-udev && make install && \
    rm -rf /repos/libusb

RUN apt update && apt-get install --no-install-recommends -y \
     libopencv-dev \
     cmake
        
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules

ADD https://api.github.com/repos/constructiverealities/depthai/branches/cr/develop cache-check
RUN git clone -b cr/develop https://github.com/constructiverealities/depthai.git --recursive /repos/depthai_core && \
    mkdir -p /build/depthai-core/cr/develop && \
    cd /build/depthai-core/cr/develop && cmake -DDEPTHAI_BUILD_EXAMPLES=OFF -DDEPTHAI_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_SHARED_LIBS=On /repos/depthai_core && make -j4 install && \
    rm -rf /build /repos/depthai_core

ADD . /repos/source

RUN --mount=type=ssh --mount=type=cache,target=/root/.hunter  \
    mkdir -p /build/depthai_tof_demo && \
    cd /build/depthai_tof_demo && \
    cmake -DBUILD_DEPTHAI=OFF /repos/source && \
    make -j 4 install && \
    rm -rf /repos

ENV DEPTHAI_MODEL_NAME depthai
ENV DEPTHAI_LEVEL debug
ENTRYPOINT ["/build/depthai_tof_demo/depthai-tof-demo"]
