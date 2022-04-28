xhost +local:root
DEPTHAI_MODEL_NAME="${DEPTHAI_MODEL_NAME:-depthai}"
docker run  -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.Xauthority:/root/.Xauthority -it --rm --privileged -e DEPTHAI_MODEL_NAME=$DEPTHAI_MODEL_NAME -e DISPLAY=$DISPLAY ghcr.io/constructiverealities/depthai_tof_demo:latest DEPTHAI_DEVICE_BINARY=/mvcmds/$DEPTHAI_MODEL_NAME.mvcmd /build/depthai_tof_demo/depthai-tof-demo $@
xhost -local:root
