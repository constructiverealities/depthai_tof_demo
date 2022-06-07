xhost +local:root
docker run  -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.Xauthority:/root/.Xauthority -it --rm --privileged -e DISPLAY=$DISPLAY ghcr.io/constructiverealities/depthai_tof_demo:main $@
xhost -local:root
