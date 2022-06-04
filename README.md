# DepthAI Demo 

This runs a minimal demo for the ToF sensor in DepthAI. It runs both the HQVGA and VGA PMD sensors. 

# Quickstart

`./run-docker.h`

### Run 5fps:
`./run-docker.h --fps 5`

# Requirements

To build locally, you will need DepthAI built or installed locally as well as OpenCV. 

To build in the docker environment, you will need docker installed and your user added to the docker group. Then just run `./build-docker.sh`. 

# Running

With a device plugged in, run the local-built binary or use the `./run_docker.sh` script. Both take the same arguments / environment variables:

### Arguments
- `--right`: The demo assumes the RGB camera is mapped to the 'left' socket on the device. This uses the right socket instead.
- `--camD`: This uses the 'camD' socket on device for RGB.
- `--old-ext-cal`: Specify the on-device extrinsics is in mm instead of cm. This is only useful for older development devices which have bugged calibrations.
- `--fps <fps>`: Sets ToF into a specific FPS setting
- `--invert`: Debug flag that inverts the transform from color to depth.
- `--flip`: Flip the RGB/registered frames to be 180 degrees their normal orientation.
