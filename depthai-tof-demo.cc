#include <depthai/depthai.hpp>
#include <depthai/device/Device.hpp>

#include <depthai/pipeline/node/ToF.hpp>
#include <depthai/pipeline/node/Camera.hpp>


void initUndistortRectifyMap( const float* lp,
                              int height, int width, float* map12 ) {
    float u0 = lp[2],  v0 = lp[3];
    float fx = lp[0],  fy = lp[1];

    float k1 = lp[4];
    float k2 = lp[5];
    float p1 = lp[6];
    float p2 = lp[7];
    float k3 = lp[8];
    float k4 = lp[9];
    float k5 = lp[10];
    float k6 = lp[11];

    float ir[] = {
            1.f/fx,       0,       -u0/fx,
            0,       1.f/fy,       -v0/fy,
            0,           0,       1
    };

    for( int i = 0; i < height; i++ )
    {
        float _x = i*ir[1] + ir[2], _y = i*ir[4] + ir[5], _w = i*ir[7] + ir[8];

        for( int j = 0; j < width; j++, _x += ir[0], _y += ir[3], _w += ir[6] )
        {
            float w = 1.f/_w, x = _x*w, y = _y*w;
            float x2 = x*x, y2 = y*y;
            float r2 = x2 + y2, _2xy = 2*x*y;
            float kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2)/(1 + ((k6*r2 + k5)*r2 + k4)*r2);
            float u = fx*(x*kr + p1*_2xy + p2*(r2 + 2*x2)) + u0;
            float v = fy*(y*kr + p1*(r2 + 2*y2) + p2*_2xy) + v0;

            map12[(width*i + j) * 3 + 0] = u;
            map12[(width*i + j) * 3 + 1] = v;
        }
    }
}

#define PMD_TOF_ROWS 172
#define PMD_TOF_COLS 224

void fill_pointcloud_map(const float* lp, float* pc_map) {
    float u0 = lp[2],  v0 = lp[3];
    float fx = lp[0],  fy = lp[1];

    initUndistortRectifyMap(lp, PMD_TOF_ROWS, PMD_TOF_COLS, pc_map);
    for(int y = 0;y < PMD_TOF_ROWS;y++) {
        for(int x = 0;x < PMD_TOF_COLS;x++) {
            size_t index = (PMD_TOF_COLS * y + x) * 3;
            float xs = pc_map[index + 0];
            float ys = pc_map[index + 1];
            float rx = (xs - u0) / fx;
            float ry = (ys - v0) / fy;

            pc_map[index + 0] = 1./sqrt(rx*rx + ry*ry + 1);
            pc_map[index + 1] = rx;
            pc_map[index + 2] = ry;
        }
    }
}

float ext_scale = .01;

#define RESTRICT_PTR *__restrict__
static inline void transform_point(const std::vector<std::vector<float>>& tx, const float *xyz, float *tx_xyz) {
    float X = xyz[0], Y = xyz[1], Z = xyz[2];
    tx_xyz[0] = tx[0][0]*X + tx[0][1]*Y + tx[0][2]*Z + tx[0][3] * ext_scale;
    tx_xyz[1] = tx[1][0]*X + tx[1][1]*Y + tx[1][2]*Z + tx[1][3] * ext_scale;
    tx_xyz[2] = tx[2][0]*X + tx[2][1]*Y + tx[2][2]*Z + tx[2][3] * ext_scale;
}

static inline void project_points(const float RESTRICT_PTR lp, const float  RESTRICT_PTR xyz,
                                  float RESTRICT_PTR uv) {
    const float fx = lp[0], fy = lp[1], cx = lp[2], cy = lp[3];

    float x = xyz[0], y = xyz[1], z = xyz[2];
    z = z ? 1./z : 1;
    x *= z; y *= z;

    const float r2 = x*x + y*y;
    const float r4 = r2*r2;
    const float r6 = r4*r2;
    const float a1 = 2*x*y;
    const float a2 = r2 + 2*x*x;
    const float a3 = r2 + 2*y*y;

    const float cdist = 1 + lp[4+0]*r2 + lp[4+1]*r4 + lp[4+4]*r6;
    const float icdist2 = 1.f/(1 + lp[4+5]*r2 + lp[4+6]*r4 + lp[4+7]*r6);
    const float xd0 = x*cdist*icdist2 + lp[4+2]*a1 + lp[4+3]*a2 + lp[4+8]*r2+lp[4+9]*r4;
    const float yd0 = y*cdist*icdist2 + lp[4+2]*a3 + lp[4+3]*a1 + lp[4+10]*r2+lp[4+11]*r4;

    const float vecTilt[3] = {xd0, yd0, 1};
    const float invProj = vecTilt[2] != 0 ? 1.f/vecTilt[2] : 1;
    const float xd = invProj * vecTilt[0];
    const float yd = invProj * vecTilt[1];

    uv[0] = xd*fx + cx;
    uv[1] = yd*fy + cy;
}
cv::Mat lastRGB;
struct DepthaAICamera {
    std::string name;

    DepthaAICamera(const std::string &name)
            : name(name) {}

  void publish(const cv::Mat &cvFrame, const cv::Mat *blend = 0) const {
        cv::namedWindow(name, cv::WINDOW_GUI_EXPANDED); 
        if (cvFrame.channels() == 3) {
            cv::imshow(name, cvFrame);
	    lastRGB = cvFrame;
        } else {
            cv::Mat as8u;
            cv::convertScaleAbs(cvFrame, as8u, 1 / 3000. * 255., 0);
            //cv::normalize(cvFrame, as8u, 0, 255, cv::NORM_MINMAX, CV_8U);

            cv::Mat img_color;
            applyColorMap(as8u, img_color, cv::COLORMAP_JET);

	    cv::Mat mask;
	    cv::inRange(cvFrame, cv::Scalar(0), cv::Scalar(0), mask);
	    img_color.setTo(cv::Scalar(0, 0, 0), mask);

	    if(blend && blend->data)
	      addWeighted( img_color, .5, *blend, .5, 0.0, img_color);

            cv::imshow(name, img_color);
        }
    }
};

std::vector<float> create_lp(const std::vector<std::vector<float>>& k, const std::vector<float>& d) {
    std::vector<float> rtn = {
            k[0][0],
            k[1][1],
            k[0][2],
            k[1][2],
            d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8]
    };
    rtn.resize(12);
    return rtn;
}

static inline void xyz_from_depth(float depth, const float RESTRICT_PTR zxy, float RESTRICT_PTR  xyz) {
    float z_scale = /*zxy[0]*/1, nx_over_z = zxy[1], ny_over_z = zxy[2];
    const float z = fmax(0, depth * z_scale);

    xyz[0] = nx_over_z * z;
    xyz[1] = ny_over_z * z;
    xyz[2] = z;
}

cv::Mat registerDepth(const cv::Mat& depth, const float* pc_map, const std::vector<std::vector<float>>& tx,
                      const std::vector<float>& depth_lp, const std::vector<float>& rgb_lp, int height, int width, float scale) {
    cv::Mat_<uint16_t> registeredDepth = cv::Mat_<uint16_t>::zeros(height / scale, width / scale);
    auto scaled_rgb = rgb_lp;
    for(int i = 0;i < 4;i++) scaled_rgb[i] = rgb_lp[i] / scale;

    for(int i = 0;i < depth.cols;i++){
        for(int j = 0;j < depth.rows;j++){
            uint16_t d = depth.at<uint16_t>(j,i);
            if(d == 0) continue;
            float xyz[3];
            float uv[2];
            xyz_from_depth(d / 1000., pc_map + 3 * (i + j * depth.cols), xyz);
            transform_point(tx, xyz, xyz);
            project_points(scaled_rgb.data(), xyz, uv);
            if(uv[0] >= 0 && uv[0] < registeredDepth.cols)
                if(uv[1] >= 0 && uv[1] < registeredDepth.rows) {
                    registeredDepth(uv[1], uv[0]) = d;
                }
        }
    }
    return registeredDepth;
}

dai::CameraBoardSocket rgb_socket = dai::CameraBoardSocket::AUTO;
dai::CameraBoardSocket tof_socket = dai::CameraBoardSocket::CENTER;

float pc_map[PMD_TOF_COLS * PMD_TOF_ROWS * 3] = { 0 };
int start(dai::Device &device, int argc, char **argv) {

    auto cnow = std::chrono::steady_clock::now();
    std::vector<std::shared_ptr<DepthaAICamera>> sockets = {
            std::make_shared<DepthaAICamera>("rgb"),
            //std::make_shared<DepthaAICamera>("raw"),
            std::make_shared<DepthaAICamera>("depth"),
            std::make_shared<DepthaAICamera>("registered_depth"),
            //std::make_shared<DepthaAICamera>("amplitude"),
            //std::make_shared<DepthaAICamera>("error"),
    };

    std::shared_ptr<DepthaAICamera> rgb_camera = sockets[0];
    std::map<std::string, std::shared_ptr<DepthaAICamera>> cameras;
    for (auto &s : sockets) {
        cameras[s->name] = s;
    }

    auto names = device.getOutputQueueNames();
    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> namedStreams;

    std::map<std::string, double> latency;
    for (auto &n : names) {
        namedStreams[n] = device.getOutputQueue(n, 1, false);
    }

    auto calibration = device.readCalibration();
    std::vector<std::vector<float>> tx;
    std::vector<float> depth_lp, rgb_lp;    
    try {
      auto depth_intrinsics = calibration.getCameraIntrinsics(tof_socket);
      auto depth_distortion = calibration.getDistortionCoefficients(tof_socket);
      auto rgb_intrinsics = calibration.getCameraIntrinsics(rgb_socket);
      auto rgb_distortion = calibration.getDistortionCoefficients(rgb_socket);
      tx = calibration.getCameraExtrinsics(tof_socket, rgb_socket);

      depth_lp = create_lp(depth_intrinsics, depth_distortion);
      rgb_lp = create_lp(rgb_intrinsics, rgb_distortion);
      fill_pointcloud_map(depth_lp.data(), pc_map);
    } catch(std::exception& e) {
      fprintf(stderr, "Error trying to access calibration. Your device must be calibrated for the full visualization. Error: %s\n", e.what());
    }

    int width = 0, height = 0;

    while (!device.isClosed()) {
        auto ch = cv::waitKey(5);
	if(ch == 'q' || ch == 'Q') break;
	
        auto event = device.getQueueEvent(names, std::chrono::milliseconds(100));
        if (event.empty()) {
            continue;
        }
        auto data = namedStreams[event]->tryGet<dai::ImgFrame>();

        if (data) {
            cv::Mat cvFrame;
            std::string encoding;
            {
                cvFrame = data->getFrame();

                if (data->getType() == dai::ImgFrame::Type::NV12) {
                    cvFrame = data->getCvFrame();
                } else if (data->getType() == dai::ImgFrame::Type::YUV420p) {
                    // Allow the conversion
                    cvFrame = data->getCvFrame();
                } else if (event == "raw") {
#define ROWS 172
#define COLS 224
                    cvFrame = cv::Mat(9 * (ROWS + 1), COLS, CV_16S, data->getData().data());
                    encoding = "16UC1";
                }
            }

            if (cameras[event] && cvFrame.rows > 1 && cvFrame.cols > 1)
                cameras[event]->publish(cvFrame);
            if(event == "rgb") {
                width = cvFrame.cols;
                height = cvFrame.rows;
            }
            if(width != 0 && event == "depth" && cvFrame.rows > 1 && cvFrame.cols > 1 && !tx.empty()) {
                auto registered_depth = registerDepth(cvFrame, pc_map, tx, depth_lp, rgb_lp, height, width, 4);
		cv::Mat downsample;
		cv::resize(lastRGB, downsample, cv::Size(registered_depth.cols, registered_depth.rows));
                cameras["registered_depth"]->publish(registered_depth, &downsample);
            }
        }
    }

    return 0;
}

int main(int argc, char **argv) {
    dai::Pipeline pipeline;

    for(int i = 1;i < argc;i++) {
      if(std::string(argv[i]) == "--right") {
	rgb_socket = dai::CameraBoardSocket::RIGHT;
      } if(std::string(argv[i]) == "--left") {
	rgb_socket = dai::CameraBoardSocket::LEFT;
      } else if(std::string(argv[i]) == "--camD") {
	rgb_socket = dai::CameraBoardSocket::CAM_D;
      }

      if(std::string(argv[i]) == "--old-ext-cal") {
	ext_scale = 1;
      }
    }

    dai::Device device;
    dai::CameraBoardSocket possible_rgb_socket = dai::CameraBoardSocket::AUTO;
    bool rgbSocketFound = false;
    for(auto& feature : device.getConnectedCameraFeatures()) {
      if(feature.socket == dai::CameraBoardSocket::RIGHT && feature.sensorName == "IMX378") {
	// Right and left share an i2c bus and IMX378's don't have an option to have different device IDs. So
	// we just assume they are in the left socket always for now.
	continue;
      }
      fprintf(stderr, "Camera %s on socket %d\n", feature.sensorName.c_str(), (int)feature.socket);
      bool canBeRGB = false;
      bool canBeMono = false;
      for(auto& type : feature.supportedTypes) {
	canBeRGB |= type == dai::CameraSensorType::COLOR;
	canBeMono |= type == dai::CameraSensorType::MONO;
      }
      if(rgb_socket == feature.socket && !canBeRGB) {
	rgb_socket = dai::CameraBoardSocket::AUTO;
      }
      rgbSocketFound |= rgb_socket == feature.socket; 
      if(canBeRGB) {
	possible_rgb_socket = feature.socket;
	if(!canBeMono) break;
      }
    }

    if(!rgbSocketFound || rgb_socket == dai::CameraBoardSocket::AUTO) {
      rgb_socket = possible_rgb_socket;
    }

    if(rgb_socket == dai::CameraBoardSocket::AUTO) {
      fprintf(stderr, "Could not find RGB socket; exiting\n");
      return -1;
    }
    
    auto xinPicture = pipeline.create<dai::node::Camera>();
    auto tof = pipeline.create<dai::node::ToF>();

    auto rgbPicture = pipeline.create<dai::node::ColorCamera>();
    rgbPicture->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    rgbPicture->setFps(30);
    rgbPicture->setBoardSocket(rgb_socket);
    //rgbPicture->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);

    std::list<std::pair<std::string, typeof(rgbPicture->isp) *>> outs = {
            {"depth",     &tof->out},
            {"amplitude", &tof->amp_out},
            {"raw",       &xinPicture->raw},
            {"rgb",       &rgbPicture->isp},
    };

    xinPicture->raw.link(tof->inputImage);

    for (auto &kv : outs) {
        auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
        xoutVideo->setStreamName(kv.first);
        kv.second->link(xoutVideo->input);
    }

    device.startPipeline(pipeline);
    device.setLogLevel(dai::LogLevel::INFO);

    return start(device, argc, argv);
}
