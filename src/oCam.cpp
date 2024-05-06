#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
//#include <dynamic_reconfigure/server.h>
// #include <ocam/camConfig.h>

#include "withrobot_camera.hpp"

class Camera
{
    Withrobot::Camera* camera;
    Withrobot::camera_format camFormat;

private:
    int width_;
    int height_;
    std::string devPath_;

public:

    Camera(int resolution, double frame_rate): camera(NULL) {

        enum_dev_list();

        camera = new Withrobot::Camera(devPath_.c_str());

        if (resolution == 0) { width_ = 1280; height_ = 960;}
        if (resolution == 1) { width_ = 1280; height_ = 720;}
        if (resolution == 2) { width_ = 640; height_  = 480;}
        if (resolution == 3) { width_ = 320; height_  = 240;}

        camera->set_format(width_, height_, Withrobot::fourcc_to_pixformat('G','R','B','G'), 1, (unsigned int)frame_rate);

        /*
         * get current camera format (image size and frame rate)
         */
        camera->get_current_format(camFormat);

        camFormat.print();

        /* Withrobot camera start */
        camera->start();
	}

    ~Camera() {
        camera->stop();
        delete camera;

	}

    void enum_dev_list()
    {
        /* enumerate device(UVC compatible devices) list */
        std::vector<Withrobot::usb_device_info> dev_list;
        int dev_num = Withrobot::get_usb_device_info_list(dev_list);

        if (dev_num < 1) {
            dev_list.clear();

            return;
        }

        for (unsigned int i=0; i < dev_list.size(); i++) {
            if (dev_list[i].product == "oCam-1CGN-U")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCam-1CGN-U-T")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCam-1MGN-U")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCam-1MGN-U-T")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
        }
    }

    void uvc_control(int exposure, int gain, int blue, int red, bool ae)
    {
        /* Exposure Setting */
        camera->set_control("Exposure (Absolute)", exposure);

        /* Gain Setting */
        camera->set_control("Gain", gain);

        /* White Balance Setting */
        camera->set_control("White Balance Blue Component", blue);
        camera->set_control("White Balance Red Component", red);

        /* Auto Exposure Setting */
        if (ae)
            camera->set_control("Exposure, Auto", 0x3);
        else
            camera->set_control("Exposure, Auto", 0x1);

    }

    bool getImages(cv::Mat &image) {

        cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
        cv::Mat dstImg;

        if (camera->get_frame(srcImg.data, camFormat.image_size, 1) != -1)
        {
            cvtColor(srcImg, dstImg, cv::COLOR_BayerGR2RGB);
            image = dstImg;

            return true;
        } else {
            return false;
        }
	}
};

/**
 * @brief       the camera ros warpper class
 */
class oCamROS {

private:
    int resolution_;
    double frame_rate_;
    int exposure_, gain_, wb_blue_, wb_red_;
    bool autoexposure_;
    bool show_image_;
    bool config_changed_;

    std::string camera_frame_id_;
    Camera* ocam;

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<image_transport::ImageTransport> img_transport;
    image_transport::Publisher img_pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> cam_info_pub;
    // image_transport::CameraPublisher camera_image_pub;

public:
    void device_poll() {
//         //Reconfigure confidence
//         dynamic_reconfigure::Server<ocam::camConfig> server;
//         dynamic_reconfigure::Server<ocam::camConfig>::CallbackType f;
//         f = boost::bind(&oCamROS::callback, this ,_1, _2);
//         server.setCallback(f);

//         // setup publisher stuff
//         image_transport::ImageTransport it(nh);
//         image_transport::Publisher camera_image_pub = it.advertise("camera/image_raw", 1);

//         ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);

        sensor_msgs::msg::CameraInfo camera_info;
        RCLCPP_INFO(node->get_logger(),"Loading from ROS calibration files");


//         // get config from the left, right.yaml in config
        // camera_info_manager::CameraInfoManager info_manager(node.get());
        // info_manager.setCameraName("ocam");
        // info_manager.loadCameraInfo( "package://ocam/config/camera.yaml");
        // camera_info = info_manager.getCameraInfo();

        // camera_info.header.frame_id = camera_frame_id_;

        RCLCPP_INFO(node->get_logger(), "Got camera calibration files");

        // loop to publish images;
        cv::Mat camera_image;
        rclcpp::Rate r(frame_rate_);
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        while (rclcpp::ok())
        {
            auto now = node->get_clock()->now();

            if (!ocam->getImages(camera_image)) {
                usleep(1000);
                continue;
            } else {
                RCLCPP_INFO_ONCE(node->get_logger(), "Success, found camera");
            }

            // if (camera_image_pub.getNumSubscribers() > 0) {
            //     std_msgs::msg::Header hdr;
            //     sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(hdr, "bgr8", camera_image).toImageMsg();
            //     // img_msg->header.stamp = now;
            //     // img_msg->header.frame_id = camera_frame_id_;
            //     camera_info.header.stamp = now;
            //     camera_image_pub.publish(img_msg, camera_info, now);
            // }
            if (img_pub.getNumSubscribers() >0)
            {
                std_msgs::msg::Header hdr;
                sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(hdr, "bgr8", camera_image).toImageMsg();
                img_msg->header.stamp = now;
                img_msg->header.frame_id = camera_frame_id_;
                img_pub.publish(img_msg);
            }
            //
            // if (cam_info_pub->get_subscription_count() > 0)
            // {
            //     camera_info.header.stamp = now;
            //     camera_info.header.frame_id = camera_frame_id_;
            //     cam_info_pub->publish(camera_info);
            // }

            if (show_image_) {
                cv::imshow("image", camera_image);
                cv::waitKey(10);
            }
            //
            executor.spin_some();
            r.sleep();
        }
    }

//     void callback(ocam::camConfig &config, uint32_t level) {
//         ocam->uvc_control(config.exposure, config.gain, config.wb_blue, config.wb_red, config.auto_exposure);
    // }

    /**
	 * @brief      { function_description }
	 *
	 * @param[in]  resolution  The resolution
	 * @param[in]  frame_rate  The frame rate
     */
    oCamROS(){

        node = rclcpp::Node::make_shared("ocam_node", rclcpp::NodeOptions());
        /* default parameters */
        resolution_ = 1;
        frame_rate_ = 30.0;
        exposure_ = 100;
        gain_ = 150;
        wb_blue_ = 200;
        wb_red_ = 160;
        autoexposure_= false;
        camera_frame_id_ = "camera";
        show_image_ = false;

        /* get parameters */
        // priv_nh.getParam("resolution", resolution_);
        // priv_nh.getParam("frame_rate", frame_rate_);
        // priv_nh.getParam("exposure", exposure_);
        // priv_nh.getParam("gain", gain_);
        // priv_nh.getParam("wb_blue", wb_blue_);
        // priv_nh.getParam("wb_red", wb_red_);
        // priv_nh.getParam("camera_frame_id", camera_frame_id_);
        // priv_nh.getParam("show_image", show_image_);
        // priv_nh.getParam("auto_exposure", autoexposure_);

        /* initialize the camera */
        ocam = new Camera(resolution_, frame_rate_);
        ocam->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        RCLCPP_INFO(node->get_logger(), "Initialized the camera");

        img_transport = std::make_shared<image_transport::ImageTransport>(node);
        // camera_image_pub = img_transport->advertiseCamera("ocam", 10);
        img_pub = img_transport->advertise("image", 30);
        cam_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());
	}

    ~oCamROS() {
        delete ocam;
    }
};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    oCamROS ocam;
    ocam.device_poll();
    rclcpp::shutdown();
    return 0;
}