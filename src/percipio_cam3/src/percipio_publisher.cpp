/** talker.cpp **/
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.h"
#include "builtin_interfaces/msg/time.hpp"

#include "../include/common.hpp"
#include "../include/TYImageProc.h"

using namespace std::chrono_literals;

static cv::Mat depth_image, color_image;
static TY_INTERFACE_HANDLE hIface = NULL;
static TY_ISP_HANDLE hColorIspHandle = NULL;
static TY_DEV_HANDLE hDevice = NULL;

static TY_CAMERA_CALIB_INFO depth_calib; 
static int32_t m_depth_width;
static int32_t m_depth_height;

static TY_CAMERA_CALIB_INFO color_calib; 
static int32_t m_color_width;
static int32_t m_color_height;

static sensor_msgs::msg::Image __ros2_depth_image;
static sensor_msgs::msg::Image __ros2_rgb_image;
static sensor_msgs::msg::PointCloud2 __ros2_pcl2_output;

static pcl::PointCloud<pcl::PointXYZ> cloud;

static std::string device_sn = "";
static bool b_rgb_enable = true;
static bool b_depth_enable = true;
static std::string szDepthResolution = "640x480";
static std::string szColorResolution = "640x480";
static bool b_depth_registration = false;
static bool b_rgb_undistortion = false;
static bool b_map_depth2rgb = false;

using namespace std::placeholders;

void set_now(builtin_interfaces::msg::Time & time)
{
  std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
  if (now <= std::chrono::nanoseconds(0)) {
    time.sec = time.nanosec = 0;
  } else {
    time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
    time.nanosec = now.count() % 1000000000;
  }
}


std::string mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
    case CV_16U:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

static inline bool fillImage(
  sensor_msgs::msg::Image & image,
  const std::string & encoding_arg,
  uint32_t rows_arg,
  uint32_t cols_arg,
  uint32_t step_arg,
  const void * data_arg)
{
  image.encoding = encoding_arg;
  image.height = rows_arg;
  image.width = cols_arg;
  image.step = step_arg;
  size_t st0 = (step_arg * rows_arg);
  image.data.resize(st0);
  std::memcpy(&image.data[0], data_arg, st0);

  image.is_bigendian = 0;
  return true;
}

/// Clear the data of an image message.
/**
 * \details All fields but `data` are kept the same.
 * \param[out]image Image to be cleared.
 */
static inline void clearImage(sensor_msgs::msg::Image & image)
{
  image.data.resize(0);
}

class MinimalDepthSubscriber : public rclcpp::Node {
  public:
    MinimalDepthSubscriber()
        : Node("percipio_depth_cam") {

        /* Note: it is very important to use a QOS profile for the subscriber that is compatible
         * with the QOS profile of the publisher.
         * The ZED component node uses a default QoS profile with reliability set as "RELIABLE"
         * and durability set as "VOLATILE".
         * To be able to receive the subscribed topic the subscriber must use compatible
         * parameters.
         */

        // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings

        //rclcpp::QoS depth_qos(10);
        //depth_qos.keep_last(10);
        //depth_qos.best_effort();
        //depth_qos.durability_volatile();

        mDepthSub = rclcpp::Node::create_publisher<sensor_msgs::msg::Image>("percipio_depth", 1);//, rclcpp::SensorDataQoS());
        mColorSub = rclcpp::Node::create_publisher<sensor_msgs::msg::Image>("percipio_rgb", 1);//, rclcpp::SensorDataQoS());
        pcl_pub   = rclcpp::Node::create_publisher<sensor_msgs::msg::PointCloud2> ("percipio_pcl", 1);
        timer_ = this->create_wall_timer(1ms, std::bind(&MinimalDepthSubscriber::timer_callback, this));
    }

  protected:
    void timer_callback()
    {
        //auto message = std_msgs::msg::String();
        //message.data = "Hello, world! " + std::to_string(count_++);
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        
        /*
        auto message = tutorial_interfaces::msg::Num();                               // CHANGE
        message.num = this->count_++;                                        // CHANGE
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);    // CHANGE
        publisher_->publish(message);
        */
        static int m_frame_idx = 0;
    	TY_FRAME_DATA frame;
    	int err = TYFetchFrame(hDevice, &frame, 2000);
    	if( err == TY_STATUS_OK ) {
    	    cv::Mat depth, irl, irr, color;
            parseFrame(frame, &depth, &irl, &irr, &color, hColorIspHandle);
            if(!depth.empty()){
                //sensor_msgs::
                fillImage(__ros2_depth_image, sensor_msgs::image_encodings::MONO16, depth.rows, depth.cols, 2*depth.cols, depth.data);
                __ros2_depth_image.header.frame_id = "map";
                mDepthSub->publish(__ros2_depth_image);

                std::vector<TY_VECT_3F> p3d;
                p3d.resize(depth.size().area());
                ASSERT_OK(TYMapDepthImageToPoint3d(&depth_calib, depth.cols, depth.rows, (uint16_t*)depth.data, &p3d[0]));

                cloud.width  = depth.size().area();
                cloud.height = 1;
                cloud.points.resize(depth.size().area());
                for (size_t i = 0; i < cloud.points.size (); ++i)
                {
                    cloud.points[i].x = p3d[i].x / 1000;
                    cloud.points[i].y = p3d[i].y / 1000;
                    cloud.points[i].z = p3d[i].z / 1000;
                }

                pcl::toROSMsg(cloud, __ros2_pcl2_output);
                __ros2_pcl2_output.header.frame_id = "map";
                pcl_pub->publish(__ros2_pcl2_output);
            }
            if(!color.empty()){
                //sensor_msgs::
                fillImage(__ros2_rgb_image, sensor_msgs::image_encodings::TYPE_8UC3, color.rows, color.cols, 3*color.cols, color.data);
                __ros2_rgb_image.header.frame_id = "map";
                mColorSub->publish(__ros2_rgb_image);
            }
            m_frame_idx++;
            TYISPUpdateDevice(hColorIspHandle);
            ASSERT_OK( TYEnqueueBuffer(hDevice, frame.userBuffer, frame.bufferSize) );
    	}
      }
  
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDepthSub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mColorSub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    ////////////////////////////////////////////////////////////////////////////////////////
    LOGD("Init lib");
    ASSERT_OK( TYInitLib() );
    TY_VERSION_INFO ver;
    ASSERT_OK( TYLibVersion(&ver) );
    LOGD("     - lib version: %d.%d.%d", ver.major, ver.minor, ver.patch);

    std::vector<TY_DEVICE_BASE_INFO> selected;
    ASSERT_OK( selectDevice(TY_INTERFACE_ALL, device_sn, "", 1, selected) );
    ASSERT(selected.size() > 0);
    TY_DEVICE_BASE_INFO& selectedDev = selected[0];

    ASSERT_OK( TYOpenInterface(selectedDev.iface.id, &hIface) );
    ASSERT_OK( TYOpenDevice(hIface, selectedDev.id, &hDevice) );

    int32_t allComps;
    ASSERT_OK( TYGetComponentIDs(hDevice, &allComps) );

    int _img_width;
    ///try to enable color camera
    if(allComps & TY_COMPONENT_RGB_CAM  && b_rgb_enable) {
        int32_t image_mode;
        ASSERT_OK(get_default_image_mode(hDevice, TY_COMPONENT_RGB_CAM, image_mode));
        ASSERT_OK(TYSetEnum(hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, image_mode));
        
        _img_width = atoi(szColorResolution.data());
        LOGD("RGB Width: %d\n", _img_width);
        std::vector<TY_ENUM_ENTRY> image_mode_list;
	    get_feature_enum_list(hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, image_mode_list);
	    for (uint32_t idx = 0; idx < image_mode_list.size(); idx++){
            TY_ENUM_ENTRY &entry = image_mode_list[idx];
            if (TYImageWidth(entry.value) == _img_width || TYImageHeight(entry.value) == _img_width){
                LOGD("Select RGB Image Mode: %s\n", entry.description);
                TYSetEnum(hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, entry.value);
                image_mode = entry.value;
                break;
            }
        }
        
        m_color_width = TYImageWidth(image_mode);
        m_color_height = TYImageHeight(image_mode);
        color_image = cv::Mat::zeros(m_color_height, m_color_width, CV_8UC3);
        
        LOGD("Has RGB camera, open RGB cam\n");
        ASSERT_OK( TYEnableComponents(hDevice, TY_COMPONENT_RGB_CAM) );
        
        TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, &color_calib, sizeof(color_calib));
        
        //create a isp handle to convert raw image(color bayer format) to rgb image
        ASSERT_OK(TYISPCreate(&hColorIspHandle));
        //Init code can be modified in common.hpp
        //NOTE: Should set RGB image format & size before init ISP
        ASSERT_OK(ColorIspInitSetting(hColorIspHandle, hDevice));
        //You can  call follow function to show  color isp supported features
    }

    /*
    if (allComps & TY_COMPONENT_IR_CAM_LEFT && ir) {
        //LOGD("Has IR left camera, open IR left cam");
        ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_LEFT));
    }

    if (allComps & TY_COMPONENT_IR_CAM_RIGHT && ir) {
        //LOGD("Has IR right camera, open IR right cam");
        ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_RIGHT));
    }
    */
    
    if (allComps & TY_COMPONENT_DEPTH_CAM && b_depth_enable) {
        int32_t image_mode;
        ASSERT_OK(get_default_image_mode(hDevice, TY_COMPONENT_DEPTH_CAM, image_mode));
        ASSERT_OK(TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, image_mode));
        
        _img_width = atoi(szDepthResolution.data());
        //LOGD("DEPTH Width: %d\n", _img_width);
        std::vector<TY_ENUM_ENTRY> image_mode_list;
	    get_feature_enum_list(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, image_mode_list);
	    for (uint32_t idx = 0; idx < image_mode_list.size(); idx++){
            TY_ENUM_ENTRY &entry = image_mode_list[idx];
            if (TYImageWidth(entry.value) == _img_width || TYImageHeight(entry.value) == _img_width){
                LOGD("Select Depth Image Mode: %s\n", entry.description);
                TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, entry.value);
                image_mode = entry.value;
                break;
            }
        }
        
        m_depth_width = TYImageWidth(image_mode);
        m_depth_height = TYImageHeight(image_mode);
        depth_image = cv::Mat::zeros(m_depth_height, m_depth_width, CV_16U);
        
        ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_DEPTH_CAM));
        
        //depth map pixel format is uint16_t ,which default unit is  1 mm
        //the acutal depth (mm)= PixelValue * ScaleUnit 
        float scale_unit = 1.;
        TYGetFloat(hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale_unit);
        TYGetStruct(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA, &depth_calib, sizeof(depth_calib));
    }
  
    LOGD("Prepare image buffer");
    uint32_t frameSize;
    ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
    LOGD("     - Get size of framebuffer, %d", frameSize);

    LOGD("     - Allocate & enqueue buffers");
    char* frameBuffer[2];
    frameBuffer[0] = new char[frameSize];
    frameBuffer[1] = new char[frameSize];
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize) );
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize) );
    
    bool hasTrigger;
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &hasTrigger));
    if (hasTrigger) {
        LOGD("Disable trigger mode");
        TY_TRIGGER_PARAM trigger;
        trigger.mode = TY_TRIGGER_MODE_OFF;
        ASSERT_OK(TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger)));
    }

    LOGD("Start capture");
    ASSERT_OK( TYStartCapture(hDevice) );
    
    auto depth_node = std::make_shared<MinimalDepthSubscriber>();

    rclcpp::spin(depth_node);
    rclcpp::shutdown();
    return 0;
  }
