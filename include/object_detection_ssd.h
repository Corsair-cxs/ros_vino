// #include <gflags/gflags.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <chrono>
#include <iostream>
#include <fstream>
// #include <memory>
// #include <vector>
#include <string>
// #include <algorithm>
#include <boost/bind.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros_vino/Object.h>
#include <ros_vino/Objects.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <inference_engine.hpp>
#include <samples/common.hpp>
#include <samples/ocv_common.hpp>

#ifdef WITH_EXTENSIONS
    #include <ext_list.hpp>
#endif

using namespace InferenceEngine;

class ObjectDetectionSSD
{

public:

	ObjectDetectionSSD();

	~ObjectDetectionSSD();

	void run();

private:

    void initInferenceEngine();

    bool getParameters(ros::NodeHandle nh_private);

    void initROSInterface();

    void rosCallbackImage(const sensor_msgs::Image::ConstPtr& image_msg);

    void frameToBlob(const cv::Mat& frame, InferRequest::Ptr& inferRequest, const std::string& inputName);

    std_msgs::Header getHeader();


    // **************************
	// ROS INTERFACE
	// **************************
    ros::NodeHandle 		nh;
    ros::NodeHandle         nh_private;

	ros::Subscriber 		sub_image_rgb;
	ros::Publisher 		    pub_image_rects;
	ros::Publisher 		    pub_objects;

    std::string 			topic_image_input;
	std::string 			topic_image_output;
	std::string 			topic_objects_output;

    // **************************
	// IE VARIABLES
	// **************************
    std::string 			model_path;
    std::string 			device;
    double 	        		score_threshold;
    bool                    bool_auto_resize;
    bool                    bool_pc;
    bool                    bool_raw;

    Core                    ie;

    // **************************
	// Application Variables
	// **************************
    InferRequest::Ptr       async_infer_request_curr;
    InferRequest::Ptr       async_infer_request_next;

    std::string             imageInputName;
    std::string             imageInfoInputName;
    std::string             outputName;

    SizeVector              outputDims;
    int                     maxProposalCount;
    int                     objectSize;

    std::vector<std::string> labels;

    cv::Mat                 image;
    cv::Mat                 curr_frame;
    cv::Mat                 next_frame;

    size_t                  width;
    size_t                  height;

    bool                    is_new_image;
    bool                    is_first_image;

};