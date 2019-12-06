#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <ros_vino/Object.h>
#include <ros_vino/Objects.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "utilities/tracked_object.h"

using arma::mat;
using arma::vec;

class MultiKfTracker
{

public:

	MultiKfTracker();

	~MultiKfTracker();

	void run();

private:

    bool getParameters(ros::NodeHandle nh_private);

    void setParameters();

    void initROSInterface();

    void rosCallbackImageRgb(const sensor_msgs::Image::ConstPtr& image_msg);

    void rosCallbackDetectedObjects(const ros_vino::Objects::ConstPtr& objects);

    void updateObjects();

    void publishObjects();

    double objectCostFunction(const ros_vino::Object& new_object, const ros_vino::Object& old_object);

    void calculateCostMatrix(const std::vector<ros_vino::Object>& new_objects, mat& cost_matrix);

    void calculateRowMinIndices(const mat& cost_matrix, std::vector<int>& row_min_indices);
    

    // **************************
	// ROS INTERFACE
	// **************************
    ros::NodeHandle 		nh;
    ros::NodeHandle         nh_private;

	ros::Subscriber 		sub_image_rgb;
	ros::Subscriber 		sub_detected_objects;
	ros::Publisher 		    pub_image_tracked;
	ros::Publisher 		    pub_tracked_objects;

    std::string 			topic_image_input;
	std::string 			topic_image_output;
	std::string 			topic_objects_input;
	std::string 			topic_objects_output;

    // **************************
	// Variables
	// **************************
    
    cv::Mat                         current_image;
    std::vector<ros_vino::Object>   new_detected_objects;
    std::vector<TrackedObject>      tracked_objects;
    std::vector<TrackedObject>      untracked_objects;
    std::vector<TrackedObject>      new_tracked_objects;
    std::vector<TrackedObject>      new_untracked_objects;

    int                             id_counter;

    double                          p_min_correspondence_cost;
    double                          p_loop_rate;
    double                          p_tracking_duration;

    int                             p_fade_counter_size;
    double                          p_sampling_time;
    double                          p_process_variance;
    double                          p_process_rate_variance;
    double                          p_measurement_variance;

};