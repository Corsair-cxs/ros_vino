#include "multi_kf_tracker.h"

MultiKfTracker::MultiKfTracker()
{
    id_counter = -1;
}

MultiKfTracker::~MultiKfTracker()
{
}

void MultiKfTracker::run()
{
    ros::NodeHandle nh_private("~");

    getParameters(nh_private);

    setParameters();

    initROSInterface();

    ROS_INFO("Start Tracking Bounding Boxes");

    ros::Rate loop_rate(p_loop_rate);

    while(ros::ok())
	{
        ros::spinOnce();
        publishObjects();
        updateObjects();
        loop_rate.sleep();
    }
}

void MultiKfTracker::updateObjects()
{
    for (int i = 0; i < tracked_objects.size(); ++i) {
        if (!tracked_objects[i].hasFaded())
        {
            tracked_objects[i].predictState();
        }
        else
        {
            tracked_objects.erase(tracked_objects.begin() + i--);
        }
    }
}

void MultiKfTracker::rosCallbackImageRgb(const sensor_msgs::Image::ConstPtr& image_msg)
{
    current_image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
}

void MultiKfTracker::rosCallbackDetectedObjects(const ros_vino::Objects::ConstPtr& detected_objects)
{
    new_detected_objects = detected_objects->objects;
    int N = new_detected_objects.size();
    int T = tracked_objects.size();
    int U = untracked_objects.size();

    if (T + U == 0) {
        untracked_objects.assign(new_detected_objects.begin(), new_detected_objects.end());
        return;
    }
    
    mat cost_matrix;
    calculateCostMatrix(new_detected_objects, cost_matrix);

    // std::cout << "cost_matrix:\n" << cost_matrix << "\n";

    std::vector<int> row_min_indices;
    calculateRowMinIndices(cost_matrix, row_min_indices);

    new_tracked_objects.clear();
    new_untracked_objects.clear();

    for (int n = 0; n < N; ++n) {

        if (row_min_indices[n] == -1) {
            new_untracked_objects.push_back(new_detected_objects[n]);
        }
        else {
            if (row_min_indices[n] >= 0 && row_min_indices[n] < T) {
                tracked_objects[row_min_indices[n]].correctState(new_detected_objects[n]);
            }
            else if (row_min_indices[n] >= T) { // New detection
                TrackedObject to(untracked_objects[row_min_indices[n] - T]);
                to.correctState(new_detected_objects[n]);
                id_counter++;
                to.setId(id_counter);
                
                new_tracked_objects.push_back(to);
            }
        }
    }

    tracked_objects.insert(tracked_objects.end(), new_tracked_objects.begin(), new_tracked_objects.end());

    // Remove old untracked objects and save new ones
    untracked_objects.clear();
    untracked_objects.assign(new_untracked_objects.begin(), new_untracked_objects.end());
}

double MultiKfTracker::objectCostFunction(const ros_vino::Object& new_object, const ros_vino::Object& old_object) {

    double label_cost = (new_object.label != old_object.label) ? 1 : 0; 

    cv::Rect new_rect = cv::Rect(new_object.x, new_object.y, new_object.width, new_object.height);
    cv::Rect old_rect = cv::Rect(old_object.x, old_object.y, old_object.width, old_object.height);
    cv::Rect union_rect = new_rect | old_rect;
    cv::Rect intersect_rect = new_rect & old_rect;
    double iou_cost = 1.0 - (double(intersect_rect.area()) / double(union_rect.area()));

    double cost = 1000*label_cost + 100*iou_cost;
    return cost;
}

void MultiKfTracker::calculateCostMatrix(const std::vector<ros_vino::Object>& new_objects, mat& cost_matrix) {
    /*
    * Cost between two objects represents their difference.
    * The bigger the cost, the less similar they are.
    * N rows of cost_matrix represent new objects.
    * T+U columns of cost matrix represent old tracked and untracked objects.
    */
    int N = new_objects.size();
    int T = tracked_objects.size();
    int U = untracked_objects.size();
    cost_matrix = mat(N, T + U);
    cost_matrix.fill(0.0);

    for (int n = 0; n < N; ++n) {
        for (int t = 0; t < T; ++t)
        {
            cost_matrix(n, t) = objectCostFunction(new_objects[n], tracked_objects[t].getObject());
        }
            
        for (int u = 0; u < U; ++u)
        {
            cost_matrix(n, u + T) = objectCostFunction(new_objects[n], untracked_objects[u].getObject());
        }  
    }
}

void MultiKfTracker::calculateRowMinIndices(const mat& cost_matrix, std::vector<int>& row_min_indices) {
    /*
    * Vector of row minimal indices keeps the indices of old objects (tracked and untracked)
    * that have the minimum cost related to each of new objects, i.e. row_min_indices[n]
    * keeps the index of old object that has the minimum cost with n-th new object.
    */
    int N,T,U;
    N = cost_matrix.n_rows;
    T = tracked_objects.size();
    U = untracked_objects.size();

    row_min_indices.assign(N, -1); // Minimum index -1 means no correspondence has been found

    for (int n = 0; n < N; ++n) {
        double min_cost = p_min_correspondence_cost;

        for (int t = 0; t < T; ++t) {
            if (cost_matrix(n, t) < min_cost) {
                min_cost = cost_matrix(n, t);
                row_min_indices[n] = t;
            }
        }

        for (int u = 0; u < U; ++u) {
            if (cost_matrix(n, u + T) < min_cost) {
                min_cost = cost_matrix(n, u + T);
                row_min_indices[n] = u + T;
            }
        }
    }
}

bool MultiKfTracker::getParameters(ros::NodeHandle nh_private)
{
    ROS_INFO("[MultiKfTracker] Getting parameters ...");
    // ROS parameters
	nh_private.param("topic_image_input", topic_image_input, std::string("/camera/color/image_raw"));
    ROS_INFO("Set topic_image_input to: %s", topic_image_input.c_str());
    nh_private.param("topic_objects_input", topic_objects_input, std::string("/camera/detected_objects"));
    ROS_INFO("Set topic_objects_input to: %s", topic_objects_input.c_str());
    nh_private.param("topic_image_output", topic_image_output, std::string("/camera/image_tracked"));
    ROS_INFO("Set topic_image_output to: %s", topic_image_output.c_str());
    nh_private.param("topic_objects_output", topic_objects_output, std::string("/camera/tracked_objects"));
    ROS_INFO("Set topic_objects_output to: %s", topic_objects_output.c_str());

    // Hungarian parameters
    nh_private.param("min_correspondence_cost", p_min_correspondence_cost, double(100.0));

    // Kalman parameters
    nh_private.param("p_sampling_time", p_sampling_time, double(30.0));
    nh_private.param("p_loop_rate", p_loop_rate, double(30.0));
    nh_private.param("p_tracking_duration", p_tracking_duration, double(0.5));
    nh_private.param("p_process_variance", p_process_variance, double(1.0));
    nh_private.param("p_process_rate_variance", p_process_rate_variance, double(10.0));
    nh_private.param("p_measurement_variance", p_measurement_variance, double(10000.0));

    p_fade_counter_size = p_loop_rate * p_tracking_duration;
    
	return true;
}

void MultiKfTracker::setParameters()
{
    TrackedObject::setSamplingTime(p_sampling_time);
    TrackedObject::setCounterSize(p_fade_counter_size);
    TrackedObject::setCovariances(p_process_variance, p_process_rate_variance, p_measurement_variance);
}

void MultiKfTracker::initROSInterface()
{
    ROS_INFO("Subscribing to %s", topic_image_input.c_str());
	sub_image_rgb = nh.subscribe(topic_image_input, 1, &MultiKfTracker::rosCallbackImageRgb, this);
    ROS_INFO("Subscribing to %s", topic_objects_input.c_str());
	sub_detected_objects = nh.subscribe(topic_objects_input, 1, &MultiKfTracker::rosCallbackDetectedObjects, this);
    ROS_INFO("Publishing to %s", topic_image_output.c_str());
    pub_image_tracked = nh.advertise<sensor_msgs::Image>(topic_image_output, 1);
    ROS_INFO("Publishing to %s", topic_objects_output.c_str());
    pub_tracked_objects = nh.advertise<ros_vino::Objects>(topic_objects_output, 1);
}

void MultiKfTracker::publishObjects()
{
    // Publish object
    ros_vino::Objects objects_msg;
    for (int i=0; i < tracked_objects.size(); i++)
    {
        ros_vino::Object object = tracked_objects[i].getObject();
        objects_msg.objects.push_back(object);

        // Draw each bounding box
        std::ostringstream conf;
        conf << ":" << std::fixed << std::setprecision(3) << object.confidence;
        srand(object.id);
        int b = std::rand()%256;
        int g = std::rand()%256;
        int r = std::rand()%256;
        cv::putText(current_image,
                    "(" + std::to_string(object.id) + ") " + object.label,
                    cv::Point2f(object.x, object.y + 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5,
                    cv::Scalar(b, g, r), 2);
        cv::rectangle(current_image, 
                        cv::Point2f(object.x, object.y), 
                        cv::Point2f(object.x+object.height, object.y+object.width), 
                        cv::Scalar(b, g, r), 4);
    }
    pub_tracked_objects.publish(objects_msg);

    // Publish image_tracked
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();
    pub_image_tracked.publish(*img_msg);
    
}

// Ugly initialization of static members of tracked obstacles...
int    TrackedObject::s_fade_counter_size     = 0;
double TrackedObject::s_sampling_time         = 100.0;
double TrackedObject::s_process_variance      = 0.0;
double TrackedObject::s_process_rate_variance = 0.0;
double TrackedObject::s_measurement_variance  = 0.0;