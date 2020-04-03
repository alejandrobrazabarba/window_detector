#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>

static const std::string CORNER_DETECTION_WINDOW = "Detected corners";

class WindowsDetector 
{
public:
    WindowsDetector();
    ~WindowsDetector();
private:
    void depthCallback(const sensor_msgs::ImageConstPtr &image_ptr, const sensor_msgs::CameraInfoConstPtr &info_ptr);
    inline uint8_t mapFloatDepthImageToUInt8(float input);
    void detectWindows();
    void get3DPointCameraModel(geometry_msgs::Point &point, float &depth, int pixel_row, int pixel_col);
    void bubbleSort(std::vector<cv::Point2f>& vector_point_pair);
    void swapPairs(std::vector<cv::Point2f>& vector_point_pair, int index_1, int index_2);
    bool checkWindowSize(geometry_msgs::Vector3& windowDimensions);
    float calcDistanceBetween3DPoints(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2);
    geometry_msgs::Point calcWindowCenter(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2, 
                                          geometry_msgs::Point& point_3, geometry_msgs::Point& point_4);
    geometry_msgs::Vector3 normalizedVectorFrom2Points(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2);
    geometry_msgs::Quaternion calcWindowOrientation(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2, 
                                                    geometry_msgs::Point& point_3, geometry_msgs::Point& point_4);
    geometry_msgs::Vector3 normalizedCrossProduct(geometry_msgs::Vector3& vector_1, geometry_msgs::Vector3& vector_2);
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport image_transport_;
    image_transport::CameraSubscriber camera_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher corners_marker_pub_;
    ros::Publisher window_axis_markers_pub_;

    visualization_msgs::Marker corners_markers_msg_;
    visualization_msgs::Marker window_x_axis_marker_msg_;
    visualization_msgs::Marker window_y_axis_marker_msg_;
    visualization_msgs::Marker window_z_axis_marker_msg_;

    int image_counter_;

    image_geometry::PinholeCameraModel model_;

    cv_bridge::CvImage depth_image_8_;
    cv_bridge::CvImageConstPtr received_depth_image_opencv_ptr_;

    geometry_msgs::Point corner_upper_left_point_;
    geometry_msgs::Point corner_upper_right_point_;
    geometry_msgs::Point corner_lower_left_point_;
    geometry_msgs::Point corner_lower_right_point_;

    int maxCorners;
    cv::RNG rng;

    tf2_ros::TransformBroadcaster tf_broadcaster;
};
 
WindowsDetector::WindowsDetector():
image_transport_(nh_),
image_counter_(0),
rng(12345)
{
    image_transport::TransportHints hints("raw", ros::TransportHints(), ros::NodeHandle("~"));
    camera_sub_ = image_transport_.subscribeCamera("depth_camera/image_raw", 1, &WindowsDetector::depthCallback, this, hints);
    corners_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("corners_markers", 1);
    window_axis_markers_pub_ = nh_.advertise<visualization_msgs::Marker>("window_axis", 1);

    // Corner markers
    corners_markers_msg_.header.frame_id = "uav_1";
    corners_markers_msg_.id = 1;
    corners_markers_msg_.lifetime = ros::Duration(0);
    corners_markers_msg_.ns = "corners";
    corners_markers_msg_.type = visualization_msgs::Marker::SPHERE_LIST;
    corners_markers_msg_.action = visualization_msgs::Marker::ADD;
    corners_markers_msg_.pose.orientation.w = 1.0;
    corners_markers_msg_.scale.x = corners_markers_msg_.scale.y = corners_markers_msg_.scale.z = 0.2;
    corners_markers_msg_.color.a = 1.0;

    std_msgs::ColorRGBA color_upper_left, color_upper_right, color_lower_left, color_lower_right;
    color_upper_left.a = color_upper_right.a = color_lower_left.a = color_lower_right.a = 1.0;
    color_upper_left.r = 1.0; // Upper left corner RED
    color_upper_right.g = 1.0; // Upper right corner GREEN
    color_lower_left.r = color_lower_left.g = 1.0; // Lower left corner YELLOW
    color_lower_right.b = 1.0; // Lower right corner BLUE

    corners_markers_msg_.colors.push_back(color_upper_left);
    corners_markers_msg_.colors.push_back(color_upper_right);
    corners_markers_msg_.colors.push_back(color_lower_left);
    corners_markers_msg_.colors.push_back(color_lower_right);

    // Window axis markers initialization
    window_x_axis_marker_msg_.header.frame_id = "uav_1";
    window_x_axis_marker_msg_.ns = "window_axis";
    window_x_axis_marker_msg_.id = 0;
    window_y_axis_marker_msg_.header.frame_id = "uav_1";
    window_x_axis_marker_msg_.ns = "window_axis";
    window_y_axis_marker_msg_.id = 1;
    window_z_axis_marker_msg_.header.frame_id = "uav_1";
    window_z_axis_marker_msg_.ns = "window_axis";
    window_z_axis_marker_msg_.id = 2;
    window_x_axis_marker_msg_.action = visualization_msgs::Marker::ADD;
    window_y_axis_marker_msg_.action = visualization_msgs::Marker::ADD;
    window_z_axis_marker_msg_.action = visualization_msgs::Marker::ADD;
    window_x_axis_marker_msg_.type = visualization_msgs::Marker::ARROW;
    window_y_axis_marker_msg_.type = visualization_msgs::Marker::ARROW;
    window_z_axis_marker_msg_.type = visualization_msgs::Marker::ARROW;
    window_x_axis_marker_msg_.scale.x = window_y_axis_marker_msg_.scale.x = window_z_axis_marker_msg_.scale.x = 0.01;
    window_x_axis_marker_msg_.scale.y = window_y_axis_marker_msg_.scale.y = window_z_axis_marker_msg_.scale.y = 0.02;
    window_x_axis_marker_msg_.color.a = window_y_axis_marker_msg_.color.a = window_z_axis_marker_msg_.color.a = 1.0;
    window_x_axis_marker_msg_.color.r = 1.0;
    window_y_axis_marker_msg_.color.g = 1.0;
    window_z_axis_marker_msg_.color.b = 1.0;

    // OpenCV Windows config
    cv::namedWindow(CORNER_DETECTION_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::moveWindow(CORNER_DETECTION_WINDOW, 250, 250);
}

WindowsDetector::~WindowsDetector()
{
    cv::destroyWindow(CORNER_DETECTION_WINDOW);
}

void WindowsDetector::detectWindows()
{
    this->maxCorners = 4;

    /// Parameters for Shi-Tomasi algorithm
    std::vector<cv::Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;


    cv::goodFeaturesToTrack( this->depth_image_8_.image,
                             corners,
                             this->maxCorners,
                             qualityLevel,
                             minDistance,
                             cv::Mat(),
                             blockSize,
                             useHarrisDetector,
                             k );

    std::cout << "Number of corners detected: " << corners.size() << std::endl;

    // Show openCV window with grayscale image with circles around corners ##########################

    cv::Mat depth_image_color;
    cv::cvtColor(depth_image_8_.image, depth_image_color,CV_GRAY2BGR);

    int r = 4;
    for ( int i = 0; i < corners.size(); i++)
    {
        cv::circle( depth_image_color, corners[i], r, cv::Scalar(this->rng.uniform(0,255), this->rng.uniform(0,255), this->rng.uniform(0,255)), -1, 8, 0);
        //std::cout << "Corner " << i << " : (x,y)" << corners[i].x << "," << corners[i].y << std::endl;
    }
    cv::imshow(CORNER_DETECTION_WINDOW, depth_image_color);
    // ###################################################

    if (corners.size() == 4)
    {
        const cv::Mat & original_image_ref = this->received_depth_image_opencv_ptr_->image;
        original_image_ref.data + original_image_ref.step[0];
        int row,col;

        // Order corners so they appear in the required position of the vector
        this->bubbleSort(corners);

        // Instead of calculating 3D position of corner, calc position of a near pixel, because corner has a NaN value depth.
        // The selection of the near pixel depends on the corner position (i.e. upper left, upper right, ...).
        float *  value_original_ptr;
        // First corner: Upper Left
        row = corners[0].y - 1;
        col = corners[0].x - 1;
        value_original_ptr = (float *) (original_image_ref.data + (original_image_ref.step[0]) * row + (original_image_ref.step[1])* col ); 
        float corner_upper_left_depth = *value_original_ptr;
        this->get3DPointCameraModel(corner_upper_left_point_, corner_upper_left_depth, row, col);

        // Second corner: Lower Left
        row = corners[1].y + 1;
        col = corners[1].x - 1;
        value_original_ptr = (float *) (original_image_ref.data + (original_image_ref.step[0]) * row + (original_image_ref.step[1])* col );
        float corner_lower_left_depth = *value_original_ptr;
        this->get3DPointCameraModel(corner_lower_left_point_, corner_lower_left_depth, row, col);

        // Third corner: Upper Right
        row = corners[2].y - 1;
        col = corners[2].x + 1;
        value_original_ptr = (float *) (original_image_ref.data + (original_image_ref.step[0]) * row + (original_image_ref.step[1])* col ); 
        float corner_upper_right_depth = *value_original_ptr;
        this->get3DPointCameraModel(corner_upper_right_point_, corner_upper_right_depth, row, col);

        // Fourth corner: Lower Right
        row = corners[3].y + 1;
        col = corners[3].x + 1;
        value_original_ptr = (float *) (original_image_ref.data + (original_image_ref.step[0]) * row + (original_image_ref.step[1])* col );
        float corner_lower_right_depth = *value_original_ptr;
        this->get3DPointCameraModel(corner_lower_right_point_, corner_lower_right_depth, row, col);

        geometry_msgs::Vector3 window_dimensions;
        if (this->checkWindowSize(window_dimensions))
        {
            this->corners_markers_msg_.points.push_back(corner_upper_left_point_);
            this->corners_markers_msg_.points.push_back(corner_upper_right_point_);
            this->corners_markers_msg_.points.push_back(corner_lower_left_point_);
            this->corners_markers_msg_.points.push_back(corner_lower_right_point_);
            this->corners_markers_msg_.header.stamp = ros::Time::now();
            this->corners_marker_pub_.publish(this->corners_markers_msg_);
            this->corners_markers_msg_.points.clear();

            geometry_msgs::Point window_center = this->calcWindowCenter(corner_upper_left_point_, corner_lower_left_point_, 
                                                                        corner_upper_right_point_, corner_lower_right_point_);
            
            geometry_msgs::Quaternion window_orientation = this->calcWindowOrientation(corner_upper_left_point_, corner_lower_left_point_,
                                                                                       corner_upper_right_point_, corner_lower_right_point_);

            // Publish tf2 transformation between uav_1 frame and window frame for debugging
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "uav_1";
            transformStamped.child_frame_id = "window";
            transformStamped.transform.rotation = window_orientation;
            transformStamped.transform.translation.x = window_center.x;
            transformStamped.transform.translation.y = window_center.y;
            transformStamped.transform.translation.z = window_center.z;
            tf_broadcaster.sendTransform(transformStamped);
        }
    }
    else
    {
        std::cout << "Not enough corners detected" << std::endl;
    }
    
}

bool WindowsDetector::checkWindowSize(geometry_msgs::Vector3& window_dimensions)
{
    float distance_upper_left_upper_right = this->calcDistanceBetween3DPoints(this->corner_upper_left_point_, this->corner_upper_right_point_);
    float distance_lower_left_lower_right = this->calcDistanceBetween3DPoints(this->corner_lower_left_point_, this->corner_lower_right_point_);
    float distance_upper_left_lower_left = this->calcDistanceBetween3DPoints(this->corner_upper_left_point_, this->corner_lower_left_point_);
    float distance_upper_right_lower_right = this->calcDistanceBetween3DPoints(this->corner_upper_right_point_, this->corner_lower_right_point_);

    bool valid_distance_upper_left_upper_right = false;
    bool valid_distance_lower_left_lower_right = false;
    bool valid_distance_upper_left_lower_left = false;
    bool valid_distance_upper_right_lower_right = false;

    std::cout << "Distance between upper left and upper right corners: " << distance_upper_left_upper_right << std::endl;
    std::cout << "Distance between lower left and lower right corners: " << distance_lower_left_lower_right << std::endl;
    std::cout << "Distance between upper left and lower left corners: " << distance_upper_left_lower_left << std::endl;
    std::cout << "Distance between upper right and lower right corners: " << distance_upper_right_lower_right << std::endl;

    // Check window dimensions against a square of 2m x 2m
    if (distance_upper_left_upper_right > 1.9 && distance_upper_left_upper_right < 2.1)
    {
        valid_distance_upper_left_upper_right = true;
        window_dimensions.x = distance_upper_left_upper_right;
    }
    if (distance_lower_left_lower_right > 1.9 && distance_lower_left_lower_right < 2.1)
    {
        valid_distance_lower_left_lower_right = true;
    }
    if (distance_upper_left_lower_left > 1.9 && distance_upper_right_lower_right < 2.1)
    {
        valid_distance_upper_left_lower_left = true;
        window_dimensions.y = distance_upper_left_lower_left;
    }
    if (distance_upper_right_lower_right > 1.9 && distance_upper_right_lower_right < 2.1)
    {
        valid_distance_upper_right_lower_right = true;
    }

    // Check window dimensions against a rectangular window of 1.5m x 4m
    if (distance_upper_left_upper_right > 3.9 && distance_upper_left_upper_right < 4.1)
    {
        valid_distance_upper_left_upper_right = true;
        window_dimensions.x = distance_upper_left_upper_right;
    }
    if (distance_lower_left_lower_right > 3.9 && distance_lower_left_lower_right < 4.1)
    {
        valid_distance_lower_left_lower_right = true;
    }
    if (distance_upper_left_lower_left > 1.4 && distance_upper_right_lower_right < 1.6)
    {
        valid_distance_upper_left_lower_left = true;
        window_dimensions.y = distance_upper_left_lower_left;
    }
    if (distance_upper_right_lower_right > 1.4 && distance_upper_right_lower_right < 1.6)
    {
        valid_distance_upper_right_lower_right = true;
    }

    window_dimensions.z = 0;
    if (valid_distance_upper_left_upper_right && valid_distance_lower_left_lower_right && valid_distance_upper_left_lower_left && valid_distance_upper_right_lower_right)
    {
        std::cout << "Valid Window" << std::endl;
        return true;
    }
    else
    {
        std::cout << "Invalid Window" << std::endl;
        return false;
    }
}

geometry_msgs::Quaternion WindowsDetector::calcWindowOrientation(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2, geometry_msgs::Point& point_3, geometry_msgs::Point& point_4)
{
    tf2::Quaternion tf2_quaternion;
    /*
        point_1 -> upper left corner
        point_2 -> lower left corner
        point_3 -> upper right corner
        point_4 -> lower right corner
    */
    geometry_msgs::Quaternion orientation;

    // 1st Get x,y,z axis of the window reference frame
    // The axis orientation will be similar to how the camera frame axis are oriented.
    // z perpendicular to the plane containing the corners
    // x parallel to the vector defined by upper left corner and upper right corner
    // y parallel to the vector defined by upper left corner and lower left corner
    geometry_msgs::Vector3 x_axis_window, y_axis_window, z_axis_window;

    x_axis_window = this->normalizedVectorFrom2Points(point_1, point_3);
    y_axis_window = this->normalizedVectorFrom2Points(point_1, point_2);
    z_axis_window = this->normalizedCrossProduct(x_axis_window, y_axis_window);

    // Publish axis markers (arrows) for debugging
    geometry_msgs::Point window_center = calcWindowCenter(point_1,point_2,point_3,point_4);

    
    geometry_msgs::Point end_point_x_axis, end_point_y_axis, end_point_z_axis;

    end_point_x_axis.x = window_center.x + x_axis_window.x;
    end_point_x_axis.y = window_center.y + x_axis_window.y;
    end_point_x_axis.z = window_center.z + x_axis_window.z;

    end_point_y_axis.x = window_center.x + y_axis_window.x;
    end_point_y_axis.y = window_center.y + y_axis_window.y;
    end_point_y_axis.z = window_center.z + y_axis_window.z;

    end_point_z_axis.x = window_center.x + z_axis_window.x;
    end_point_z_axis.y = window_center.y + z_axis_window.y;
    end_point_z_axis.z = window_center.z + z_axis_window.z;

    window_x_axis_marker_msg_.points.push_back(window_center);
    window_x_axis_marker_msg_.points.push_back(end_point_x_axis);
    window_y_axis_marker_msg_.points.push_back(window_center);
    window_y_axis_marker_msg_.points.push_back(end_point_y_axis);
    window_z_axis_marker_msg_.points.push_back(window_center);
    window_z_axis_marker_msg_.points.push_back(end_point_z_axis);

    window_axis_markers_pub_.publish(window_x_axis_marker_msg_);
    window_axis_markers_pub_.publish(window_y_axis_marker_msg_);
    window_axis_markers_pub_.publish(window_z_axis_marker_msg_);
    window_x_axis_marker_msg_.points.clear();
    window_y_axis_marker_msg_.points.clear();
    window_z_axis_marker_msg_.points.clear();

    // Get M matrix which is the matrix that allows us to obtain window reference frame axis from camera referenc frame axis
    //     |a11 a12 a13|
    // M = |a21 a22 a23|
    //     |a31 a32 a33|
    // aij is the dot product of the i vector of the transformed basis and the j vector of the original basis
    // As the original basis is composed of the normal vectors î,ĵ and k (^), the dot product is simplified
    // and the resulting matrix elements are just the components of the transformed basis vectors, arranged by rows
    // The first vector in the first row, second vector in second row, ...
    double a11, a12, a13, a21, a22, a23, a31, a32, a33;

    a11 = x_axis_window.x;
    a12 = x_axis_window.y;
    a13 = x_axis_window.z;

    a21 = y_axis_window.x;
    a22 = y_axis_window.y;
    a23 = y_axis_window.z;

    a31 = z_axis_window.x;
    a32 = z_axis_window.y;
    a33 = z_axis_window.z;

    // Get M transpose (Rotation matrix that allows us to transform coordinates expressed in transformed basis to original basis)
    //      |a11 a21 a31|   |r11 r12 r13|
    // Mt = |a12 a22 a32| = |r21 r22 r23|
    //      |a13 a23 a33|   |r31 r32 r33|

    // Get yaw, pitch and roll from M transpose
    double yaw = atan2(a12, a11);
    double pitch = atan2(-a13, sqrt(a23*a23+a33*a33));
    double roll = atan2(a23, a33);

    // Get quaternion from roll,pitch and yaw
    tf2_quaternion.setRPY(roll, pitch, yaw);
    tf2::convert(tf2_quaternion, orientation);
    return orientation;
}

geometry_msgs::Vector3 WindowsDetector::normalizedVectorFrom2Points(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2)
{
    geometry_msgs::Vector3 vector;
    vector.x = point_2.x - point_1.x;
    vector.y = point_2.y - point_1.y;
    vector.z = point_2.z - point_1.z;

    double vector_norm = sqrt(pow(vector.x,2) + pow(vector.y,2) + pow(vector.z,2));
    vector.x = vector.x/vector_norm;
    vector.y = vector.y/vector_norm;
    vector.z = vector.z/vector_norm;
    return vector;
}

geometry_msgs::Vector3 WindowsDetector::normalizedCrossProduct(geometry_msgs::Vector3& vector_1, geometry_msgs::Vector3& vector_2)
{
    geometry_msgs::Vector3 vector;
    vector.x = vector_1.y * vector_2.z - vector_1.z * vector_2.y;
    vector.y = - (vector_1.x * vector_2.z - vector_1.z * vector_2.x);
    vector.z = vector_1.x * vector_2.y - vector_1.y * vector_2.x;

    double vector_norm = sqrt(pow(vector.x,2) + pow(vector.y,2) + pow(vector.z,2));
    vector.x = vector.x/vector_norm;
    vector.y = vector.y/vector_norm;
    vector.z = vector.z/vector_norm;
    return vector;
}

geometry_msgs::Point WindowsDetector::calcWindowCenter(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2, geometry_msgs::Point& point_3, geometry_msgs::Point& point_4)
{
    geometry_msgs::Point center;

    center.x = (point_1.x + point_2.x + point_3.x + point_4.x)/4;
    center.y = (point_1.y + point_2.y + point_3.y + point_4.y)/4;
    center.z = (point_1.z + point_2.z + point_3.z + point_4.z)/4;

    return center;
}

float WindowsDetector::calcDistanceBetween3DPoints(geometry_msgs::Point& point_1, geometry_msgs::Point& point_2)
{
    return sqrt( (point_1.x-point_2.x)*(point_1.x-point_2.x) + (point_1.y-point_2.y)*(point_1.y-point_2.y) + (point_1.z-point_2.z) );
}

void WindowsDetector::get3DPointCameraModel(geometry_msgs::Point &point, float &depth, int pixel_row, int pixel_col)
{
    //std::cout << "Depth: " << depth << std::endl;
    //std::cout << "Pixel column: " << pixel_col << std::endl;
    //std::cout << "Pixel row: " << pixel_row << std::endl;
    point.x = (pixel_col - model_.cx())/model_.fx() * depth;
    point.y = (pixel_row - model_.cy())/model_.fy() * depth;
    point.z = depth;
}

void WindowsDetector::bubbleSort(std::vector<cv::Point2f>& vector_point_pair)
{
    int n = vector_point_pair.size();
    int i,j;
    bool swapped;

    // First order using x coordinate
    for (i=0; i < n-1; i++)
    {
        swapped = false;
        for (j=0; j < n-i-1; j++)
        {
            if (vector_point_pair[j].x > vector_point_pair[j+1].x)
            {
                swapPairs(vector_point_pair, j, j+1);
                swapped = true;
            }
        }
        if (swapped == false)
            break;
    }
    // Then order using y coordinate (half vector first, then the second half)
    for (i=0; i < n/2-1; i++)
    {
        swapped = false;
        for (j=0; j < n/2-i-1; j++)
        {
            if (vector_point_pair[j].y > vector_point_pair[j+1].y)
            {
                swapPairs(vector_point_pair, j, j+1);
                swapped = true;
            }
        }
        if (swapped == false)
            break;
    }
    for (i=0; i < n/2-1; i++)
    {
        swapped = false;
        for (j=n/2; j < n-i-1; j++)
        {
            if (vector_point_pair[j].y > vector_point_pair[j+1].y)
            {
                swapPairs(vector_point_pair, j, j+1);
                swapped = true;
            }
        }
        if (swapped == false)
            break;
    }    
}

void WindowsDetector::swapPairs(std::vector<cv::Point2f>& vector_point_pair, int index_1, int index_2)
{
    cv::Point2f temp;
    temp = vector_point_pair.at(index_1);
    vector_point_pair.at(index_1) = vector_point_pair.at(index_2);
    vector_point_pair.at(index_2) = temp;
}

#define DEPTH_MAX_VALUE 6
inline uint8_t WindowsDetector::mapFloatDepthImageToUInt8(float input)
{
    return floor((input/DEPTH_MAX_VALUE)*255);
}

void WindowsDetector::depthCallback(const sensor_msgs::ImageConstPtr &image_ptr, const sensor_msgs::CameraInfoConstPtr &info_ptr)
{
    // For each 4 messages process 1 and discard the other 3
    if (image_counter_ == 0 || image_counter_ == 4)
    {
        image_counter_ = 0;
        // Extract camera parameters from 
        model_.fromCameraInfo(info_ptr);
    
        /* Some depth cameras use TYPE_32FC1 encoding. They encode depth measured in meters as float
           Other cameras use MONO16 encoding. In that case, they encode depth measured in mm as uint16 */
        try
        {
            //std::cout << "received image encoding " << image_ptr->encoding << std::endl;
            received_depth_image_opencv_ptr_ = cv_bridge::toCvShare(image_ptr, sensor_msgs::image_encodings::TYPE_32FC1);
            //std::cout << "opencv bridge image encoding " << received_depth_image_opencv_ptr_->encoding << std::endl;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        int image_height = image_ptr->height;
        int image_width = image_ptr->width;


        // Received image must be converted to 8bit grayscale because it's a requirement of the used OpenCV function
        depth_image_8_.image = cv::Mat(cv::Size(image_width, image_height), CV_8UC1);
        depth_image_8_.header.frame_id = image_ptr->header.frame_id;
        depth_image_8_.header.stamp = image_ptr->header.stamp;

        for(int i1 = 0; i1 < received_depth_image_opencv_ptr_->image.rows; i1++)
        {
            for(int j1 = 0; j1 < received_depth_image_opencv_ptr_->image.cols; j1++)
            {
                const cv::Mat & original_image_ref = received_depth_image_opencv_ptr_->image;
                const cv::Mat & converted_image_ref = depth_image_8_.image;
                float *  value_original_ptr = (float *) (original_image_ref.data + original_image_ref.step[0]*i1 + original_image_ref.step[1]*j1); 
                uint8_t * value_converted_ptr = (uint8_t *) (converted_image_ref.data + converted_image_ref.step[0] * i1 + converted_image_ref.step[1] * j1);

                uint32_t integer_aux;
                std::memcpy(&integer_aux, value_original_ptr, sizeof(float));
                /* 0x7FC00000 is the 32 bit float memory representation of a NaN value, used by depth cameras to signal that
                   it wasn't able to measure the depth associated with some pixel
                */
                if (integer_aux == 0X7FC00000)
                {
                    *value_converted_ptr = 0;
                }
                else
                {
                    // Convert float value to 8bit value using linear function
                    *value_converted_ptr = this->mapFloatDepthImageToUInt8(*value_original_ptr);
                } 
            }
        }
        std::cout << std::endl;

        this->detectWindows();
    }
    image_counter_++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "windows_detector");
    WindowsDetector windows_detector;
    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        cv::waitKey(1);
        rate.sleep();
    }

    return 0;
}