#include "read_image.h"

using namespace std;

//#define PAINT_OUTPUT
#define PUBLISH_DEBUG_OUTPUT

static const uint32_t MY_ROS_QUEUE_SIZE = 1;



cLaneDetectionFu::cLaneDetectionFu(ros::NodeHandle nh): nh_(nh), priv_nh_("~")
{
    std::string node_name = ros::this_node::getName();

    ROS_INFO("Node name: %s",node_name.c_str());

    priv_nh_.param<std::string>(node_name+"/camera_name", camera_name, "/usb_cam/image_raw"); 
    priv_nh_.param<int>(node_name+"/cam_w", cam_w, 640);
    priv_nh_.param<int>(node_name+"/cam_h", cam_h, 480);
    read_images_ = nh.subscribe(nh_.resolveName(camera_name), MY_ROS_QUEUE_SIZE, &cLaneDetectionFu::ProcessInput,this);

    image_transport::ImageTransport image_transport(nh);
    image_publisher = image_transport.advertiseCamera("/lane_model/lane_model_image", MY_ROS_QUEUE_SIZE);

}

cLaneDetectionFu::~cLaneDetectionFu()
{
}

void cLaneDetectionFu::ProcessInput(const sensor_msgs::Image::ConstPtr& msg)
{
    //use ROS image_proc or opencv instead of ip mapper?

    cv_bridge::CvImagePtr cv_ptr;
    //convert to cvImagePtr and make it black and white (8bit)
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    // matrix of image
    cv::Mat image = cv_ptr->image.clone();
    //cut 
    cv::Mat cut_image = image(cv::Rect(0,cam_h/2,cam_w,cam_h/2));
    
    #ifdef PAINT_OUTPUT
        cv::imshow("IPmapped image", cut_image);
        cv::waitKey(1);
    #endif

    /// read a point of an image ...
    for (int i = 0; i < cut_image.rows; ++i) {
        for (int j = 0; j < cut_image.cols; ++j) {
            int read_point = 0;                
            //cv::Mat uses ROW-major system -> .at(y,x)
            read_point = cut_image.at<uint8_t>(j, i);
        }
    }
}

void cLaneDetectionFu::pubRGBImageMsg(cv::Mat& rgb_mat)
{
    sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

    ros::Time head_time_stamp=ros::Time::now();
    std::string rgb_frame_id = "_rgb_optical_frame";
    sensor_msgs::CameraInfoPtr rgb_camera_info;
    unsigned int head_sequence_id = 0;


    rgb_img->header.seq = head_sequence_id;
    rgb_img->header.stamp = head_time_stamp;
    rgb_img->header.frame_id = rgb_frame_id;

    rgb_img->width = rgb_mat.cols;
    rgb_img->height = rgb_mat.rows;

    rgb_img->encoding = sensor_msgs::image_encodings::BGR8;
    rgb_img->is_bigendian = 0;

    int step = sizeof(unsigned char) * 3 * rgb_img->width;
    int size = step * rgb_img->height;
    rgb_img->step = step;
    rgb_img->data.resize(size);
    memcpy(&(rgb_img->data[0]), rgb_mat.data, size);

    rgb_camera_info->header.frame_id = rgb_frame_id;
    rgb_camera_info->header.stamp = head_time_stamp;
    rgb_camera_info->header.seq = head_sequence_id;

    image_publisher.publish(rgb_img, rgb_camera_info);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cLaneDetectionFu");
    ros::NodeHandle nh;
    
    cLaneDetectionFu node=cLaneDetectionFu(nh);

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
