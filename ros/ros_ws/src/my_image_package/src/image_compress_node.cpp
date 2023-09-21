#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
        cv::Mat image = cv_ptr->image;

        std::vector<uchar> buffer;
        cv::imencode(".jpg", image, buffer);

        sensor_msgs::CompressedImage compressed_msg;
        compressed_msg.header = msg->header;
        compressed_msg.format = "jpeg";
        compressed_msg.data = buffer;

        pub.publish(compressed_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_compress_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("/ur_front_left_color_image_raw", 1, imageCallback);
    pub = nh.advertise<sensor_msgs::CompressedImage>("/ur_front_left_color_image_compressed", 1);

    ros::spin();
    return 0;
}