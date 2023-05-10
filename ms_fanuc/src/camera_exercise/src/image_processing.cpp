#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter() : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/camera/output_video", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Convert the image to HSV color space, filter, convert back to BGR
        GaussianBlur(cv_ptr->image, cv_ptr->image, Size(5, 5), 0, 0);
        cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2HSV);
        inRange(cv_ptr->image, Scalar(40, 0, 0), Scalar(100, 255, 255), cv_ptr->image);

        // Find contours and mark them in red
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(cv_ptr->image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        cvtColor(cv_ptr->image, cv_ptr->image, CV_GRAY2BGR);
        for (int i = 0; i < contours.size(); i++)
        {
            Scalar color = Scalar(0, 0, 255);
            drawContours(cv_ptr->image, contours, i, color, 2, 8, hierarchy, 0, Point());
        }

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}