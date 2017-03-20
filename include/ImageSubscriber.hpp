#ifndef IMAGE_SUBSCRIBER_HPP_
#define IMAGE_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "CombinedCircleDetector.h"
#include "ColourFilterCircleDetector.h"

class ImageSubscriber
{
protected:
    ros::NodeHandle *nh;
    image_transport::ImageTransport *it;
    image_transport::Subscriber sub;
    std::string window;
    ICircleDetector * detector;

public:
    cv::Mat image;

public:
    ImageSubscriber(){}

    ImageSubscriber(ros::NodeHandle *nh, image_transport::ImageTransport *it, std::string topic, std::string window){
        this->detector =  new CombinedCircleDetector( cv::Scalar(30, 100, 75), cv::Scalar(90, 255, 255) );
        this->nh = nh;
        this->it = it;
        this->window = window;
        sub = it->subscribe(topic, 1, &ImageSubscriber::MsgInterrupt, this);
        cv::namedWindow(this->window, CV_WINDOW_NORMAL);
        cv::namedWindow("debug", CV_WINDOW_NORMAL);
    }

    ~ImageSubscriber(){
        cv::destroyWindow(this->window);
    }

    void ResizeWindow(int width, int length){
        cv::resizeWindow(this->window, width, length);
        cv::resizeWindow("debug", width, length);
    }

    void MoveWindow(int x, int y){
        cv::moveWindow(this->window, x, y);
        cv::moveWindow("debug", x+640, y);
    }

    void MsgInterrupt(const sensor_msgs::ImageConstPtr& msg){
        try{
            this->image = cv_bridge::toCvShare(msg, "bgr8")->image;
            this->detector->Detect(this->image);
            cv::imshow(this->window, this->image);
            cv::waitKey(10);
        }
        catch (cv_bridge::Exception& e){
             ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
};

#endif
