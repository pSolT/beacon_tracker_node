#ifndef BEACON_LOCATOR_HPP_
#define BEACON_LOCATOR_HPP_

#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>

#include "CombinedCircleDetector.h"
#include "ColourFilterCircleDetector.h"

class BeaconLocator
{
protected:
    ros::NodeHandle *nh;
    ICircleDetector * detector;
    std::string imageTopic_;
    std::string pointCloudTopic_;
    ros::Subscriber imageSubscriber;
    ros::Subscriber pointCloudSubscriber;
    ros::Subscriber depthSubscriber;
    std::thread * worker;
    std::mutex m;
    std::condition_variable changed;

    struct FullData
    {
      const sensor_msgs::PointCloud2 * pc = nullptr;
      const sensor_msgs::Image * image = nullptr;
      const sensor_msgs::Image * depth = nullptr;
    };

    std::map<uint32_t, FullData> events;

public:
    BeaconLocator() = default;
    BeaconLocator(ros::NodeHandle *nh, std::string imageTopic, std::string pointCloudTopic, std::string depthTopic)
    {
        this->detector =  new CombinedCircleDetector( cv::Scalar(30, 100, 75), cv::Scalar(90, 255, 255) );
        this->nh = nh;
        this->pointCloudSubscriber = nh->subscribe(pointCloudTopic, 1, &BeaconLocator::PointCloudHandler, this);
        this->imageSubscriber = nh->subscribe(imageTopic, 1, &BeaconLocator::ImageHandler, this);
        this->depthSubscriber = nh->subscribe(depthTopic, 1, &BeaconLocator::DepthHandler, this);
        worker = new std::thread(&BeaconLocator::doWork, this);
        cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
    }

    ~BeaconLocator() {}

    void PointCloudHandler(const sensor_msgs::PointCloud2& pcl)
    {
        std::unique_lock<std::mutex> lock(m);
        events[pcl.header.seq].pc = &pcl;
        changed.notify_one();
    }

    void DepthHandler(const sensor_msgs::Image& image)
    {
        std::unique_lock<std::mutex> lock(m);
        events[image.header.seq].depth = &image;
        changed.notify_one();
    }

    void ImageHandler(const sensor_msgs::Image& image)
    {
        std::unique_lock<std::mutex> lock(m);
        events[image.header.seq].image = &image;
        changed.notify_one();
    }

    void doWork()
    {
        while(1)
        {
              std::unique_lock<std::mutex> lock(m);
              changed.wait(lock);
              for (auto it = events.begin(); it != events.end();)
              {
                  if((*it).second.pc and (*it).second.image and (*it).second.depth )
                  {
                      std::cout << (*it).first << " done!" << std::endl;
                      events.erase(it++);
                  }
                  else
                  {
                    ++it;
                  }

              }
              lock.unlock();
        }
    }

};

#endif
