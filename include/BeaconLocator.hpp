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
#include <geometry_msgs/PoseStamped.h>

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
    ros::Publisher beacon1LocationPublisher;
    ros::Publisher beacon2LocationPublisher;

    std::thread * worker;
    std::mutex m;
    std::condition_variable changed;

    struct Event
    {
      sensor_msgs::PointCloud2 pointCloud;
      bool pointCloundInitialized = false;
      sensor_msgs::Image image;
      bool imageInitialized = false;
      sensor_msgs::Image depth;
      bool depthInitialized = false;
    };

    std::map<uint32_t, Event> events;

public:
    BeaconLocator() = default;
    BeaconLocator
    (
       ros::NodeHandle *nh,
       std::string imageTopic,
       std::string pointCloudTopic,
       std::string depthTopic,
       std::string beacon1LocationTopic,
       std::string beacon2LocationTopic
     )
    {
        this->detector =  new CombinedCircleDetector( cv::Scalar(30, 100, 75), cv::Scalar(90, 255, 255) );
        this->nh = nh;
        this->pointCloudSubscriber = nh->subscribe(pointCloudTopic, 1, &BeaconLocator::PointCloudHandler, this);
        this->imageSubscriber = nh->subscribe(imageTopic, 1, &BeaconLocator::ImageHandler, this);
        this->depthSubscriber = nh->subscribe(depthTopic, 1, &BeaconLocator::DepthHandler, this);
        this->beacon1LocationPublisher = nh->advertise<geometry_msgs::PoseStamped>(beacon1LocationTopic, 100);
        this->beacon2LocationPublisher = nh->advertise<geometry_msgs::PoseStamped>(beacon2LocationTopic, 100);

        worker = new std::thread(&BeaconLocator::ProcessEvents, this);
    }

    ~BeaconLocator() {}

private:
    void PointCloudHandler(const sensor_msgs::PointCloud2& pcl)
    {
        std::unique_lock<std::mutex> lock(m);
        events[pcl.header.seq].pointCloud = pcl;
        events[pcl.header.seq].pointCloundInitialized = true;
        changed.notify_one();
    }

    void DepthHandler(const sensor_msgs::Image& image)
    {
        std::unique_lock<std::mutex> lock(m);
        events[image.header.seq].depth = image;
        events[image.header.seq].depthInitialized = true;
        changed.notify_one();
    }

    void ImageHandler(const sensor_msgs::Image& image)
    {
        std::unique_lock<std::mutex> lock(m);
        events[image.header.seq].image = image;
        events[image.header.seq].imageInitialized = true;
        changed.notify_one();
    }

    void ProcessEvents()
    {
        while(1)
        {
              std::unique_lock<std::mutex> lock(m);
              changed.wait(lock);
              for (auto it = events.begin(); it != events.end();)
              {
                  if((*it).second.pointCloundInitialized and (*it).second.imageInitialized and (*it).second.depthInitialized )
                  {
                      ProcessEvent((*it).second);
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

    void ProcessEvent
    (
        const Event & event
    )
    {
        try
        {
            auto cvImage = cv_bridge::toCvCopy(event.image);
            std::vector<cv::Point> beaconCentres = detector->Detect(cvImage->image);
            std::cout << beaconCentres.size() << std::endl;
            for(const auto& point: beaconCentres)
            {
              uint32_t offset = point.y * event.pointCloud.row_step + point.x * event.pointCloud.point_step;

              int arrayPosX = offset + event.pointCloud.fields[0].offset; // X has an offset of 0
              int arrayPosY = offset + event.pointCloud.fields[1].offset; // Y has an offset of 4
              int arrayPosZ = offset + event.pointCloud.fields[2].offset; // Z has an offset of 8

              float x = 0.0;
              float y = 0.0;
              float z = 0.0;

              memcpy(&x, &event.pointCloud.data[arrayPosX], sizeof(float));
              memcpy(&y, &event.pointCloud.data[arrayPosY], sizeof(float));
              memcpy(&z, &event.pointCloud.data[arrayPosZ], sizeof(float));

              geometry_msgs::PoseStamped beaconPosition;
              beaconPosition.pose.position.x = x;
              beaconPosition.pose.position.y = y;
              beaconPosition.pose.position.z = z;
              this->beacon1LocationPublisher.publish(beaconPosition);
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

};

#endif
