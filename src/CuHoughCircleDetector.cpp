//
// Created by psolt on 18.02.17.
//

#include "include/CuHoughCircleDetector.h"

std::vector<cv::Point> CuHoughCircleDetector::Detect(cv::Mat &image)
{
    const float dp = 1.0f;
    const float minDist = 5.0f;
    const int minRadius = 1;
    const int maxRadius = 100;
    const int cannyThreshold = 120;
    const int votesThreshold = 10;
    cv::Mat gray;
    cv::Mat hsv;
    cv::Mat rgb;
    cv::Mat mask;
    cv::cvtColor(image, hsv, CV_BGR2HSV);

    inRange(hsv,cv::Scalar(30, 100, 100), cv::Scalar(90, 255, 255) ,mask);
    GaussianBlur( mask, mask, cv::Size(9, 9), 2, 2 );
    cv::cuda::GpuMat dGray;

    dGray.upload(mask);
    cv::cuda::GpuMat dCircles;
    {

        cv::Ptr<cv::cuda::HoughCirclesDetector> houghCircles = cv::cuda::createHoughCirclesDetector(dp, minDist, cannyThreshold, votesThreshold, minRadius, maxRadius, 100);

        houghCircles->detect(dGray, dCircles);

    }
    std::vector<cv::Vec3f> circlesGPU;
    if(!dCircles.empty())
    {

        circlesGPU.resize(dCircles.size().width);
        dCircles.row(0).download(cv::Mat(circlesGPU).reshape(3, 1));
    }

    std::vector<cv::Point> result;

    for( size_t i = 0; i < circlesGPU.size(); i++ )
    {
        cv::Point center(cvRound(circlesGPU[i][0]), cvRound(circlesGPU[i][1]));
        result.push_back(center);
        int radius = cvRound(circlesGPU[i][2]);
        // draw the circle center
        std::cout << center.x << " " << center.y  << " " << radius << std::endl;
        circle( image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // draw the circle outline
        circle( image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );

    }

    return result;
}
