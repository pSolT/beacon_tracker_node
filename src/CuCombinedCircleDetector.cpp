//
// Created by psolt on 12.03.17.
//

#include "../include/CuCombinedCircleDetector.h"

std::vector<cv::Point> CuCombinedCircleDetector::Detect(cv::Mat &image)
{
    const float dp = 1.0f;
    const float minDist = 5.0f;
    const int minRadius = 1;
    const int maxRadius = 100;
    const int cannyThreshold = 100;
    const int votesThreshold = 20;

    cv::cuda::GpuMat dImage;
    dImage.upload(image);
    cv::cuda::GpuMat dGray(dImage.rows, dImage.cols, CV_8UC1);
    inRange_gpu(dImage,colourLowerBoundary_,colourUpperBoundary_,dGray);

    cv::Mat filtered;
    dGray.download(filtered);
    imshow("filtered", filtered);
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
