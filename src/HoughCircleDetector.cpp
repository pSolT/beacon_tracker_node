//
// Created by psolt on 18.02.17.
//

#include "../include/HoughCircleDetector.h"

std::vector<cv::Point> HoughCircleDetector::Detect(cv::Mat &image)
{
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    GaussianBlur( gray, gray, cv::Size(9, 9), 2, 2 );
    std::vector<cv::Vec3f> circles;
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows/4, 200, 100 );

    std::vector<cv::Point> result;

    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        result.push_back(center);
        int radius = cvRound(circles[i][2]);
        // draw the circle center
        circle( image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // draw the circle outline
        circle( image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

    return result;
}
