//
// Created by psolt on 12.03.17.
//

#include "../include/CombinedCircleDetector.h"

void CombinedCircleDetector::Detect(cv::Mat &image)
{
    const float dp = 1.0f;
    const float minDist = 100.0f;
    const int minRadius = 10;
    const int maxRadius = 100;
    const int cannyThreshold = 100;
    const int votesThreshold = 10;

    cv::Mat hsv;
    cv::Mat mask;


    cv::cvtColor(image, hsv, CV_BGR2HSV);
    std::vector<std::vector<cv::Point> > contours;

    inRange(hsv,colourLowerBoundary_,colourUpperBoundary_,mask);
    cv::imshow("debug", mask);
    std::vector<cv::Vec3f> circles;

    HoughCircles(mask, circles, CV_HOUGH_GRADIENT, dp, minDist, cannyThreshold, votesThreshold, minRadius, maxRadius);
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // draw the circle center
        circle( image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // draw the circle outline
        circle( image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }


}
