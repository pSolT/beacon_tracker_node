//
// Created by psolt on 18.02.17.
//

#include "../include/ColourFilterCircleDetector.h"

void ColourFilterCircleDetector::Detect(cv::Mat &image)
{
    cv::Mat hsv;
    cv::Mat mask;
    cv::Mat imgH;
    image.convertTo(imgH, -1, 2, 0);

    cv::cvtColor(imgH, hsv, CV_BGR2HSV);
    std::vector<std::vector<cv::Point> > contours;

    inRange(hsv,colourLowerBoundary_,colourUpperBoundary_,mask);
    erode(mask, mask, cv::Mat(), cv::Point{0, 0}, 2);
    dilate(mask, mask, cv::Mat(), cv::Point{0, 0}, 2);
    cv::imshow("debug", imgH);
    findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point{0, 0} );
    std::cout << contours.size() << std::endl;
    if(contours.size() > 0)
    {
        auto contour = std::max_element
                (
                        contours.begin(),
                        contours.end(),
                        [&](const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2)
                        {
                            return contourArea(contour1, false) < contourArea(contour2, false);
                        }
                );

        cv::Point2f center;
        float radius;
        minEnclosingCircle(*contour, center, radius);

        circle( image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // draw the circle outline
        circle( image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

}
