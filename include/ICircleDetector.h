//
// Created by psolt on 18.02.17.
//

#ifndef BEACONDETECTIONPLAYGROUND_ICIRCLEDETECTOR_H
#define BEACONDETECTIONPLAYGROUND_ICIRCLEDETECTOR_H

#include <opencv2/opencv.hpp>

class ICircleDetector {

public:
    virtual ~ICircleDetector() {}
    virtual void Detect(cv::Mat& image) = 0;

};


#endif //BEACONDETECTIONPLAYGROUND_ICIRCLEDETECTOR_H
