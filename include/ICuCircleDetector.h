//
// Created by psolt on 18.02.17.
//

#ifndef BEACONDETECTIONPLAYGROUND_ICUCIRCLEDETECTOR_H
#define BEACONDETECTIONPLAYGROUND_ICUCIRCLEDETECTOR_H

#include "ICircleDetector.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"

class ICuCircleDetector: public ICircleDetector {
public:
    virtual ~ICuCircleDetector() {}
    virtual void Detect(cv::Mat& image) = 0;
};


#endif //CIRCLEDETECTIONTEST_ICUCIRCLEDETECTOR_H
