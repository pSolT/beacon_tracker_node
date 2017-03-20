//
// Created by psolt on 18.02.17.
//

#ifndef BEACONDETECTIONPLAYGROUND_HOUGHCIRCLEDETECTOR_H
#define BEACONDETECTIONPLAYGROUND_HOUGHCIRCLEDETECTOR_H

#include "ICircleDetector.h"

class HoughCircleDetector: public ICircleDetector  {

public:
    std::vector<cv::Point> Detect(cv::Mat& image);

};


#endif //BEACONDETECTIONPLAYGROUND_HOUGHCIRCLEDETECTOR_H
