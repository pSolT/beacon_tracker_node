//
// Created by psolt on 18.02.17.
//

#ifndef BEACONDETECTIONPLAYGROUND_CUHOUGHCIRCLEDETECTOR_H
#define BEACONDETECTIONPLAYGROUND_CUHOUGHCIRCLEDETECTOR_H

#include "ICuCircleDetector.h"


class CuHoughCircleDetector : public ICuCircleDetector {

public:
    void Detect(cv::Mat& image) override;
};


#endif //BEACONDETECTIONPLAYGROUND_CUHOUGHCIRCLEDETECTOR_H
