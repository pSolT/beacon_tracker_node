//
// Created by psolt on 12.03.17.
//

#ifndef BEACONDETECTIONPLAYGROUND_CUCOMBINEDCIRCLEDETECTOR_H
#define BEACONDETECTIONPLAYGROUND_CUCOMBINEDCIRCLEDETECTOR_H

#include "ICuCircleDetector.h"
#include "gpuHelper.cuh"

class CuCombinedCircleDetector : public ICuCircleDetector {
private:
    cv::Scalar colourLowerBoundary_;
    cv::Scalar colourUpperBoundary_;

public:
    ~CuCombinedCircleDetector() {}
    CuCombinedCircleDetector(cv::Scalar colourLowerBoundar, cv::Scalar colourUpperBoundary)
    : colourLowerBoundary_(colourLowerBoundar), colourUpperBoundary_(colourUpperBoundary) {}
    std::vector<cv::Point> Detect(cv::Mat& image);

};


#endif //BEACONDETECTIONPLAYGROUND_CUCOMBINEDCIRCLEDETECTOR_H
