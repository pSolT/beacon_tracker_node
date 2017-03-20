#ifndef BEACONDETECTIONPLAYGROUND_COMBINEDCIRCLEDETECTOR_H
#define BEACONDETECTIONPLAYGROUND_COMBINEDCIRCLEDETECTOR_H

#include "ICircleDetector.h"

class CombinedCircleDetector : public ICircleDetector {
private:
    cv::Scalar colourLowerBoundary_;
    cv::Scalar colourUpperBoundary_;

public:
    CombinedCircleDetector(cv::Scalar colourLowerBoundar, cv::Scalar colourUpperBoundary)
    : colourLowerBoundary_(colourLowerBoundar), colourUpperBoundary_(colourUpperBoundary) {}
    void Detect(cv::Mat& image);

};

#endif //BEACONDETECTIONPLAYGROUND_COMBINEDCIRCLEDETECTOR_H
