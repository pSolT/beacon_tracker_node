//
// Created by psolt on 18.02.17.
//

#ifndef BEACONDETECTIONPLAYGROUND_COLOURMASKCIRCLEDETECTOR_H
#define BEACONDETECTIONPLAYGROUND_COLOURMASKCIRCLEDETECTOR_H

#include "ICircleDetector.h"

class ColourFilterCircleDetector: public ICircleDetector {
private:
    cv::Scalar colourLowerBoundary_;
    cv::Scalar colourUpperBoundary_;

public:
    ColourFilterCircleDetector(cv::Scalar colourLowerBoundar, cv::Scalar colourUpperBoundary)
            : colourLowerBoundary_(colourLowerBoundar), colourUpperBoundary_(colourUpperBoundary) {}
    void Detect(cv::Mat& image);

};


#endif //BEACONDETECTIONPLAYGROUND_COLOURMASKCIRCLEDETECTOR_H
