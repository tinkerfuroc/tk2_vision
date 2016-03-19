#ifndef FUSION_H

#define FUSION_H

#include "tinker_object_recognition/CMT/common.h"

namespace cmt {

class Fusion
{
public:
    void preferFirst(const vector<Point2f> & firstPoints, const vector<int> & firstClasses,
           const vector<Point2f> & secondPoints, const vector<int> & secondClasses,
           vector<Point2f> & fusedPoints, vector<int> & fusedClasses);
};

} /* namespace CMT */

#endif /* end of include guard: FUSION_H */
