//
// Created by 郭嘉丞 on 15/9/25.
//

#ifndef OBJECTFINDER_KINECTPARAMETERS_H
#define OBJECTFINDER_KINECTPARAMETERS_H

namespace tinker
{
namespace vision
{
    extern double depthToZ[2];  //from depth data(0~1) to Z in the camera coordinate(cm)
    extern double projectionParameter[3][4];  //the projection matrix values
    extern double projectionParameter1080[3][4];
}
}

#endif //OBJECTFINDER_KINECTPARAMETERS_H
