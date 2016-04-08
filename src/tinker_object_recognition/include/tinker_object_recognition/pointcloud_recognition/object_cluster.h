/*
 * Created by sugar10w, 2016.2.25
 * Last edited by sugar10w, 2016.2.25
 *
 * TODO 记录用于描述物体的点云
 *
 */

#ifndef __OBJECTFINDER_OBJECT_CLUSTER_H__
#define __OBJECTFINDER_OBJECT_CLUSTER_H__


#include <pcl/visualization/pcl_visualizer.h>

#include "tinker_object_recognition/common.h"

namespace tinker {
namespace vision {

class ObjectCluster
{
public:
    /* 新建物体*/
    ObjectCluster(const PointCloudPtr& input_cloud, float leaf_size=0.003);
    /* 在viewer上绘制方框 */
    void DrawBoundingBox(pcl::visualization::PCLVisualizer& viewer, int object_number);
    inline bool isValid() { return valid_; }
    inline PointCloudPtr GetCloud() { return cloud; }
private:
    float leaf_size_;
    /* 大概的位置和颜色信息*/
    float x_min, x_max, y_min, y_max, z_min, z_max, xx, yy, zz;
    int r_avg, g_avg, b_avg;
    /* 点云*/
    PointCloudPtr cloud;
    
    bool valid_;

};

}
}

#endif //OBJECTFINDER_OBJECT_CLUSTER_H__
