/*
  * Created by sugar10w, 2016.2.25
  * Last edited by sugar10w, 2016.2.25
  *
  * TODO 记录用于描述物体的点云
  *
  */
 
 #include "tinker_object_recognition/pointcloud_recognition/object_cluster.h"
 
 #include <sstream>
 
 namespace tinker {
 namespace vision {
 
 /* 新建物体 */
 ObjectCluster::ObjectCluster(const PointCloudPtr& input_cloud, float leaf_size)
     : cloud(input_cloud), leaf_size_(leaf_size)
 {
     int n = cloud->width * cloud->height;
     int r_sum = 0, g_sum = 0, b_sum = 0, cnt = 0;
 
     valid_ = true;

     x_min = y_min = z_min = 10000;
     x_max = y_max = z_max = -10000;
 
     for (int i=0; i<n; ++i)
     {
         PointT point = cloud->points[i];
         if (!(point.x||point.y||point.z)) continue;
         ++cnt;

         if (point.x<x_min) x_min = point.x;
         if (point.x>x_max) x_max = point.x;
         if (point.y<y_min) y_min = point.y;
         if (point.y>y_max) y_max = point.y;
         if (point.z<z_min) z_min = point.z;
         if (point.z>z_max) z_max = point.z;

         if (point.z>2.00)
         {
             valid_= false;
             break;
         }

         r_sum += point.r;
         g_sum += point.g;
         b_sum += point.b;
     }
     
     if (valid_)
     {
         r_avg = r_sum / cnt; if (r_avg>255) r_avg = 255;
         g_avg = g_sum / cnt; if (g_avg>255) g_avg = 255;
         b_avg = b_sum / cnt; if (b_avg>255) b_avg = 255;
     }

     //r_avg = g_avg = b_avg = 255;
     xx = x_max - x_min;
     yy = y_max - y_min;
     zz = z_max - z_min;
    
     /* 确认是否为目标的条件 */
     valid_ = 
         valid_ 
         && xx < 0.50 
         && yy < 0.50
         && yy*xx < cnt*leaf_size_*leaf_size_*3
         && 6 * (x_max - x_min) > (y_max - y_min)
         && 6 * (y_max - y_min) > (x_max - x_min);
     /*if (leaf_size_!=0) valid_ = 
         valid_ ;
     else
         valid_ && zz*yy*xx < cnt*0.005*0.005*0.005*3;
     */    
 }
 
 /* 在viewer上绘制方框 */
 void ObjectCluster::DrawBoundingBox(pcl::visualization::PCLVisualizer& viewer, int object_number)
 {
     if (!valid_) return;

     if (xx<0.05 || yy<0.05 || zz<0.05) return;

     std::stringstream box_name;
     box_name << "object_box_" << object_number;
     viewer.removeShape(box_name.str());
 
     viewer.addCube(
             x_min, x_max, y_min, y_max, z_min, z_max,
             (double)r_avg/256, (double)g_avg/256, (double)b_avg/256,
             box_name.str());
     viewer.setShapeRenderingProperties(
             pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2,
             box_name.str());
 
 }
 
 }
 }
