#ifndef _RASTERPROCESSOR_H_
#define _RASTERPROCESSOR_H_
#include "my_include.h"
#define MAX_BOLCK_POINTS_NUM  100000

#define SQUARE_DIST(x1, y1, x2, y2) \
    (((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2)))
    
class RasterProcessor
{
private:
    double grid_size;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> origin_cloud;
public:
    double pos_x_min,pos_x_max,pos_y_min,pos_y_max;
    int points_num;
    int rows,cols;
    std::tuple<pcl::PointXYZL**, cv::Mat , int**> RasterProcess();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ConvexHull();
public:
    RasterProcessor();
    RasterProcessor(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in,double size_grid);
    ~RasterProcessor();
};

#endif