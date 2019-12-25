#ifndef _TRIANGULATION_H_
#define _TRIANGULATION_H_
#include "my_include.h"

#define Rad2deg  180.0f/M_PI
#define Deg2Rad  M_PI/180.0f
class Triangulation
{
private:
    double Angle_threshold;
    double Distance_threshold;
    GEOM_FADE25D::Fade_2D* pDt;
    
public: 
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> origin_cloud;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> triangulation_process(std::vector<cv::Point3f> seed_points);
public:
    Triangulation();
    Triangulation(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in, double angle_threshold, double distance_threshold);
    ~Triangulation() {}
};

#endif 