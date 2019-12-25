#ifndef _MORPHOLOGY_H_
#define _MORPHOLOGY_H_
#include "my_include.h"
#include "RasterProcessor.h"

struct Points_info {
        int index;
        int flag;
};

struct Params {
    double  grid_size;
    int Max_window_size;
    double Terrian_slop;
    double height_diff_threshold_init;
    double height_diff_threshold_max;
};

class Morphology
{
private:
    RasterProcessor *Raster_Process_obj;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> origin_cloud;
public:
    Params params;
    std::vector<cv::Point3f> ExtractSeedByPCL();
    std::tuple<std::vector<cv::Point3f>,pcl::PointIndices> My_ExtractGroundSeed();
    cv::Mat strelDisk(int Radius);
public:
    Morphology();
    Morphology(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in,Params input_param);
    ~Morphology();
};

#endif