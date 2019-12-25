#ifndef _TERRAINFILTER_H_
#define _TERRAINFILTER_H_
#include "my_include.h"

class TerrainFilter
{
private:
    
public:
    double size_grid;
    double height_difference_threshold;
    int Disk_Radius;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> Triangulation(int Num_points_for_computing_mean, double Num_of_std_dev_for_removing_outliers, double Angle_threshold, double Distance_threshold);
public:
    TerrainFilter();
    TerrainFilter(const char* input_file_name, double grid_size, int disk_radius, double height_difference);
    ~TerrainFilter();  
};

#endif