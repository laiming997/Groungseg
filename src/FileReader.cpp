#include "FileReader.h"
using namespace std;
FileReader::FileReader():
    file_path(NULL)
{
    
}

FileReader::FileReader(const char* input_file_name):
    file_path(input_file_name)
{
    cout<< "the file path:"<< file_path << endl;
}

FileReader::~FileReader()
{

}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> FileReader::txtReader()
{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ifstream fin(file_path,ios::in);
    char line[500];
    std::string x,y,z;
    while (fin.getline(line,sizeof(line)))
    {
        stringstream words(line);
        words >> x;
        words >> y;
        words >> z;
        cloud->push_back(pcl::PointXYZ(atof(x.c_str()), atof(y.c_str()), atof(z.c_str())));
    }
    printf("the file %s has read %zd points !\n",file_path,cloud->size());
    return cloud;
}

std::tuple<pcl::PointCloud<pcl::PointXYZL> , pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointXYZ>> FileReader::RetxtReader()
{
    pcl::PointCloud<pcl::PointXYZL> cloud;
    pcl::PointCloud<pcl::PointXYZ> Ground_cloud;
    pcl::PointCloud<pcl::PointXYZ> Object_cloud;
    ifstream fin(file_path,ios::in);
    char line[500];
    std::string x,y,z,label;
    while (fin.getline(line,sizeof(line)))
    {
        stringstream words(line);
        words >> x;
        words >> y;
        words >> z;
        words >> label; //
        pcl::PointXYZL points;
        points.x = atof(x.c_str());
        points.y = atof(y.c_str());
        points.z = atof(z.c_str());
        points.label = atoi(label.c_str());
        cloud.push_back(points);
    }
    for (int i = 0; i < cloud.size(); i++)
    {
        if(cloud.points[i].label == 0)
        {//ground point
            Ground_cloud.push_back(pcl::PointXYZ(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z));
        }
        else if(cloud.points[i].label == 1)
        {//non ground point
            Object_cloud.push_back(pcl::PointXYZ(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z));
        }
    }
    printf("the point cloud has %zd ground points!\n",Ground_cloud.size());
    printf("the point cloud has %zd non ground points!\n",Object_cloud.size());
    return std::make_tuple(cloud,Ground_cloud,Object_cloud);
}