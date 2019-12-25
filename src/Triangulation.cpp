#include "Triangulation.h"


Triangulation::Triangulation():
    Angle_threshold(0),
    Distance_threshold(0)
{

}

Triangulation::Triangulation(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in, double angle_threshold, double distance_threshold):
    origin_cloud(cloud_in),
    Angle_threshold(angle_threshold),
    Distance_threshold(distance_threshold)
{
    printf("TIN Iteration(angle threshold = %f, distance threshold = %f)...\n", Angle_threshold, Distance_threshold);
    printf("the size of the input origin points: %zd\n", origin_cloud->size());
}


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> Triangulation::triangulation_process(std::vector<cv::Point3f> seed_points)
{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<GEOM_FADE25D::Point2> vInputPoints;
    for (int i = 0; i < seed_points.size(); i++)
    {
        ground_cloud->push_back(pcl::PointXYZ(seed_points.at(i).x, seed_points.at(i).y, seed_points.at(i).z));
        vInputPoints.push_back(GEOM_FADE25D::Point2(seed_points.at(i).x, seed_points.at(i).y, seed_points.at(i).z));
    }
    printf("the size of the input seed points: %zd\n", seed_points.size());
    GEOM_FADE25D::Fade_2D* pDt = new GEOM_FADE25D::Fade_2D;
    pDt->insert(vInputPoints);
    GEOM_FADE25D::Triangle2 *temp_TR;
    GEOM_FADE25D::Vector2   temp_norm;
    GEOM_FADE25D::Point2    temp_pointO;
    
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = origin_cloud->points.begin(); it != origin_cloud->points.end(); it++)
    {
        GEOM_FADE25D::Point2 P(it->x, it->y , it->z);
        temp_TR = pDt->locate(P);//定位到包含p点的三角形
        if( temp_TR != NULL)
        {
            temp_norm = temp_TR->getNormalVector();//获得三角形平面的归一化法向量
            temp_pointO = temp_TR->getBarycenter();//获得三角形的重心点O
            /*Point projected onto the plane of the triangle*/
            GEOM_FADE25D::Vector2 temp_PO(P - temp_pointO);//得到向量PO
            double dist = (temp_PO.x() * temp_norm.x()) + (temp_PO.y() * temp_norm.y()) + (temp_PO.z() * temp_norm.z());//将向量PO向法向量投影，得到P到三角形的垂距
            GEOM_FADE25D::Point2 proj_P = GEOM_FADE25D::Point2(P - (dist*temp_norm));//将p点投影到三角形平面内
            
            /*Compute the normal distance to the surface of the triangle*/
            double dis_N = sqrt(pow(proj_P.x() - P.x(), 2) + pow(proj_P.y() - P.y(), 2) + pow(proj_P.z() - P.z(), 2));
            /*Compute the angles to the three vertices of the triangle*/
            double dis_P0 = sqrt(pow(P.x() -temp_TR->getCorner(0)->x(), 2) + pow(P.y() - temp_TR->getCorner(0)->y(), 2) + pow(P.z() - temp_TR->getCorner(0)->z(), 2));
            double dis_P1 = sqrt(pow(P.x() -temp_TR->getCorner(1)->x(), 2) + pow(P.y() - temp_TR->getCorner(1)->y(), 2) + pow(P.z() - temp_TR->getCorner(1)->z(), 2));
            double dis_P2 = sqrt(pow(P.x() -temp_TR->getCorner(2)->x(), 2) + pow(P.y() - temp_TR->getCorner(2)->y(), 2) + pow(P.z() - temp_TR->getCorner(2)->z(), 2));
            
            /*Convert the angles to radiuans*/
            double alpha = asin(dis_N / dis_P0) * Rad2deg;//弧度变角度
            double beta  = asin(dis_N / dis_P1) * Rad2deg;
            double gamma = asin(dis_N / dis_P2) * Rad2deg;
            
            if(alpha < Angle_threshold && beta < Angle_threshold && gamma < Angle_threshold && dis_N < Distance_threshold)
            {
                pDt->insert(GEOM_FADE25D::Point2(it->x, it->y , it->z));
                ground_cloud -> push_back(pcl::PointXYZ(it->x, it->y , it->z));
            }
        }
    }
    printf("the size of the ground points after TIN Iteration: %zd\n", ground_cloud->size());
    return ground_cloud;
}