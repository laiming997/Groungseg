#include "RasterProcessor.h"
using namespace std;
RasterProcessor::RasterProcessor():
    grid_size(0)
{
    pos_x_max = std::numeric_limits<double>::min();
    pos_x_min = std::numeric_limits<double>::max();
    pos_y_max = std::numeric_limits<double>::min();
    pos_y_min = std::numeric_limits<double>::max();
}

RasterProcessor::RasterProcessor(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in,double size_grid):
    origin_cloud(cloud_in),
    grid_size(size_grid)
{
    points_num = origin_cloud->size();
    pos_x_max = std::numeric_limits<double>::min();
    pos_x_min = std::numeric_limits<double>::max();
    pos_y_max = std::numeric_limits<double>::min();
    pos_y_min = std::numeric_limits<double>::max(); 
    cout << "Block Processor object has been instantiated, and the grid size is:" << grid_size << endl;
    std::cout << "cloud befor Block Processor has points:" << points_num << std::endl;
    for (int i = 0; i < origin_cloud->size(); i++)
    {
        double pos_x = origin_cloud->points[i].x;
        double pos_y = origin_cloud->points[i].y;
        if(pos_x_min > pos_x) pos_x_min = pos_x;
        if(pos_x_max < pos_x) pos_x_max = pos_x;
        if(pos_y_min > pos_y) pos_y_min = pos_y;
        if(pos_y_max < pos_y) pos_y_max = pos_y;
    }
    printf("the edge of zone:(%lf,%lf),(%lf,%lf)\n",pos_x_min,pos_x_max,pos_y_min,pos_y_max);
    rows = floor((pos_y_max - pos_y_min)/grid_size) +1;
    cols = floor((pos_x_max - pos_x_min)/grid_size) +1;
    printf("the rows of the raster is %d, the cols of the raster is %d\n",rows,cols);
}

RasterProcessor::~RasterProcessor()
{

}

std::tuple<pcl::PointXYZL**, cv::Mat , int**> RasterProcessor::RasterProcess(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> CHULL = RasterProcessor::ConvexHull();//寻找包围点云的凸包，返回凸包的顶点
    pcl::PointXYZL** lowraster = (pcl::PointXYZL**)malloc(sizeof(double)*(rows+1));
    int** lowraster_indices = (int**)malloc(sizeof(double)*(rows+1));
    for (int i = 0; i < rows+1; i++)
    {
        lowraster[i] = (pcl::PointXYZL*)malloc(4*sizeof(double)*(cols+1));
        lowraster_indices[i] = (int*)malloc(1*sizeof(double)*(cols+1));
        for (int j = 0; j < cols+1; j++)
        {
            lowraster[i][j].x = 0.0;
            lowraster[i][j].y = 0.0;
            lowraster[i][j].z = 0.0;
            lowraster[i][j].label = 0;
            lowraster_indices[i][j] = -1;
        }        
    }
    int max_temp_idy = 0;
	int max_temp_idx = 0; 
    for (int i = 0; i < origin_cloud->size(); i++)
    {
        pcl::PointXYZ points(origin_cloud->points[i]);
        int temp_idx = floor((points.x - pos_x_min)/ grid_size);//向下取整,列
        int temp_idy = floor((points.y - pos_y_min)/ grid_size);//行
        if(max_temp_idx < temp_idx)  max_temp_idx = temp_idx;
        if(max_temp_idy < temp_idy)  max_temp_idy = temp_idy;  
        /*The grid is empty hence insert the value*/
        if(lowraster[temp_idy][temp_idx].label == 0)
        {//第一次在该cell放入点
            lowraster[temp_idy][temp_idx].label = 1;//表示已经放入了点
            lowraster[temp_idy][temp_idx].x = points.x;
            lowraster[temp_idy][temp_idx].y = points.y;
            lowraster[temp_idy][temp_idx].z = points.z;
            lowraster_indices[temp_idy][temp_idx] = i;
        }
        else
        {
            lowraster[temp_idy][temp_idx].label = 2;//表示一个cell有多个点落入
            if(lowraster[temp_idy][temp_idx].z > points.z)
            {
                lowraster[temp_idy][temp_idx].x = points.x;
                lowraster[temp_idy][temp_idx].y = points.y;
                lowraster[temp_idy][temp_idx].z = points.z;
                lowraster_indices[temp_idy][temp_idx] = i;
            }
        }
    }
    for(int i = 0; i <rows+1; i++)
    {
        for(int j = 0; j <cols+1; j++)
        {
            if(lowraster[i][j].label == 0)
            {
                double x_nn = (j*grid_size)+pos_x_min;
                double y_nn = (i*grid_size)+pos_y_min;
                double z_nn = 0;
                if(i>0 && j>0 && i<rows-1 && j<cols-1)
                {
                    double dis_thresh = std::numeric_limits<double>::max();
                    for (int ii = i-1; ii <= i+1; ii++)
                    {
                        for (int jj = j-1; jj <= j+1; jj++)
                        {
                            if(ii != i && jj != j && lowraster[ii][jj].label != 0)
                            {
                                double dis = sqrt(SQUARE_DIST(x_nn,y_nn,lowraster[ii][jj].x,lowraster[ii][jj].y));
                                if(dis_thresh >dis)
                                {
                                    dis_thresh = dis;
                                    z_nn = lowraster[ii][jj].z;//获得一个八领域内距离最近的点的高程
                                }
                            }
                        }    
                    }
                }
                if(z_nn != 0)
                {
                    lowraster[i][j].label = 3;//插值获得
                    lowraster[i][j].x = x_nn;
                    lowraster[i][j].y = y_nn;
                    lowraster[i][j].z = z_nn;
                }
            }
        }
    }
    int lable_0 = 0, lable_1 = 0, lable_2 = 0, lable_3 = 0;
    for (int i = 0; i < rows+1; i++)
    {
        for (int j = 0; j < cols+1; j++)
        {
            if(lowraster[i][j].label == 0) lable_0++;
            else if(lowraster[i][j].label == 1) lable_1++;
            else if(lowraster[i][j].label == 2) lable_2++;
            else if(lowraster[i][j].label == 3) lable_3++;
        }   
    }
    printf("lable_0:%d lable_1:%d lable_2:%d,lable_3:%d\n",lable_0,lable_1,lable_2,lable_3);
    cv::Mat rasterize(rows,cols,CV_32FC1,cv::Scalar(0.0));
    for (int i = 0; i < rows+1; i++)
    {
        for (int j = 0; j < cols+1; j++)
        {
            if(lowraster[i][j].label != 0) rasterize.at<float>(i,j) = lowraster[i][j].z;
        }   
    }

    return std::make_tuple(lowraster, rasterize , lowraster_indices);
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> RasterProcessor::ConvexHull(){
//使用pcl库计算凸包
    pcl::ConvexHull<pcl::PointXYZ> chull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr chull_projection_points(new pcl::PointCloud<pcl::PointXYZ>);//原始点云投影到一个平面
    pcl::PointCloud<pcl::PointXYZ> chull_points;
    for (int i = 0; i < origin_cloud->size(); i++)
    {//先把点云都投影到一个平面上，再做凸包检测
        pcl::PointXYZ points;
        points.x = origin_cloud->points[i].x;
        points.y = origin_cloud->points[i].y;
        points.z = 1.0;
        chull_projection_points->push_back(points);
    }
    
    chull.setInputCloud(chull_projection_points);//放入点云
    chull.reconstruct(chull_points);//chull_point:位于凸包上的点；  polygons：凸包多边形顶点的索引号
    pcl::PointIndices hull_point_indicies;//获得凸包顶点在原始点云上的索引号
    chull.getHullPointIndices(hull_point_indicies);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> CHULL_vertices = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < hull_point_indicies.indices.size(); i++)
    {
        CHULL_vertices->push_back(pcl::PointXYZ(origin_cloud->points[hull_point_indicies.indices[i]].x, origin_cloud->points[hull_point_indicies.indices[i]].y, 1.0));
    }
    std::cout<< "CHULL_vertices has points :" << CHULL_vertices->size() << std::endl;
    return CHULL_vertices;
}