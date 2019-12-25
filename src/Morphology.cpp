#include "Morphology.h"
using namespace std;

Morphology::Morphology()
{

}

Morphology::Morphology(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in,Params input_param):
    origin_cloud(cloud_in)
{
    params = input_param;
    cout << "Morphology object has been instantiated, and the origin_cloud size is: " << origin_cloud->size() << endl;
    /* Raster_Process_obj = new RasterProcessor(origin_cloud,input_param.grid_size); */
}

Morphology::~Morphology()
{
}

std::vector<cv::Point3f> Morphology::ExtractSeedByPCL()
{
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pcl::PointIndicesPtr ground_pmf (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_ground_pmf (new pcl::PointCloud<pcl::PointXYZ>);
    std::cerr << "Ground cloud before pmf filtering: " << origin_cloud->size() << std::endl;
    pmf.setInputCloud(origin_cloud);
    pmf.setMaxWindowSize(params.Max_window_size);
    pmf.setSlope(params.Terrian_slop);
    pmf.setInitialDistance(params.height_diff_threshold_init);
    pmf.setMaxDistance(params.height_diff_threshold_max);
    pmf.extract(ground_pmf->indices);
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(origin_cloud);
    extract.setIndices(ground_pmf);
    extract.filter(*Cloud_ground_pmf);
    std::cerr << "Ground cloud after pmf filtering: " << Cloud_ground_pmf->size() << std::endl;
    
    vector<cv::Point3f> vec3;
    for (int i = 0; i <Cloud_ground_pmf->size();i++) 
    {
        cv::Point3f points(Cloud_ground_pmf->at(i).x, Cloud_ground_pmf->at(i).y,Cloud_ground_pmf->at(i).z);
        vec3.push_back(points);
    }
    return vec3;
}


std::tuple<std::vector<cv::Point3f>,pcl::PointIndices> Morphology::My_ExtractGroundSeed()
{   
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (origin_cloud);
    // Compute the series of window sizes and height thresholds
    std::vector<float> height_thresholds;
    std::vector<float> window_sizes;
    int iteration = 0;
    int window_size = 0;
    float height_threshold  = 0.0f;
    
    while (window_size < params.Max_window_size)
    {
        // Determine the initial window size.
        window_size = (int)(std::pow (2.0, iteration) + 1.0f);
        // window_size = 2* iteration + 1;
        if(iteration == 0)
        {
            height_threshold = params.height_diff_threshold_init;
        }
        else height_threshold = params.Terrian_slop *(window_size - window_sizes[iteration - 1]) * params.grid_size + params.height_diff_threshold_init;
        if(height_threshold > params.height_diff_threshold_max)  height_threshold = params.height_diff_threshold_max;
        
        window_sizes.push_back(window_size);
        height_thresholds.push_back(height_threshold);
        
        iteration++;
    }
    
    // Progressively filter ground returns using morphological open
    Raster_Process_obj = new RasterProcessor(origin_cloud,params.grid_size);//对输入的点云进行规则网格化
    cv::Mat rasterize;//规则网格化后的Mat;
    pcl::PointXYZL** lowraster;//网格点云
    int** lowraster_indices;//每个网格的点云序号
    std::tie(lowraster, rasterize , lowraster_indices) = Raster_Process_obj ->RasterProcess();
    Points_info** rasterInfo =  (Points_info**)malloc(sizeof(int*)*(Raster_Process_obj->rows+1));//保存放入原始点云的序号和地物点标志位
    for (int i = 0; i < Raster_Process_obj->rows+1; i++)
    {
        rasterInfo[i] = (Points_info*)malloc(2*sizeof(int)*(Raster_Process_obj->cols+1));
        for (int j = 0; j < Raster_Process_obj->cols +1 ; j++)
        {
            if(lowraster_indices[i][j] != -1 && (lowraster[i][j].label == 1 || lowraster[i][j].label == 2) ) 
            {
                rasterInfo[i][j].index = lowraster_indices[i][j];
                rasterInfo[i][j].flag  = 0;
            }
            else
            {
                rasterInfo[i][j].index = -1;//表示放的非原始点云
                rasterInfo[i][j].flag  = -1;
            }
        }
    }
    for (int i = 0; i < window_sizes.size(); i++)
    {
        printf("      Iteration %d (height threshold = %f, window size = %f)...\n",i, height_thresholds[i], window_sizes[i]);
        // opening operation at the current window size.
        cv::Mat channel_blur = rasterize.clone();//对网格化后的Mat进行高斯滤波
        cv::Mat strl = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(window_sizes[i],window_sizes[i]));//根据window_size获得数学形态法的滤波窗口
        cv::Mat dst;
        cv::morphologyEx(rasterize,dst,cv::MORPH_OPEN,strl);//进行开运算
        rasterize = dst.clone();
        int non_ground_num = 0;
        for (int ii = 0; ii < Raster_Process_obj->rows; ii++)
        {
            for (int jj = 0; jj < Raster_Process_obj->cols; jj++)
            {
                if(abs(channel_blur.at<float>(ii,jj) - dst.at<float>(ii,jj))>height_thresholds[i])
                {
                    if(rasterInfo[ii][jj].index != -1 && rasterInfo[ii][jj].flag == 0)
                    {
                        rasterInfo[ii][jj].flag = window_sizes[i];
                        non_ground_num ++ ;
                    }
                }
            }
        }
        cout << "non_ground_num:" << non_ground_num << endl;
    }
    vector<cv::Point3f> vec3;
    pcl::PointIndices gound_indices;
    for (int ii = 0; ii < Raster_Process_obj->rows; ii++)
    {
        for (int jj = 0; jj < Raster_Process_obj->cols; jj++)
        {
            if(rasterInfo[ii][jj].index != -1 && rasterInfo[ii][jj].flag == 0)
            {
                vec3.push_back(cv::Point3f(lowraster[ii][jj].x, lowraster[ii][jj].y,lowraster[ii][jj].z));
                gound_indices.indices.push_back(rasterInfo[ii][jj].index);
            }
        }
    }
    std::cout<< "gound_indices:"<<gound_indices.indices.size()<<endl;
    return std::make_tuple(vec3,gound_indices);
}




cv::Mat Morphology::strelDisk(int Radius)
{
    cv::Mat sel((2*Radius -1),(2*Radius -1),CV_8U,cv::Scalar(1));
    int borderWidth;
    switch (Radius)
    {
        case 1:  borderWidth = 1; break;
        case 2:  borderWidth = 2; break;
        case 3:  borderWidth = 0; break;
        case 4:  borderWidth = 2; break;
        case 5:  borderWidth = 2; break;
        case 6:  borderWidth = 2; break;
        case 7:  borderWidth = 2; break;
        case 8:  borderWidth = 4; break;
        case 9:  borderWidth = 4; break;
        case 10: borderWidth = 4; break;
        case 11: borderWidth = 6; break;
        case 12: borderWidth = 6; break;
        case 13: borderWidth = 6; break;
        case 14: borderWidth = 6; break;
        case 15: borderWidth = 8; break;
        case 16: borderWidth = 8; break;
        case 17: borderWidth = 8; break;
        case 18: borderWidth = 10; break;
        case 19: borderWidth = 10; break;
        case 20: borderWidth = 10; break;
        case 21: borderWidth = 10; break;
        case 22: borderWidth = 10; break;
        case 23: borderWidth = 12; break;
        case 24: borderWidth = 12; break;
        case 25: borderWidth = 14; break;
        case 26: borderWidth = 14; break;
        case 27: borderWidth = 14; break;
        case 28: borderWidth = 14; break;
        case 29: borderWidth = 16; break;
        case 30: borderWidth = 16; break;
        case 31: borderWidth = 16; break;
        case 32: borderWidth = 18; break;
        case 33: borderWidth = 18; break;
        case 34: borderWidth = 18; break;
        case 35: borderWidth = 18; break;
        case 36: borderWidth = 20; break;
        case 37: borderWidth = 20; break;
        case 38: borderWidth = 20; break;
        case 39: borderWidth = 20; break;
    }
    for (int i = 0; i < borderWidth; i++)
    {
        for (int j = borderWidth -1 -i; j >= 0; j--)
        {
            sel.at<uchar>(i,j) = 0;
            sel.at<uchar>(sel.rows - 1 - i,j) = 0;
            sel.at<uchar>(i,sel.cols - 1 - j) = 0;
            sel.at<uchar>(sel.rows - 1 - i,sel.cols - 1 - j) = 0;
        }
    }
    return sel;
}
