#include "FileReader.h"
#include "Morphology.h"
#include "Triangulation.h"
#include "Config.h"
int main(int argc, char const *argv[])
{
    if (argc < 2)
    {
		fprintf(stderr, "Usage: %s, Input Filename\n", argv[0]);
		system("PAUSE");
		return -1;
	}	
	FileReader reader(argv[1]);

	// cloud_in = reader.txtReader();
	pcl::PointCloud<pcl::PointXYZL> cloud_ref;
    pcl::PointCloud<pcl::PointXYZ> Ground_cloud;
    pcl::PointCloud<pcl::PointXYZ> Object_cloud;
	std::tie(cloud_ref, Ground_cloud,Object_cloud)= reader.RetxtReader();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZL> sor;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> cloud_filter(new pcl::PointCloud<pcl::PointXYZL>);
	sor.setInputCloud(cloud_ref.makeShared());
	sor.setMeanK(20);
	sor.setStddevMulThresh(3.0);
	sor.filter(*cloud_filter);//先预处理点云去掉一些孤立点
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i <cloud_filter->size();i++)
	{
		cloud_in->push_back(pcl::PointXYZ(cloud_filter->points[i].x, cloud_filter->points[i].y, cloud_filter->points[i].z));
	}
	
	std::string ConfigFile= argv[2];
	Config configSettings(ConfigFile);
	Params param;
	param.grid_size = configSettings.Read("grid_size",0.25);
	param.height_diff_threshold_init = configSettings.Read("height_diff_threshold_init",0.5);
	param.height_diff_threshold_max = configSettings.Read("height_diff_threshold_max",3);
	param.Max_window_size = configSettings.Read("Max_window_size",38);
	param.Terrian_slop = configSettings.Read("Terrian_slop",1.0);//数学形态法的参数
	double angle_threshold =  configSettings.Read("angle_threshold",2.0); // angle in radians
	double distance_threshold = configSettings.Read("distance_threshold",0.2);
	
	printf("grid_size:%lf\n",param.grid_size);
	printf("height_diff_threshold_init:%lf\n",param.height_diff_threshold_init);
	printf("Max_window_size:%d\n",param.Max_window_size);
	printf("Terrian_slop:%lf\n",param.Terrian_slop);
	printf("angle_threshold:%lf\n",angle_threshold);
	printf("distance_threshold:%lf\n",distance_threshold);
	
	Morphology mor_filt(cloud_in,param);//建立数学形态学滤波器的对象
	std::vector<cv::Point3f> vec3,pmr_vec3;
	pcl::PointIndices gound_indices;
	std::tie(vec3,gound_indices) = mor_filt.My_ExtractGroundSeed();//自己写的渐进式数学形态学滤波器
	Triangulation PDT_filter(cloud_in,angle_threshold,distance_threshold);//对数学形态学滤波器获得地面点的种子点后，进行TIN不规则三角网迭代
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_groundTIN(new pcl::PointCloud<pcl::PointXYZ>);
	std::tie(cloud_groundTIN,gound_indices) = PDT_filter.triangulation_process(gound_indices,vec3);
	
	pcl::ExtractIndices<pcl::PointXYZ> filter;
	pcl::PointIndices::Ptr ground_indices_ptr(new pcl::PointIndices);
	std::vector<int> obj_indices;
	for (int i = 0; i<gound_indices.indices.size();i++)
	{
		ground_indices_ptr->indices.push_back(gound_indices.indices[i]);
	}
	
	filter.setInputCloud(cloud_in);
	filter.setIndices(ground_indices_ptr);
	filter.setNegative(true);
	filter.filter(obj_indices);
	cout<< "obj_indices" << obj_indices.size()<<endl;
	// 错误率计算
	cout << "/**********************Precision analysis************************************/"<<endl;
	pcl::PointIndices::Ptr ground2obj_index(new pcl::PointIndices);
	pcl::PointIndices::Ptr obj2ground_index(new pcl::PointIndices);
	double type_1_error = 0.0, type_2_error = 0.0, total_error = 0.0;
	int ground_num_truth=0,object_num_truth=0;
	int ground2obj=0,obj2ground=0;
	for (int i = 0; i <cloud_filter->size();i++)
	{
		if(cloud_filter->points[i].label ==1)
		{//地物点真值
			object_num_truth++;
		}
		else if(cloud_filter->points[i].label == 0)
		{//地面点真值
			ground_num_truth++;
		}
	}
	for (int i = 0; i <obj_indices.size(); i++)
	{
		if(cloud_filter->points[obj_indices[i]].label ==0)
		{//把地面点识别成地物点
			ground2obj++;
			ground2obj_index->indices.push_back(obj_indices.at(i));
		}
	}	
	for (int i = 0; i <gound_indices.indices.size(); i++)
	{
		if(cloud_filter->points[gound_indices.indices[i]].label ==1)
		{//把地物点识别成地面点
			obj2ground++;
			obj2ground_index->indices.push_back(gound_indices.indices.at(i));
		}
	}
	type_1_error = float(ground2obj)/float(ground_num_truth);
	type_2_error = float(obj2ground)/float(object_num_truth);
	total_error  = float(ground2obj+obj2ground)/float(cloud_filter->size());
	cout <<"object_num_truth:" << object_num_truth<<endl;
	cout <<"ground_num_truth:" << ground_num_truth<<endl;
	cout << "obj2ground:" << obj2ground<<endl;
	cout << "ground2obj:" << ground2obj<<endl;
	printf("type_1_error:%f\n",type_1_error);
	printf("type_2_error:%f\n",type_2_error);
	printf("total_error:%f\n",total_error);
	// 可视化
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground2obj (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obj2ground (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> get_error_points;
	get_error_points.setInputCloud(cloud_in);
	get_error_points.setIndices(ground2obj_index);
	get_error_points.filter(*cloud_ground2obj);
	get_error_points.setIndices(obj2ground_index);
	get_error_points.filter(*cloud_obj2ground);//获得错误点
	
	pcl::visualization::PCLVisualizer viewer("view");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_viewer_handler(cloud_groundTIN, 255, 255, 255); // green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_g2o_handler(cloud_ground2obj, 255, 0, 0); // red
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_o2g_handler(cloud_obj2ground, 0, 255, 0); // green
	viewer.addPointCloud<pcl::PointXYZ>(cloud_groundTIN,cloud_viewer_handler,"cloud_view");
	viewer.addPointCloud<pcl::PointXYZ>(cloud_ground2obj,cloud_g2o_handler,"cloud_ground2obj");
	viewer.addPointCloud<pcl::PointXYZ>(cloud_obj2ground,cloud_o2g_handler,"cloud_obj2ground");
	viewer.spin();
	system("PAUSE");
    return 0;
}
