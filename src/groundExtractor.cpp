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
	
	Morphology mor_filt(cloud_in,param);
	std::vector<cv::Point3f> vec3,pmr_vec3;
	pcl::PointIndices gound_indices;
	std::tie(vec3,gound_indices) = mor_filt.My_ExtractGroundSeed();//自己写的渐进式数学形态学滤波器
	Triangulation PDT_filter(cloud_in,angle_threshold,distance_threshold);//对数学形态学滤波器获得地面点的种子点后，进行TIN不规则三角网迭代
	PDT_filter.triangulation_process(vec3);
	// pmr_vec3 = mor_filt.ExtractSeedByPCL();
	
	
	// 错误率计算
	double type_1_error = 0.0, type_2_error = 0.0;
	pcl::PointCloud<pcl::PointXYZL> my_seg_cloud;
	pcl::PointIndices non_gound_indices;
	int ground_num_truth=0,object_num_truth=0;
	int ground2obj=0,obj2ground=0;
	bool flag=false;
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
		// // for (int j = 0; j < gound_indices.indices.size(); j++)
		// // {
		// // 	if(i == gound_indices.indices[j])
		// // 	{//该索引号在滤波后地面点的索引号里
		// // 		flag = true;
		// // 	}
		// // }
		// if(flag == false)
		// {
		// 	non_gound_indices.indices.push_back(i);
		// }
		// else flag = false; 
	}
	cout << "non_gound_indices:" << non_gound_indices.indices.size()<<endl;
	for (int i = 0; i <gound_indices.indices.size(); i++)
	{
		if(cloud_filter->points[gound_indices.indices[i]].label ==1)
		{//把地物点识别成地面点
			obj2ground++;
		}
	}	
	cout <<"object_num_truth" << object_num_truth<<endl;
	cout <<"ground_num_truth" << ground_num_truth<<endl;
	cout << "obj2ground:" << obj2ground<<endl;
	// 可视化
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_view(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pmr(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < vec3.size(); i++)
	{
		cloud_view->push_back(pcl::PointXYZ(vec3[i].x,vec3[i].y,vec3[i].z));
	}
	// for (int i = 0; i < pmr_vec3.size(); i++)
	// {
	// 	cloud_pmr->push_back(pcl::PointXYZ(pmr_vec3[i].x,pmr_vec3[i].y,pmr_vec3[i].z));
	// }
	pcl::visualization::PCLVisualizer viewer("view");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_viewer_handler(cloud_view, 255, 0, 0); // green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_pmr_handler(Ground_cloud.makeShared(), 0, 255, 0); // green
	viewer.addPointCloud<pcl::PointXYZ>(cloud_view,cloud_viewer_handler,"cloud_view");
	viewer.addPointCloud<pcl::PointXYZ>(Ground_cloud.makeShared(),cloud_pmr_handler,"cloud_pmr");
	viewer.spin();
	system("PAUSE");
    return 0;
}
