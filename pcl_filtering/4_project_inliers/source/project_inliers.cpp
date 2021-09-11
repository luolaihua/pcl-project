#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include<pcl/visualization/pcl_visualizer.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);


	pcl::visualization::PCLVisualizer viewer("project inliers");
	int v1(1);
	int v2(2);
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);

	//// 填入点云数据
	//pcl::PCDReader reader;
	//// 把路径改为自己存放文件的路径
	//reader.read<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud);
	//std::cerr << "Cloud before filtering: " << std::endl;
	//std::cerr << *cloud << std::endl;


	// 填入点云数据，使用随机数据，用作原理示例
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	std::cerr << "Cloud before projection: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;

	std::cerr << "Cloud size before projection: " << cloud->points.size()<< std::endl;

	// 平面公式：ax + by + cz + d = 0
	// 创建一个系数为X=Y=0,Z=1的平面,相当于是x-y平面
	// a = b = d = 0, c = 1;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;
	// 创建滤波器对象
	pcl::ProjectInliers<pcl::PointXYZ> proj;//创建滤波器对象
	proj.setModelType(pcl::SACMODEL_PLANE); //设置对象对应的投影模型
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);//设置投影模型的参数因子
	proj.filter(*cloud_projected);

	std::cerr << "Cloud after projection: " << std::endl;
	for (size_t i = 0; i < cloud_projected->points.size(); ++i)
		std::cerr << "    " << cloud_projected->points[i].x << " "
		<< cloud_projected->points[i].y << " "
		<< cloud_projected->points[i].z << std::endl;

	std::cerr << "Cloud size after projection: " << cloud_projected->points.size() << std::endl;

	viewer.addPointCloud(cloud, "c1", v1);
	viewer.addPointCloud(cloud_projected, "c2", v2);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c1", v1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c2", v2);
	viewer.addCoordinateSystem(1, cloud_projected->points[0].x, cloud_projected->points[0].y, cloud_projected->points[0].z, v2);
	viewer.addCoordinateSystem(1, cloud->points[0].x, cloud->points[0].y, cloud->points[0].z, v1);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return (0);
}
