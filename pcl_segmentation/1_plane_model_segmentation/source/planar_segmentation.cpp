#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/pcd_io.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//填充点云数据
	(*cloud).width = 15;
	(*cloud).height = 1;
	(*cloud).points.resize((*cloud).width * (*cloud).height);
	//生成数据
	for (size_t i = 0; i < (*cloud).points.size(); ++i)
	{
		(*cloud).points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		(*cloud).points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		(*cloud).points[i].z = 1.0;
	}
	//设置几个局外点
	(*cloud).points[0].z = 2.0;
	(*cloud).points[3].z = -2.0;
	(*cloud).points[6].z = 4.0;
	std::cerr << "Point cloud data: " << (*cloud).points.size() << " points" << std::endl;
	for (size_t i = 0; i < (*cloud).points.size(); ++i)
		std::cerr << "    " << (*cloud).points[i].x << " "
		<< (*cloud).points[i].y << " "
		<< (*cloud).points[i].z << std::endl;


	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//可选设置
	seg.setOptimizeCoefficients(true);
	//必须设置
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}
	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;
	std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

	for (size_t i = 0; i < inliers->indices.size(); ++i)
		std::cerr << inliers->indices[i] << "    " << (*cloud).points[inliers->indices[i]].x << " "
		<< (*cloud).points[inliers->indices[i]].y << " "
		<< (*cloud).points[inliers->indices[i]].z << std::endl;

	//pcl::io::savePCDFileASCII("test.pcd", *cloud);
	////图形化显示
	////创建PCLVisualzer对象
	//pcl::visualization::PCLVisualizer viewer("Plane Model Segmentation");

	//int v1(1);
	//int v2(2);

	//viewer.createViewPort(0.0, 0.0, 0.5, 1.0,v1);
	//viewer.createViewPort(0.5, 0.0, 1.0, 1.0,v2);
	//viewer.setBackgroundColor(255, 255, 255, v1);
	//viewer.setBackgroundColor(0, 0, 255, v2);
	//viewer.addCoordinateSystem(1, v1);
	//
	////设置点云颜色
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_origin(cloud, 255, 0, 0);
	//viewer.addPointCloud(cloud, cloud_origin, "v1", v1);
	//viewer.addPointCloud(cloud, cloud_origin, "v2", v2);
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1000,"v1",v1);
	//viewer.spin();

	return (0);
}
