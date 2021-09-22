#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	// 填入点云数据
	cloud_in->width = 5;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);

	//使用随机数填充点云
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	//打印创建的点云数据
	std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
		<< std::endl;
	for (size_t i = 0; i < cloud_in->points.size(); ++i) std::cout << "    " <<
		cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
		cloud_in->points[i].z << std::endl;

	//创建第二个点云
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		//这里只做了平移变换，所有点的x坐标平移了0.7的距离
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;


	std::cout << "Transformed " << cloud_in->points.size() << " data points:"
		<< std::endl;
	for (size_t i = 0; i < cloud_out->points.size(); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " <<
		cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

	//创建ICP对象
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//为ICP算法设置输入点云
	icp.setInputCloud(cloud_in);
	//设置目标点云
	icp.setInputTarget(cloud_out);
	//Final为验证的结果
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	//打印出验证的结果
	std::cout << "Final " << cloud_in->points.size() << " data points:"
		<< std::endl;
	for (size_t i = 0; i < Final.points.size(); ++i)
		std::cout << "    " << Final.points[i].x << " " <<
		Final.points[i].y << " " << Final.points[i].z << std::endl;

	//如果hasConverged() == 1,说明输入点云和目标点云之间的是刚性变换，并返回ICP的评估分数
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;

	//打印出输入点云和目标点云之间的转换矩阵
	std::cout << icp.getFinalTransformation() << std::endl;
	return (0);
}
