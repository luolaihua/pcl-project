//#include <iostream>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include<pcl/visualization/pcl_visualizer.h>
//#include<pcl/io/pcd_io.h>
//#include<vector>
//
//int
//main(int argc, char** argv)
//{
//	//原始点云
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	//平面上的点云
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inner(new pcl::PointCloud<pcl::PointXYZ>);	
//	//平面外的点云
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outer(new pcl::PointCloud<pcl::PointXYZ>);
//	
//	//填充点云数据
//	(*cloud).width = 15;
//	(*cloud).height = 1;
//	(*cloud).points.resize((*cloud).width * (*cloud).height);
//	//生成数据
//	for (size_t i = 0; i < (*cloud).points.size(); ++i)
//	{
//		(*cloud).points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//		(*cloud).points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//		//z 坐标始终为1，说明这些点位于同一个平面
//		(*cloud).points[i].z = 1.0;
//	}
//	//设置几个局外点，三个平面外的点
//	(*cloud).points[0].z = 200.0;
//	(*cloud).points[3].z = -200.0;
//	(*cloud).points[6].z = 400.0;
//	std::cerr << "Point cloud data: " << (*cloud).points.size() << " points" << std::endl;
//	for (size_t i = 0; i < (*cloud).points.size(); ++i)
//		std::cerr <<"index:\t"<< i<<"\t" << (*cloud).points[i].x << "\t"
//		<< (*cloud).points[i].y << "\t"
//		<< (*cloud).points[i].z << std::endl;
//
//
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//	//创建分割对象
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	//可选设置
//	seg.setOptimizeCoefficients(true);
//	//必须设置
//	seg.setModelType(pcl::SACMODEL_PLANE);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setDistanceThreshold(0.01);
//	seg.setInputCloud(cloud);
//	seg.segment(*inliers, *coefficients);
//
//	//判断是否分割成功
//	if (inliers->indices.size() == 0)
//	{
//		PCL_ERROR("Could not estimate a planar model for the given dataset.");
//		return (-1);
//	}
//	std::cerr << std::endl << "Model coefficients: " << coefficients->values[0] << " "
//		<< coefficients->values[1] << " "
//		<< coefficients->values[2] << " "
//		<< coefficients->values[3] << std::endl << std::endl;
//
//
//	//根据分割结果填充平面内和平面外点云
//	cloud_inner->width = inliers->indices.size();
//	cloud_inner->height = 1;
//	cloud_inner->points.resize(cloud_inner->width * cloud_inner->height);
//
//	cloud_outer->width = cloud->points.size() - inliers->indices.size();
//	cloud_outer->height = 1;
//	cloud_outer->points.resize(cloud_outer->width * cloud_outer->height);
//
//	//创建一个数组，大小为点云总数,初始化为0
//	std::vector<int> p_flag(cloud->points.size());
//
//	//将平面内的点标记
//	for (size_t i = 0; i < inliers->indices.size(); ++i)
//		p_flag[inliers->indices[i]] = 1;
//
//	for (size_t i = 0,j=0 ; i < (*cloud).points.size(); ++i)
//	{
//		//遍历，找出平面外的点
//		if (p_flag[i] == 0)
//		{
//			cloud_outer->points[j].x = (*cloud).points[i].x;
//			cloud_outer->points[j].y = (*cloud).points[i].y;
//			cloud_outer->points[j].z = (*cloud).points[i].z;
//			++j;
//			std::cerr << "outer points index:\t" << i << "\t" << (*cloud).points[i].x << "\t"
//				<< (*cloud).points[i].y << "\t"
//				<< (*cloud).points[i].z << std::endl;
//		}
//	}
//
//	//打印出平面外的点
//	std::cerr << std::endl << "Outer points: " << cloud_outer->points.size() << std::endl;
//	for (size_t i = 0; i < cloud_outer->points.size(); ++i)
//	{
//		std::cerr << "\t" << (*cloud_outer).points[i].x << "\t"
//			<< (*cloud_outer).points[i].y << "\t"
//			<< (*cloud_outer).points[i].z << std::endl;
//	}
//
//	//平面内的点
//	std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
//	for (size_t i = 0; i < inliers->indices.size(); ++i)
//	{
//		cloud_inner->points[i].x = (*cloud).points[inliers->indices[i]].x;
//		cloud_inner->points[i].y = (*cloud).points[inliers->indices[i]].y;
//		cloud_inner->points[i].z = (*cloud).points[inliers->indices[i]].z;
//		std::cerr << "index:\t" << inliers->indices[i] << "\t" << (*cloud).points[inliers->indices[i]].x << "\t"
//			<< (*cloud).points[inliers->indices[i]].y << "\t"
//			<< (*cloud).points[inliers->indices[i]].z << std::endl;
//	}
//
//	//图形化显示
//	//创建PCLVisualzer对象
//	pcl::visualization::PCLVisualizer viewer("Plane Model Segmentation");
//
//	int v1(1);
//	int v2(2);
//	
//	//创建视角v1，v2
//	viewer.createViewPort(0.0, 0.0, 0.5, 1.0,v1);
//	viewer.createViewPort(0.5, 0.0, 1.0, 1.0,v2);
//	//设置背景颜色为白色
//	viewer.setBackgroundColor(255, 255, 255, v1);
//	viewer.setBackgroundColor(255, 255, 255, v2);
//	//添加直角坐标，放大1000倍
//	//viewer.addCoordinateSystem(1000,v1);
//	//viewer.addCoordinateSystem(1000,v2);
//	
//	//设置点云颜色
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_origin(cloud, 255, 0, 0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in(cloud_inner, 255, 0, 0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out(cloud_outer, 0, 0, 255);
//
//
//	viewer.addPointCloud(cloud, cloud_origin, "v1", v1);
//	viewer.addPointCloud(cloud_outer, cloud_out, "v2", v2);
//	viewer.addPointCloud(cloud_inner, cloud_in, "v3", v2);
//
//	//设置点云的大小，point_size默认为1，这里设置为1000，突出显示
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1000,"v1",v1);
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1000,"v2",v2);
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1000,"v3",v2);
//	viewer.spin();
//
//	return (0);
//}
