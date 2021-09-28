#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include<pcl/visualization/pcl_visualizer.h>
/*
	在平面模型上构建凸多边形
*/
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;

	reader.read("table_scene_mug_stereo_textured.pcd", *cloud);
	// 使用直通滤波器，滤除z轴上的杂点
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.1);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: "
		<< cloud_filtered->points.size() << " data points." << std::endl;


	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// 可选的,设置优化系数
	seg.setOptimizeCoefficients(true);
	// 必须的步骤
	seg.setModelType(pcl::SACMODEL_PLANE);//平面模型
	seg.setMethodType(pcl::SAC_RANSAC);//设置采样一致性估计方法模型为SAC_RANSAC
	seg.setDistanceThreshold(0.01);//设置距离阈值为0.01，与估计平面模型距离小于0.01cm的点都为内点inliners

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);//开始分割，结果保存在inliers和coefficients里面
	std::cerr << "PointCloud after segmentation has: "
		<< inliers->indices.size() << " inliers." << std::endl;

	// 创建点云投影滤波
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);//投影滤波模型为平面SACMODEL_PLANE
	proj.setInputCloud(cloud_filtered);
	proj.setModelCoefficients(coefficients);//将分割估计得到的参数因子coefficients设置为投影平面模型系数
	proj.filter(*cloud_projected);
	std::cerr << "PointCloud after projection has: "
		<< cloud_projected->points.size() << " data points." << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	//创建多边形提取对象
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_projected);
	chull.setAlpha(0.1); //设置alpha值为0.1
	chull.reconstruct(*cloud_hull);//重建提取创建凹多边形

	std::cerr << "Concave hull has: " << cloud_hull->points.size()
		<< " data points." << std::endl;

	pcl::PCDWriter writer;
	writer.write("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

	//显示
	pcl::visualization::PCLVisualizer viewer("Restructing");
	int v1(1);
	int v2(2);
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.addPointCloud(cloud_projected, "v1", v1);
	viewer.addPointCloud(cloud_hull, "v2", v2);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return (0);
}