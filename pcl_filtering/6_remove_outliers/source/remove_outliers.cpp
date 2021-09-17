//#include <iostream>
//#include <pcl/point_types.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/filters/conditional_removal.h>
//#include<pcl/visualization/pcl_visualizer.h>
//int
//main(int argc, char** argv)
//{
//	//if (argc != 2)
//	//{
//	//	std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
//	//	exit(0);
//	//}
//	int testNum = 0;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//	// 填入点云数据
//	cloud->width = 5;
//	cloud->height = 1;
//	cloud->points.resize(cloud->width * cloud->height);
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//	{
//		cloud->points[i].x = rand() / (RAND_MAX + 1.0f);
//		cloud->points[i].y = rand() / (RAND_MAX + 1.0f);
//		cloud->points[i].z = rand() / (RAND_MAX + 1.0f);
//	}
//	if (testNum == 0) {
//		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//		// 创建滤波器
//		outrem.setInputCloud(cloud);
//		outrem.setRadiusSearch(0.5);
//		outrem.setMinNeighborsInRadius(2);
//		// 应用滤波器
//		outrem.filter(*cloud_filtered);
//	}
//	else{
//		// 创建环境
//		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new
//			pcl::ConditionAnd<pcl::PointXYZ>());
//		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
//			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
//		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
//			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
//		// 创建滤波器
//		pcl::ConditionalRemoval<pcl::PointXYZ> condrem(range_cond);
//		condrem.setInputCloud(cloud);
//		condrem.setKeepOrganized(true);
//		// 应用滤波器
//		condrem.filter(*cloud_filtered);
//	}
//
//	std::cerr << "Cloud before filtering: " << std::endl;
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//		std::cerr << "    " << cloud->points[i].x << " "
//		<< cloud->points[i].y << " "
//		<< cloud->points[i].z << std::endl;
//	// 显示滤波后的点云
//	std::cerr << "Cloud after filtering: " << std::endl;
//	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
//		std::cerr << "    " << cloud_filtered->points[i].x << " "
//		<< cloud_filtered->points[i].y << " "
//		<< cloud_filtered->points[i].z << std::endl;
//	return (0);
//}
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read("pig.pcd", *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	boundingbox_ptr->push_back(pcl::PointXYZ(0.1, 0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(0.1, -0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, 0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(-0.1, -0.1, 0));
	boundingbox_ptr->push_back(pcl::PointXYZ(0.15, 0.1, 0));

	pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(boundingbox_ptr);
	hull.setDimension(2);
	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
	hull.reconstruct(*surface_hull, polygons);

	pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropHull<pcl::PointXYZ> bb_filter;
	bb_filter.setDim(2);
	bb_filter.setInputCloud(cloud);
	bb_filter.setHullIndices(polygons);
	bb_filter.setHullCloud(surface_hull);
	bb_filter.filter(*objects);
	std::cout << objects->size() << std::endl;

	//visualize
	boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v(new pcl::visualization::PCLVisualizer("crophull display"));
	for_visualizer_v->setBackgroundColor(255, 255, 255);

	int v1(0);
	for_visualizer_v->createViewPort(0.0, 0.0, 0.33, 1, v1);
	for_visualizer_v->setBackgroundColor(255, 255, 255, v1);
	for_visualizer_v->addPointCloud(cloud, "cloud1", v1);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloud1", v1);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1", v1);
	for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull, 0, .069 * 255, 0.2 * 255, "backview_hull_polyline1", v1);

	int v2(0);
	for_visualizer_v->createViewPort(0.33, 0.0, 0.66, 1, v2);
	for_visualizer_v->setBackgroundColor(255, 255, 255, v2);
	for_visualizer_v->addPointCloud(surface_hull, "surface_hull", v2);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "surface_hull", v2);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "surface_hull", v2);
	for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull, 0, .069 * 255, 0.2 * 255, "backview_hull_polyline", v2);

	int v3(0);
	for_visualizer_v->createViewPort(0.66, 0.0, 1, 1, v3);
	for_visualizer_v->setBackgroundColor(255, 255, 255, v3);
	for_visualizer_v->addPointCloud(objects, "objects", v3);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "objects", v3);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "objects", v3);

	while (!for_visualizer_v->wasStopped())
	{

		for_visualizer_v->spinOnce(1000);
	}
	system("pause");
}
