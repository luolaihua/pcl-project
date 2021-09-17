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

//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/project_inliers.h>
//#include<pcl/visualization/pcl_visualizer.h>
//int
//main(int argc, char** argv)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::visualization::PCLVisualizer viewer("project inliers");
//	int v1(1);
//	int v2(2);
//	viewer.createViewPort(0, 0, 0.5, 1, v1);
//	viewer.createViewPort(0.5, 0, 1, 1, v2);
//
//
//	// 填入点云数据
//	cloud->width = 5;
//	cloud->height = 1;
//	cloud->points.resize(cloud->width * cloud->height);
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//	{
//		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//	}
//	std::cerr << "Cloud before projection: " << std::endl;
//	for (size_t i = 0; i < cloud->points.size(); ++i)
//		std::cerr << "    " << cloud->points[i].x << " "
//		<< cloud->points[i].y << " "
//		<< cloud->points[i].z << std::endl;
//	// 平面公式：ax + by + cz + d = 0
//	// 创建一个系数为X=Y=0,Z=1的平面,相当于是x-y平面
//	// a = b = d = 0, c = 1;
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//	coefficients->values.resize(4);
//	coefficients->values[0] = coefficients->values[1] = 0;
//	coefficients->values[2] = 1.0;
//	coefficients->values[3] = 0;
//	// 创建滤波器对象
//	pcl::ProjectInliers<pcl::PointXYZ> proj;//创建滤波器对象
//	proj.setModelType(pcl::SACMODEL_PLANE); //设置对象对应的投影模型
//	proj.setInputCloud(cloud);
//	proj.setModelCoefficients(coefficients);//设置投影模型的参数因子
//	proj.filter(*cloud_projected);
//
//	std::cerr << "Cloud after projection: " << std::endl;
//	for (size_t i = 0; i < cloud_projected->points.size(); ++i)
//		std::cerr << "    " << cloud_projected->points[i].x << " "
//		<< cloud_projected->points[i].y << " "
//		<< cloud_projected->points[i].z << std::endl;
//
//	viewer.addPointCloud(cloud, "c1", v1);
//	viewer.addPointCloud(cloud_projected, "c2", v2);
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1000, "c1", v1);
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1000, "c2", v2);
//
//	viewer.addCoordinateSystem(1000, cloud_projected->points[0].x, cloud_projected->points[0].y, cloud_projected->points[0].z, "c1", v2);
//	viewer.addCoordinateSystem(1000, cloud->points[0].x, cloud->points[0].y, cloud->points[0].z, "c2", v1);
//
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//	return (0);
//}
