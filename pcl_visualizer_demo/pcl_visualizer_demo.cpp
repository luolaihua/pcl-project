#include<iostream>
#include<thread>

#include<pcl/common/common_headers.h>
#include<pcl/features/normal_3d.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/console/parse.h>

using namespace std::chrono_literals;

// ----------------------
// -----打印使用帮助-----
// ----------------------

void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-s           Simple visualization example\n"
		<< "-r           RGB colour visualization example\n"
		<< "-c           Custom colour visualization example\n"
		<< "-n           Normals visualization example\n"
		<< "-a           Shapes visualization example\n"
		<< "-v           Viewports example\n"
		<< "-i           Interaction Customization example\n"
		<< "\n\n";
}

//************************************
// Method:    传入三个const指针，返回一个PCLVisualizer指针
// Returns:   pcl::visualization::PCLVisualizer::Ptr
// Parameter: pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud
// Parameter: pcl::PointCloud<pcl::Normal>::ConstPtr normals1
// Parameter: pcl::PointCloud<pcl::Normal>::ConstPtr normals2
//************************************
pcl::visualization::PCLVisualizer::Ptr viewportsVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
	pcl::PointCloud<pcl::Normal>::ConstPtr normals1,
	pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	// --------------------------------------------------------
	// ----------------打开3D视窗，添加点云和法线--------------
	// --------------------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius:0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius:0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "normals2", v2);
	viewer->addCoordinateSystem(1.0);

	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

	return (viewer);
}