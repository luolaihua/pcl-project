#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/visualization/pcl_visualizer.h>
int
main(int argc, char** argv)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
	pcl::visualization::PCLVisualizer viewer("voxel_grid_filter");
	int v1(1);
	int v2(2);
	viewer.createViewPort(0, 0, 0.5, 1,v1);
	viewer.createViewPort(0.5, 0, 1, 1,v2);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

	// Create the filtering object
	//pcl::VoxelGrid<pcl::PointXYZ> sor;
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.02f, 0.02f, 0.02f);//设置滤波时创建的体素体积为2 cm3的立方体
	sor.filter(*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;

	pcl::PCDWriter writer;
	//writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered);
	writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered,
		Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filtered2);
	pcl::fromPCLPointCloud2(*cloud, *cloud2);

	viewer.addPointCloud(cloud2, "cloud", v1);
	viewer.addPointCloud(cloud_filtered2, "cloud_filtered", v2);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return (0);
}