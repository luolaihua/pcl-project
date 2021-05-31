#include<iostream>
#include<pcl/io/pcd_io.h>//PCD读写类相关的头文件
#include<pcl/point_types.h>//点云类型头文件
int 
main(int argc,char** argv) {
	std::cout << "hello" << std::endl;
	//typedef boost::shared_ptr<PointCloud<PointT> > Ptr;
	//Ptr 是一个共享指针，使用pcl::PointXYZ类型来保存点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ>);
	//如果打开点云失败
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return (1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		std::cout << "    " << cloud->points[i].x
			<< " " << cloud->points[i].y
			<< " " << cloud->points[i].z << std::endl;
	}
	return (0);
}