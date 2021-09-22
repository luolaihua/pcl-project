#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	// �����������
	cloud_in->width = 5;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);

	//ʹ�������������
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	//��ӡ�����ĵ�������
	std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
		<< std::endl;
	for (size_t i = 0; i < cloud_in->points.size(); ++i) std::cout << "    " <<
		cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
		cloud_in->points[i].z << std::endl;

	//�����ڶ�������
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		//����ֻ����ƽ�Ʊ任�����е��x����ƽ����0.7�ľ���
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;


	std::cout << "Transformed " << cloud_in->points.size() << " data points:"
		<< std::endl;
	for (size_t i = 0; i < cloud_out->points.size(); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " <<
		cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

	//����ICP����
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//ΪICP�㷨�����������
	icp.setInputCloud(cloud_in);
	//����Ŀ�����
	icp.setInputTarget(cloud_out);
	//FinalΪ��֤�Ľ��
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	//��ӡ����֤�Ľ��
	std::cout << "Final " << cloud_in->points.size() << " data points:"
		<< std::endl;
	for (size_t i = 0; i < Final.points.size(); ++i)
		std::cout << "    " << Final.points[i].x << " " <<
		Final.points[i].y << " " << Final.points[i].z << std::endl;

	//���hasConverged() == 1,˵��������ƺ�Ŀ�����֮����Ǹ��Ա任��������ICP����������
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;

	//��ӡ��������ƺ�Ŀ�����֮���ת������
	std::cout << icp.getFinalTransformation() << std::endl;
	return (0);
}
