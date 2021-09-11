#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<pcl/visualization/pcl_visualizer.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	// �����������
	pcl::PCDReader reader;
	// ��·����Ϊ�Լ�����ļ���·��
	reader.read<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud);
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	// �����˲�������
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);//���ô��˲��ĵ���
	sor.setMeanK(50);//�����ڽ���ͳ��ʱ���ǲ�ѯ���ھӵ���
	sor.setStddevMulThresh(1.0);//�����ж��Ƿ�Ϊ��Ⱥ�����ֵ
	sor.filter(*cloud_filtered);//���˲����������cloud_filtered��
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
	sor.setNegative(true);


	pcl::visualization::PCLVisualizer viewer("statistic removal");
	int v1(1);
	int v2(2);
	//�����Ӵ�
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);

	viewer.addPointCloud(cloud, "cloud", v1);
	viewer.addPointCloud(cloud_filtered, "cloud_filtered", v2);

	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}
