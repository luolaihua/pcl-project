#include<pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/point_cloud_color_handlers.h>
#include<pcl/keypoints/uniform_sampling.h>
#include<pcl/surface/mls.h>
#include<iostream>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


using namespace std;

//�²��� down_sampling
void down_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr dest_cloud)
{
	//�����˲�����
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	//������Ҫ���˵ĵ���
	filter.setInputCloud(source_cloud);

	//�����˲�ʱ�������������Ϊ1cm��������
	//��������դ��Ĵ�С��1*1*1 cm
	filter.setLeafSize(0.01f, 0.01f, 0.01f);
	//ִ���˲���������������� 
	filter.filter(*dest_cloud);
}

//���Ȳ��� uniform_sampling
void uniform_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr dest_cloud)
{
	//�����˲�����
	pcl::UniformSampling<pcl::PointXYZ> filter;
	//������Ҫ���˵ĵ���
	filter.setInputCloud(source_cloud);

	//�����˲�ʱ�������������Ϊ1cm��������
	filter.setRadiusSearch(0.01f);
	//ִ���˲���������������� 
	filter.filter(*dest_cloud);
}

//�����������ؽ�
void up_sampling(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr dest_cloud)
{
	//�˲�����
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
	filter.setInputCloud(source_cloud);
	//������������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	filter.setSearchMethod(kdtree);
	//����������İ뾶Ϊ3cm
	filter.setSearchRadius(0.03);
	//Up sampling ���������У�DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
	filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
	//�����İ뾶��
	filter.setUpsamplingRadius(0.03);
	//�����Ĳ�����С
	filter.setUpsamplingStepSize(0.02);

	filter.process(*dest_cloud);

}

int main(int argc, char** argv)
{
	//���ص����ļ������ƶ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr down_sampling_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_sampling_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr up_sampling_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(argc < 2)
	{
		pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProject\\cloud_sampling\\cmake_bin\\Debug\\bunny.pcd", *cloud);
	}
	else
	{
		if (pcl::io::loadPCDFile <pcl::PointXYZ>(argv[1], *cloud) == -1)
		{
			std::cout << "Cloud reading failed." << std::endl;
			return (-1);
		}// ���������������
	}
	//�²���
	down_sampling(cloud, down_sampling_cloud);
	//���Ȳ���
	uniform_sampling(cloud, uniform_sampling_cloud);
	//��������
	up_sampling(cloud, up_sampling_cloud);

	//==================��ʾ����
	//����viewer����
	pcl::visualization::PCLVisualizer viewer("Cloud Sampling Viewer");//������ַ���Ϊ���ڵ�����
	
	int v1(1);
	int v2(2);
	int v3(3);
	int v4(4);
	viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v1);
	viewer.setBackgroundColor(0, 0, 0, v1);

	viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);

	viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v3);
	viewer.setBackgroundColor(0, 0, 0, v3);

	viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v4);
	viewer.setBackgroundColor(0, 0, 0, v4);

	// The color we will be using

	float bckgr_gray_level = 0.0;  // Black    ���ñ�����ɫ
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud, cloud_in_color_h, "cloud_in_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_green(down_sampling_cloud, 0, 255, 0);      // ��ʾ��ɫ����
	viewer.addPointCloud(down_sampling_cloud, cloud_out_green, "down_sampling_cloud", v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_orage(uniform_sampling_cloud, 255, 0, 0);     //��ʾ��ɫ����
	viewer.addPointCloud(uniform_sampling_cloud, cloud_out_orage, "uniform_sampling_cloud", v3);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_purple(up_sampling_cloud, 255, 0, 255);
	viewer.addPointCloud(up_sampling_cloud, cloud_out_purple, "up_sampling_cloud", v4);

	//��ӡ������Ϣ
	cout << "ԭʼ������Ϣ��" << endl;
	cout << "width = " << cloud->width << " , height = " << cloud->height << endl;
	cout << "�²��������" << endl;
	cout << "width = " << down_sampling_cloud->width << " , height = " << down_sampling_cloud->height << endl;
	cout << "���Ȳ��������" << endl;
	cout << "width = " << uniform_sampling_cloud->width << " , height = " << uniform_sampling_cloud->height << endl;
	cout << "�����������" << endl;
	cout << "width = " << up_sampling_cloud->width << " , height = " << up_sampling_cloud->height << endl;

	viewer.spin();
	//while (!viewer.wasStopped())
	//{
	//	//�ڴ˴����������������
	//	viewer.spinOnce();
	//	
	//} 

	// ����������ʾ
	pcl::visualization::PCLVisualizer origin_viewer("origin_viewer");
	origin_viewer.addPointCloud(cloud);
	origin_viewer.spin();

	pcl::visualization::PCLVisualizer viewer_down_samp("down_samping_viewer");
	viewer_down_samp.addPointCloud(down_sampling_cloud, cloud_out_green);
	viewer_down_samp.spin();

	pcl::visualization::PCLVisualizer viewer_uniform_samp("uniform_samping_viewer");
	viewer_uniform_samp.addPointCloud(uniform_sampling_cloud, cloud_out_orage);
	viewer_uniform_samp.spin();

	pcl::visualization::PCLVisualizer viewer_up_samp("up_samping_viewer");
	viewer_up_samp.addPointCloud(up_sampling_cloud, cloud_out_purple);
	viewer_up_samp.spin();

	return 0;
}
