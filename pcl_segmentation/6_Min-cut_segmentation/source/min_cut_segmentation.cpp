#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
using namespace pcl::console;
int main (int argc, char** argv)
{
	if(argc<2)
	{
		std::cout<<".exe xx.pcd -bc 1 -fc 1.0 -nc 0 -cx 68.97 -cy -18.55 -cz 0.57 -s 0.25 -r 3.0433856 -non 14 -sw 0.5"<<endl;

		return 0;
	}//����������С��1���������ʾ��Ϣ
	time_t start,end,diff[5],option;
	start = time(0); 
	int NumberOfNeighbours=14;//����Ĭ�ϲ���
	bool Bool_Cuting=false;//����Ĭ�ϲ���
	float far_cuting=1,near_cuting=0,C_x=0.071753,C_y= -0.309913,C_z=1.603000,Sigma=0.25,Radius=0.8,SourceWeight=0.5;//����Ĭ���������
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);// ����һ��PointCloud <pcl::PointXYZRGB>����ָ�벢����ʵ����
	if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1 )// ���ص�������
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	parse_argument (argc, argv, "-bc", Bool_Cuting);
	parse_argument (argc, argv, "-fc", far_cuting);
	parse_argument (argc, argv, "-nc", near_cuting);

	parse_argument (argc, argv, "-cx", C_x);
	parse_argument (argc, argv, "-cy", C_y);
	parse_argument (argc, argv, "-cz", C_z);

	parse_argument (argc, argv, "-s", Sigma);
	parse_argument (argc, argv, "-r", Radius);
	parse_argument (argc, argv, "-non", NumberOfNeighbours);
	parse_argument (argc, argv, "-sw", SourceWeight);//�������������ʽ

	pcl::IndicesPtr indices (new std::vector <int>);//����һ������
	if(Bool_Cuting)//�ж��Ƿ���Ҫֱͨ�˲�
	{
		pcl::PassThrough<pcl::PointXYZ> pass;//����ֱͨ�˲�������
		pass.setInputCloud (cloud);//�����������
		pass.setFilterFieldName ("z");//����ָ�����˵�ά��
		pass.setFilterLimits (near_cuting, far_cuting);//����ָ��γ�ȹ��˵ķ�Χ
		pass.filter (*indices);//ִ���˲��������˲����������������

	}
	pcl::MinCutSegmentation<pcl::PointXYZ> seg;//����һ��PointXYZRGB���͵����������ָ����
	seg.setInputCloud (cloud);//�����������
	if(Bool_Cuting)seg.setIndices (indices);//����������Ƶ�����

	pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());//����һ��PointCloud <pcl::PointXYZRGB>����ָ�벢����ʵ����
	pcl::PointXYZ point;//�������ĵ㲢��ֵ

	point.x =C_x;
	point.y = C_y;
	point.z = C_z;
	foreground_points->points.push_back(point);
	seg.setForegroundPoints (foreground_points);//����ǰ�����ƣ�Ŀ�����壩�����ĵ�

	seg.setSigma (Sigma);//����ƽ���ɱ���Sigmaֵ
	seg.setRadius (Radius);//���ñ����ͷ�Ȩ�صİ뾶
	seg.setNumberOfNeighbours (NumberOfNeighbours);//�����ٽ�����Ŀ
	seg.setSourceWeight (SourceWeight);//����ǰ���ͷ�Ȩ��

	std::vector <pcl::PointIndices> clusters;
	seg.extract (clusters);//��ȡ�ָ�Ľ�����ָ��������ڵ��������������С�

	std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;//���㲢����ָ��ڼ������������ֵ

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();//��ǰ���㸳���ɫ���Ա����㸳���ɫ��
	pcl::visualization::PCLVisualizer viewer ("���ƿ�PCLѧϰ�̵̳ڶ���-��С��ָ��");
	viewer.addPointCloud(colored_cloud);
	viewer.addSphere(point,Radius,122,122,0,"sphere");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.2,"sphere");
	
	while (!viewer.wasStopped ())
	{
		viewer.spin();
	}//���п��ӻ�

	return (0);
}