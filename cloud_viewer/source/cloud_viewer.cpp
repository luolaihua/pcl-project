#include<pcl/visualization/cloud_viewer.h>
#include<iostream>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
using namespace std;
void
foo()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{

	}
}

int user_data;

//�ص�����������������ע���ִֻ��һ��
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	//���ô��ڱ�����ɫ
	viewer.setBackgroundColor(1.0, 0.5, 1.0);//��ɫ
	//���һ��Բ�򼸺���
	pcl::PointXYZ o;//xyz Ϊ����λ��
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	//���Բ�򼸺�ͼ��
	viewer.addSphere(o, 0.25, "sphere", 0);
	cout << "i only run once" << endl;
}
//Ҳ�ǻص�����������������ע���ÿ֡��ʾ��ִ��һ��
//���幦�ܣ��ڿ��ӻ����������һ��ˢ����ʾ���ַ���
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}

int main()
{
	//���ص����ļ������ƶ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("maize.pcd", *cloud);

	//����viewer����
	pcl::visualization::CloudViewer viewer("My Cloud Viewer");//������ַ���Ϊ���ڵ�����
	//showCloud��ͬ���ģ��ڴ˴��ȴ�ֱ����Ⱦ��ʾΪֹ
	viewer.showCloud(cloud);
	//��ע�ắ������Ⱦ���ʱֻ����һ��
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	//��ע�ắ������Ⱦ���ʱÿ�ζ�����
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//�ڴ˴����������������
		user_data++;
	}
	return 0;
}
