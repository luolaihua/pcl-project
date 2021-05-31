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

//回调函数，在主函数中注册后只执行一次
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	//设置窗口背景颜色
	viewer.setBackgroundColor(1.0, 0.5, 1.0);//粉色
	//添加一个圆球几何体
	pcl::PointXYZ o;//xyz 为坐标位置
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	//添加圆球几何图像
	viewer.addSphere(o, 0.25, "sphere", 0);
	cout << "i only run once" << endl;
}
//也是回调函数，在主函数中注册后，每帧显示都执行一次
//具体功能：在可视化对象中添加一个刷新显示的字符串
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
	//加载点云文件到点云对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("maize.pcd", *cloud);

	//创建viewer对象
	pcl::visualization::CloudViewer viewer("My Cloud Viewer");//传入的字符串为窗口的名字
	//showCloud是同步的，在此处等待直到渲染显示为止
	viewer.showCloud(cloud);
	//该注册函数在渲染输出时只调用一次
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	//该注册函数在渲染输出时每次都调用
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//在此处可以添加其他处理
		user_data++;
	}
	return 0;
}
