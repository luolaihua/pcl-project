#include<vector>
#include<iostream>
#include<utility>
#include<pcl/visualization/pcl_plotter.h>

int
main(int argc, char** argv)
{
	//����һ��plottter��
	pcl::visualization::PCLPlotter * plotter = new pcl::visualization::PCLPlotter();

	////����һ������ʽ����
	////y=x^2,x^2���±�Ϊ1��������Ϊ0
	//// 0 0 1
	//std::vector<double> func1(3, 0);
	//func1[2] = 1;
	////������ʽ��ӽ�plottter��
	////X ������䷶Χ��[-10, 10] as the range in X axis and "y = x^2" as title
	//plotter->addPlotData(func1, -10, 10, "y=x^2");

	
	plotter->addPlotData(argv[1]);
	//��ʾͼ�����
	plotter->plot();
	return 0;

}