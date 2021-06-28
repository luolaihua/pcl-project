#include<vector>
#include<iostream>
#include<utility>
#include<pcl/visualization/pcl_plotter.h>

int
main(int argc, char** argv)
{
	//定义一个plottter类
	pcl::visualization::PCLPlotter * plotter = new pcl::visualization::PCLPlotter();

	////定义一个多项式函数
	////y=x^2,x^2的下标为1，常数项为0
	//// 0 0 1
	//std::vector<double> func1(3, 0);
	//func1[2] = 1;
	////将多项式添加进plottter中
	////X 轴的区间范围：[-10, 10] as the range in X axis and "y = x^2" as title
	//plotter->addPlotData(func1, -10, 10, "y=x^2");

	
	plotter->addPlotData(argv[1]);
	//显示图表，完成
	plotter->plot();
	return 0;

}