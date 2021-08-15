#include<vector>
#include<iostream>
#include<utility>
#include<cmath>
#include<pcl/visualization/pcl_plotter.h>

using namespace pcl::visualization;

void
generateData(double *ax, double *acos, double *asin, int numPoints)
{
	double inc = 7.5 / (numPoints - 1);
	for (int i = 0; i < numPoints; ++i)
	{
		ax[i] = i * inc;
		acos[i] = std::cos(i + inc);
		asin[i] = sin(i + inc);
	}
}
//-----------------------定义Y= f(X)的回调函数-------------------
double
step(double val)
{
	if (val > 0)
		return (double)(int)val;
	else
		return (double)((int)val - 1);
}

double
identity(double val)
{
	return val;
}

int
main(int argc, char** argv)
{
	//定义一个plottter类
	pcl::visualization::PCLPlotter * plotter = new pcl::visualization::PCLPlotter();

	//设置一些属性
	plotter->setShowLegend(true);

	//生成对应点
	int numPoints = 69;
	double ax[100], acos[100], asin[100];
	generateData(ax, acos, asin, numPoints);

	//添加绘制的数据
	plotter->addPlotData(ax, acos, numPoints, "cos");
	plotter->addPlotData(ax, asin, numPoints, "sin");

	//显示3秒
	plotter->spinOnce(3000);
	plotter->clearPlots();

	//-------------绘制隐式函数和自定义函数-------------------
	//设置Y轴区域
	plotter->setYRange(-10, 10);

	//定义多项式
	std::vector<double> func1(1, 0);
	func1[0] = 1;//y=1
	std::vector<double> func2(3, 0);
	func2[2] = 1;//y=x^2

	plotter->addPlotData(std::make_pair(func1, func2), -10, 10, "y = 1/x^2", 100, vtkChart::POINTS);
	plotter->spinOnce(2000);

	plotter->addPlotData(func2, -10, 10, "y = x^2");
	plotter->spinOnce(2000);

	//callbacks
	plotter->addPlotData(identity, -10, 10, "identity");
	plotter->spinOnce(2000);

	plotter->addPlotData(std::abs, -10, 10, "abs");
	plotter->spinOnce(2000);

	plotter->addPlotData(step, -10, 10, "step", 100, vtkChart::POINTS);
	plotter->spinOnce(2000);

	plotter->clearPlots();

	//一个简单的动画
	std::vector<double> fsq(3, 0);
	fsq[2] = -100;//y=x^2;
	while (!plotter->wasStopped())
	{
		if (fsq[2] == 100) fsq[2] = -100;
		fsq[2]++;
		char str[50];
		//sprintf(str, "y=%dx^2", (int)fsq[2]);
		plotter->addPlotData(fsq, -10, 10, str);

		plotter->spinOnce(100);
		plotter->clearPlots();
	}
	return 0;

}