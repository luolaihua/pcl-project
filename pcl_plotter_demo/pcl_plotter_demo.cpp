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
//-----------------------����Y= f(X)�Ļص�����-------------------
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
	//����һ��plottter��
	pcl::visualization::PCLPlotter * plotter = new pcl::visualization::PCLPlotter();

	//����һЩ����
	plotter->setShowLegend(true);

	//���ɶ�Ӧ��
	int numPoints = 69;
	double ax[100], acos[100], asin[100];
	generateData(ax, acos, asin, numPoints);

	//��ӻ��Ƶ�����
	plotter->addPlotData(ax, acos, numPoints, "cos");
	plotter->addPlotData(ax, asin, numPoints, "sin");

	//��ʾ3��
	plotter->spinOnce(3000);
	plotter->clearPlots();

	//-------------������ʽ�������Զ��庯��-------------------
	//����Y������
	plotter->setYRange(-10, 10);

	//�������ʽ
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

	//һ���򵥵Ķ���
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