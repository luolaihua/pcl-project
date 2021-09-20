#include "mainwindow.h"
#include <iostream>
#include <QApplication>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "vtkoutputwindow.h"//加入
#include"vtkFileOutputWindow.h"

//vtkOutputWindow::SetGlobalWarningDisplay(0);
int main(int argc, char *argv[])
{
    //vtkOutputWindow::SetGlobalWarningDisplay(0);     //加入
    QString logPath = QString("%1/log/Error.txt").arg(QCoreApplication::applicationDirPath());   vtkSmartPointer<vtkFileOutputWindow> fileOutputWindow = vtkSmartPointer<vtkFileOutputWindow>::New(); fileOutputWindow->SetFileName(qPrintable(logPath)); vtkOutputWindow::SetInstance(fileOutputWindow);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
