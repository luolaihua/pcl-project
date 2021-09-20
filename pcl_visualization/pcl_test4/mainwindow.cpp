#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include<qdebug.h>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // Setup the cloud pointer
    cloud.reset (new PointCloudT);
    //如果打开点云失败
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:\\PCLProject\\cloud_sampling\\cmake_bin\\Debug\\bunny.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        std::cout << "Loaded ";
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    //viewer->setBackgroundColor(255,255,255);
    ui->qvtkWidget->update ();
    viewer->addPointCloud (cloud, "cloud");
    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}

MainWindow::~MainWindow()
{
    delete ui;
}

