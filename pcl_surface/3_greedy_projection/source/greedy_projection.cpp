//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
//
//
//int
//main (int argc, char** argv)
//{
//  // 加载PCD文件为PointXYZ类型
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::io::loadPCDFile ("table_scene_lms400_downsampled.pcd", *cloud);
//
//  //* the data should be available in cloud
//
//  // 法线估计对象
//  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//  //使用KD树进行搜索
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//  tree->setInputCloud (cloud);
//  n.setInputCloud (cloud);
//  n.setSearchMethod (tree);
//  n.setKSearch (20);//设置k搜索的k值为20
//  n.compute (*normals);//估计法线的结果存储到normals中
//  //由于本例中使用的三角化算法输入必须为有向点云，所以需要使用PCL中的法线估计算法先估计出每个点的法线
//  //* normals should not contain the point normals + surface curvatures
//
//  // 连接 XYZ字段和法线字段
//  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
//  //* cloud_with_normals = cloud + normals
//
//  // 创建搜索树KDtree
//  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//  tree2->setInputCloud (cloud_with_normals);
//
//  // Initialize objects
//  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//  pcl::PolygonMesh triangles;//存储最终的三角网格模型
//
//  // Set the maximum distance between connected points (maximum edge length)
//  // 设置连接点之间的最大距离（即为三角形最大边长）为0.025
//  // 设置个参数特征值
//  gp3.setSearchRadius (0.025);
//
//  // Set typical values for the parameters
//  // 设置被样本点搜索其临近点的最远距离为2.5，为了适应点云的密度变化
//  gp3.setMu (2.5);
//  //设置样本点可搜索的邻域个数为100
//  gp3.setMaximumNearestNeighbors (100);
//  //设置某点法线方向偏离样本点法线方向的最大角度为45度
//  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//  //设置三角化后得到三角形内角最小角度为10度
//  gp3.setMinimumAngle(M_PI/18); // 10 degrees
//  //设置三角化后得到三角形内角最大角度为120度
//  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//  //设置该参数保证法线朝向一致
//  gp3.setNormalConsistency(false);
//
//  // Get result
//  gp3.setInputCloud (cloud_with_normals);
//  gp3.setSearchMethod (tree2);
//
//  gp3.reconstruct (triangles);//重建提取三角化
// // std::cout << triangles;
//  // 附加顶点信息
//  std::vector<int> parts = gp3.getPartIDs();
//  std::vector<int> states = gp3.getPointStates();
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  viewer->addPolygonMesh(triangles,"my");
// 
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();
//   // 主循环
//  while (!viewer->wasStopped ())
//  {
//    viewer->spinOnce (100);
//    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//  }
//  return (0);
//}
