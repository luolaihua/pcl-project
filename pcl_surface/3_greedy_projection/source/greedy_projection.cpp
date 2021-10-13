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
//  // ����PCD�ļ�ΪPointXYZ����
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::io::loadPCDFile ("table_scene_lms400_downsampled.pcd", *cloud);
//
//  //* the data should be available in cloud
//
//  // ���߹��ƶ���
//  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//  //ʹ��KD����������
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//  tree->setInputCloud (cloud);
//  n.setInputCloud (cloud);
//  n.setSearchMethod (tree);
//  n.setKSearch (20);//����k������kֵΪ20
//  n.compute (*normals);//���Ʒ��ߵĽ���洢��normals��
//  //���ڱ�����ʹ�õ����ǻ��㷨�������Ϊ������ƣ�������Ҫʹ��PCL�еķ��߹����㷨�ȹ��Ƴ�ÿ����ķ���
//  //* normals should not contain the point normals + surface curvatures
//
//  // ���� XYZ�ֶκͷ����ֶ�
//  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
//  //* cloud_with_normals = cloud + normals
//
//  // ����������KDtree
//  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//  tree2->setInputCloud (cloud_with_normals);
//
//  // Initialize objects
//  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//  pcl::PolygonMesh triangles;//�洢���յ���������ģ��
//
//  // Set the maximum distance between connected points (maximum edge length)
//  // �������ӵ�֮��������루��Ϊ���������߳���Ϊ0.025
//  // ���ø���������ֵ
//  gp3.setSearchRadius (0.025);
//
//  // Set typical values for the parameters
//  // ���ñ��������������ٽ������Զ����Ϊ2.5��Ϊ����Ӧ���Ƶ��ܶȱ仯
//  gp3.setMu (2.5);
//  //������������������������Ϊ100
//  gp3.setMaximumNearestNeighbors (100);
//  //����ĳ�㷨�߷���ƫ�������㷨�߷�������Ƕ�Ϊ45��
//  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//  //�������ǻ���õ��������ڽ���С�Ƕ�Ϊ10��
//  gp3.setMinimumAngle(M_PI/18); // 10 degrees
//  //�������ǻ���õ��������ڽ����Ƕ�Ϊ120��
//  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//  //���øò�����֤���߳���һ��
//  gp3.setNormalConsistency(false);
//
//  // Get result
//  gp3.setInputCloud (cloud_with_normals);
//  gp3.setSearchMethod (tree2);
//
//  gp3.reconstruct (triangles);//�ؽ���ȡ���ǻ�
// // std::cout << triangles;
//  // ���Ӷ�����Ϣ
//  std::vector<int> parts = gp3.getPartIDs();
//  std::vector<int> states = gp3.getPointStates();
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  viewer->addPolygonMesh(triangles,"my");
// 
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();
//   // ��ѭ��
//  while (!viewer->wasStopped ())
//  {
//    viewer->spinOnce (100);
//    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//  }
//  return (0);
//}
