#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
int
 main (int argc, char** argv)
{ srand(time(0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  //�����������
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = rand () / (RAND_MAX + 1.0f)-0.5;
    cloud->points[i].y = rand () / (RAND_MAX + 1.0f)-0.5;
    cloud->points[i].z = rand () / (RAND_MAX + 1.0f)-0.5;
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;
  // �����˲�������
  pcl::PassThrough<pcl::PointXYZ> pass;//�����˲�������
  pass.setInputCloud (cloud);			//���ô��˲��ĵ���
  pass.setFilterFieldName ("z");		//������Z�᷽���Ͻ����˲�
  pass.setFilterLimits (0.0, 1.0);		//�����˲���ΧΪ0~1,�ڷ�Χ֮��ĵ�ᱻ����
  //pass.setFilterLimitsNegative (true);//�Ƿ�����ˣ�Ĭ��Ϊfalse
  pass.filter (*cloud_filtered);		//��ʼ����

  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;
  return (0);
}
