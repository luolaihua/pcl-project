#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include<pcl/visualization/pcl_visualizer.h>
/*
	��ƽ��ģ���Ϲ���͹�����
*/
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;

	reader.read("table_scene_mug_stereo_textured.pcd", *cloud);
	// ʹ��ֱͨ�˲������˳�z���ϵ��ӵ�
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.1);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: "
		<< cloud_filtered->points.size() << " data points." << std::endl;


	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// �����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// ��ѡ��,�����Ż�ϵ��
	seg.setOptimizeCoefficients(true);
	// ����Ĳ���
	seg.setModelType(pcl::SACMODEL_PLANE);//ƽ��ģ��
	seg.setMethodType(pcl::SAC_RANSAC);//���ò���һ���Թ��Ʒ���ģ��ΪSAC_RANSAC
	seg.setDistanceThreshold(0.01);//���þ�����ֵΪ0.01�������ƽ��ģ�;���С��0.01cm�ĵ㶼Ϊ�ڵ�inliners

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);//��ʼ�ָ���������inliers��coefficients����
	std::cerr << "PointCloud after segmentation has: "
		<< inliers->indices.size() << " inliers." << std::endl;

	// ��������ͶӰ�˲�
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);//ͶӰ�˲�ģ��Ϊƽ��SACMODEL_PLANE
	proj.setInputCloud(cloud_filtered);
	proj.setModelCoefficients(coefficients);//���ָ���Ƶõ��Ĳ�������coefficients����ΪͶӰƽ��ģ��ϵ��
	proj.filter(*cloud_projected);
	std::cerr << "PointCloud after projection has: "
		<< cloud_projected->points.size() << " data points." << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	//�����������ȡ����
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud_projected);
	chull.setAlpha(0.1); //����alphaֵΪ0.1
	chull.reconstruct(*cloud_hull);//�ؽ���ȡ�����������

	std::cerr << "Concave hull has: " << cloud_hull->points.size()
		<< " data points." << std::endl;

	pcl::PCDWriter writer;
	writer.write("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

	//��ʾ
	pcl::visualization::PCLVisualizer viewer("Restructing");
	int v1(1);
	int v2(2);
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.addPointCloud(cloud_projected, "v1", v1);
	viewer.addPointCloud(cloud_hull, "v2", v2);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return (0);
}