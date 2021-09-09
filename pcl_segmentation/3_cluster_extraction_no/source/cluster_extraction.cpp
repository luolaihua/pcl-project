#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include<pcl/visualization/pcl_visualizer.h>
bool isPushSpace = false;
//�����¼�
void keyboard_event_occurred(const pcl::visualization::KeyboardEvent& event, void * nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
	{
		isPushSpace = true;
	}
}
int
main(int argc, char** argv)
{
	// ��PCD�ļ��ж�ȡ��������
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("table_scene_lms400.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	pcl::visualization::PCLVisualizer viewer("Cluster Extraction");
	// ע������¼�
	viewer.registerKeyboardCallback(&keyboard_event_occurred, (void*)NULL);
	int v1(1);
	int v2(2);
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);

	//�����˲�����: ʹ���²�����Ҷ�ӵĴ�СΪ 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*

	viewer.addPointCloud(cloud, "cloud1", v1);
	viewer.addPointCloud(cloud_filtered, "cloud2", v2);
	//��Ⱦ10���ټ���
	viewer.spinOnce(10000);

	// ����ƽ��ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	// �ѵ��������е�ƽ��ȫ�����˵����ظ����ˣ�ֱ����������С��ԭ����0.3��
	int i = 0, nr_points = (int)cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{	
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Write the planar inliers to disk
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);

		//������ʾ����
		viewer.updatePointCloud(cloud_filtered, "cloud1");
		viewer.updatePointCloud(cloud_f, "cloud2");
		//��Ⱦ3���ټ���
		viewer.spinOnce(3000);

		cloud_filtered = cloud_f;

	}

	viewer.removePointCloud("cloud2", v2);

	// ����KdTreee������Ϊ��������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	//�����ȡ���������һ�������У�������ÿ��Ԫ�ش����ȡ��һ��������Ƶ��±�
	ec.extract(cluster_indices);

	//������ȡ�����������ʾ������
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		//������ʱ���������ĵ���
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		//ͨ���±꣬������
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*

		//���õ�������
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "��ǰ���� "<<j<<" �����ĵ�������: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
		j++;

		//��ʾ,������ò�ͬ��ɫ�������ֲ�ͬ�ľ���
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color(cloud_cluster, rand()*100 + j * 80, rand() * 50 + j * 90, rand() * 200 + j * 100);
		viewer.addPointCloud(cloud_cluster,cluster_color, ss.str(), v2);
		viewer.spinOnce(5000);
	}
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return (0);
}
