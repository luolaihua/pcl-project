#include<pcl/point_cloud.h>
#include<pcl/octree/octree_search.h>

#include<iostream>
#include<vector>
#include<ctime>

using namespace std;

int
main(int argc, char** argv)
{
	//ʹ��ϵͳʱ�������������
	srand((unsigned int)time(NULL));
	//����һ��PointXYZ���͵���ָ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//��ʼ����������
	cloud->width = 1000;//��Ϊ1000
	cloud->height = 1;//��Ϊ1��˵��Ϊ�������
	cloud->points.resize(cloud->width * cloud->height);
	//ʹ��������������
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		//PointCloud���ж�[]���������������أ����ص��Ƕ�points������
		// (*cloud)[i].x ��ͬ�� cloud->points[i].x
		(*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);//�ƽ�д��
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);//�ƽ�д��
	}

	//�����˲�������
	float resolution = 128.0f;//���÷ֱ���
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);


	//���õ�������,����cloud������
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	//���ñ�������,����������
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	//�����ڽ�������
	//ʹ��vector�洢��������±�
	vector<int> pointIdxVec;//�����±�
	if (octree.voxelSearch(searchPoint, pointIdxVec))
	{
		cout << "Neighbors within voxel search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z
			<< ")"
			<< endl;
		for (size_t i = 0; i < pointIdxVec.size(); ++i)
		{
			cout << "    " << cloud->points[pointIdxVec[i]].x
				<< " " << cloud->points[pointIdxVec[i]].x
				<< " " << cloud->points[pointIdxVec[i]].z
				<< endl;
		}
	}

	//��ʼk���������
	int K = 10;
	//ʹ������vector�洢�������
	vector<int> pointIdxNKNSearch(K);//�����±�
	vector<float> pointNKNSquaredDistance(K);//��������ƽ��

	cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K = " << K << endl;



	/**
	 * �������ǵ�KdTree���س���0��������ھӣ�
	 * Ȼ������ӡ������10�������searchPoint������ھӵ�λ�ã�
	 * ��Щ���洢������֮ǰ������vector�С�
	 */
	if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
		{
			cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
				<< " " << cloud->points[pointIdxNKNSearch[i]].x
				<< " " << cloud->points[pointIdxNKNSearch[i]].z
				<< "( squared distance: " << pointNKNSquaredDistance[i] << " )" << endl;
		}
	}

	//���ڰ뾶����������
	//����������������������У�һ�����±꣬һ���Ǿ���
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	//���������뾶�����ֵ
	float radius = 256.0f* rand() / (RAND_MAX + 1.0f);

	cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << endl;

	/**
	 * ������ǵ�KdTree��ָ���İ뾶�ڷ��س���0���ھӣ�������ӡ����Щ�洢�������еĵ�����ꡣ
	 */
	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
		{
			cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
				<< " " << cloud->points[pointIdxRadiusSearch[i]].x
				<< " " << cloud->points[pointIdxRadiusSearch[i]].z
				<< "( squared distance: " << pointRadiusSquaredDistance[i] << " )" << endl;
		}
	}

	return 0;
}