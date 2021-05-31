#include<pcl/point_cloud.h>
#include<pcl/octree/octree_search.h>

#include<iostream>
#include<vector>
#include<ctime>

using namespace std;

int
main(int argc, char** argv)
{
	//使用系统时间做随机数种子
	srand((unsigned int)time(NULL));
	//创建一个PointXYZ类型点云指针
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//初始化点云数据
	cloud->width = 1000;//宽为1000
	cloud->height = 1;//高为1，说明为无序点云
	cloud->points.resize(cloud->width * cloud->height);
	//使用随机数填充数据
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		//PointCloud类中对[]操作符进行了重载，返回的是对points的引用
		// (*cloud)[i].x 等同于 cloud->points[i].x
		(*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);//推进写法
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);//推进写法
	}

	//创建八叉树对象
	float resolution = 128.0f;//设置分辨率
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);


	//设置点云输入,将在cloud中搜索
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	//设置被搜索点,用随机数填充
	pcl::PointXYZ searchPoint;
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	//体素内近邻搜索
	//使用vector存储搜索结果下标
	vector<int> pointIdxVec;//保存下标
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

	//开始k最近邻搜索
	int K = 10;
	//使用两个vector存储搜索结果
	vector<int> pointIdxNKNSearch(K);//保存下标
	vector<float> pointNKNSquaredDistance(K);//保存距离的平方

	cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K = " << K << endl;



	/**
	 * 假设我们的KdTree返回超过0个最近的邻居，
	 * 然后它打印出所有10个离随机searchPoint最近的邻居的位置，
	 * 这些都存储在我们之前创建的vector中。
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

	//基于半径的邻域搜索
	//搜索结果保存在两个数组中，一个是下标，一个是距离
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	//设置搜索半径，随机值
	float radius = 256.0f* rand() / (RAND_MAX + 1.0f);

	cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << endl;

	/**
	 * 如果我们的KdTree在指定的半径内返回超过0个邻居，它将打印出这些存储在向量中的点的坐标。
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