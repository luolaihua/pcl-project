#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
using namespace std;

int 
main(int argc, char** argv) 
{
	//判断输入参数个数是否正确，默认为1，如果输入一个，则为2
	if (argc != 2)
	{
		cerr << "please specify command line arg '-f' or '-p'" << endl;
		exit(0);
	}
	//创建点云类型存储
	// 点云数据拼接：点云a + 点云b = 点云c
	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
	
	// 点云字段拼接： PointXYZ点云a + Normal 点云b = PointNormal 点云c
	pcl::PointCloud<pcl::Normal> n_cloud_b;
	pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

	//通过输入参数判断是点云数据拼接还是字段拼接
	bool isCatField = strcmp(argv[1],"-f") == 0;//默认是数据拼接,-p

	//先创建点云数据
	//创建cloud_a
	cloud_a.width = 5;//宽为5，在无序点云中，宽即为点云个数
	cloud_a.height = cloud_b.height = n_cloud_b.height = 1;//高度为1，说明为无序点云
	cloud_a.points.resize(cloud_a.width*cloud_a.height);// 重置点云个数
	//使用随机数填充点云a
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
	{
		cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	//先把点云a打印出来：
	cout << "Cloud A:" << endl;
	for (size_t i = 0; i < cloud_a.points.size(); ++i) {
		cout << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << endl;
	}
	//如果是字段拼接
	if (isCatField) {
		//先初始化Normal类型点云n_cloud_b;
		//字段拼接的话，要确保两个点云个数相同
		n_cloud_b.width = cloud_a.width;//等于cloud_a的个数
		n_cloud_b.points.resize(n_cloud_b.width*n_cloud_b.height);
		for (size_t i = 0; i < n_cloud_b.points.size(); ++i) {
			n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);;
			n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);;
			n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);;
		}
		//先把点云b打印出来：
		cout << "Cloud B:" << endl;
		for (size_t i = 0; i < n_cloud_b.points.size(); ++i) {
			cout << "    " << n_cloud_b.points[i].normal[0] << " " << n_cloud_b.points[i].normal[1] << " " << n_cloud_b.points[i].normal[2] << endl;
		}

		//使用字段连接函数concatenateFields
		pcl::concatenateFields(cloud_a, n_cloud_b, p_n_cloud_c);

		//把拼接的点云输出
		cout << "Cloud C:（字段拼接）" << endl;
		for (size_t i = 0; i < p_n_cloud_c.points.size(); ++i) {
			cout << "    "<<p_n_cloud_c.points[i].x<<" "<<p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " " 
				<< p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << " " << endl;
		}
	}
	//如果是点云数据拼接
	else {
		//先初始化cloud_b点云
		//数据拼接，点数可以不一样，但是字段要一致
		cloud_b.width = 3;
		cloud_b.points.resize(cloud_b.width*cloud_b.height);
		for (size_t i = 0; i < cloud_b.points.size(); ++i)
		{
			cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		}
		//先把点云b打印出来：
		cout << "Cloud B:" << endl;
		for (size_t i = 0; i < cloud_b.points.size(); ++i) {
			cout << "    " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " "<<  cloud_b.points[i].z << endl;
		}

		//数据拼接,直接赋值，做加法
		//cloud_c = cloud_a;
		//cloud_c += cloud_b;
		//cloud_c = cloud_b + cloud_a;
		cloud_c = cloud_a + cloud_b;
		//把结果打印出来
		cout << "Cloud C:（数据拼接）" << endl;
		for (size_t i = 0; i < cloud_c.points.size(); ++i) {
			cout << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << endl;
		}
		
	}
	return(0);

}