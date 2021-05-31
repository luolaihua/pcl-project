#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
using namespace std;

int 
main(int argc, char** argv) 
{
	//�ж�������������Ƿ���ȷ��Ĭ��Ϊ1���������һ������Ϊ2
	if (argc != 2)
	{
		cerr << "please specify command line arg '-f' or '-p'" << endl;
		exit(0);
	}
	//�����������ʹ洢
	// ��������ƴ�ӣ�����a + ����b = ����c
	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
	
	// �����ֶ�ƴ�ӣ� PointXYZ����a + Normal ����b = PointNormal ����c
	pcl::PointCloud<pcl::Normal> n_cloud_b;
	pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

	//ͨ����������ж��ǵ�������ƴ�ӻ����ֶ�ƴ��
	bool isCatField = strcmp(argv[1],"-f") == 0;//Ĭ��������ƴ��,-p

	//�ȴ�����������
	//����cloud_a
	cloud_a.width = 5;//��Ϊ5������������У���Ϊ���Ƹ���
	cloud_a.height = cloud_b.height = n_cloud_b.height = 1;//�߶�Ϊ1��˵��Ϊ�������
	cloud_a.points.resize(cloud_a.width*cloud_a.height);// ���õ��Ƹ���
	//ʹ�������������a
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
	{
		cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	//�Ȱѵ���a��ӡ������
	cout << "Cloud A:" << endl;
	for (size_t i = 0; i < cloud_a.points.size(); ++i) {
		cout << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << endl;
	}
	//������ֶ�ƴ��
	if (isCatField) {
		//�ȳ�ʼ��Normal���͵���n_cloud_b;
		//�ֶ�ƴ�ӵĻ���Ҫȷ���������Ƹ�����ͬ
		n_cloud_b.width = cloud_a.width;//����cloud_a�ĸ���
		n_cloud_b.points.resize(n_cloud_b.width*n_cloud_b.height);
		for (size_t i = 0; i < n_cloud_b.points.size(); ++i) {
			n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);;
			n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);;
			n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);;
		}
		//�Ȱѵ���b��ӡ������
		cout << "Cloud B:" << endl;
		for (size_t i = 0; i < n_cloud_b.points.size(); ++i) {
			cout << "    " << n_cloud_b.points[i].normal[0] << " " << n_cloud_b.points[i].normal[1] << " " << n_cloud_b.points[i].normal[2] << endl;
		}

		//ʹ���ֶ����Ӻ���concatenateFields
		pcl::concatenateFields(cloud_a, n_cloud_b, p_n_cloud_c);

		//��ƴ�ӵĵ������
		cout << "Cloud C:���ֶ�ƴ�ӣ�" << endl;
		for (size_t i = 0; i < p_n_cloud_c.points.size(); ++i) {
			cout << "    "<<p_n_cloud_c.points[i].x<<" "<<p_n_cloud_c.points[i].y << " " << p_n_cloud_c.points[i].z << " " 
				<< p_n_cloud_c.points[i].normal[0] << " " << p_n_cloud_c.points[i].normal[1] << " " << p_n_cloud_c.points[i].normal[2] << " " << endl;
		}
	}
	//����ǵ�������ƴ��
	else {
		//�ȳ�ʼ��cloud_b����
		//����ƴ�ӣ��������Բ�һ���������ֶ�Ҫһ��
		cloud_b.width = 3;
		cloud_b.points.resize(cloud_b.width*cloud_b.height);
		for (size_t i = 0; i < cloud_b.points.size(); ++i)
		{
			cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		}
		//�Ȱѵ���b��ӡ������
		cout << "Cloud B:" << endl;
		for (size_t i = 0; i < cloud_b.points.size(); ++i) {
			cout << "    " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " "<<  cloud_b.points[i].z << endl;
		}

		//����ƴ��,ֱ�Ӹ�ֵ�����ӷ�
		//cloud_c = cloud_a;
		//cloud_c += cloud_b;
		//cloud_c = cloud_b + cloud_a;
		cloud_c = cloud_a + cloud_b;
		//�ѽ����ӡ����
		cout << "Cloud C:������ƴ�ӣ�" << endl;
		for (size_t i = 0; i < cloud_c.points.size(); ++i) {
			cout << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << endl;
		}
		
	}
	return(0);

}