#include<iostream>
#include<thread>

#include<pcl/common/common_headers.h>
#include<pcl/features/normal_3d.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/console/parse.h>

using namespace std::chrono_literals;

// ----------------------
// -----��ӡʹ�ð���-----
// ----------------------

void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-s           Simple visualization example\n"
		<< "-r           RGB colour visualization example\n"
		<< "-c           Custom colour visualization example\n"
		<< "-n           Normals visualization example\n"
		<< "-a           Shapes visualization example\n"
		<< "-v           Viewports example\n"
		<< "-i           Interaction Customization example\n"
		<< "\n\n";
}

//************************************
// Method:    ��������constָ�룬����һ��PCLVisualizerָ��
// ͨ����ͬ���ӿڣ�ViewPort�����ƶ������
// ���ò�ͬ�������뾶������ͬһ���Ƽ������Ӧ��ͬ�뾶�����鷨��
// Returns:   pcl::visualization::PCLVisualizer::Ptr
// Parameter: pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud
// Parameter: pcl::PointCloud<pcl::Normal>::ConstPtr normals1
// Parameter: pcl::PointCloud<pcl::Normal>::ConstPtr normals2
//************************************
pcl::visualization::PCLVisualizer::Ptr viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
	pcl::PointCloud<pcl::Normal>::ConstPtr normals1,
	pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	// --------------------------------------------------------
	// ----------------��3D�Ӵ�����ӵ��ƺͷ���--------------
	// --------------------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	//ʹ��Ĭ��ֵ��ʼ���������
	viewer->initCameraParameters();


	int v1(0);
	//�����ӿ�
	/**����һ���ӿڴ� [xmin,ymin] �� [xmax,ymax].
	* ���� xmin ���ӿ���X�����Сֵ (0.0 <= 1.0)
	* ���� ymin ���ӿ���Y�����Сֵ (0.0 <= 1.0)
	* ���� xmax ���ӿ���X������ֵ (0.0 <= 1.0)
	* ���� ymax�� �ӿ���Y������ֵ (0.0 <= 1.0)
	* ���� viewport �� ���ӿڵ�id��������Ǹ����ò���
	*
	* ע�⣺
	*�����ǰ���ڲ�������Ⱦ����������һ��������*�ӿڽ�������Ϊ0 ('all')�����һ��������Ⱦ
	*�����ڣ��ӿ�ID��������Ϊ��Ⱦ��������- 1
	*/
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);

	//����viewport�ı�����ɫ�����ĸ�����Ϊָ���ӿڵ�id����������룬Ĭ��Ϊȫ���ӿ����ñ�����ɫ
	//setBackgroundColor(const double &r, const double &g, const double &b, int viewport = 0);
	viewer->setBackgroundColor(0, 0, 0, v1);

	//Ϊ�ӿ�����ı�,����ı���������غ���������������ɫ���������
	/*
	addText (const std::string &text,
		int xpos, int ypos,
		const std::string &id = "", int viewport = 0);
		*/
	viewer->addText("Radius:0.01", 10, 10, "v1 text", v1);

	//����һ����ɫ������
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	//Ϊָ����viewport������ƣ���ɫ��������
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius:0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);

	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

	return (viewer);
}

// ----------------
// -----������-----
// ----------------
int 
main(int argc, char** argv)
{
	// --------------------------------------
	// ----------�����в�������--------------
	// --------------------------------------
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	bool simple(false), rgb(false), custom_c(false), normals(false),
		shapes(false), viewports(false), interaction_customization(false);
	if (pcl::console::find_argument(argc, argv, "-s") >= 0)
	{
		simple = true;
		std::cout << "Simple visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-c") >= 0)
	{
		custom_c = true;
		std::cout << "Custom colour visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-r") >= 0)
	{
		rgb = true;
		std::cout << "RGB colour visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-n") >= 0)
	{
		normals = true;
		std::cout << "Normals visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-a") >= 0)
	{
		shapes = true;
		std::cout << "Shapes visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-v") >= 0)
	{
		viewports = true;
		std::cout << "Viewports example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-i") >= 0)
	{
		interaction_customization = true;
		std::cout << "Interaction Customization example\n";
	}
	else
	{
		printUsage(argv[0]);
		return 0;
	}
	// ------------------------------------
	// ----------����ʾ����������----------
	// ------------------------------------
	pcl::PointCloud < pcl::PointXYZ > ::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Generating example point clouds.\n\n";
	/*
	 * ����һ����z�ἷѹ����Բ����ɫ�ɰ���������˳�򽥱�
	 */
	std::uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = 0.5*std::cos(pcl::deg2rad(angle));
			basic_point.y = sinf(pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);

			pcl::PointXYZRGB point;
			point.x = basic_point.x;
			point.y = basic_point.y;
			point.z = basic_point.z;
			std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 | static_cast<std::uint32_t>(g) << 8
				| static_cast<std::uint32_t>(b));
			//��rgb�����ݱ�����һ��������
			point.rgb = *reinterpret_cast<float*> (&rgb);
			point_cloud_ptr->points.push_back(point);
		}
		if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	}
	basic_cloud_ptr->width = basic_cloud_ptr->size();
	basic_cloud_ptr->height = 1;//�������
	point_cloud_ptr->width = point_cloud_ptr->size();
	point_cloud_ptr->height = 1;

	// ----------------------------------------------------------------
	// ----------------�������뾶0.05������淨��----------------------
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);
	// ----------------------------------------------------------------
	// ----------------�������뾶0.1������淨��----------------------
	// ----------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);

	pcl::visualization::PCLVisualizer::Ptr viewer;
	if (viewports)
	{
		viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
	}
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
}