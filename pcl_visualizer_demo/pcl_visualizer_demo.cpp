#include<iostream>
#include<thread>

#include<pcl/common/common_headers.h>
#include<pcl/features/normal_3d.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/console/parse.h>

using namespace std::chrono_literals;

// ----------------------
// -----打印使用帮助-----
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
// Method:    传入三个const指针，返回一个PCLVisualizer指针
// 通过不同的视口（ViewPort）绘制多个点云
// 利用不同的搜索半径，基于同一点云计算出对应不同半径的两组法线
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
	// ----------------打开3D视窗，添加点云和法线--------------
	// --------------------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	//使用默认值初始化相机参数
	viewer->initCameraParameters();


	int v1(0);
	//创建视口
	/**创建一个视口从 [xmin,ymin] 到 [xmax,ymax].
	* 参数 xmin ：视口在X轴的最小值 (0.0 <= 1.0)
	* 参数 ymin ：视口在Y轴的最小值 (0.0 <= 1.0)
	* 参数 xmax ：视口在X轴的最大值 (0.0 <= 1.0)
	* 参数 ymax： 视口在Y轴的最大值 (0.0 <= 1.0)
	* 参数 viewport ： 新视口的id，传入的是个引用参数
	*
	* 注意：
	*如果当前窗口不存在渲染器，将创建一个，并且*视口将被设置为0 ('all')。如果一个或多个渲染
	*器存在，视口ID将被设置为渲染器的总数- 1
	*/
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);

	//设置viewport的背景颜色，第四个参数为指定视口的id，如果不输入，默认为全部视口设置背景颜色
	//setBackgroundColor(const double &r, const double &g, const double &b, int viewport = 0);
	viewer->setBackgroundColor(0, 0, 0, v1);

	//为视口添加文本,添加文本有许多重载函数，可以设置颜色字体等属性
	/*
	addText (const std::string &text,
		int xpos, int ypos,
		const std::string &id = "", int viewport = 0);
		*/
	viewer->addText("Radius:0.01", 10, 10, "v1 text", v1);

	//创建一个颜色处理器
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	//为指定的viewport加入点云，颜色处理器，
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
// -----主函数-----
// ----------------
int 
main(int argc, char** argv)
{
	// --------------------------------------
	// ----------命令行参数解析--------------
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
	// ----------创建示例点云数据----------
	// ------------------------------------
	pcl::PointCloud < pcl::PointXYZ > ::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Generating example point clouds.\n\n";
	/*
	 * 创建一个向z轴挤压的椭圆，颜色由按红绿蓝的顺序渐变
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
			//将rgb的数据保存在一个浮点数
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
	basic_cloud_ptr->height = 1;//无序点云
	point_cloud_ptr->width = point_cloud_ptr->size();
	point_cloud_ptr->height = 1;

	// ----------------------------------------------------------------
	// ----------------以搜索半径0.05计算表面法线----------------------
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);
	// ----------------------------------------------------------------
	// ----------------以搜索半径0.1计算表面法线----------------------
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