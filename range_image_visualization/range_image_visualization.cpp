#include<iostream>

#include<pcl/common/common_headers.h>
#include<pcl/range_image/range_image.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/range_image_visualizer.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/console/parse.h>//控制台参数解析

typedef pcl::PointXYZ PointType;

// --------------------
// -----设置参数-------
// --------------------

float angular_resolution_x = 0.5f, 
		angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame =pcl::RangeImage::CAMERA_FRAME;//为0的枚举常量

bool live_update = false;//是否实时更新的标志

// --------------------
// ----打印使用帮助----
// --------------------
void
printUsage(const char* progName)
{
	std::cout<<"\n\nUsage: "<< progName<<"[options] <scene.pcd>\n\n"
		<<"Options:\n"
		<<"-------------------------------------------------\n"
		<< "-rx <float>  angular resolution in degrees (default " << angular_resolution_x << ")\n"
		<< "-ry <float>  angular resolution in degrees (default " << angular_resolution_y << ")\n"
		<< "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
		<< "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
		<< "-h           this help\n"
		<< "\n\n";
}

//给3D视窗viewer设置观察角度
void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation()*Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	// 使用给定的参数设置相机的位置，视窗等
	viewer.setCameraPosition(
		pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}

//主函数
int
main(int argc, char** argv)
{
	// ------------------------------------------------------------------
	// -----------------------命令行参数解析---------------------------
	// ------------------------------------------------------------------

	//find_argument函数：查找命令行中是否有输入某命令，没有的话返回-1，有的话返回其起始下标
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);//argv[0]指向程序运行的全路径名
		return 0;
	}
	if (pcl::console::find_argument(argc, argv, "-l") >= 0)
	{
		live_update = true;//实时更新开关，从3D视窗中被选中的视图，实时更新到深度图像
		std::cout << "Live update is on\n";
	}
	//parse函数：返回值同find_argument函数，将命令之后的参数解析到传入的引用参数
	if (pcl::console::parse(argc, argv, "-rx", angular_resolution_x) >= 0)
		std::cout << "Setting angular resolution in x-direction to " << angular_resolution_x << "deg.\n";
	if (pcl::console::parse(argc, argv, "-ry", angular_resolution_y) >= 0)
		std::cout << "Setting angular resolution in y-direction to " << angular_resolution_y << "deg.\n";
	int tmp_coordinate_frame;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
	{
		coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
		std::cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
	}

	//度转弧度
	angular_resolution_x = pcl::deg2rad(angular_resolution_x);
	angular_resolution_y = pcl::deg2rad(angular_resolution_y);


	// ------------------------------------------------------------------
	// ------------------读取pcd文件，如果不存在则创建点云数据-----------
	// ------------------------------------------------------------------
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	//解析pcd文件名
	std::vector<int> pcd_filename_indeces = pcl::console::parse_file_extension_argument(argc,argv,"pcd");
	//如果解析结果不为空，说明使用本地pcd文件
	if (!pcd_filename_indeces.empty())
	{
		std::string filename = argv[pcd_filename_indeces[0]];
		//如果文件读取失败,直接退出
		if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
		{
			std::cout << "Was not able to open file \"" << filename << "\".\n";
			printUsage(argv[0]);//argv[0]是程序的路径全称
			return 0;
		}
		scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
			point_cloud.sensor_origin_[1],
			point_cloud.sensor_origin_[2])) *
			Eigen::Affine3f(point_cloud.sensor_orientation_);
	}
	else//解析结果为空，自己创建点云
	{
		std::cout << "\nNo *.pcd file given => Genereating example cloud.\n\n";
		for (float x = -0.5f; x <= 0.5f; x += 0.01f)
		{
			for (float y = -0.5f; y <= 0.5f; y += 0.01f)
			{
				PointType point; point.x = x; point.y = y, point.z = 2.0f - y;
				point_cloud.points.push_back(point);
			}
		}
		point_cloud.width = point_cloud.size();
		point_cloud.height = 1;//无序点云
	}

	// -----------------------------------------------
	// --------------为点云数据创建深度图像-----------
	// -----------------------------------------------
	// 
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
		pcl::deg2rad(360.0f), pcl::deg2rad(180.f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	// -----------------------------------------------
	// -------------打开3D视窗并添加点云--------------
	// -----------------------------------------------
	pcl::visualization::PCLVisualizer viewer("3D Viewer");//创建窗口，并设置名字为3D Viewer
	viewer.setBackgroundColor(1, 1, 1);//设置背景颜色,白色
	/**
	 * 添加颜色为黑色，point size为1的深度图像
	 */
	pcl::visualization::PointCloudColorHandlerCustom < pcl::PointWithRange>range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	/**
	 * 添加一个坐标系，并可视化原始点云
	 */
	//viewer.addCoordinateSystem(1.0f, "global");
	//pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud");)
	viewer.initCameraParameters();
	/**
	 * setViewerPose，将视窗的查看位置设置为深度图像中的传感器位置
	 */
	setViewerPose(viewer, range_image.getTransformationToWorldSystem());

	// -----------------------------------------------
	// -----------------显示深度图像------------------
	// -----------------------------------------------
	pcl::visualization::RangeImageVisualizer range_image_wiget("Range Image");
	range_image_wiget.showRangeImage(range_image);
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer.wasStopped())
	{
		range_image_wiget.spinOnce();//处理深度图像可视化类
		viewer.spinOnce();//处理3D窗口的当前事件
		pcl_sleep(0.01);

		//实时更新2D深度图像，以响应可视化窗口中的当前视角，通过命令 -l 来激活
		if (live_update)
		{
			scene_sensor_pose = viewer.getViewerPose();
			range_image.createFromPointCloud(point_cloud,angular_resolution_x,angular_resolution_y,
				pcl::deg2rad(360.0f),pcl::deg2rad(180.0f), scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
			range_image_wiget.showRangeImage(range_image);
		}

	}
}