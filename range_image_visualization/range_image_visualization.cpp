#include<iostream>

#include<pcl/common/common_headers.h>
#include<pcl/range_image/range_image.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/range_image_visualizer.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/console/parse.h>//����̨��������

typedef pcl::PointXYZ PointType;

// --------------------
// -----���ò���-------
// --------------------

float angular_resolution_x = 0.5f, 
		angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame =pcl::RangeImage::CAMERA_FRAME;//Ϊ0��ö�ٳ���

bool live_update = false;//�Ƿ�ʵʱ���µı�־

// --------------------
// ----��ӡʹ�ð���----
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

//��3D�Ӵ�viewer���ù۲�Ƕ�
void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation()*Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	// ʹ�ø����Ĳ������������λ�ã��Ӵ���
	viewer.setCameraPosition(
		pos_vector[0], pos_vector[1], pos_vector[2],
		look_at_vector[0], look_at_vector[1], look_at_vector[2],
		up_vector[0], up_vector[1], up_vector[2]);
}

//������
int
main(int argc, char** argv)
{
	// ------------------------------------------------------------------
	// -----------------------�����в�������---------------------------
	// ------------------------------------------------------------------

	//find_argument�������������������Ƿ�������ĳ���û�еĻ�����-1���еĻ���������ʼ�±�
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);//argv[0]ָ��������е�ȫ·����
		return 0;
	}
	if (pcl::console::find_argument(argc, argv, "-l") >= 0)
	{
		live_update = true;//ʵʱ���¿��أ���3D�Ӵ��б�ѡ�е���ͼ��ʵʱ���µ����ͼ��
		std::cout << "Live update is on\n";
	}
	//parse����������ֵͬfind_argument������������֮��Ĳ�����������������ò���
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

	//��ת����
	angular_resolution_x = pcl::deg2rad(angular_resolution_x);
	angular_resolution_y = pcl::deg2rad(angular_resolution_y);


	// ------------------------------------------------------------------
	// ------------------��ȡpcd�ļ�������������򴴽���������-----------
	// ------------------------------------------------------------------
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	//����pcd�ļ���
	std::vector<int> pcd_filename_indeces = pcl::console::parse_file_extension_argument(argc,argv,"pcd");
	//������������Ϊ�գ�˵��ʹ�ñ���pcd�ļ�
	if (!pcd_filename_indeces.empty())
	{
		std::string filename = argv[pcd_filename_indeces[0]];
		//����ļ���ȡʧ��,ֱ���˳�
		if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
		{
			std::cout << "Was not able to open file \"" << filename << "\".\n";
			printUsage(argv[0]);//argv[0]�ǳ����·��ȫ��
			return 0;
		}
		scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
			point_cloud.sensor_origin_[1],
			point_cloud.sensor_origin_[2])) *
			Eigen::Affine3f(point_cloud.sensor_orientation_);
	}
	else//�������Ϊ�գ��Լ���������
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
		point_cloud.height = 1;//�������
	}

	// -----------------------------------------------
	// --------------Ϊ�������ݴ������ͼ��-----------
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
	// -------------��3D�Ӵ�����ӵ���--------------
	// -----------------------------------------------
	pcl::visualization::PCLVisualizer viewer("3D Viewer");//�������ڣ�����������Ϊ3D Viewer
	viewer.setBackgroundColor(1, 1, 1);//���ñ�����ɫ,��ɫ
	/**
	 * �����ɫΪ��ɫ��point sizeΪ1�����ͼ��
	 */
	pcl::visualization::PointCloudColorHandlerCustom < pcl::PointWithRange>range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	/**
	 * ���һ������ϵ�������ӻ�ԭʼ����
	 */
	//viewer.addCoordinateSystem(1.0f, "global");
	//pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud");)
	viewer.initCameraParameters();
	/**
	 * setViewerPose�����Ӵ��Ĳ鿴λ������Ϊ���ͼ���еĴ�����λ��
	 */
	setViewerPose(viewer, range_image.getTransformationToWorldSystem());

	// -----------------------------------------------
	// -----------------��ʾ���ͼ��------------------
	// -----------------------------------------------
	pcl::visualization::RangeImageVisualizer range_image_wiget("Range Image");
	range_image_wiget.showRangeImage(range_image);
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer.wasStopped())
	{
		range_image_wiget.spinOnce();//�������ͼ����ӻ���
		viewer.spinOnce();//����3D���ڵĵ�ǰ�¼�
		pcl_sleep(0.01);

		//ʵʱ����2D���ͼ������Ӧ���ӻ������еĵ�ǰ�ӽǣ�ͨ������ -l ������
		if (live_update)
		{
			scene_sensor_pose = viewer.getViewerPose();
			range_image.createFromPointCloud(point_cloud,angular_resolution_x,angular_resolution_y,
				pcl::deg2rad(360.0f),pcl::deg2rad(180.0f), scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
			range_image_wiget.showRangeImage(range_image);
		}

	}
}