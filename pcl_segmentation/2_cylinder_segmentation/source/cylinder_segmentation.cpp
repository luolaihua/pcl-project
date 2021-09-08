#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<string>

typedef pcl::PointXYZ PointT;
// All the objects needed
pcl::PCDReader reader;
pcl::PassThrough<PointT> pass;
pcl::NormalEstimation<PointT, pcl::Normal> ne;
pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
pcl::PCDWriter writer;
pcl::ExtractIndices<PointT> extract;
pcl::ExtractIndices<pcl::Normal> extract_normals;
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

// Datasets
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
//PCLVisualizer
pcl::visualization::PCLVisualizer viewer("Cylinder Segmentation");

int step_count;

void pass_through_filter()
{
	// Build a passthrough filter to remove spurious NaNs
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

}

void normals_estimate()
{
	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);
}

void plane_seg()
{
	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
}

void get_plane()
{
	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	// Write the planar inliers to disk
	
	extract.filter(*cloud_plane);
	std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
	writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);
}

void remove_plane()
{
	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*cloud_normals2);
}

void cylinder_seg()
{
	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0, 0.1);
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(cloud_normals2);

	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
}
void get_cylinder()
{
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else
	{
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
		writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
	}
}

void keyboard_event_occurred(const pcl::visualization::KeyboardEvent& event,
	void * nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
	{
		++step_count;
	}
}
int
main(int argc, char** argv)
{
	step_count = 1;
	std::string str = "STEP";
	//初始化PCLVisualizer
	//viewer.addCoordinateSystem(1);
	//viewer.setBackgroundColor(0, 0, 255);
	viewer.addText(str+ std::to_string(step_count), 10, 10,16, 200,200,100,"text");
	
	//注册键盘回调函数，按下空格键将显示下一个点云处理结果
	viewer.registerKeyboardCallback(&keyboard_event_occurred, (void*)NULL);


	// Read in the cloud data
	reader.read("table_scene_mug_stereo_textured.pcd", *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

	// 显示原始点云
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> color_cloud(cloud, 255, 0, 0);
	viewer.addPointCloud(cloud,"cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"cloud");


	//滤波去噪
	pass_through_filter();

	// 法线估计
	normals_estimate();

	// 把平面分割出来
	plane_seg();

	// 利用平面点云的下标将平面抽取出来，并保存
	get_plane();

	// 移除平面及其法线，将结果保存在cloud_filtered2，cloud_normals2
	remove_plane();

	// 将圆柱分割出来，得到系数因子和下标
	cylinder_seg();

	// 将圆柱抽取并保存
	get_cylinder();
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		if (step_count < 8)
		{
			viewer.updateText(str + std::to_string(step_count), 10, 10, 16, 50, 100, 200, "text");
			switch (step_count)
			{
			case 2:
				//滤波去噪	
				viewer.updatePointCloud(cloud_filtered, "cloud"); break;
			case 3:
				//法线估计
				break;
			case 4:
				//利用平面点云的下标将平面抽取出来，并保存
				viewer.updatePointCloud(cloud_plane, "cloud"); break;
			case 5:
				//移除平面及其法线，将结果保存在cloud_filtered2，cloud_normals2
				viewer.updatePointCloud(cloud_filtered2, "cloud"); break;
			case 6:
				// 将圆柱分割出来，得到系数因子和下标
				break;
			case 7:
				//// 将圆柱抽取并保存
				viewer.updatePointCloud(cloud_cylinder, "cloud"); break;
			default:
				break;
			}
		}
		else {
			break;
		}

	}

	return (0);
}
