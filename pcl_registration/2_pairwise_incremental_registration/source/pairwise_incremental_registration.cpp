/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/
#include <boost/make_shared.hpp>//boostָ��
#include <pcl/point_types.h>//�����Ͷ���
#include <pcl/point_cloud.h>//������
#include <pcl/point_representation.h>//���ʾ
#include <pcl/io/pcd_io.h>//pcd�����������
#include <pcl/filters/voxel_grid.h>//���������˲�
#include <pcl/filters/filter.h>//�˲�
#include <pcl/features/normal_3d.h>//��������
#include <pcl/registration/icp.h>//icp��
#include <pcl/registration/icp_nl.h> //������icp��
#include <pcl/registration/transforms.h>//�任������
#include <pcl/visualization/pcl_visualizer.h>//���ӻ���

//��ɫ������
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
//�����Ͷ���
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT; //������
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;//���з������ĵ���

//�������ӻ�����
pcl::visualization::PCLVisualizer *p;
//���������ӵ�
int vp_1, vp_2;
//������Ƶķ���Ľṹ����
struct PCD
{
	PointCloud::Ptr cloud;//����ָ��
	std::string f_name;//�ļ���
	PCD() : cloud(new PointCloud) {};
};
struct PCDComparator
{
	bool operator () (const PCD& p1, const PCD& p2)
	{
		return (p1.f_name < p2.f_name);//�ļ����Ƿ���ͬ
	}
};
//��< x, y, z, curvature >��ʽ����һ���µĵ㣬��ʾx,y,z+����
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;//ά��
public:
	MyPointRepresentation()
	{
		//����ߴ�ֵ
		nr_dimensions_ = 4;
	}
	//����copyToFloatArray�������������ǵ�����ʸ��
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;//����
	}
};
////////////////////////////////////////////////////////////////////////////////
/** �ڿ��ӻ����ڵĵ�һ�ӵ���ʾԴ���ƺ�Ŀ�����
 *  ����ͼ������ʾδƥ���Դ���ƺ�Ŀ�����
 *	��ɫ����Դ����,��ɫ����Ŀ�����
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
	p->removePointCloud("vp1_target");
	p->removePointCloud("vp1_source");
	PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);//green
	PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);//red
	p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);
	PCL_INFO("Press q to begin the registration.\n");
	p->spin();
}
////////////////////////////////////////////////////////////////////////////////
/**�ڿ��ӻ����ڵĵڶ��ӵ���ʾԴ���ƺ�Ŀ�����
 *	�ұ���ʾ��׼���Դ���ƺ�Ŀ�����
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
	p->removePointCloud("source");
	p->removePointCloud("target");
	// �������ֶ������Ƶ�����ɫ����ʾ
	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
	if (!tgt_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");
	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
	if (!src_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");
	p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);
	p->spinOnce();
}
////////////////////////////////////////////////////////////////////////////////
/**����һ��������Ҫƥ����һ���PCD�ļ�
  * ����argc�ǲ��������� (pass from main ())
  *���� argv ʵ�ʵ������в��� (pass from main ())
  *����models�������ݼ��ĺϳ�ʸ��
  */
void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
	std::string extension(".pcd");
	//�ٶ���һ��������ʵ�ʲ���ģ��
	for (int i = 1; i < argc; i++)
	{
		std::string fname = std::string(argv[i]);
		// ������Ҫ5���ַ�������Ϊ.pcd����4���ַ���
		if (fname.size() <= extension.size())
			continue;
		std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);
		//��������һ��pcd�ļ�
		if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
		{
			//���ص��Ʋ������������ģ���б���
			PCD m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile(argv[i], *m.cloud);
			//�ӵ������Ƴ�NAN��
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);
			models.push_back(m);
		}
	}
}
////////////////////////////////////////////////////////////////////////////////
/**ƥ��һ�Ե������ݼ����ҷ������
  *���� cloud_src ��Դ����
  *���� cloud_tgt ��Ŀ�����
  *���� output �������׼�����Դ����
  *���� final_transform ������Դ��Ŀ��֮���ת��
  *���� downsample �Ƿ���Ҫ�����²���
  */
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	//Ϊ��һ���Ժ͸��ٵ��²���
	//ע�⣺Ϊ�˴����ݼ���Ҫ��������
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;//�����˲�
	// �Ƿ��ȶԵ��ƽ����²���
	if (downsample)
	{
		grid.setLeafSize(0.05, 0.05, 0.05);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}
	//�������淨�ߺ�����
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;//���Ʒ��߹��ƶ���
	// ʹ��KD����������
	//��������㷨��kdtree search ����ļ����㡡����ƽ�桡Э�������PCA�ֽ⡡��ⷨ��
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30); //��������ھӵ�����k��k���������
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);//������淨��������������
	pcl::copyPointCloud(*src, *points_with_normals_src);//��ͬ���͵ĵ���֮���ת��
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);
	
	//����˵�������Զ����ı�ʾ�����϶��壩
	MyPointRepresentation point_representation;
	//����'curvature'�ߴ�Ȩ���Ա�ʹ����x, y, zƽ��
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);
	
	// ��׼
	//IterativeClosestPointNonLinear��һ��ICP���壬ʹ��Levenberg-Marquardt�Ż���ˡ��ϳɵ�ת�������Ż�Ϊ��Ԫ����
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;

	reg.setTransformationEpsilon(1e-6);//���������ж���������ֵԽС�򾫶�Խ������ҲԽ��
	//��������Ӧ��ϵ֮���(src<->tgt)����������Ϊ10����
	//ע�⣺����������ݼ���С������
	reg.setMaxCorrespondenceDistance(0.1);//����10cm�ĵ㲻����
	//���õ��ʾ
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));
	reg.setInputCloud(points_with_normals_src);//ԭ����
	reg.setInputTarget(points_with_normals_tgt);//Ŀ�����
	//
	//��һ��ѭ����������ͬ�����Ż�����ʹ������ӻ�
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(2);//������������ÿ�ε������ξ���Ϊ����
	//�ֶ�������ÿ�ֶ�����һ�Σ��ڴ��ڶ����µĵ����������ˢ����ʾ
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO("Iteration Nr. %d.\n", i);
		//Ϊ�˿��ӻ���Ŀ�ı������
		points_with_normals_src = reg_result;
		//����
		reg.setInputCloud(points_with_normals_src);

		// ��ʼ��׼���������������reg_result
		reg.align(*reg_result);
		//��ÿһ������֮���ۻ�ת��
		Ti = reg.getFinalTransformation() * Ti;
		//������ת����֮ǰת��֮��Ĳ���С����ֵ
		//��ͨ����С����Ӧ���������Ƴ���
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		prev = reg.getLastIncrementalTransformation();
		//���ӻ���ǰ״̬
		showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}
	//
  // �õ�Ŀ����Ƶ�Դ���Ƶı任
	targetToSource = Ti.inverse();
	//
	//��Ŀ�����ת����Դ���
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	p->removePointCloud("source");
	p->removePointCloud("target");
	PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);//��׼��ĵ���Ϊ��ɫ
	PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);//ԭʼ����Ϊ��ɫ
	p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
	PCL_INFO("Press q to continue the registration.\n");
	p->spin();
	p->removePointCloud("source");
	p->removePointCloud("target");
	//���Դ���Ƶ�ת��Ŀ��
	*output += *cloud_src;
	final_transform = targetToSource;
}
/* ---[ */
int main(int argc, char** argv)
{
	// ��������
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
	loadData(argc, argv, data);
	//����û�����
	if (data.empty())
	{
		PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]\n", argv[0]);
		PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc\n");
		PCL_INFO("Example: %s `rospack find pcl`/test/bun0.pcd `rospack find pcl`/test/bun4.pcd\n", argv[0]);
		return (-1);
	}
	PCL_INFO("Loaded %d datasets.", (int)data.size());
	//����һ��PCL���ӻ�����
	p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registration example");
	p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);
	PointCloud::Ptr result(new PointCloud), source, target;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
	//ѭ���������е���
	for (size_t i = 1; i < data.size(); ++i)
	{
		source = data[i - 1].cloud;//������׼
		target = data[i].cloud;//�����������
		//��ӿ��ӻ�����
		showCloudsLeft(source, target);
		PointCloud::Ptr temp(new PointCloud);
		PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());
		
		//pairTransform���ش�Ŀ�����target��source�ı任����
		pairAlign(source, target, temp, pairTransform, true);

		//�ѵ�ǰ������׼��ĵ���tempת����ȫ������ϵ�·���result
		pcl::transformPointCloud(*temp, *result, GlobalTransform);

		//�õ�ǰ���������֮��ı任����ȫ�ֱ任
		std::cout << "GlobalTransform:" << std::endl;
		std::cout << GlobalTransform << std::endl;
		GlobalTransform = pairTransform * GlobalTransform;

		//������׼�ԣ�ת������һ�����ƿ����
		std::stringstream ss;
		ss << i << ".pcd";
		pcl::io::savePCDFile(ss.str(), *result, true);
	}
}
/* ]--- */
