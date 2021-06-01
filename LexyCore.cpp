#pragma warning(disable:4996)

#include "LexyCore.h"
#include "pcv.h"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class ToolsPrivate
{
public:

	void initParam();
	void appendParam(const std::string &decription, double value, double min_value, double max_value);
	AlgoParam *findParam(const std::string &description);
	const AlgoParam *findParam(const std::string &description) const;
	///
	std::vector<AlgoParam> m_szParams;

	Reprojector rj;
};

void ToolsPrivate::initParam()
{
	appendParam("参数1", 1.0, 0., 2.);
}

void ToolsPrivate::appendParam(const std::string & description, double value, double min_value, double max_value)
{
	if (findParam(description)) {
		/// 已经存在
		assert(0);
		return;
	}
	if (value < min_value || value > max_value) {
		/// 输入的参数值不正确
		assert(0);
		return;
	}
	m_szParams.push_back(AlgoParam());
	auto &param = m_szParams.back();
	param.description = description;
	param.current_value = value;
	param.def_value = value;
	param.min_value = min_value;
	param.max_value = max_value;
	return;
}

AlgoParam * ToolsPrivate::findParam(const std::string & description)
{
	if (description.empty())return nullptr;
	for (auto &param : m_szParams) {
		if (!param.description.compare(description)) {
			return &param;
		}
	}
	return nullptr;
}

const AlgoParam * ToolsPrivate::findParam(const std::string & description)const
{
	if (description.empty())return nullptr;
	for (auto &param : m_szParams) {
		if (!param.description.compare(description)) {
			return &param;
		}
	}
	return nullptr;
}

///

LaxyCore::LaxyCore()
	:d(new ToolsPrivate())
{
	d->initParam();
}
LaxyCore::~LaxyCore()
{
	delete d;
}

void LaxyCore::init()
{

}

void LaxyCore::print() const
{
}

bool LaxyCore::setParam(const std::string & description, double value)
{
	d->appendParam(description, value, 0, 10000);
	return false;
}

bool LaxyCore::setPath(const std::string & path)
{
	template_path = path;
	return true;
}

double LaxyCore::getParam(const std::string & description, bool * pOk) const
{
	return d->findParam(description)->current_value;
}

bool LaxyCore::reload_template(std::string & templatename)
{
	//template_path = "./LexyData/";
	std::string template_file = template_path + templatename;
	if(d->rj.reload_template(template_path, templatename)) return true;
	return false;
}

bool LaxyCore::reload_template(const PtCloud & pt_data, pcl::PointCloud<pcl::PointXYZ>::Ptr key_points, const std::string & ini_file)
{
	if (d->rj.reload_template(pt_data, key_points, ini_file)) return true;
	return false;
}

bool LaxyCore::generate(const PtCloud & pt_data)
{
	cv::Mat dimage_2;
	//if (getParam("enable_debug") > 0) d->rj.setdebug(true);
	if (!d->rj.generate_dimage(pt_data, dimage_2)) return false;
	if (!d->rj.compare_dimage(dimage_2)) return false;
	if (d->rj.getKeypoints().size() < 1) return false;
	return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LaxyCore::get_transformed_key_points()
{
	return d->rj.cloud_result;
}

bool LaxyCore::generate(const PtCloud& pt_data, std::vector <LaxyRes> & resvalue)
{

	cv::Mat dimage_2;
	LaxyRes res;
	std::vector<cacRes> pcv_res;
	//if (getParam("enable_debug") > 0) d->rj.setdebug(true);
	if (!d->rj.generate_dimage(pt_data, dimage_2)) return false;
	if (!d->rj.compare_dimage(dimage_2)) return false;
	if (d->rj.getKeypoints().size() < 1) return false;
	pcv_res = d->rj.caculate(template_path);
	for (int i=0;i<pcv_res.size();i++)
	{
		res.current_value = pcv_res[i].current_value;
		res.defult_max = pcv_res[i].defult_max;
		res.defult_min = pcv_res[i].defult_min;
		res.isgood = pcv_res[i].isgood;
		resvalue.push_back(res);
	}
	return true;
}

bool LaxyCore::generate(const std::vector<PtFrame>& pt_data, std::vector <LaxyRes> & resvalue)
{
	PtCloud source_pt_data(pt_data);
	if(!generate(source_pt_data, resvalue)) return false;
	return true;
}

bool LaxyCore::vistual()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;
	pcl_viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(d->rj.cloud_in, "z");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> high_light(d->rj.cloud_result, 255, 255, 255);
	pcl_viewer->addPointCloud<pcl::PointXYZ>(d->rj.cloud_in, fildColor, "centered cloud");
	pcl_viewer->addPointCloud<pcl::PointXYZ>(d->rj.cloud_result, high_light, "trajectory");
	pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "trajectory");
	pcl_viewer->spin();
	return false;
}

