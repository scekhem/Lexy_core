#pragma once

#ifndef LEXY_CORE_H
#define LEXY_CORE_H 1

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ptcloud.h>

#ifdef LEXY_CORE_EXPORTS
#define LEXY_CORE_API __declspec(dllexport)
#else
#define LEXY_CORE_API __declspec(dllimport)
#endif

typedef struct LaxyCore_res
{
	// 规定最小值
	double defult_min;

	// 规定最大值
	double defult_max;

	// 当前值
	double current_value;

	// 是否达标 true-ok、false-ng;
	bool isgood;

}LaxyRes;

typedef struct tagAlgoParam
{
	// 参数说明
	std::string description;

	// 默认值
	double def_value;

	// 当前值
	double current_value;

	// 最大值
	double max_value;

	// 最小值
	double min_value;
}AlgoParam;

class ToolsPrivate;
class LEXY_CORE_API LaxyCore
{

public:

	LaxyCore();
	~LaxyCore();

public:

	void init();
	void print() const;

public:
	/// 参数设置
	bool setParam(const std::string &description, double value);
	bool setPath(const std::string & path);
	double getParam(const std::string &description, bool *pOk = nullptr) const;

public:
	std::string template_path = "./LexyData/";
	/// 生成轨迹
	/**
	* @param templatename: 模板名称（零件名称+零件号--即文件夹名）
	*/
	bool reload_template(std::string & templatename);

	///
	/**
	* 载入模板
	* @param pt_data: 模板数据
	* @param key_points: 模板轨迹点
	* @param ini_file: 配置文件路径（ini文件）
	*/
	bool reload_template(const PtCloud& pt_data, pcl::PointCloud<pcl::PointXYZ>::Ptr key_points, const std::string & ini_file);

	/**
	* 匹配数据
	* @param pt_data: 待寻找对应轨迹点的数据
	*/
	bool generate(const PtCloud& pt_data);

	/**
	* 获取结果
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_transformed_key_points();
	///

	/**
	* @param pt_data: 扫描当前零件得到的Ptcloud数据
	* @param res_index: 输出计算得到该零件上的关键测量值
	*/
	bool generate(const PtCloud& pt_data, std::vector <LaxyRes>& resvalue);
	bool generate(const std::vector<PtFrame>& pt_data, std::vector <LaxyRes>& resvalue);
	bool vistual();




private:
	ToolsPrivate *d;
	
};

//CELLTAP_NAMESPACE_END

#endif // LEXY_CORE_H