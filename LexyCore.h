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
	// �涨��Сֵ
	double defult_min;

	// �涨���ֵ
	double defult_max;

	// ��ǰֵ
	double current_value;

	// �Ƿ��� true-ok��false-ng;
	bool isgood;

}LaxyRes;

typedef struct tagAlgoParam
{
	// ����˵��
	std::string description;

	// Ĭ��ֵ
	double def_value;

	// ��ǰֵ
	double current_value;

	// ���ֵ
	double max_value;

	// ��Сֵ
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
	/// ��������
	bool setParam(const std::string &description, double value);
	bool setPath(const std::string & path);
	double getParam(const std::string &description, bool *pOk = nullptr) const;

public:
	std::string template_path = "./LexyData/";
	/// ���ɹ켣
	/**
	* @param templatename: ģ�����ƣ��������+�����--���ļ�������
	*/
	bool reload_template(std::string & templatename);

	///
	/**
	* ����ģ��
	* @param pt_data: ģ������
	* @param key_points: ģ��켣��
	* @param ini_file: �����ļ�·����ini�ļ���
	*/
	bool reload_template(const PtCloud& pt_data, pcl::PointCloud<pcl::PointXYZ>::Ptr key_points, const std::string & ini_file);

	/**
	* ƥ������
	* @param pt_data: ��Ѱ�Ҷ�Ӧ�켣�������
	*/
	bool generate(const PtCloud& pt_data);

	/**
	* ��ȡ���
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr get_transformed_key_points();
	///

	/**
	* @param pt_data: ɨ�赱ǰ����õ���Ptcloud����
	* @param res_index: �������õ�������ϵĹؼ�����ֵ
	*/
	bool generate(const PtCloud& pt_data, std::vector <LaxyRes>& resvalue);
	bool generate(const std::vector<PtFrame>& pt_data, std::vector <LaxyRes>& resvalue);
	bool vistual();




private:
	ToolsPrivate *d;
	
};

//CELLTAP_NAMESPACE_END

#endif // LEXY_CORE_H