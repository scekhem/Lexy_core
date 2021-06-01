#pragma once
#pragma warning(disable:4996)

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <iostream>

#include <ptcloud.h>
#include <ptframe.h>
#include <PtRobot.h>
#include <frameio.h>
#include "SimpleIni.h"

typedef struct caculate_res
{
	// 规定最小值
	double defult_min;

	// 规定最大值
	double defult_max;

	// 当前值
	double current_value;

	// 是否达标 true-ok、false-ng;
	bool isgood;

}cacRes;

class  Reprojector
{
public:
	// initialize
	void _allocateMemory();
	Reprojector();
	~Reprojector(void);

	PtCloud loadPTData(std::string& filename);

	bool reload_template(const std::string & path, const std::string & templatename);

	bool reload_template(const PtCloud& pt_data, pcl::PointCloud<pcl::PointXYZ>::Ptr key_points, const std::string & ini_file);

	bool generate_dimage(std::string & templatename, cv::Mat & dist_image, const bool & load_template = false);
	bool generate_dimage(const PtCloud& pt_data, cv::Mat & dist_image, const bool & load_template = false); // reprojet points xyz to matrix

	/**
	 * @param src_1: ptcloud file 1 path
	 * @param src_2: ptcloud file 2 path
	 */
	bool compare_dimage(std::string & src);	

	/**
	* @param dimage_1: depth image 1 (GRAY)
	* @param dimage_2: depth image 2 (GRAY)
	*/
	bool compare_dimage(cv::Mat & dimage_2);
	bool points_refine( cv::Mat & template_homo, cv::Mat & dimage, const std::vector<cv::Point2f>& key_pix_homo, std::vector<cv::Point2f>& points_refine);
	bool points_check(cv::Mat & template_refine, cv::Mat & dimage_roi, const cv::Point2f & point_refine, cv::Point2f & point_final);
	std::vector<int> getKeypoints();

	

	std::vector<cacRes> caculate(const std::string& config_path);

	void setdebug(const bool value);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result;

private:

	//pcl::PointCloud<pcl::PointXYZ>::Ptr _updatePoints(Eigen::MatrixXi* depth_mask); // process & show gray img
	CSimpleIniA inifile;
	bool debug_model = false;
	bool use_HEQ = false;
	pcl::PointXYZ min_p;
	pcl::PointXYZ max_p;

	double gap_wide;
	double gap_height;
	float depth_range;

	// ini config params
	double crop_z_min = 0.25;
	int blur_size = 7;
	int margin_x = 400;
	int margin_y = 400;
	int margin_check_x = 150;
	int margin_check_y = 150;
	double compress_x = 1;
	double compress_y = 2.45;
		 

	cv::Mat template_img;
	std::vector<cv::Point2f> key_pix;
	std::vector<cv::Point2f> key_pix_dist;
	std::vector<cv::Point2f> key_pix_check;


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centered;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_selected;
	pcl::PointCloud<pcl::PointXYZ>::Ptr selected_points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr selected_points_center;

	Eigen::MatrixXi matrix_index;

};	//reprojector