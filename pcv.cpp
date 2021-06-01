#include "pcv.h"
#include "vfc.h"
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

#include <fstream>
#include <math.h>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

bool compare_AKAZE(const cv::Mat & dimage_1, const cv::Mat & dimage_2, cv::Mat & Homography, cv::Mat & Matches_resfine,bool use_Affine, bool debug_model)
{
	using namespace cv;
	//cv::Mat dimage_1, dimage_2;
	cv::Mat img1 = dimage_1.clone();
	cv::Mat img2 = dimage_2.clone();
	cv::normalize(img1, img1, 0, 255, cv::NORM_MINMAX);
	cv::normalize(img2, img2, 0, 255, cv::NORM_MINMAX);

	cv::Mat desc1, desc2;
	Ptr<ORB> detector = ORB::create();
	//Ptr<AKAZE> detector = AKAZE::create();
	//Ptr<cv::BRISK> detector = cv::BRISK::create();

	std::vector<cv::KeyPoint> aff_kpts1, aff_kpts2;

	detector->detectAndCompute(img1, cv::Mat(), aff_kpts1, desc1);
	detector->detectAndCompute(img2, cv::Mat(), aff_kpts2, desc2);


	vector<cv::KeyPoint> kpts1, kpts2;
	kpts1 = aff_kpts1;
	kpts2 = aff_kpts2;

	//FlannBasedMatcher matcher;
	cv::BFMatcher matcher(4,true);
	vector<cv::DMatch> matches;

	
	matcher.match(desc1, desc2, matches);

	//cout << "VFC Start ... \n ";

	//-- Step: 匹配点对精确提纯 (VFC)
	vector<cv::Point2f> X;
	vector<cv::Point2f> Y;
	X.clear();          
	Y.clear();
	for (unsigned int i = 0; i < matches.size(); i++) {
		int idx1 = matches[i].queryIdx;
		int idx2 = matches[i].trainIdx;
		X.push_back(kpts1[idx1].pt);
		Y.push_back(kpts2[idx2].pt);
	}
	// VFC process
	VFC myvfc;
	myvfc.setData(X, Y);
	myvfc.optimize();
	vector<int> matchIdx = myvfc.obtainCorrectMatch();
	//t = ((double)getTickCount() - t) / getTickFrequency();
	//cout << "MATCH + VFC Time (s): " << t << endl;

	if (matchIdx.size() < 8) { 
		std::cout << " BAD MATCH RESULT!";
		return false; }
	vector< cv::DMatch > correctMatches;
	correctMatches.clear();
	for (unsigned int i = 0; i < matchIdx.size(); i++) {
		int idx = matchIdx[i];
		correctMatches.push_back(matches[idx]);
	}


	vector<cv::Point2f> obj;
	vector<cv::Point2f> objInScene;
	for (size_t t = 0; t < correctMatches.size(); t++) {
		obj.push_back(kpts1[correctMatches[t].queryIdx].pt);//返回对象在模板图特征点坐标
		objInScene.push_back(kpts2[correctMatches[t].trainIdx].pt);//返回对象在背景查找图的坐标
	}
	//Homography = findHomography(obj, objInScene, RANSAC);
	if (!use_Affine)
		Homography = findHomography(obj, objInScene, RHO);
	else
		Homography = estimateAffinePartial2D(obj, objInScene);

	if (debug_model) {
		vector<cv::Point2f> obj_corner(4);
		vector<cv::Point2f> scene_corner(4);
		obj_corner[0] = cv::Point(0, 0);
		obj_corner[1] = cv::Point(img1.cols, 0);
		obj_corner[2] = cv::Point(img1.cols, img1.rows);
		obj_corner[3] = cv::Point(0, img1.rows);
		if (!use_Affine)
			perspectiveTransform(obj_corner, scene_corner, Homography);//透视变换
		else
			cv::transform(obj_corner, scene_corner, Homography);

		drawMatches(img1, kpts1, img2, kpts2, correctMatches, Matches_resfine);
		line(Matches_resfine, scene_corner[0] + cv::Point2f(img1.cols, 0), scene_corner[1] + cv::Point2f(img1.cols, 0), cv::Scalar(0, 0, 255), 2, 8, 0);
		line(Matches_resfine, scene_corner[1] + cv::Point2f(img1.cols, 0), scene_corner[2] + cv::Point2f(img1.cols, 0), cv::Scalar(0, 0, 255), 2, 8, 0);
		line(Matches_resfine, scene_corner[2] + cv::Point2f(img1.cols, 0), scene_corner[3] + cv::Point2f(img1.cols, 0), cv::Scalar(0, 0, 255), 2, 8, 0);
		line(Matches_resfine, scene_corner[3] + cv::Point2f(img1.cols, 0), scene_corner[0] + cv::Point2f(img1.cols, 0), cv::Scalar(0, 0, 255), 2, 8, 0);

		cv::cvtColor(img1, img1, cv::COLOR_GRAY2RGB);
		cv::cvtColor(img2, img2, cv::COLOR_GRAY2RGB);

		namedWindow("PreciseMatchWithVFC", cv::WINDOW_NORMAL);
		imshow("PreciseMatchWithVFC", Matches_resfine);
		waitKey(0);
	}
	return true;
}

bool heq_func(const cv::Mat & dimage, std::vector<cv::Point2f>& key_pix_heq, const int iter_time)
{
	cv::Mat heq = dimage.clone();
	std::vector<std::vector<cv::Point>> contours, selected_contour;
	cv::threshold(heq,heq,10,255, cv::THRESH_BINARY);
	cv::findContours(heq, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	double maxArea = 0;
	std::vector<cv::Point> maxContour;
	for (size_t i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area > maxArea)
		{
			maxArea = area;
			maxContour = contours[i];
		}
	}
	cv::RotatedRect box = cv::minAreaRect(maxContour);
	cv::Point2f rect[4];
	cv::Point2f temp_pix;
	box.points(rect);
	temp_pix.x = (rect[0].x + rect[3].x) / 2;
	temp_pix.y = (rect[0].y + rect[3].y) / 2;
	rect[0] = temp_pix;
	key_pix_heq.push_back(temp_pix);
	temp_pix.x = (rect[1].x + rect[2].x) / 2;
	temp_pix.y = (rect[1].y + rect[2].y) / 2;
	rect[3] = temp_pix;
	key_pix_heq.push_back(temp_pix);
	
	for (int i = 0; i < iter_time; i++)
	{
		cv::Rect ccomp;
		cv::floodFill(heq, cv::Point(1, 1), cv::Scalar(255), &ccomp, cv::Scalar(20), cv::Scalar(20));
		cv::threshold(heq, heq, 10, 255, cv::THRESH_BINARY_INV);
		try {
			cv::findContours(heq, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			if (contours.size() < 0) { throw(-1); }     // can not find any shape
			double maxArea = 0;
			//std::vector<cv::Point> maxContour;
			for (size_t i = 0; i < contours.size(); i++)
			{
				double area = cv::contourArea(contours[i]);
				if (area > maxArea)
				{
					maxArea = area;
					maxContour = contours[i];
				}
			}
			selected_contour.push_back(maxContour);
			box = cv::minAreaRect(cv::Mat(maxContour));
			box.points(rect);
			temp_pix.x = (rect[0].x + rect[3].x) / 2;
			temp_pix.y = (rect[0].y + rect[3].y) / 2;
			key_pix_heq.push_back(temp_pix);
			temp_pix.x = (rect[1].x + rect[2].x) / 2;
			temp_pix.y = (rect[1].y + rect[2].y) / 2;
			key_pix_heq.push_back(temp_pix);
			line(heq, rect[0], rect[3], cv::Scalar(255), 4, 8);
		}
		catch (int e) { return false; }
	}

	//cv::namedWindow("heq", cv::WINDOW_NORMAL);
	//cv::imshow("heq", heq);
	//cv::waitKey(0);
	return true;
}

void Reprojector::_allocateMemory()
{
	cloud_in.reset(new pcl::PointCloud< pcl::PointXYZ >());
	cloud_centered.reset(new pcl::PointCloud< pcl::PointXYZ >());
	cloud_result.reset(new pcl::PointCloud< pcl::PointXYZ >());
	cloud_selected.reset(new pcl::PointCloud< pcl::PointXYZ >());
	selected_points.reset(new pcl::PointCloud< pcl::PointXYZ >());
	selected_points_center.reset(new pcl::PointCloud< pcl::PointXYZ >());
}

Reprojector::~Reprojector(void)
{
	matrix_index.setRandom();
}

Reprojector::Reprojector()
{
	_allocateMemory();
}

PtCloud Reprojector::loadPTData(std::string& filename)
{
	PtCloud container;
	std::string full_path = filename;

	std::cout << "loading..." << std::endl;
	container = restoreCloudDataBinary(full_path);

	__int64 data_size = 0;

	std::cout << "[info] load frames from :" << full_path.c_str() << std::endl;
		//<< "[info] countPerLine :" << container.getNodeCountPerLine() << std::endl
		//<< "[info] lineCount :" << container.getLineCount() << std::endl;
	return container;
}

bool Reprojector::reload_template(const std::string & path, const std::string & templatename)
{

	string templatename_product = path +  templatename + "/" + templatename +".ptcloud";
	string templatename_point = path +  templatename + "/" + templatename +".bin";
	fstream input(templatename_point, ios::in | ios::binary);
	if (!input.good()) 
	{
		cerr << "Could not read file: " << templatename_point << endl;
		return false;
	}
	input.seekg(0, ios::beg);
	this->selected_points->clear();
	for (int i = 0; input.good() && !input.eof(); i++) {
		pcl::PointXYZ point;
		input.read((char *)&point.x, 3 * sizeof(float));
		this->selected_points->push_back(point);
	}
	input.close();

	std::string config_file = path + templatename + "/" + "config.ini";
	//CSimpleIniA ini;
	SI_Error rc = inifile.LoadFile(config_file.data());
	if (rc < 0) return false;
	const char* pv;
	pv = inifile.GetValue("check_config", "debug");
	debug_model = std::stoi(pv);
	pv = inifile.GetValue("check_config", "crop_z_min");
	crop_z_min = std::stof(pv);
	pv = inifile.GetValue("check_config", "margin_x");
	margin_x = std::stof(pv);
	pv = inifile.GetValue("check_config", "margin_y");
	margin_y = std::stof(pv);
	pv = inifile.GetValue("check_config", "margin_check_x");
	margin_check_x = std::stof(pv);
	pv = inifile.GetValue("check_config", "margin_check_y");
	margin_check_y = std::stof(pv);
	pv = inifile.GetValue("check_config", "compress_x");
	compress_x = std::stof(pv);
	pv = inifile.GetValue("check_config", "compress_y");
	compress_y = std::stof(pv);
	pv = inifile.GetValue("check_config", "use_HEQ");
	use_HEQ = std::stoi(pv);
	generate_dimage(templatename_product, template_img, true);
	//cv::equalizeHist(template_img, template_img);
	cv::normalize(template_img, template_img, 0, 255, cv::NORM_MINMAX);
	return true;
}

bool Reprojector::reload_template(const PtCloud & pt_data, pcl::PointCloud<pcl::PointXYZ>::Ptr key_points, const std::string & ini_file)
{
	this->selected_points->clear();
	this->selected_points = key_points;

	std::string config_file = ini_file;
	//CSimpleIniA ini;
	SI_Error rc = inifile.LoadFile(config_file.data());
	if (rc < 0) return false;
	const char* pv;
	pv = inifile.GetValue("check_config", "debug");
	debug_model = std::stoi(pv);
	pv = inifile.GetValue("check_config", "crop_z_min");
	crop_z_min = std::stof(pv);
	pv = inifile.GetValue("check_config", "blur_size");
	blur_size = std::stof(pv);
	pv = inifile.GetValue("check_config", "margin_x");
	margin_x = std::stof(pv);
	pv = inifile.GetValue("check_config", "margin_y");
	margin_y = std::stof(pv);
	pv = inifile.GetValue("check_config", "margin_check_x");
	margin_check_x = std::stof(pv);
	pv = inifile.GetValue("check_config", "margin_check_y");
	margin_check_y = std::stof(pv);
	pv = inifile.GetValue("check_config", "compress_x");
	compress_x = std::stof(pv);
	pv = inifile.GetValue("check_config", "compress_y");
	compress_y = std::stof(pv);
	pv = inifile.GetValue("check_config", "use_HEQ");
	use_HEQ = std::stoi(pv);
	generate_dimage(pt_data, template_img, true);

	cv::normalize(template_img, template_img, 0, 255, cv::NORM_MINMAX);
	return true;
}

bool Reprojector::generate_dimage(std::string & templatename, cv::Mat & dist_image, const bool & load_template)
{
	PtCloud container = loadPTData(templatename);
	generate_dimage(container, dist_image, load_template);
	return true;
}

bool Reprojector::generate_dimage(const PtCloud & pt_data, cv::Mat & dist_image, const bool & load_template)
{
	int line_count = pt_data.getLineCount();
	int cols_count = pt_data.getNodeCountPerLine();
	if(line_count*cols_count < 32000)
	{
		std::cout << "[error] ptdata size error:" << line_count << " x " << cols_count << std::endl;
		return false;
	}
	std::cout << "[info] get ptdata:" << line_count << " x " << cols_count << std::endl;
	pcl::PointXYZ temp_point;
	pcl::PointCloud< pcl::PointXYZ > temp_cloud;
	std::vector<int> intensity;
	std::vector<PtNode> pt_nodes = pt_data.getNodes();
	std::cout << "get Nodes:" << pt_nodes.size() << std::endl;
	double max_x = 0, max_y = 0, min_x = 99999, min_y = 99999;
	for (int i = 0; i < pt_nodes.size(); i++)
	{
		if (!isnan(pt_nodes[i].laser.x()))
		{
			if (pt_nodes[i].laser.x() > max_x) max_x = pt_nodes[i].laser.x();
			else if (pt_nodes[i].laser.x() < min_x) min_x = pt_nodes[i].laser.x();
		}
		if (!isnan(pt_nodes[i].laser.y()))
		{
			if (pt_nodes[i].laser.y() > max_y) max_y = pt_nodes[i].laser.y();
			else if (pt_nodes[i].laser.y() < min_y) min_y = pt_nodes[i].laser.y();
		}
		if (pt_nodes[i].laser.hasNaN()) continue;
		else {

			temp_point.x = pt_nodes[i].laser.x();
			temp_point.y = pt_nodes[i].laser.y();
			temp_point.z = -pt_nodes[i].laser.z();

			intensity.push_back(i);
			temp_cloud.push_back(temp_point);
		}
	}
	std::cout << "[info] frame range:" << max_x << " - " << min_x << " ; " << max_y << " - " << min_y << endl;
	*cloud_in = temp_cloud;
	pcl::getMinMax3D(*cloud_in, min_p, max_p);				// get points max & min range in xyz

	gap_wide = (max_x - min_x) / (cols_count - 1) * compress_x;
	gap_height = (max_y - min_y) / (line_count - 1) * compress_y;
	if (gap_wide <= 0 || gap_height <= 0) {
		std::cout << "[error] pt points axis error !" << std::endl;
		return false;
	}
	line_count = (max_p.y - min_p.y) / gap_height + 1;
	cols_count = (max_p.x - min_p.x) / gap_wide + 1;
	Eigen::MatrixXd matrix_depth = Eigen::MatrixXd::Zero(line_count + (margin_y*2), cols_count + (margin_x*2));
	matrix_index = Eigen::MatrixXi::Zero(line_count + (margin_y * 2), cols_count + (margin_x * 2));

	std::cout << "[info] point range:" << max_p << min_p << std::endl;
	std::cout << "[info] depth img size:" << cols_count + 1 << " - " << line_count + 1 << std::endl;

	depth_range = max_p.z - min_p.z;
	std::cout << "[info] depth range:" << depth_range << std::endl;
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	/* move points to cameraframe center */
	transform_2.translation() << -min_p.x, -min_p.y, 0.0;
	transform_2.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())); // 0 deg in rotation as default
	pcl::transformPointCloud(*cloud_in, *cloud_centered, transform_2);

	for (int i = 0; i < cloud_centered->points.size(); i++)
	{
		int pos_x = cloud_centered->points[i].x / gap_wide + margin_x;
		int pos_y = cloud_centered->points[i].y / gap_height + margin_y;
		//matrix_depth(pos_y, pos_x) = (cloud_centered->points[i].z - min_p.z - (depth_range * crop_z_min)) / (depth_range * (1- crop_z_min)) * 255;
		//matrix_depth(pos_y, pos_x) = (cloud_centered->points[i].z - min_p.z- crop_z_min) / (depth_range) * 255;
		matrix_depth(pos_y, pos_x) = (cloud_centered->points[i].z - min_p.z- crop_z_min) / (depth_range) * 32767;
		matrix_index(pos_y, pos_x) = i;
	}
	if (load_template) 
	{
		key_pix.clear();
		pcl::transformPointCloud(*selected_points, *selected_points_center, transform_2);
		std::cout << "key point num: " << selected_points_center->points.size() <<endl;
		for (int i = 0; i < selected_points_center->points.size(); i++)
		{
			cv::Point2f temp_keypix;
			temp_keypix.x = selected_points_center->points[i].x / gap_wide + margin_x;
			temp_keypix.y = selected_points_center->points[i].y / gap_height + margin_y;
			if (temp_keypix.x > margin_x && temp_keypix.y > margin_y)
			{
				key_pix.push_back(temp_keypix);
				std::cout << "load key point: " << temp_keypix.x << " - " << temp_keypix.y << endl;
			}
		}
	}
	cv::Mat result_UP;
	cv::Mat img_temp, image_hist, image_hist_x, image_hist_y;
	cv::eigen2cv(matrix_depth, img_temp);
	img_temp.convertTo(img_temp, CV_16UC1);
	img_temp.convertTo(image_hist, CV_16SC1);
	cv::medianBlur(image_hist, image_hist, 3);
	cv::Sobel(image_hist, image_hist, CV_16S, 1, 0, 5);
	//cv::bitwise_and(image_hist, image_hist_x, image_hist);
	//cv::Sobel(image_hist, image_hist_y, CV_16S, 1, 0, 5.0);
	//cv::bitwise_and(image_hist_y, image_hist, image_hist);
	cv::normalize(image_hist, image_hist, 0, 255, cv::NORM_MINMAX);

	image_hist.convertTo(image_hist, CV_8UC1);
	cv::medianBlur(image_hist, image_hist, blur_size);
	dist_image = image_hist.clone();
	cv::namedWindow("dist_image", cv::WINDOW_GUI_NORMAL);
	cv::imshow("dist_image", dist_image);
	cv::waitKey(0);
	return true;
}

bool Reprojector::compare_dimage(std::string & src)
{
	cv::Mat dimage_2;
	generate_dimage(src,dimage_2);
	compare_dimage(dimage_2);
	return true;
}

bool Reprojector::compare_dimage(cv::Mat & dimage) {
	cv::Mat Homography, goodMathcesImage, template_img_homo;

	//cv::equalizeHist(dimage, dimage);
	cv::normalize(dimage, dimage, 0, 255, cv::NORM_MINMAX);

	key_pix_check.clear();
	key_pix_dist.clear();
	std::vector<cv::Point2f> key_pix_homo, key_pix_ref;
	if (use_HEQ) {
		heq_func(dimage, key_pix_check,1);
		key_pix_dist = key_pix_check;
	}
	else {
		if (!compare_AKAZE(template_img, dimage, Homography, goodMathcesImage, false, debug_model)) return false;
		cv::perspectiveTransform(key_pix, key_pix_homo, Homography);
		cv::warpPerspective(template_img, template_img_homo, Homography, template_img.size());
		points_refine(template_img_homo, dimage, key_pix_homo, key_pix_ref);
		key_pix_dist = key_pix_check;
		//key_pix_dist = key_pix_ref;
	}

	if (debug_model) {
		cv::cvtColor(dimage, dimage, cv::COLOR_GRAY2RGB);
		for (int i = 0; i < key_pix_ref.size(); i++)
		{
			//std::cout << key_pix_ref[i].x << " - " << key_pix_ref[i].y << endl;
			cv::circle(dimage, key_pix_ref[i], 2, cv::Scalar(0, 0, 255), 4);
			cv::putText(dimage, std::to_string(i), key_pix_ref[i],0,2, cv::Scalar(0, 0, 255),4);
		}
		for (int i = 0; i < key_pix_check.size(); i++)
		{
			//std::cout << key_pix_ref[i].x << " - " << key_pix_ref[i].y << endl;
			cv::circle(dimage, key_pix_check[i], 2, cv::Scalar(0, 255, 0), 4);
			cv::putText(dimage, std::to_string(i), key_pix_check[i], 0, 2, cv::Scalar(0, 255, 0), 3);
		}
		for (int i = 0; i < key_pix_homo.size(); i++)
		{
			cv::circle(dimage, key_pix_homo[i], 2, cv::Scalar(255, 0, 0), 4);
			cv::putText(dimage, std::to_string(i), key_pix_homo[i], 0, 2, cv::Scalar(255, 0, 0), 2);
		}
		cv::namedWindow("dimage", cv::WINDOW_NORMAL);
		cv::imshow("dimage", dimage);
		//cv::waitKey(0);
	}
	return true;
}

bool Reprojector::points_refine(cv::Mat & template_homo,cv::Mat & dimage, const std::vector<cv::Point2f>& key_pix_homo, std::vector<cv::Point2f>& points_refine)
{
	std::vector<cv::Point2f> point_res, center, center_realative, center_refine;
	cv::Rect key_roi, halcon_roi;
	cv::Mat Homography, goodMathces;
	for (int i = 0; i < key_pix_homo.size(); i++) 
	{
		if (key_pix_homo[i].x < margin_x || key_pix_homo[i].y < margin_y) continue;
		if (key_pix_homo[i].x > (dimage.cols-margin_x) || key_pix_homo[i].y > (dimage.rows-margin_y)) continue;
		center = { key_pix_homo[i] };
		center_realative = { cv::Point2f(margin_x,margin_y) };
		key_roi = cv::Rect(center[0].x - margin_x, center[0].y - margin_y, margin_x*2, margin_y*2);
		cv::Mat temp_crop = template_homo(key_roi);
		cv::Mat dimage_crop = dimage(key_roi);
		std::cout << key_pix_homo[i] << "";

		if (!compare_AKAZE(temp_crop, dimage_crop, Homography, goodMathces, true, debug_model)) {
			std::cout << " fail to refine !";
			center_refine.push_back(cv::Point2f(0, 0));
			center_refine[0].x = key_pix_homo[i].x;
			center_refine[0].y = key_pix_homo[i].y;
			cv::Point2f point_final;
			points_check(temp_crop, dimage_crop, center_realative[0], point_final);
			point_final.x = key_pix_homo[i].x + point_final.x - center_realative[0].x;
			point_final.y = key_pix_homo[i].y + point_final.y - center_realative[0].y;
			key_pix_check.push_back(point_final);
			point_res.push_back(center_refine[0]);
		}
		else {
			//cv::perspectiveTransform(center_realative, center_refine, Homography);
			cv::transform(center_realative, center_refine, Homography);
			
			std::cout << center_refine[0] <<" --";
			if (center_refine[0].x > margin_x && center_refine[0].y > margin_y) {
				cv::Mat temp_crop_homo;
				cv::warpAffine(temp_crop, temp_crop_homo, Homography, temp_crop.size());

				

				cv::Point2f point_final;
				points_check(temp_crop_homo, dimage_crop, center_refine[0], point_final);
				point_final.y = key_pix_homo[i].y + point_final.y - center_realative[0].y;
				point_final.x = key_pix_homo[i].x + point_final.x - center_realative[0].x;
				key_pix_check.push_back(point_final);
				//center_refine[0] = point_final;
				center_refine[0].x = key_pix_homo[i].x + center_refine[0].x - center_realative[0].x;
				center_refine[0].y = key_pix_homo[i].y + center_refine[0].y - center_realative[0].y;
				point_res.push_back(center_refine[0]);

			}
			else {
				center_refine.push_back(cv::Point2f(0, 0));
				center_refine[0].x = key_pix_homo[i].x;
				center_refine[0].y = key_pix_homo[i].y;
				cv::Point2f point_final;
				points_check(temp_crop, dimage_crop, center_realative[0], point_final);
				point_final.x = key_pix_homo[i].x + point_final.x - center_realative[0].x;
				point_final.y = key_pix_homo[i].y + point_final.y - center_realative[0].y;
				key_pix_check.push_back(point_final);
				point_res.push_back(center_refine[0]);
			}
		}
	}
	points_refine = point_res;
	return true;
}

bool Reprojector::points_check(cv::Mat & template_refine, cv::Mat & dimage_roi, const cv::Point2f & point_refine, cv::Point2f & point_final)
{

	//int margin_check_x = 250;
	//int margin_check_y = 250;
	cv::Point2f point_refine_centered(point_refine.x + margin_check_x, point_refine.y + margin_check_y);
	cv::Rect key_roi = cv::Rect(point_refine.x, point_refine.y, margin_check_x *2, margin_check_y *2);

	cv::copyMakeBorder(template_refine, template_refine, margin_check_y, margin_check_y, margin_check_x, margin_check_x, cv::BORDER_CONSTANT, cv::Scalar(0));
	cv::copyMakeBorder(dimage_roi, dimage_roi, margin_check_y, margin_check_y, margin_check_x, margin_check_x, cv::BORDER_CONSTANT, cv::Scalar(0));
	cv::Mat temp_crop = template_refine(key_roi);
	cv::Mat dimage_crop = dimage_roi(key_roi);
	cv::Mat Homography, goodMathces;

	std::vector<cv::Point2f> center_realative, center_ref;
	center_realative.push_back(cv::Point2f(margin_check_x, margin_check_y));
	if (!compare_AKAZE(temp_crop, dimage_crop, Homography, goodMathces, true, debug_model)) {
		point_final = point_refine;
		std::cout <<" --\n";
	}
	else {
		//cv::perspectiveTransform(center_realative, center_ref, Homography);
		cv::transform(center_realative, center_ref, Homography);

		center_ref[0].x = point_refine.x + center_ref[0].x - center_realative[0].x;
		center_ref[0].y = point_refine.y + center_ref[0].y - center_realative[0].y;
		std::cout << center_ref[0] << " --\n";
		point_final = center_ref[0];
	}
	return false;
}

std::vector<int> Reprojector::getKeypoints()
{
	std::vector<int> res;
	cloud_result.reset(new pcl::PointCloud< pcl::PointXYZ >());
	int range_r = 0;
	int range_c = 0;
	std::vector<cv::Point3f> raw3d, dist3d;
	for (int i = 0; i < key_pix_dist.size(); i++) 
	{
		int co = key_pix_dist[i].x - range_c;
		int ro = key_pix_dist[i].y - range_r;
		pcl::PointXYZ temp_point(0, 0, -1000);
		int final_index = -1;
		int index = -1;
		for (int diff_r = -range_r; diff_r <= range_r; diff_r++) {
			for (int diff_c = -range_c; diff_c <= range_c; diff_c++) {
				int index = matrix_index(ro + diff_r, co + diff_c);
				///todo: if no points get 
				if (index > 1)
				{
					if (index < cloud_in->points.size())
					{
						if (cloud_in->points[index].z > temp_point.z) {
							temp_point = cloud_in->points[index];
							final_index = index;
						}
					}
				}
			}
		}
		temp_point.x = (key_pix_dist[i].x - margin_x)*gap_wide + min_p.x;
		temp_point.y = (key_pix_dist[i].y - margin_y)*gap_height + min_p.y;
		if (final_index > 0) {
			cloud_result->points.push_back(temp_point);
			res.push_back(final_index);
			/*cv::Point3f temp_pointcv;
			temp_pointcv.x = temp_point.x;
			temp_pointcv.y = temp_point.y;
			temp_pointcv.z = temp_point.z;
			dist3d.push_back(temp_pointcv);*/
		}
	}
	/*for (int i = 0; i < res.size(); i++)
	{
		cv::Point3f temp_point;
		temp_point.x = cloud_result->points[i].x;
		temp_point.y = cloud_result->points[i].y;
		temp_point.z = cloud_result->points[i].z;
		raw3d.push_back(temp_point);
	}
	cv::Mat disthomo,inliners;
	cv::estimateAffine3D(raw3d, dist3d, disthomo, inliners);
	std::cout << disthomo << endl;

	raw3d.clear();
	dist3d.clear();
	for (int i = 0; i < selected_points->points.size(); i++)
	{
		cv::Point3f temp_point;
		temp_point.x = selected_points->points[i].x;
		temp_point.y = selected_points->points[i].y;
		temp_point.z = selected_points->points[i].z;
		raw3d.push_back(temp_point);
	}
	cv::transform(raw3d, dist3d, disthomo);
	cloud_result->points.clear();
	for (int i = 0; i < dist3d.size(); i++)
	{
		pcl::PointXYZ temp_point;
		temp_point.x = dist3d[i].x;
		temp_point.y = dist3d[i].y;
		temp_point.z = dist3d[i].z;
		cloud_result->points.push_back(temp_point);
	}*/
	std::cout << "[info] fetch result num:" << res.size() << endl;
	return res;
}

std::vector<cacRes> Reprojector::caculate(const std::string & config_path)
{
	//std::string config_file = config_path + "config.ini";
	//CSimpleIniA ini;
	//SI_Error rc = ini.LoadFile(config_file.data());
	//if (rc < 0) return std::vector<cacRes>();
	std::vector <cacRes> cacres;
	const char* pv;
	pv = inifile.GetValue("check_config", "all_num");
	int check_num = std::stoi(pv);
	const char* check_type;
	for(int i =0;i<check_num;i++)
	{
		char sname = 'A' + i ;
		string b(1,sname);
		check_type = inifile.GetValue(b.data(), "pnum");
		if (*check_type == '2')
		{
			const char* pindex;
			int index_1, index_2;
			const char* default_value;
			double default_min, default_max, diff=0;
			pindex = inifile.GetValue(b.data(), "pindex_1");
			index_1 = std::stoi(pindex);
			pindex = inifile.GetValue(b.data(), "pindex_2");
			index_2 = std::stoi(pindex);
			default_value = inifile.GetValue(b.data(), "default_min");
			default_min = std::stof(default_value);
			default_value = inifile.GetValue(b.data(), "default_max");
			default_max = std::stof(default_value);
			default_value = inifile.GetValue(b.data(), "diff");
			diff = std::stof(default_value);
			pcl::PointXYZ pt_A, pt_B;
			pt_A = cloud_result->points[index_1];
			pt_B = cloud_result->points[index_2];
			double distance = sqrt(pow((pt_A.x - pt_B.x),2) + pow((pt_A.y - pt_B.y), 2)) + diff;
			cacRes temp_res;
			temp_res.current_value = distance;
			temp_res.defult_min = default_min;
			temp_res.defult_max = default_max;
			temp_res.isgood = false;
			if (distance > default_min && distance < default_max) temp_res.isgood = true;
			cacres.push_back(temp_res);
			std::cout << "[result]" << sname << "-" <<
						"distance: " << temp_res.current_value <<
						" default: " << temp_res.defult_min << " - " << temp_res.defult_max << " " <<
						" status: " << temp_res.isgood <<
						"\n";
		}
		else if (*check_type == '0')
		{
			const char* pindex;
			int index_1, index_2;
			const char* default_value;
			double default_min, default_max, diff = 0;
			pindex = inifile.GetValue(b.data(), "pindex_1");
			index_1 = std::stoi(pindex);
			pindex = inifile.GetValue(b.data(), "pindex_2");
			index_2 = std::stoi(pindex);
			default_value = inifile.GetValue(b.data(), "default_min");
			default_min = std::stof(default_value);
			default_value = inifile.GetValue(b.data(), "default_max");
			default_max = std::stof(default_value);
			default_value = inifile.GetValue(b.data(), "diff");
			diff = std::stof(default_value);
			pcl::PointXYZ pt_A, pt_B;
			pt_A = cloud_result->points[index_1];
			pt_B = cloud_result->points[index_2];
			double distance = sqrt(pow((pt_A.x - pt_B.x), 2) + pow((pt_A.y - pt_B.y), 2)) + diff;
			cacRes temp_res;
			temp_res.current_value = distance;
			temp_res.defult_min = default_min;
			temp_res.defult_max = default_max;
			temp_res.isgood = false;
			if (distance > default_min && distance < default_max) temp_res.isgood = true;
			cacres.push_back(temp_res);
			std::cout << "[result]" << sname << "-" <<
				"distance: " << temp_res.current_value <<
				" default: " << temp_res.defult_min << " - " << temp_res.defult_max << " " <<
				" status: " << temp_res.isgood <<
				"\n";
		}
		else if (*check_type == '4')
		{
			const char* pindex;
			int index_1, index_2, index_3, index_4;
			const char* default_value;
			double default_min, default_max, diff=0;
			pindex = inifile.GetValue(b.data(), "pindex_1");
			index_1 = std::stoi(pindex);
			pindex = inifile.GetValue(b.data(), "pindex_2");
			index_2 = std::stoi(pindex);
			pindex = inifile.GetValue(b.data(), "pindex_3");
			index_3 = std::stoi(pindex);
			pindex = inifile.GetValue(b.data(), "pindex_4");
			index_4 = std::stoi(pindex);
			default_value = inifile.GetValue(b.data(), "default_min");
			default_min = std::stof(default_value);
			default_value = inifile.GetValue(b.data(), "default_max");
			default_max = std::stof(default_value);
			default_value = inifile.GetValue(b.data(), "diff");
			diff = std::stof(default_value);

			pcl::PointXYZ pt_A, pt_B, pt_C, pt_D;
			pt_A = cloud_result->points[index_1];
			pt_B = cloud_result->points[index_2];
			pt_C = cloud_result->points[index_3];
			pt_D = cloud_result->points[index_4];
			pt_A.x = (pt_A.x + pt_B.x) / 2;
			pt_A.y = (pt_A.y + pt_B.y) / 2;
			pt_B.x = (pt_C.x + pt_D.x) / 2;
			pt_B.y = (pt_C.y + pt_D.y) / 2;
			double distance = sqrt(pow((pt_A.x - pt_B.x), 2) + pow((pt_A.y - pt_B.y), 2)) + diff;
			cacRes temp_res;
			temp_res.current_value = distance;
			temp_res.defult_min = default_min;
			temp_res.defult_max = default_max;
			temp_res.isgood = false;
			if (distance > default_min && distance < default_max) temp_res.isgood = true;
			cacres.push_back(temp_res);
			std::cout << "[result]" << sname << "-" <<
				"distance: " << temp_res.current_value <<
				" default: " << temp_res.defult_min << " - " << temp_res.defult_max << " " <<
				" status: " << temp_res.isgood <<
				"\n";
		}
	}
	return cacres;
}

void Reprojector::setdebug(const bool value)
{
	debug_model = value;
}


