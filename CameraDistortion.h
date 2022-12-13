#pragma once
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

class CameraDistortion
{
	double fx_, fy_, cx_, cy_, k1_, k2_, k3_, p1_, p2_, skew_c_;

public:
	CameraDistortion();
	~CameraDistortion();
	void SetParams(double& fx, double& fy, double& cx, double& cy, double& k1, double& k2, double& k3, double& p1, double& p2, double& skew_c);
	void UndistortImage(const cv::Mat& distort_image, cv::Mat& undistort_image);
	void PixelToNormalized(double& px, double& py);
	void DistortionModel(double& px, double& py);
	void NormalizedToPixel(double& px, double& py);
	void CalculateCameraParameters(std::string path,int checkerboard[]);
};

