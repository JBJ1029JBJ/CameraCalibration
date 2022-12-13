#include "CameraDistortion.h"

#define PATH "C:\\pattern\\*.png"

int main()
{
	int checkboard[2] = { 4,7 };

	CameraDistortion cameradistortion;
	cameradistortion.CalculateCameraParameters(PATH, checkboard);

	cv::Mat distort_image = cv::imread("C:\\pattern\\distpattern1.png",cv::IMREAD_GRAYSCALE);

	cv::Mat undistort_image = cv::Mat::zeros(distort_image.rows, distort_image.cols, distort_image.type());
	
	cameradistortion.UndistortImage(distort_image, undistort_image);

	cv::imshow("src", distort_image);
	cv::imshow("dst", undistort_image);

	cv::waitKey(0);
	return 0;

}