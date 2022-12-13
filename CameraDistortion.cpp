#include "CameraDistortion.h"

CameraDistortion::CameraDistortion()
{
	fx_ = fy_ = cx_ = cy_ = k1_ = k2_ = k3_ = p1_ = p2_ = skew_c_ = 0;
}

CameraDistortion::~CameraDistortion(){}

void CameraDistortion::SetParams(double& _fx, double& _fy, double& _cx, double& _cy, double& _k1, double& _k2, double& _k3, double& _p1, double& _p2, double& _skew_c)
{
	fx_ = _fx;
	fy_ = _fy;
	cx_ = _cx;
	cy_ = _cy;
	k1_ = _k1;
	k2_ = _k2;
	k3_ = _k3;
	p1_ = _p1;
	p2_ = _p2;
	skew_c_ = _skew_c;
}

void CameraDistortion::UndistortImage(const cv::Mat& distort_image, cv::Mat& undistort_image)
{
	int height = distort_image.rows;
	int width = distort_image.cols;
	int channel = distort_image.channels();

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			double px = x;
			double py = y;

			PixelToNormalized(px, py);
			DistortionModel(px, py);
			NormalizedToPixel(px, py);

			if (px >= 0 && py >= 0 && px < width && py < height)
			{
				undistort_image.at<uchar>(y, x) = distort_image.at<uchar>(py, px);
			}
		}
	}

}

void CameraDistortion::PixelToNormalized(double& px, double& py)
{
	double x_n_u, y_n_u;

	y_n_u = (py - cy_) / fy_;
	x_n_u = (px - cx_) / fx_ - skew_c_ * y_n_u;

	px = x_n_u;
	py = y_n_u;
}

void CameraDistortion::DistortionModel(double& px, double& py)
{
	double ru2 = px * px + py * py;
	double radial_distortion = (1 + k1_ * ru2 + k2_ * ru2 * ru2 + k3_ * ru2 * ru2 * ru2);

	double x_n_d, y_n_d;

	x_n_d = radial_distortion * px + 2 * p1_ * px * py + p2_ * (radial_distortion * 2 * px * px);
	y_n_d = radial_distortion * py + p1_ * (radial_distortion * 2 * py * py) + 2 * p2_ * px * py;

	px = x_n_d;
	py = y_n_d;
}

void CameraDistortion::NormalizedToPixel(double& px, double& py)
{
	double x_p_d, y_p_d;

	x_p_d = fx_ * (px + skew_c_ * py) + cx_;
	y_p_d = fy_ * py + cy_;

	px = x_p_d;
	py = y_p_d;
}

void CameraDistortion::CalculateCameraParameters(std::string path, int checkerboard[])
{
	std::vector<std::vector<cv::Point3f>> object_points;									// 3D 실세계 좌표
	std::vector<std::vector<cv::Point2f>> image_points;										// 2D 이미지 좌표

	std::vector<cv::Point3f> temp_object_points;
	for (int i = 0; i < checkerboard[1]; i++)
	{
		for (int j = 0; j < checkerboard[0]; j++)
		{
			temp_object_points.push_back(cv::Point3f(j,i,0));
		}
	}

	std::vector<cv::String> images;

	cv::glob(path, images);

	cv::Mat frame, gray_frame;

	std::vector<cv::Point2f> corner_points;
	bool success;

	for (int i = 0; i < images.size(); i++)
	{
		frame = cv::imread(images[i]);
		cv::cvtColor(frame, gray_frame, CV_RGB2GRAY);

		// findChessboardCorners : 입력 image가 chessboard pattern인지 확인하고 체스판 모서리를 찾는다.
		// 모서리의 배치가 특정 순서로 배치된 경우 true 반환, 그렇지 않으면 false
		// 보다 정확한 좌표를 결정하기 위해 cornerSubPix 함수와 사용

		// CV_CALIB_CB_ADAPTIVE_THRESH = 1	CV_CALIB_CB_NORMALIZE_IMAGE = 2	 CV_CALIB_CB_FAST_CHECK = 8
		// CV_CALIB_CB_ADAPTIVE_THRESH : image 흑백 변환
		// CV_CALIB_CB_NORMALIZE_IMAGE : equalizeHist를 사용하여 이미지 감마를 정규화
		// CV_CALIB_CB_FAST_CHECK : 체스판 모서리를 찾는 이미지에 대한 빠른 검사
		success = cv::findChessboardCorners(gray_frame, cv::Size(checkerboard[0], checkerboard[1]), corner_points, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FAST_CHECK);

		if (success)
		{
			// TermCriteria
			// {
			//		int type;			CV_TERMCRIT_ITER 혹은 CV_TERMCRIT_EPS 의 OR 연산	
			//		int max_iter;		최대 반복 횟수
			//		double epsilon;		정확도
			// }
			// CV_TERMCRIT_ITER : 알고리즘 수행 최대 반복 회수, max_iter 변수
			// CV_TERMCRIT_EPS  : 얼마나 정확한 정도까지 수행할 것인지 나타내는 정확도, epsilon 변수
			cv::TermCriteria criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.001);

			cv::cornerSubPix(gray_frame, corner_points, cv::Size(11, 11), cv::Size(-1, -1), criteria);

			cv::drawChessboardCorners(frame, cv::Size(checkerboard[0], checkerboard[1]), corner_points, success);

			object_points.push_back(temp_object_points);
			image_points.push_back(corner_points);
		}
		cv::imshow("img", frame);
		cv::waitKey(0);
	}
	cv::destroyAllWindows();

	cv::Mat camera_matrix, distortion_coefficients, R, T;

	cv::calibrateCamera(object_points, image_points, cv::Size(gray_frame.rows, gray_frame.cols), camera_matrix, distortion_coefficients, R, T);
	// camera_matrix = { fx,   0,   cx
	//					  0,   fy,  cy
	//					  0    0    0}

	//distortion_coefficients = (k1, k2, p1, p2[, k3, [, k4, k5, k6[, s1, s2, s3, s4[, taux, tauy]]]])

	std::cout << "cameraMatrix : " << camera_matrix << std::endl;
	std::cout << "distCoeffs : " << distortion_coefficients << std::endl;
	std::cout << "Rotation vector : " << R << std::endl;
	std::cout << "Translation vector : " << T << std::endl;

	fx_ = camera_matrix.at<double>(0, 0);
	skew_c_ = camera_matrix.at<double>(0, 1);
	cx_ = camera_matrix.at<double>(0, 2);
	fy_ = camera_matrix.at<double>(1, 1);
	cy_ = camera_matrix.at<double>(1, 2);

	k1_ = distortion_coefficients.at<double>(0, 0);
	k2_ = distortion_coefficients.at<double>(0, 1);
	p1_ = distortion_coefficients.at<double>(0, 2);
	p2_ = distortion_coefficients.at<double>(0, 3);
	k3_ = distortion_coefficients.at<double>(0, 4);

}