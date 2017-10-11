/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 9 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>

class CameraCalibrator {

    // input points输入点
    //位于世界坐标的点
    std::vector<std::vector<cv::Point3f>> objectPoints;
    //像素坐标的点
    std::vector<std::vector<cv::Point2f>> imagePoints;

    // output Matricesfiiii
    // 输出矩阵，内参数阵和畸变系数真
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

	// flag to specify how calibration is done
    // 标定的方式
	int flag;
	// used in image undistortion 
    // 用于图像去畸变
    cv::Mat map1,map2; 
	bool mustInitUndistort;

  public:
	CameraCalibrator() : flag(0), mustInitUndistort(true) {};

	// Open the chessboard images and extract corner points
    // 打开棋盘图像并提取角点
	int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);
	// Add scene points and corresponding image points
    void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
	// Calibrate the camera
    double calibrate(const cv::Size &imageSize, std::vector<cv::Mat> &ri, std::vector<cv::Mat> &ti);
    // Set the calibration flag
    void setCalibrationFlag(bool radial8CoeffEnabled=false, bool tangentialParamEnabled=false);
	// Remove distortion in an image (after calibration)
	cv::Mat /*CameraCalibrator::*/remap(const cv::Mat &image);

    // Getters
    //获得相机的内参数矩阵
    cv::Mat getCameraMatrix() { return cameraMatrix; }

    //获取相机的畸变系数
    cv::Mat getDistCoeffs()   { return distCoeffs; }
	//cv::Mat getrvecs() { return rvecs; }
};

#endif // CAMERACALIBRATOR_H
