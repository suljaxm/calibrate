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

#include <iostream>
#include <iomanip>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "CameraCalibrator.h"

int main()
{

	cv::namedWindow("Image");
	cv::Mat image;
	std::vector<std::string> filelist;

    for (int i=1; i<20; i++)
    {

		std::stringstream str;
        str << "../chessboards/chessboard" <<std::setw(2) << std::setfill('0') << i << ".jpg";  //load raw pictures
		std::cout << str.str() << std::endl;

		filelist.push_back(str.str());
		image= cv::imread(str.str(),0);
		cv::imshow("Image",image);
	
		 cv::waitKey(100);
	}
    std::cout << "picture over " << std::endl;
	// Create calibrator object
    CameraCalibrator cameraCalibrator;
    // add the corners from the chessboard
    cv::Size boardSize(6,4);
    //cv::Size boardSize(9,6);
	cameraCalibrator.addChessboardPoints
	(
		filelist,	// filenames of chessboard image
		boardSize// size of chessboard
	);	
		// calibrate the camera
    //	cameraCalibrator.setCalibrationFlag(true,true);
	std::vector<cv::Mat> rvecs, tvecs;
    cameraCalibrator.calibrate(image.size(),rvecs,tvecs);
	for(int i=0;i<tvecs.size();++i)
	std::cout <<tvecs[i] <<std::endl;
    std::cout << std::endl;
    // Image Undistortion
    image = cv::imread(filelist[6]);
	cv::Mat uImage= cameraCalibrator.remap(image);

	// display camera matrix
//    cv::Mat cameraMatrix= cameraCalibrator.getCameraMatrix();
//    std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
//    std::cout << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(0,1) << " " << cameraMatrix.at<double>(0,2) << std::endl;
//    std::cout << cameraMatrix.at<double>(1,0) << " " << cameraMatrix.at<double>(1,1) << " " << cameraMatrix.at<double>(1,2) << std::endl;
//    std::cout << cameraMatrix.at<double>(2,0) << " " << cameraMatrix.at<double>(2,1) << " " << cameraMatrix.at<double>(2,2) << std::endl;

    cv::Mat cameraMatrix = cameraCalibrator.getCameraMatrix();
    std::cout << "  Camera intrinsic" << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
	for (int i = 0; i<cameraMatrix.rows; i++)
		for (int j = 0; j<cameraMatrix.cols; j++)
		{
			std::cout << std::setw(10) << cameraMatrix.at<double>(i, j);
			if (j == 2)
				std::cout << std::endl;
		}

	cv::Mat distCoeffs = cameraCalibrator.getDistCoeffs();
    std::cout << " camera distortion " << distCoeffs.rows << "x" << distCoeffs.cols << std::endl;
    for (int i = 0; i<distCoeffs.rows; i++)
        for (int j = 0; j<distCoeffs.cols; j++)
            std::cout << distCoeffs.at<double>(i, j) << "\t";


    imshow("Original Image", image);
    imshow("Undistorted Image", uImage);

	cv::waitKey();
	return 0;
}
