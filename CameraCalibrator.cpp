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

#include "CameraCalibrator.h"

// Open chessboard images and extract corner points
/************************************************************************/
/* ????            ????????????
 * @Input              filelist????
 * @Input              boardSize????
 @ ??                ??????????*/
/************************************************************************/
int CameraCalibrator::addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize)
{

	// the points on the chessboard
    // ?????????
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Point3f> objectCorners;

    // 3D Scene Points:3D ?????
    // Initialize the chessboard corners ??????????????
    // in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z)= (i,j,0) ?????(X,Y,Z)=(i,j,0)
	for (int i=0; i<boardSize.height; i++)
	{
		for (int j=0; j<boardSize.width; j++) 
		{
			objectCorners.push_back(cv::Point3f(i, j, 0.0f));
		}
    }

    // 2D Image points:
    cv::Mat image; // to contain chessboard image ????????
    int successes = 0;
    // for all viewpoints  ????
    for (int i=0; i<filelist.size(); i++) 
	{
        // Open the image ????
        image = cv::imread(filelist[i],0);

        // Get the chessboard corners ????
        bool found = cv::findChessboardCorners(image, boardSize, imageCorners);

        // Get subpixel accuracy on the corners????????
        cv::cornerSubPix(image, imageCorners,
                         cv::Size(5,5),
                         cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER +cv::TermCriteria::EPS,
                                          30,// max number of iterations maxCount=30
                                          0.1)// min accuracy????
                         );

          // If we have a good board, add it to our data ????????????????????????????
		  if (imageCorners.size() == boardSize.area()) {

            // Add image and scene points from one view???????????????
            addPoints(imageCorners, objectCorners);
            successes++;
          }

        //Draw the corners
        cv::drawChessboardCorners(image, boardSize, imageCorners, found);
        cv::imshow("Corners on Chessboard", image);
        cv::waitKey(100);
    }

	return successes;
}

// Add scene points and corresponding image points
/************************************************************************/
/* ?????       ????????????
 * @Input           imageCorners???
 * @Input           objectCorners???*/
/************************************************************************/
void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners)
{

    // 2D image points from one view 2D ???
	imagePoints.push_back(imageCorners);          
    // corresponding 3D scene points ??3D?????
	objectPoints.push_back(objectCorners);
}

// Calibrate the camera
// returns the re-projection error
/************************************************************************/
/* ?????     ????????????
 * @Input         imageSize????*/
/************************************************************************/
double CameraCalibrator::calibrate(const cv::Size &imageSize, std::vector<cv::Mat> &ri, std::vector<cv::Mat> &ti)
{
    // undistorter must be reinitialized ???????
	mustInitUndistort= true;
    //???????
	std::vector<cv::Mat> rvecs, tvecs;
	//Output rotations and translations
	//std::vector<std::vector<cv::Point3f>> rvecs, tvecs;
	//_InputOutputArray::OutputArrayOfArrays rvecs;
	// start calibration
     calibrateCamera(objectPoints, // the 3D points 3D?
                    imagePoints,  // the image points ???
                    imageSize,    // image size ????
                    cameraMatrix, // output camera matrix ???????
                    distCoeffs,   // output distortion matrix ???????
                    rvecs, tvecs, // Rs, Ts ?????
                    flag);        // set options ????
//					,CV_CALIB_USE_INTRINSIC_GUESS);
	 ri = rvecs;
	 ti = tvecs;
	 
	 return 0;

}

// remove distortion in an image (after calibration)
/************************************************************************/
/* ?????            ?????????????
 * ???@Input          image????????
 * ???@Return         ???????
 */
 /************************************************************************/
cv::Mat CameraCalibrator::remap(const cv::Mat &image) {

	cv::Mat undistorted;

    if (mustInitUndistort) { // called once per calibration ???????????
    
		cv::initUndistortRectifyMap(
            cameraMatrix,  // computed camera matrix ??????
            distCoeffs,    // computed distortion matrix ?????????
            cv::Mat(),     // optional rectification (none) 
			cv::Mat(),     // camera matrix to generate undistorted
			cv::Size(640,480),
//            image.size(),  // size of undistorted ????????
            CV_32FC1,      // type of output map ?????????
            map1, map2);   // the x and y mapping functions x???y??????

        mustInitUndistort= false;
	}

    // Apply mapping functions  ??????
    cv::remap(image, undistorted, map1, map2, 
		cv::INTER_LINEAR); // interpolation type

	return undistorted;
}


// Set the calibration options
// 8radialCoeffEnabled should be true if 8 radial coefficients are required (5 is default)
// tangentialParamEnabled should be true if tangeantial distortion is present
void CameraCalibrator::setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled) {

    // Set the flag used in cv::calibrateCamera()
    flag = 0;
    if (!tangentialParamEnabled) flag += CV_CALIB_ZERO_TANGENT_DIST;
	if (radial8CoeffEnabled) flag += CV_CALIB_RATIONAL_MODEL;
}

