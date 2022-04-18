#ifndef PROJECTION_H
#define PROJECTION_H

#include "tools/rotation.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

// camera : 9 dims array with 
// [0-2] : angle-axis rotation 
// [3-5] : translateion
// [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion
// point : 3D location. 
// predictions : 2D predictions with center of the image plane. 

// camera : 11 dims array with 
// [0-2] : angle-axis rotation 
// [3-4] : translateion
// [5-6] : k1, k2 of camera1
// [7-8] : fx, cy of camera2
// [9-10] : k1, k2 of camera2
// point : 3D location.  
// predictions : 2D predictions with center of the image plane. 

// [11] : baseline 
// [12-15] : fx, fy, cx, cy of camera1
// [16-18] : fx, fy, cx of camera2

using namespace cv;
using namespace std;

template<typename T>
inline bool CamProjectionWithDistortion(const T* camera, const T* point, T* predictions){
    // Rodrigues' formula
    T p[3];
    
    AngleAxisRotatePoint(camera, point, p);
    // camera[3,4] are the translation
    p[0] += camera[11]; p[1] += camera[3]; p[2] += camera[4];

    //inner parameters
    T xp[2], yp[2], l1[2], l2[2], r2[2], distortion[2];
    T pixel_x[2], pixel_y[2];
    
    const T& c1_fx = camera[12];
    const T& c1_fy = camera[13];
    const T& c1_cx = camera[14];
    const T& c1_cy = camera[15];
    const T& c2_f_ratio = camera[16]/camera[17];
    const T& c2_cx = camera[18];
    
    T c2_fx = camera[7];
    T c2_fy = camera[7]/c2_f_ratio;
    T c2_cy = camera[8];
    
    // Compute the center fo distortion
    xp[0] = -point[0]/point[2];
    yp[0] = -point[1]/point[2];
    
    xp[1] = -p[0]/p[2];
    yp[1] = -p[1]/p[2];

    // Apply second and fourth order radial distortion
    l1[0] = camera[5];
    l2[0] = camera[6];
    
    l1[1] = camera[9];
    l2[1] = camera[10];
    
    r2[0] = xp[0] * xp[0] + yp[0] * yp[0];
    distortion[0] = T(1.0) + r2[0] * (l1[0] + l2[0] * r2[0]);

    r2[1] = xp[1] * xp[1] + yp[1] * yp[1];
    distortion[1] = T(1.0) + r2[1] * (l1[1] + l2[1] * r2[1]);

    pixel_x[0] = c1_fx * distortion[0] * xp[0] + c1_cx;
    pixel_y[0] = c1_fy * distortion[0] * yp[0] + c1_cy;
    
    pixel_x[1] = c2_fx * distortion[1] * xp[1] + c2_cx;
    pixel_y[1] = c2_fy * distortion[1] * yp[1] + c2_cy;
    
    // rectification
    
    Mat cameraMatrix[2], distCoeffs[2];
    Size imageSize = Size(1920,1080);
    
    Mat cameraMatrix1 = (Mat_<double>(3,3) << c1_fx, 0, c1_cx, 0, c1_fy, c1_cy, 0, 0, 1);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << c2_fx, 0, c2_cx, 0, c2_fy, c2_cy, 0, 0, 1);
    Mat distCoeffs1 = (Mat_<double>(1, 2) << l1[0], l2[0]);
	Mat distCoeffs2 = (Mat_<double>(1, 2) << l1[1], l2[1]);
    Mat par_R = cv::Mat::zeros(3, 3, CV_64F);
    Mat R1, R2, P1, P2, Q;
	Mat E, F;
    
    Mat vec = (Mat_<double>(1, 3) << camera[0], camera[1], camera[2]);
	Rodrigues(vec, par_R);
    
    Mat par_T = (Mat_<double>(1, 3) << camera[11], camera[3], camera[4]);
    
    stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
                  imageSize, par_R, par_T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize);
                  
    vector<cv::Point2f> c1_point;
    vector<cv::Point2f> c2_point;
    vector<cv::Point2f> c1_point_rect;
    vector<cv::Point2f> c2_point_rect;
    
    c1_point.push_back(Point2f(pixel_x[0], pixel_y[0]));
    c2_point.push_back(Point2f(pixel_x[1], pixel_y[1]));
    
    undistortPoints(c1_point, c1_point_rect, cameraMatrix1, distCoeffs1, R1, P1);
    undistortPoints(c2_point, c2_point_rect, cameraMatrix2, distCoeffs2, R2, P2);
    
    predictions[0] = c1_point_rect.front().x;
    predictions[1] = c2_point_rect.front().y;

    return true;
}



#endif // projection.h
