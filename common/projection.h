#ifndef PROJECTION_H
#define PROJECTION_H

#include "tools/rotation.h"
#include <opencv2/core/core.hpp>

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

template<typename T>
inline bool CamProjectionWithDistortion(const T* const_parameters, const T* camera, const T* point, T* predictions){
    // Rodrigues' formula
    T p[3];
    
    AngleAxisRotatePoint(camera, point, p);
    // camera[3,4] are the translation
    p[0] += const_parameters[0]; p[1] += camera[3]; p[2] += camera[4];

    //inner parameters
    T xp[2], yp[2], l1[2], l2[2], r2[2], distortion[2];
    T pixel_x[2], pixel_y[2];
    
    const T& c1_fx = const_parameters[1];
    const T& c1_fy = const_parameters[2];
    const T& c1_cx = const_parameters[3];
    const T& c1_cy = const_parameters[4];
    const T& c2_f_ratio = const_parameters[5]/const_parameters[6];
    const T& c2_cx = const_parameters[7];
    
    T& c2_fx = camera[7];
    T& c2_fy = camera[7]/c2_f_ratio;
    T& c2_cy = camera[8];
    
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
    
    
    predictions[0] = c1_fy * distortion[0] * yp[0] + c1_cy;
    predictions[1] = focal * c2_distortion * c2_yp;

    return true;
}



#endif // projection.h
