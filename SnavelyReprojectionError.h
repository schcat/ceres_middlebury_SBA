#ifndef SnavelyReprojection_H
#define SnavelyReprojection_H

#include <iostream>
#include "ceres/ceres.h"


#include "common/tools/rotation.h"
#include "common/projection.h"

class SnavelyReprojectionError
{
public:
//   SnavelyReprojectionError(double observation_x, double observation_y):observed_x(observation_x),observed_y(observation_y){}

    SnavelyReprojectionError(){}
    
template<typename T>
    bool operator()(const T* const const_parameters, const T* const camera, const T* const point, T* residuals)const{
        // camera[0,1,2] are the angle-axis rotation
        T predictions[2];
        CamProjectionWithDistortion(const_parameters, camera, point, predictions);
        residuals[0] = predictions[0] - predictions[1]; // y1-y2

        return true;
    }

    static ceres::CostFunction* Create(){
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError,1,11,3>(
            new SnavelyReprojectionError()));
    }

};

#endif // SnavelyReprojection.h

