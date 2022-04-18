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
    
    bool operator()(const double* const camera, const double* const point, double* residuals)const{
        // camera[0,1,2] are the angle-axis rotation
        double predictions[2];
        CamProjectionWithDistortion(camera, point, predictions);
        residuals[0] = predictions[0] - predictions[1]; // y1-y2
     //   std::cout<<"residuals"<<std::endl;
     //   std::cout<<residuals[0]<<std::endl;

        return true;
    }

    static ceres::CostFunction* Create(){
        return (new ceres::NumericDiffCostFunction<SnavelyReprojectionError,ceres::FORWARD,1,11,3>(
            new SnavelyReprojectionError()));
    }

};

#endif // SnavelyReprojection.h

