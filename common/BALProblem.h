#ifndef BALPROBLEM_H
#define BALPROBLEM_H

#include <stdio.h>
#include <string>
#include <iostream>


class BALProblem
{
public:
    explicit BALProblem(const std::string& filename, bool use_quaternions = false);
    ~BALProblem(){
        delete[] parameters_;
    }

    void WriteToFile(const std::string& filename)const;
    void WriteToPLYFile(const std::string& filename)const;

    void Normalize();

    void Perturb(const double rotation_sigma,
                 const double translation_sigma,
                 const double point_sigma);
    
    
    int camera_block_size()             const{ return 11;                        }
    int point_block_size()              const{ return 3;                         }             
    int num_points()                    const{ return num_points_;               }
    int num_parameters()                const{ return num_parameters_;           }
    const double* parameters()          const{ return parameters_;               }
    const double* cameras()             const{ return parameters_;               }
    const double* points()              const{ return parameters_ + 11;          }
    double* mutable_cameras()                { return parameters_;               }
    double* mutable_points()                 { return parameters_ + 11;          }

private:
    void CameraToAngelAxisAndCenter(const double* camera,
                                    double* angle_axis,
                                    double* center)const;

    void AngleAxisAndCenterToCamera(const double* angle_axis,
                                    const double* center,
                                    double* camera)const;

    int num_points_;
    int num_parameters_;
    
    bool use_quaternions_;

    double* parameters_; 

};

#endif // BALProblem.h
