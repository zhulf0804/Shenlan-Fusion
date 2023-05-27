#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <string>
#include <vector>

struct HandEyeCostFunction {
    HandEyeCostFunction(const Eigen::Affine3d &src,
                        const Eigen::Affine3d &base)
            : src_pose(src), base_pose(base) {
    
    }

    template<typename T>
    bool operator()(const T *x, const T *y, const T *z, const T *rotation,
                    T *residual) const {
        
        // TODO: homework

        //////////////////////////////////////////////////////////////

        Eigen::Quaternion<T> quat(rotation[3], rotation[0], rotation[1],
                                  rotation[2]);
        Eigen::Transform<T, 3, Eigen::Affine> ext;
        ext.translation() = Eigen::Matrix<T, 3, 1>(*x, *y, *z);
        ext.linear() = quat.toRotationMatrix();
        T error = T(0);
        













        *residual = ceres::sqrt(error);

        ////////////////////////////////////////////////////////////////

        return true;
    }

    Eigen::Affine3d src_pose;
    Eigen::Affine3d base_pose;
};

