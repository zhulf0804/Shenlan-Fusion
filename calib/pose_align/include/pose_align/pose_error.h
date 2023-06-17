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

        Eigen::Transform<T, 3, Eigen::Affine> src_pose_t = src_pose.cast<T>();
        Eigen::Transform<T, 3, Eigen::Affine> base_pose_t = base_pose.cast<T>();
        Eigen::Transform<T, 3, Eigen::Affine> ext_src = ext * src_pose_t;
        Eigen::Transform<T, 3, Eigen::Affine> base_ext = base_pose_t * ext;
        Eigen::Transform<T, 3, Eigen::Affine> error_mat = ext_src.inverse() * base_ext;
        
        // error += error_mat.matrix().cwiseAbs().sum();
        // 用四元数比矩阵更加紧凑

        Eigen::Matrix<T, 3, 3> rotation_matrix = error_mat.linear(); 
        Eigen::Quaternion<T> rotation_quaternion(rotation_matrix); 
        // 当 x^2 + y^2 + z^2 = 0 时，w = 1, 表示单位矩阵
        error += rotation_quaternion.x() * rotation_quaternion.x();
        error += rotation_quaternion.y() * rotation_quaternion.y();
        error += rotation_quaternion.z() * rotation_quaternion.z();

        Eigen::Matrix<T, 3, 1> translation = error_mat.translation(); 
        error += translation.squaredNorm();

        *residual = ceres::sqrt(error);
        ////////////////////////////////////////////////////////////////

        return true;
    }

    Eigen::Affine3d src_pose;
    Eigen::Affine3d base_pose;
};

