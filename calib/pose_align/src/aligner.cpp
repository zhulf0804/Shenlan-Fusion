#include "pose_align/aligner.h"
#include "pose_align/eigen_quaternion_parameterization.h"
#include "pose_align/pose_error.h"
#include "pose_align/sensors.h"
#include "pose_align/transform.h"

namespace pose_align {

    void Aligner::align(Odom &odom_source, Odom &odom_target) {
        int odom_size = odom_target.getOdomTransformSize();
        std::cout<<"odom size is "<<odom_size<<std::endl;

        std::vector<Transform> odom_source_delta;
        std::vector<Transform> odom_target_delta;

        ceres::Problem problem;
        ceres::LossFunction *loss_func_ptr = new ceres::CauchyLoss(0.2);
        ceres::Solver::Summary summary;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 500;
        options.num_threads = 8;

        double translation[3] = {0, 0, 0};
        double quaternion[4] = {0, 0, 0., 1};

        int last_source_id = 0;

        for (int i = 0; i < odom_size; i++) {

            Transform last_src_pos = odom_target.getOdomTransformOfidx(last_source_id);
            Transform cur_src_pos = odom_target.getOdomTransformOfidx(i);
            Transform delta_src_pos = cur_src_pos.inverse() * last_src_pos;

            if(delta_src_pos.translation_.norm() < 1)
            {
                continue;
            }
            last_source_id = i;
            int last_id = i;
            for (int j = i + 1; j < odom_size; j++) {
                Transform src_pos = odom_target.getOdomTransformOfidx(last_id);
                Transform base_pos = odom_target.getOdomTransformOfidx(j);
                Transform delta_pos = base_pos.inverse() * src_pos;

                Transform delta_src_pos = base_pos.inverse() * cur_src_pos;

                Eigen::Vector3f euler_angle = delta_pos.rotation_.matrix().eulerAngles(2, 1, 0);
                double yaw = std::abs(euler_angle[0] * 180 / 3.1415926);
                double dist = delta_pos.translation_.norm();

                if (dist > 2 && delta_src_pos.translation_.norm() < 5) {
                    odom_source_delta.push_back(odom_source.getOdomTransformOfidx(i).inverse() * odom_source.getOdomTransformOfidx(j));
                    odom_target_delta.push_back(odom_target.getOdomTransformOfidx(i).inverse() * odom_target.getOdomTransformOfidx(j));
                    last_id = j;
                }
                
            }
        }

        std::cout <<" delta data size is "<<odom_target_delta.size()<<std::endl;

        for (size_t i = 0; i < odom_target_delta.size(); ++i) {
            Eigen::Affine3d source_transform;
            Eigen::Affine3d target_transform;

            source_transform.translation() = odom_source_delta[i].translation_.cast<double>();
            source_transform.linear() = odom_source_delta[i].rotation_.toRotationMatrix().cast<double>();

            target_transform.translation() = odom_target_delta[i].translation_.cast<double>();
            target_transform.linear() = odom_target_delta[i].rotation_.toRotationMatrix().cast<double>();

            ceres::CostFunction *cost_func =
                    new ceres::AutoDiffCostFunction<HandEyeCostFunction, 1, 1, 1, 1, 4>
                    (new HandEyeCostFunction(target_transform, source_transform));
            problem.AddResidualBlock(cost_func, loss_func_ptr, translation,
                                        translation + 1, translation + 2, quaternion);
        }

        ceres::LocalParameterization *local_para_ptr = new EigenQuaternionParameterization;
        problem.SetParameterization(quaternion, local_para_ptr);

        if (fix_extrinsic_z_) {
            problem.SetParameterBlockConstant(translation + 2);
        }

        ceres::Solve(options, &problem, &summary);
        Eigen::Quaternionf quat_result(quaternion[3], quaternion[0], quaternion[1],
                                        quaternion[2]);
        Eigen::Vector3f tras_result = Eigen::Vector3f(translation[0], translation[1], translation[2]);
        Transform pose_result;
        pose_result.rotation_ = quat_result;
        pose_result.translation_ = tras_result;

        pose_result = pose_result.inverse();

        std::cout<<"the extrinsic is "<<std::endl;
        std::cout<<"pose_result quaternion is "<<pose_result.rotation_.coeffs().transpose()<<std::endl;
        std::cout<<"pose_result eular is "<<pose_result.rotation_.matrix().eulerAngles(2, 0, 1).transpose() * 180 / 3.1415926<<std::endl;
        std::cout<<"pose_result translation is "<<pose_result.translation_.transpose()<<std::endl;

        Transform::Rotation gt_ext_quat(0.707, 0, 0, 0.707);
        Transform::Translation gt_ext_trans(0.3, 0.4, 0.0);
        Transform gt_ext_pose = Transform(gt_ext_trans, gt_ext_quat);

        std::cout<<"gt_ext_pose quaternion is "<<gt_ext_pose.rotation_.coeffs().transpose()<<std::endl;
        std::cout<<"gt_ext_pose eular is "<<gt_ext_pose.rotation_.matrix().eulerAngles(2, 0, 1).transpose() * 180 / 3.1415926<<std::endl;
        std::cout<<"gt_ext_pose translation is "<<gt_ext_pose.translation_.transpose()<<std::endl;
    }

}  // namespace pose_align
