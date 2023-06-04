#include "CostFunctionFactory.h"
#include "ceres/ceres.h"
#include "PinholeCamera.h"
#include <Eigen/Core>
#include <typeinfo>

class ReprojectionErrorAutoDiff
{
public:

    ReprojectionErrorAutoDiff(const Eigen::Vector3d& observed_P,
                       const Eigen::Vector2d& observed_p)
        : m_observed_P(observed_P), m_observed_p(observed_p) {}

    // variables: camera intrinsics and camera extrinsics
    template <typename T>
    bool operator()(const T* const params,
                    const T* const q,
                    const T* const t,
                    T* residuals) const
    {
        Eigen::Matrix<T, 3, 1> P = m_observed_P.cast<T>();
        Eigen::Matrix<T, 2, 1> predicted_p;

        Eigen::Matrix<T, 2, 1> e = Eigen::Matrix<T, 2, 1>::Zero();

        // TODO: homework2

        // 完成相机的投影过程，计算重投影误差
        T P_w[3];
        P_w[0] = T(P(0));
        P_w[1] = T(P(1));
        P_w[2] = T(P(2));

        // Convert quaternion from Eigen convention (x, y, z, w)
        // to Ceres convention (w, x, y, z)
        T q_ceres[4] = {q[3], q[0], q[1], q[2]};
        // project 3D object point to the image plane
        T P_c[3];
        ceres::QuaternionRotatePoint(q_ceres, P_w, P_c);
        P_c[0] += t[0];
        P_c[1] += t[1];
        P_c[2] += t[2];

        // Transform to model plane
        T x, y;
        x = P_c[0] / P_c[2];
        y = P_c[1] / P_c[2];

        // params: k1, k2, p1, p2, fx, fy, cx, cy
        T r2 = pow(x, 2) + pow(y, 2);
        T k1, k2, p1, p2;
        k1 = params[0];
        k2 = params[1];
        p1 = params[2];
        p2 = params[3];
        T x_distored = x + x * k1 * r2 + x * k2 * pow(r2, 2) + T(2.0) * p1 * x * y + p2 * (r2 + T(2.0) * pow(x, 2));
        T y_distored = y + y * k1 * r2 + y * k2 * pow(r2, 2) + p1 * (r2 + T(2.0) * pow(y, 2)) + T(2.0) * p2 * x * y;
        predicted_p(0) = params[4] * x_distored + params[6];
        predicted_p(1) = params[5] * y_distored + params[7];

        // e = (predicted_p - m_observed_p).array().abs();
        e = (predicted_p - m_observed_p.cast<T>());

        residuals[0] = e(0);
        residuals[1] = e(1);

        return true;
    }

    // observed 3D point
    Eigen::Vector3d m_observed_P;

    // observed 2D point
    Eigen::Vector2d m_observed_p;
};


boost::shared_ptr<CostFunctionFactory> CostFunctionFactory::m_instance;

CostFunctionFactory::CostFunctionFactory()
{

}

boost::shared_ptr<CostFunctionFactory>
CostFunctionFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CostFunctionFactory);
    }

    return m_instance;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const PinholeCameraConstPtr& camera,
        const Eigen::Vector3d& observed_P,
        const Eigen::Vector2d& observed_p) const
{

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);
    ceres::CostFunction* costFunction = nullptr;
    
    costFunction = new ceres::AutoDiffCostFunction<ReprojectionErrorAutoDiff, 2, 8, 4, 3>(
                  new ReprojectionErrorAutoDiff(observed_P, observed_p));

    return costFunction;
}

