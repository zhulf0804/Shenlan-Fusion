#include "CostFunctionFactory.h"
#include "ceres/ceres.h"
#include "PinholeCamera.h"
#include <Eigen/Core>

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







        // project 3D object point to the image plane






        // Transform to model plane








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

