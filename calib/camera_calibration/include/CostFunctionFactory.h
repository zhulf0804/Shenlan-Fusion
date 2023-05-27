#pragma once

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "PinholeCamera.h"

namespace ceres
{
    class CostFunction;
}

class CostFunctionFactory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CostFunctionFactory();

    static boost::shared_ptr<CostFunctionFactory> instance(void);

    ceres::CostFunction* generateCostFunction(const PinholeCameraConstPtr& camera,
                                              const Eigen::Vector3d& observed_P,
                                              const Eigen::Vector2d& observed_p) const;
private:
    static boost::shared_ptr<CostFunctionFactory> m_instance;
};


