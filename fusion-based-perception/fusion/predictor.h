#pragma once

#include <Eigen/Core>
#include "input_data_type.h"
#include "object.h"

namespace kit {
namespace perception {
namespace fusion {

class Predictor {
 public:
    Predictor() = default;
    bool Predict(const FusionObjectListPtr &fusion_obj_list, double ts, proto_input::Pose &pose);

}; // class Predictor

}  // namespace fusion
}  // namespace perception
}  // namespace kit
