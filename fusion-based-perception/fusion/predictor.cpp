#include "predictor.h"

namespace kit {
namespace perception {
namespace fusion {

bool Predictor::Predict(const FusionObjectListPtr &fusion_obj_list, double ts, proto_input::Pose &pose) {
    // TODO: Predict global objects to local timestamp
    double state_time_ns = fusion_obj_list->time_ns;

    // for the uninitialized obj list, prediction is not needed.
    if (state_time_ns < 1) return true;

    // using ego-vehicle coordinates, prediction is not needed.
    // double dt = ts - state_time_ns;
    // for (size_t j = 0; j < fusion_obj_list->objs.size(); ++j) {
    //     auto &fusion_obj = fusion_obj_list->objs[j]; 
    //     fusion_obj->x = fusion_obj->x + pose.linear_velocity.x * dt + 0.5 * dt * dt * pose.linear_acceleration.x;
    //     fusion_obj->y = fusion_obj->y + pose.linear_velocity.y * dt + 0.5 * dt * dt * pose.linear_acceleration.y;
    //     fusion_obj->z = fusion_obj->z + pose.linear_velocity.z * dt + 0.5 * dt * dt * pose.linear_acceleration.z;
    // }

    return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace kit
