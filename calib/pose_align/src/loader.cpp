#include "pose_align/loader.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <iostream>

#include "pose_align/sensors.h"
#include "pose_align/transform.h"


namespace pose_align {

    Loader::Loader() {}

    bool Loader::loadPoses(std::string &data_path) {
        std::ifstream fp(data_path.c_str());
        
        if (!fp.eof()) {
            std::string line;

            int index = 0;

            while (std::getline(fp, line)) {
                std::vector<std::string> split_str;
                boost::split(split_str, line, boost::is_any_of("  ,\t"));
                // add one judge condition

                if (split_str.size() == 7) {
                    Transform::Rotation quat(
                            std::atof(split_str[6].c_str()), std::atof(split_str[3].c_str()),
                            std::atof(split_str[4].c_str()), std::atof(split_str[5].c_str()));
                    Transform::Translation trans(std::atof(split_str[0].c_str()),
                                                 std::atof(split_str[1].c_str()),
                                                 std::atof(split_str[2].c_str()));

                    Transform pose = Transform(trans, quat);
                    double timestamp = 0.1 * (index++);
                    odom_target.addTransformData(timestamp, pose);

                    Transform::Rotation ext_quat(0.707, 0, 0, 0.707);
                    Transform::Translation ext_trans(0.3, 0.4, 0.0);
                    Transform ext_pose = Transform(ext_trans, ext_quat);

                    Transform pose_ = pose * ext_pose;

                    odom_source.addTransformData(timestamp, pose_);
                } else {
                    std::cerr << "The format of poses.txt is wrong." << std::endl;
                }
            }
        }

        std::cout << "odom_size is " << odom_target.getOdomTransformSize()
                  << std::endl;
    }

}  // namespace pose_align
