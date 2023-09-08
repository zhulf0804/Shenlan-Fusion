import os
import re
import numpy as np


def read_file(file_path):
    with open(file_path, 'r') as file:
        content = file.read()
        pose_content = re.search('pose {.*}', content, re.DOTALL)
        if pose_content:
            position = re.search('position {.*?}', pose_content.group(), re.DOTALL)
            orientation = re.search('orientation {.*?}', pose_content.group(), re.DOTALL)
            linear_velocity = re.search('linear_velocity {.*?}', pose_content.group(), re.DOTALL)
            linear_acceleration = re.search('linear_acceleration {.*?}', pose_content.group(), re.DOTALL)
            if position and orientation and linear_velocity:
                x = re.search('x: (.*)\n', position.group())
                y = re.search('y: (.*)\n', position.group())
                z = re.search('z: (.*)\n', position.group())
                qx = re.search('qx: (.*)\n', orientation.group())
                qy = re.search('qy: (.*)\n', orientation.group())
                qz = re.search('qz: (.*)\n', orientation.group())
                qw = re.search('qw: (.*)\n', orientation.group())
                vx = re.search('x: (.*)\n', linear_velocity.group())
                vy = re.search('y: (.*)\n', linear_velocity.group())
                vz = re.search('z: (.*)\n', linear_velocity.group())
                ax = re.search('x: (.*)\n', linear_acceleration.group())
                ay = re.search('y: (.*)\n', linear_acceleration.group())
                az = re.search('z: (.*)\n', linear_acceleration.group())
                if x and y and z and qx and qy and qz and qw and vx and vy and vz:
                    return (float(x.group(1)), float(y.group(1)), float(z.group(1))), \
                           (float(qx.group(1)), float(qy.group(1)), float(qz.group(1)), float(qw.group(1))), \
                           (float(vx.group(1)), float(vy.group(1)), float(vz.group(1))), \
                           (float(ax.group(1)), float(ay.group(1)), float(az.group(1)))
    return None


def main():
    root = '/home/lifa/code/calib/shenlanxueyuan_code/catkin_ws/src/fusion-based-perception/test_data/mutil_obj/localization'
    filenames = os.listdir(root)
    print(len(filenames))
    # for filename in filenames:
    #     print(filename, filename[:-4])
    filenames = sorted(filenames, key=lambda x:float(x[:-4]))
    position_list, orientation_list, linear_velocity_list, linear_acceleration_list = [], [], [], []
    for filename in filenames:
        print(filename, filename[:-4])
        position, orientation, linear_velocity, linear_acceleration = read_file(os.path.join(root, filename))
        position_list.append(position)
        orientation_list.append(orientation)
        linear_velocity_list.append(linear_velocity)
        linear_acceleration_list.append(linear_acceleration)
    print(len(position_list), len(orientation_list), len(linear_velocity_list), len(linear_acceleration_list))
    # for item in pos_list:
    #     print(item)
    print('position_list_min', np.min(position_list, axis=0).tolist())
    print('position_list_max', np.max(position_list, axis=0).tolist())
    print('orientation_list_min', np.min(orientation_list, axis=0).tolist())
    print('orientation_list_max', np.max(orientation_list, axis=0).tolist())
    print('linear_velocity_list_min', np.min(linear_velocity_list, axis=0).tolist())
    print('linear_velocity_list_max', np.max(linear_velocity_list, axis=0).tolist())
    print('linear_acceleration_list_min', np.min(linear_acceleration_list, axis=0).tolist())
    print('linear_acceleration_list_max', np.max(linear_acceleration_list, axis=0).tolist())
    


def test():
    file_path = '/home/lifa/code/calib/shenlanxueyuan_code/catkin_ws/src/fusion-based-perception/test_data/mutil_obj/localization/1650511777.1221306.txt'
    position, orientation, linear_velocity, linear_acceleration = read_file(file_path)
    print('position:', position)
    print('orientation:', orientation)
    print('linear_velocity:', linear_velocity)
    print('linear_acceleration: ', linear_acceleration)


if __name__ == '__main__':
    main()
    # test()
