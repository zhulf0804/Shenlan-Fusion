import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R


def quat2mat(q, t=None):
    # mat = np.array([[1-2*q[2]**2-2*q[3]**2, 2*q[1]*q[2]-2*q[0]*q[3], 2*q[1]*q[3]+2*q[0]*q[2]],
    #               [2*q[1]*q[2]+2*q[0]*q[3], 1-2*q[1]**2-2*q[3]**2, 2*q[2]*q[3]-2*q[0]*q[1]],
    #               [2*q[1]*q[3]-2*q[0]*q[2], 2*q[2]*q[3]+2*q[0]*q[1], 1-2*q[1]**2-2*q[2]**2]])

    mat = R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    if t is None:
        return mat
    T = np.eye(4)
    T[:3, :3] = mat
    T[:3, -1] = np.array(t)
    return T


def draw_traj(ori_pos, gt_pos, pred_pos):
    fig = plt.figure()
    ax1 = fig.add_subplot(211, projection='3d')
    ax1.plot(ori_pos[:, 0], ori_pos[:, 1], ori_pos[:, 2], c='b', marker='x', label='origin')
    ax1.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2], c='g', marker='x', label='gt')
    ax1.legend(loc='upper right')
    # 设置坐标轴标签
    ax1.set_xlabel('X Label')
    ax1.set_ylabel('Y Label')
    ax1.set_zlabel('Z Label')

    ax2 = fig.add_subplot(212, projection='3d')
    # 绘制散点图
    ax2.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2], c='g', marker='x', label='gt')
    ax2.plot(pred_pos[:, 0], pred_pos[:, 1], pred_pos[:, 2], c='r', label='pred')
    ax2.legend(loc='upper right')
    # 设置坐标轴标签
    ax2.set_xlabel('X Label')
    ax2.set_ylabel('Y Label')
    ax2.set_zlabel('Z Label')

    # 显示图形
    plt.show()


if __name__ == '__main__':
    # quat = [w:0.7, x:0, y:0, z:0.7] 
    # t = [x:0.2, y:0.3, z:0]
    q = [0.7, 0, 0, 0.7]
    t = [0.3, 0.4, 0]
    gt_T = quat2mat(q, t)
    q_p = [0.707105, -9.20458e-07, 8.57004e-07, 0.707109]
    t_p = [0.299682, 0.400395, -9.79583e-07]
    pred_T = quat2mat(q_p, t_p)


    ori_traj, gt_traj, pred_traj = [], [], []
    with open('../data/slam_poses.txt') as f:
        lines = f.readlines()
        pose_data = []
        for line in lines:
            line_split = list(map(float, line.strip().split()))
            # print(line_split, '.....', line_split[6:] + line_split[3:6])
            # (x, y, z, w) -> (q0, q1, q2, q3) = (w, x, y, z)
            cur_mat = quat2mat(line_split[6:] + line_split[3:6], line_split[:3])
            ori_traj.append(cur_mat)
            gt_traj.append(np.dot(cur_mat, gt_T))
            pred_traj.append(np.dot(cur_mat, pred_T))  
    ori_pos = np.array(ori_traj)[:, :3, -1] # (n, 3)
    gt_pos = np.array(gt_traj)[:, :3, -1] # (n, 3)
    pred_pos = np.array(pred_traj)[:, :3, -1] # (n, 3)
    draw_traj(ori_pos, gt_pos, pred_pos)
