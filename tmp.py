import argparse
from pathlib import Path

import open3d as o3d
import glob, os
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()

    # 各パラメータは点群に応じて調整
    parser.add_argument('--max_correspondence_distance', type=float, default=0.02)
    parser.add_argument('--max_iteration', type=int, default=500)
    parser.add_argument('--estimate_normals_max_nn', type=int, default=30)
    parser.add_argument('--estimate_normals_radius', type=float, default=0.1)

    parser.add_argument('--pcd_source', type=Path)
    parser.add_argument('--pcd_target', type=Path)

    return parser.parse_args()

def fnc1():
    bb = o3d.geometry.AxisAlignedBoundingBox(
        np.array([[-1],[-1],[0]]),
        np.array([[1],[1],[2]]),
    )
    # bb2 = o3d.geometry.AxisAlignedBoundingBox(
    #     np.array([[-1],[0.2],[0.6]]),
    #     np.array([[1],[0.4],[1.4]]),
    # )
    folder_path = '/home/aichi2204/Desktop/rosbag2_2024_06_07-23_12_38/points/'
    new_folder_path = '/home/aichi2204/Desktop/rosbag2_2024_06_07-23_12_38/points_new'
    pcd_files = sorted(glob.glob(os.path.join(folder_path, '**/*.pcd'), recursive=True))
    
    pcd_target = o3d.io.read_point_cloud("/home/aichi2204/Desktop/rosbag2_2024_06_07-23_12_38/points/1717769558_189695801.pcd").crop(bb)
    for i in range(5, len(pcd_files), 5):
        pcd_file = pcd_files[i]
        pcd_source = o3d.io.read_point_cloud(pcd_files[i]).crop(bb)
    # 事前に点群の重なり具合の評価値を確認しておくと、ICP の有効性が確認しやすい
        # evaluation = o3d.pipelines.registration.evaluate_registration
        #     source = pcd_source,
        #     target = pcd_target,
        #     max_correspondence_distance = 0.02)

        # target の点群の法線推定
        pcd_target.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1,
                max_nn=30
            )
        )

        # point-to-plane ICP を適用
        reg = o3d.pipelines.registration.registration_icp(
            source = pcd_source,
            target = pcd_target,
            max_correspondence_distance = 0.02,
            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500)
        )

        # reg.transformation が結果の変換行列
        pcd_source.transform(reg.transformation)
        new_pcd_file = pcd_file.replace(folder_path, new_folder_path)

        o3d.io.write_point_cloud(new_pcd_file, pcd_source)
        pcd_target = pcd_source

        print(pcd_file)

if __name__ == '__main__':
    fnc1()
