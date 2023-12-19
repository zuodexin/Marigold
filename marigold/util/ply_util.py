import numpy as np
import open3d as o3d


def save_colored_point_cloud(depth, rgb, filename="colored_point_cloud_open3d.ply"):
    """
    将带颜色的点云保存为PLY文件使用Open3D

    参数：
    - depth: 深度图的NumPy数组
    - rgb: RGB图像的NumPy数组
    - filename: 保存的PLY文件名，默认为'colored_point_cloud_open3d.ply'
    """
    # 将深度图和RGB图像转换为Open3D格式
    scene_depth = depth * 600 + 400
    depth_o3d = o3d.geometry.Image(scene_depth)
    rgb_o3d = o3d.geometry.Image(rgb)
    width, height = rgb.shape[:2]
    fx, fy = 800, 800
    cx, cy = width / 2, height / 2

    # 创建RGBD图像
    # d / depth_scale, then truncate
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d, convert_rgb_to_intensity=False, depth_trunc=1
    )

    # 创建点云
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    )
    # 保存PLY文件
    o3d.io.write_point_cloud(filename, pcd)
