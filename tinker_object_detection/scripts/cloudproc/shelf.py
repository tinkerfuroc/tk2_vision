import pcl
import sys
import pypclviewer
import matplotlib.pyplot as plt
import sklearn
from sklearn import linear_model, datasets import numpy as np
from math import cos, sin
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from sklearn.cluster import DBSCAN
from cloudproc import split_back, remove_sides, remove_horizontal, cluster_cloud


def rotate_normalize(points, back_points):
    xs, ys, zs = points.T
    xs_back = back_points[:,0]
    zs_back = back_points[:,1]
    xz_model = linear_model.LinearRegression()
    xz_model.fit(xs_back.reshape(-1, 1), zs_back)
    xz_slope = xz_model.coef_
    rotate_angle = -np.arctan(xz_slope)
    rotate_mat = np.array([[cos(rotate_angle), -sin(rotate_angle)],
                           [sin(rotate_angle), cos(rotate_angle)]], dtype=np.float32)
    xz_rotated = rotate_mat.dot(np.stack([xs, zs]))
    xs = xz_rotated[0]
    zs = xz_rotated[1]
    return np.vstack([xs, ys, zs]).T, rotate_angle


cloud = pcl.load(sys.argv[1])
fil = cloud.make_voxel_grid_filter()
fil.set_leaf_size(0.02, 0.02, 0.02)
cloud = fil.filter()
fil = cloud.make_passthrough_filter()
fil.set_filter_field_name('z')
fil.set_filter_limits(0, 2)
cloud = fil.filter()

point_arr = cloud.to_array()
points_back, points_remain = split_back(point_arr)
points_remain, rotate_angle = rotate_normalize(points_remain, points_back)
points_remain, width = remove_sides(points_remain)
points_final = points_remain
points_final = remove_horizontal(points_remain, width)
cluster_cloud(points_final)
