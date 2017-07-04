import sklearn
from sklearn import linear_model, datasets
import numpy as np
from math import cos, sin
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt


def split_back(points):
    model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression(), residual_threshold=0.03)
    model_ransac.fit(xs.reshape(-1, 1), zs)
    inlier_mask = model_ransac.inlier_mask_
    z_cut = zs[inlier_mask].mean() - 0.03 # to ensure clean cut
    return np.vstack([xs[inlier_mask], ys[inlier_mask], zs[inlier_mask]]).T, \
           points[np.where(points[:,2] < z_cut)]


def split_side(points):
    xs, ys, zs = points.T
    model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression(), 
            residual_threshold=0.04, max_trials=1000,
            is_model_valid=lambda model, X, y: np.abs(model.coef_).max() < 7e-2)
    while True:
        try:
            model_ransac.fit(np.vstack([ys, zs]).T, xs)
            break
        except:
            pass
    inlier_mask = model_ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)
    return np.vstack([xs[inlier_mask], ys[inlier_mask], zs[inlier_mask]]).T,\
           np.vstack([xs[outlier_mask], ys[outlier_mask], zs[outlier_mask]]).T


def remove_sides(points):
    side1, remain = split_side(points)
    side2, remain = split_side(remain)
    x1 = side1[:,0].mean()
    x2 = side2[:,0].mean()
    if x1 > x2:
        side2, side1 = side1, side2
    cut_left = side1[:,0].max() + 0.02
    cut_right = side2[:,0].min() - 0.02 # to ensure clean cut
    print(cut_left, cut_right)
    cut_z  = side1[:,2].max()
    points = points[np.where(points[:, 0] < cut_right)]
    points = points[np.where(points[:, 0] > cut_left)]
    return points, cut_right - cut_left


def uniform_enough(x):
    return (np.float32(np.histogram(x, bins=5)[0]) / len(x)).min() != 0


def remove_horizontal(points, width):
    xs, ys, zs = points.T
    model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression(), 
            residual_threshold=0.01, max_trials=1000, 
            is_model_valid=lambda model, X, y: np.abs(model.coef_).max() < 3e-2 and X.max() - X.min() >= width * 0.3)
    fail_cnt = 0
    while True:
        try:
            model_ransac.fit(xs.reshape(-1, 1), ys)
            inlier_mask = model_ransac.inlier_mask_
            remove_xs = xs[inlier_mask]
            if remove_xs.max() - remove_xs.min() >= 0.3 * width and \
               len(remove_xs) > 30 and uniform_enough(remove_xs):
                outlier_mask = np.logical_not(inlier_mask)
                xs = xs[outlier_mask]
                ys = ys[outlier_mask]
                zs = zs[outlier_mask]
                print('Remove line!', remove_xs.shape)
            else:
                fail_cnt += 1
        except:
            fail_cnt += 1
        if fail_cnt >= 5:
            break
    return np.vstack([xs, ys, zs]).T


def show_cluster_cloud(points, thres=0.05, filter_flat=True):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    dbscan = DBSCAN(eps=0.05)
    dbscan.fit(points)
    labels = dbscan.labels_
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
    core_samples_mask = np.zeros_like(dbscan.labels_, dtype=bool)
    core_samples_mask[dbscan.core_sample_indices_] = True
    for k, col in zip(unique_labels, colors):
        class_member_mask = (labels == k)
        xyz = points[class_member_mask & core_samples_mask]
        if len(xyz) > 5:
            x, y, z = xyz.T
            height = y.max() - y.min()
            width = x.max() - x.min()
            if not filter_flat or width < height * 4:
                ax.plot(x, y, z, 'o', markerfacecolor=tuple(col))
    plt.show()


def show_cluster_cloud(points, thres=0.05, filter_flat=True):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    dbscan = DBSCAN(eps=0.05)
    dbscan.fit(points)
    labels = dbscan.labels_
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
    core_samples_mask = np.zeros_like(dbscan.labels_, dtype=bool)
    core_samples_mask[dbscan.core_sample_indices_] = True
    for k, col in zip(unique_labels, colors):
        class_member_mask = (labels == k)
        xyz = points[class_member_mask & core_samples_mask]
        if len(xyz) > 5:
            x, y, z = xyz.T
            height = y.max() - y.min()
            width = x.max() - x.min()
            if not filter_flat or width < height * 4:
                ax.plot(x, y, z, 'o', markerfacecolor=tuple(col))
    plt.show()


def cluster_cloud(points, thres=0.05, filter_flat=True):
    dbscan = DBSCAN(eps=0.05)
    dbscan.fit(points)
    labels = dbscan.labels_
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
    core_samples_mask = np.zeros_like(dbscan.labels_, dtype=bool)
    core_samples_mask[dbscan.core_sample_indices_] = True
    boxes_3d = []
    for k, col in zip(unique_labels, colors):
        class_member_mask = (labels == k)
        xyz = points[class_member_mask & core_samples_mask]
        if len(xyz) > 5:
            x, y, z = xyz.T
            height = y.max() - y.min()
            width = x.max() - x.min()
            if not filter_flat or width < height * 4:
                boxes_3d.append([
                    x.min(), x.max(),
                    y.min(), y.max(),
                    z.min(), z.max()
                ])
    return boxes_3d

