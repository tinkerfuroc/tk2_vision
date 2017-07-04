import pcl
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cloudproc import split_back, remove_sides, remove_horizontal
from sklearn import linear_model, datasets
import numpy as np
from cloudproc import cluster_cloud
from projection import proj_3dboxes_to_2d, proj_2dbox_to_cloud
import cv2

cloud = pcl.load(sys.argv[1])
fil = cloud.make_voxel_grid_filter()
fil.set_leaf_size(0.02, 0.02, 0.02)
cloud = fil.filter()
fil = cloud.make_passthrough_filter()
fil.set_filter_field_name('z')
fil.set_filter_limits(0, 2)
cloud = fil.filter()
fil = cloud.make_passthrough_filter()
fil.set_filter_field_name('x')
fil.set_filter_limits(-0.6, 0.6)
cloud = fil.filter()
points = cloud.to_array()
xs, ys, zs = points.T

model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression(), 
        is_model_valid=lambda model, X, y: np.abs(model.coef_).max() < 7e-2,
        residual_threshold=0.03)
model_ransac.fit(np.vstack([xs, zs]).T, ys)
inlier_mask = model_ransac.inlier_mask_
outlier_mask = np.logical_not(inlier_mask)
y_cut = ys[inlier_mask].mean() - 0.01
print(y_cut)
points = points[np.where(points[:,1] < y_cut)]

P = np.array([540.68603515625, 0.0, 460.75, 0.0, 
              0.0, 540.68603515625, 280.75, 0.0, 
              0.0, 0.0, 1.0, 0.0]).reshape(3,4)
boxes_3d = cluster_cloud(points, filter_flat=False)
boxes_2d = proj_3dboxes_to_2d(boxes_3d, P)
img = cv2.imread('./table_pipeline/image.png')
for b in boxes_2d:
    xmin, xmax, ymin, ymax = b
    obj_points = proj_2dbox_to_cloud(b, P, points)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(obj_points[:,0], obj_points[:,1], obj_points[:,2])
    cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 1)
    cv2.imshow('img', img)
    cv2.waitKey(20)
    plt.show()

