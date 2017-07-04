import numpy as np

def proj_points_to_2d(points, P):
    n = points.shape[0]
    points = points.T
    o = np.ones((1, n))
    points = np.vstack([points, o])
    pixel_points = (P.dot(points)).T
    pixel_points = pixel_points[:,:2] / pixel_points[:,2:]
    return pixel_points


def proj_3dbox_to_2d(box_3d, P):
    xmin, xmax, ymin, ymax, zmin, zmax = box_3d
    proj_points = np.array([
        [xmin, ymin, zmax],
        [xmin, ymin, zmin],
        [xmin, ymax, zmax],
        [xmin, ymax, zmin],
        [xmax, ymin, zmax],
        [xmax, ymin, zmin],
        [xmax, ymax, zmax],
        [xmax, ymax, zmin],
    ])
    pixel_points = proj_points_to_2d(proj_points, P)
    xs = pixel_points[:,0]
    ys = pixel_points[:,1]
    return tuple(map(int, (xs.min(), xs.max(), ys.min(), ys.max())))


def proj_3dboxes_to_2d(boxes_3d, P):
    return [proj_3dbox_to_2d(b, P) for b in boxes_3d]


def proj_2dbox_to_cloud(box_2d, P, points):
    xmin, xmax, ymin, ymax = box_2d
    pixel_points = proj_points_to_2d(points, P)
    points = points[np.where((pixel_points[:,0] >= xmin)
        & (pixel_points[:,0] < xmax) & (pixel_points[:,1] >= ymin) &\
        (pixel_points[:,1] < ymax))]
    return points


