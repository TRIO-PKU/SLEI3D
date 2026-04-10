import numpy as np
from scipy.interpolate import BSpline
import rospy
from geometry_msgs.msg import Point
from datetime import datetime
from bspline.msg import Bspline


def astar_to_bspline(astar_path: list) -> Bspline:
    # 假设你自定义的 B-spline 类如下：
    class MyBSpline:
        def __init__(self, order, traj_id, start_time, end_time, knots, pos_pts, yaw_pts, yaw_dt):
            self.order = order
            self.traj_id = traj_id
            self.start_time = start_time
            self.end_time = end_time
            self.knots = knots
            self.pos_pts = pos_pts
            self.yaw_pts = yaw_pts
            self.yaw_dt = yaw_dt

    # 控制点和 B-spline 参数设置
    control_points = np.array(astar_path)
    degree = 3  # Cubic B-spline
    order = degree + 1  # 阶次
    n_control_points = len(control_points)
    n_knots = n_control_points + degree + 1
    knot_vector = np.concatenate((
        np.zeros(degree),  
        np.linspace(0, 1, n_control_points - degree + 1),  
        np.ones(degree)   
    ))

    # 提取控制点的 x, y, z 坐标
    cx, cy, cz = control_points[:, 0], control_points[:, 1], control_points[:, 2]

    # 创建 B-spline 对象并采样生成位置点
    spline_x = BSpline(knot_vector, cx, degree)
    spline_y = BSpline(knot_vector, cy, degree)
    spline_z = BSpline(knot_vector, cz, degree)
    t_vals = np.linspace(0, 1, 100)
    x_fine, y_fine, z_fine = spline_x(t_vals), spline_y(t_vals), spline_z(t_vals)

    # 生成 pos_pts (geometry_msgs/Point[])
    pos_pts = [Point(x, y, z) for x, y, z in zip(x_fine, y_fine, z_fine)]

    # 计算航向角 yaw_pts
    yaw_pts = []
    for i in range(1, len(pos_pts)):
        dx = pos_pts[i].x - pos_pts[i-1].x
        dy = pos_pts[i].y - pos_pts[i-1].y
        yaw_pts.append(np.arctan2(dy, dx))
    yaw_dt = 0.1  # 假设航向角时间间隔为 0.1 秒

    # 填充 MyBSpline 实例
    traj_id = 1  # 样例 ID
    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(len(t_vals) * yaw_dt)

    my_bspline = MyBSpline(
        order=order,
        traj_id=traj_id,
        start_time=start_time,
        end_time=end_time,
        knots=knot_vector.tolist(),
        pos_pts=pos_pts,
        yaw_pts=yaw_pts,
        yaw_dt=yaw_dt
    )
        
    result = Bspline()
    result.order = my_bspline.order
    result.traj_id = my_bspline.traj_id
    result.start_time = my_bspline.start_time
    result.end_time = my_bspline.end_time
    result.knots = my_bspline.knots
    result.pos_pts = my_bspline.pos_pts
    result.yaw_pts = my_bspline.yaw_pts
    result.yaw_dt = my_bspline.yaw_dt
    return result

