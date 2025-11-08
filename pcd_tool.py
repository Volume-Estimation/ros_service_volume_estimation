# save as app_select_and_export.py
import numpy as np
import sys
import os
os.environ["QT_API"] = "pyside6"
import pyvista as pv
import traceback
import vtk  # 用于从 box representation 读取六个平面

from pyvistaqt import BackgroundPlotter
from PySide6.QtWidgets import QFileDialog

# ---------- 算法 ----------
def pca_obb(points):
    P = points.astype(np.float64)
    c = P.mean(0)
    X = P - c
    C = (X.T @ X) / max(len(P) - 1, 1)
    vals, vecs = np.linalg.eigh(C)
    order = np.argsort(vals)[::-1]
    U = vecs[:, order]  # 列向量: u1,u2,u3
    Y = X @ U
    mins, maxs = Y.min(0), Y.max(0)
    lengths = maxs - mins
    center_local = 0.5 * (mins + maxs)
    center_world = c + U @ center_local
    half = 0.5 * lengths
    signs = np.array([
        [-1,-1,-1],[ 1,-1,-1],[-1, 1,-1],[ 1, 1,-1],
        [-1,-1, 1],[ 1,-1, 1],[-1, 1, 1],[ 1, 1, 1]
    ], dtype=np.float64)
    corners = center_world + (signs * half) @ U.T
    return c, U, lengths, corners

def pca_plane(points):
    P = points.astype(np.float64)
    c = P.mean(0)
    X = P - c
    C = (X.T @ X) / max(len(P) - 1, 1)
    vals, vecs = np.linalg.eigh(C)
    n = vecs[:, 0]
    n /= (np.linalg.norm(n) + 1e-12)
    d = -float(n @ c)  # n·x + d = 0
    return c, n, d

def rot_n_to_z(n):
    n = n / (np.linalg.norm(n) + 1e-12)
    z = np.array([0.,0.,1.])
    v = np.cross(n, z)
    s = np.linalg.norm(v)
    c = float(n @ z)
    if s < 1e-10:
        if c > 0:  # 已对齐
            return np.eye(3)
        # 反向，绕任意与 z 垂直的轴旋转 180°（取 x 轴）
        K = np.array([[0, -0, 0],
                      [0, 0, -1],
                      [0, 1, 0]], dtype=np.float64)
        return np.eye(3) + K + K@K
    # 使用未归一化交叉积的闭式公式，生成正交旋转矩阵
    K = np.array([[0, -v[2], v[1]],
                  [v[2], 0, -v[0]],
                  [-v[1], v[0], 0]], dtype=np.float64)
    R = np.eye(3) + K + K@K * ((1 - c) / (s**2))
    return R

def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]], dtype=np.float64)

# ---------- DEM 体积估计（基于高程格栅） ----------
def _is_point_inside_polygon(point_xy, polygon_xy) -> bool:
    x, y = point_xy
    poly = polygon_xy
    num_vertices = len(poly)
    inside = False
    if num_vertices < 3:
        return False
    x_prev, y_prev = poly[-1]
    for vx, vy in poly:
        intersects = ((vy > y) != (y_prev > y)) and (
            x < (x_prev - vx) * (y - vy) / (y_prev - vy + 1e-12) + vx
        )
        if intersects:
            inside = not inside
        x_prev, y_prev = vx, vy
    return inside

def _compute_convex_hull_monotonic_chain(points_xy: np.ndarray) -> np.ndarray:
    if points_xy.shape[0] < 3:
        return points_xy.copy()
    pts = points_xy[np.lexsort((points_xy[:, 1], points_xy[:, 0]))]
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(tuple(p))
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(tuple(p))
    hull = lower[:-1] + upper[:-1]
    return np.array(hull, dtype=float)

def _create_height_map(points: np.ndarray, bounds, grid_res_x: float, grid_res_y: float, height_method: str):
    min_x, max_x, min_y, max_y = bounds
    num_x = max(1, int(np.ceil((max_x - min_x) / grid_res_x)))
    num_y = max(1, int(np.ceil((max_y - min_y) / grid_res_y)))
    height_map = np.zeros((num_x, num_y), dtype=np.float64)
    count_map = np.zeros((num_x, num_y), dtype=np.int64) if height_method == 'mean' else None
    for x, y, z in points:
        gx = min(int((x - min_x) / grid_res_x), num_x - 1)
        gy = min(int((y - min_y) / grid_res_y), num_y - 1)
        if height_method == 'max':
            height_map[gx, gy] = max(height_map[gx, gy], z)
        else:
            height_map[gx, gy] += z
            count_map[gx, gy] += 1
    if height_method == 'mean':
        empty_mask = (count_map == 0)
        height_map = np.divide(height_map, count_map, out=np.zeros_like(height_map), where=count_map!=0)
    else:
        empty_mask = (height_map == 0)
    return height_map, empty_mask, (num_x, num_y)

def _get_convex_hull_mask(points: np.ndarray, bounds, grid_shape, grid_res_x: float, grid_res_y: float):
    min_x, max_x, min_y, max_y = bounds
    num_x, num_y = grid_shape
    try:
        hull_polygon = _compute_convex_hull_monotonic_chain(points[:, :2])
        valid_mask = np.zeros((num_x, num_y), dtype=bool)
        for i in range(num_x):
            for j in range(num_y):
                cx = min_x + (i + 0.5) * grid_res_x
                cy = min_y + (j + 0.5) * grid_res_y
                if _is_point_inside_polygon((cx, cy), hull_polygon):
                    valid_mask[i, j] = True
        return valid_mask
    except Exception:
        return np.ones((num_x, num_y), dtype=bool)

def _interpolate_height_map(height_map: np.ndarray, empty_mask: np.ndarray, valid_mask: np.ndarray, max_passes: int):
    interpolated_map = np.copy(height_map)
    has_value_mask = ~empty_mask
    initial_empty_count = int(np.sum(empty_mask & valid_mask))
    for pass_num in range(max_passes):
        current_empty = np.argwhere(~has_value_mask & empty_mask & valid_mask)
        if len(current_empty) == 0:
            return interpolated_map, {
                'passes_used': pass_num,
                'stop_reason': 'all_cells_filled',
                'initial_empty_cells': initial_empty_count,
                'final_empty_cells': 0,
                'filled_cells': initial_empty_count,
                'convex_hull_area': int(np.sum(valid_mask))
            }
        num_changes = 0
        newly_filled = []
        for r, c in current_empty:
            neighbors = []
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = r+dr, c+dc
                if 0 <= nr < height_map.shape[0] and 0 <= nc < height_map.shape[1] and has_value_mask[nr, nc]:
                    neighbors.append(interpolated_map[nr, nc])
            if neighbors:
                interpolated_map[r, c] = float(np.mean(neighbors))
                newly_filled.append((r, c))
                num_changes += 1
        for r, c in newly_filled:
            has_value_mask[r, c] = True
        if num_changes == 0:
            final_empty_count = int(np.sum(~has_value_mask & empty_mask & valid_mask))
            return interpolated_map, {
                'passes_used': pass_num + 1,
                'stop_reason': 'converged',
                'initial_empty_cells': initial_empty_count,
                'final_empty_cells': final_empty_count,
                'filled_cells': initial_empty_count - final_empty_count,
                'convex_hull_area': int(np.sum(valid_mask))
            }
    final_empty_count = int(np.sum(~has_value_mask & empty_mask & valid_mask))
    return interpolated_map, {
        'passes_used': max_passes,
        'stop_reason': 'max_passes_reached',
        'initial_empty_cells': initial_empty_count,
        'final_empty_cells': final_empty_count,
        'filled_cells': initial_empty_count - final_empty_count,
        'convex_hull_area': int(np.sum(valid_mask))
    }

def estimate_volume_dem_np(points: np.ndarray, grid_resolution_x: float = 0.1, grid_resolution_y: float = 0.1,
                           height_method: str = 'max', interpolate_empty_cells: bool = False,
                           max_interpolation_passes: int = 50):
    if points is None or len(points) == 0:
        return 0.0, {'used_interpolation': False, 'stop_reason': 'no_points'}, None
    P = np.asarray(points, dtype=np.float64)
    mins = P[:, :2].min(axis=0)
    maxs = P[:, :2].max(axis=0)
    bounds = (mins[0], maxs[0], mins[1], maxs[1])
    height_map, empty_mask, grid_shape = _create_height_map(P, bounds, grid_resolution_x, grid_resolution_y, height_method)
    valid_mask = _get_convex_hull_mask(P, bounds, grid_shape, grid_resolution_x, grid_resolution_y)
    initial_empty_count = int(np.sum(empty_mask & valid_mask))
    interpolation_info = {
        'used_interpolation': False,
        'convex_hull_area': int(np.sum(valid_mask)),
        'initial_empty_cells': initial_empty_count
    }
    if interpolate_empty_cells and max_interpolation_passes > 0:
        height_map, interp_info = _interpolate_height_map(height_map, empty_mask, valid_mask, max_interpolation_passes)
        interpolation_info.update({'used_interpolation': True, **interp_info})
    else:
        interpolation_info['final_empty_cells'] = initial_empty_count
        interpolation_info['filled_cells'] = 0
    cell_area = grid_resolution_x * grid_resolution_y
    total_volume = float(np.sum(height_map) * cell_area)
    return total_volume, interpolation_info, height_map

def level_transforms(P_sel):
    # 平面与主轴
    c, n, d = pca_plane(P_sel)
    _, U, _, _ = pca_obb(P_sel)  # 取第一主轴 u1
    u1 = U[:,0]

    # 1) 法向对齐 +Z
    R_level = rot_n_to_z(n)              # R_level * n = +Z
    # 旋转后按“平均 z 移到 0”来计算平移量
    P_rot = (R_level @ P_sel.T).T
    z_mean = float(P_rot[:, 2].mean())
    t_level = np.array([0,0,-z_mean])    # 平移到 z=0（平均 z）
    T_level = np.eye(4); T_level[:3,:3] = R_level; T_level[:3,3] = t_level

    # 2) 额外对齐主轴到 +X（绕Z修正偏航）
    u1_after = R_level @ u1
    u1_xy = u1_after.copy(); u1_xy[2] = 0
    if np.linalg.norm(u1_xy) > 1e-8:
        yaw = np.arctan2(u1_xy[1], u1_xy[0])  # 当前指向
    else:
        yaw = 0.0
    R_yaw = rot_z(-yaw)                 # 把投影转到 +X
    R2 = R_yaw @ R_level
    T_level_yaw = np.eye(4); T_level_yaw[:3,:3] = R2; T_level_yaw[:3,3] = t_level
    return T_level, T_level_yaw

# ---------- GUI ----------
class App:
    def __init__(self):
        pv.set_plot_theme("dark")
        self.pl = BackgroundPlotter(title="Selector / Exporter", window_size=(1200,800))

        # 保留默认的鼠标交互，只添加自定义键盘事件处理
        # 尝试禁用默认的 Q 键退出行为
        try:
            if hasattr(self.pl.interactor, 'SetExitOnQuit'):
                self.pl.interactor.SetExitOnQuit(False)
        except Exception:
            pass

        self.points = None
        self.box_widget = None
        self.box_last_transform = None
        self._actors = {}
        ########## DEM 配置 ##########
        self.grid_res_x = 0.1
        self.grid_res_y = 0.1
        self.height_method = 'mean'  # 'max' or 'mean'
        self.interpolate_empty_cells = False
        self.max_interpolation_passes = 100

        self._bind()

    def load_from_path(self, path: str):
        try:
            self.points = self._read_points(path)
            print(f"加载点数: {0 if self.points is None else len(self.points)}")
            self._reset_scene()
        except Exception as e:
            print(f"读取点云失败: {e}")

    def _bind(self):
        self.pl.add_key_event("l", self.on_load)
        self.pl.add_key_event("b", self.on_box)
        self.pl.add_key_event("o", self.on_export_obb)
        self.pl.add_key_event("t", self.on_export_transform)
        self.pl.add_key_event("f", self.on_apply_transform_txt)
        self.pl.add_key_event("v", self.on_volume_dem)
        self.pl.add_key_event("space", self.on_level_and_apply)

        # 添加盒子移动和旋转控制
        self.pl.add_key_event("a", self.on_move_box_left)   # A: 左移
        self.pl.add_key_event("d", self.on_move_box_right)  # D: 右移
        self.pl.add_key_event("w", self.on_move_box_up)     # W: 上移
        self.pl.add_key_event("x", self.on_move_box_down)   # X: 下移
        self.pl.add_key_event("z", self.on_move_box_z_up)   # Z: Z轴上移
        self.pl.add_key_event("c", self.on_move_box_z_down) # C: Z轴下移

        # 添加盒子旋转控制
        self.pl.add_key_event("Left", self.on_rotate_box_left)   # 方向键左: 绕Z轴逆时针旋转
        self.pl.add_key_event("Right", self.on_rotate_box_right) # 方向键右: 绕Z轴顺时针旋转

        self.pl.add_text("L:Load  B:Box  O:Export OBB  T:Export Level Transform  F:Apply Transform  Space:Level+Apply  V:Volume(DEM)", font_size=10)
        self.pl.add_text("W/A/D/X:Move Box XY  Z:Box Z+  C:Box Z-  Left/Right:Rotate Z", font_size=10, position='upper_right')

    def on_load(self):
        dlg = QFileDialog(self.pl.app_window)
        dlg.setFileMode(QFileDialog.ExistingFile)
        dlg.setNameFilter("PointCloud (*.npy *.ply *.obj *.vtp *.vtk *.stl *.pcd);;All (*)")
        if not dlg.exec(): return
        path = dlg.selectedFiles()[0]
        try:
            self.points = self._read_points(path)
            print(f"加载点数: {0 if self.points is None else len(self.points)}")
            self._reset_scene()
        except Exception as e:
            print(f"读取点云失败: {e}")

    def _read_points(self, path: str):
        if path.lower().endswith(".npy"):
            arr = np.load(path)
            assert arr.ndim==2 and arr.shape[1]>=3
            pts = arr[:, :3].astype(np.float64)
            pts = pts[np.isfinite(pts).all(axis=1)]
            return pts
        if path.lower().endswith('.pcd'):
            # 优先使用 open3d（如果已安装），否则回退到简单的 ASCII PCD 解析
            try:
                import open3d as o3d
                pcd = o3d.io.read_point_cloud(path)
                pts = np.asarray(pcd.points, dtype=np.float64)
                if pts.size == 0:
                    raise ValueError('Open3D 读取到空点集')
                pts = pts[np.isfinite(pts).all(axis=1)]
                return pts
            except Exception:
                # 简单 ASCII PCD 解析：只支持 ASCII 格式并假设包含 x y z 字段
                with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                    header = []
                    for line in f:
                        line_strip = line.strip()
                        header.append(line_strip)
                        if line_strip.upper().startswith('DATA'):
                            data_type = line_strip.split()[1].lower() if len(line_strip.split())>1 else 'ascii'
                            break
                    # 找到字段定义
                    fields = None
                    for h in header:
                        if h.lower().startswith('fields') or h.lower().startswith('field'):
                            parts = h.split()
                            fields = parts[1:]
                            break
                    if data_type != 'ascii':
                        raise RuntimeError('仅支持 ASCII PCD，或者请安装 open3d 来支持二进制 PCD')
                    if fields is None:
                        raise RuntimeError('无法解析 PCD header：缺少 FIELDS')
                    # 读取剩余的行作为数据
                    data = []
                    for line in f:
                        if not line.strip():
                            continue
                        parts = line.strip().split()
                        if len(parts) < len(fields):
                            continue
                        data.append(parts)
                    data = np.asarray(data)
                    # 找到 x,y,z 索引
                    lower_fields = [ff.lower() for ff in fields]
                    try:
                        xi = lower_fields.index('x')
                        yi = lower_fields.index('y')
                        zi = lower_fields.index('z')
                    except ValueError:
                        raise RuntimeError('PCD 文件不包含 x y z 字段')
                    pts = data[:, [xi, yi, zi]].astype(np.float64)
                    pts = pts[np.isfinite(pts).all(axis=1)]
                    return pts

        mesh = pv.read(path)
        pts = np.asarray(mesh.points, dtype=np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        return pts

    def _reset_scene(self):
        self.pl.clear()
        self._actors.clear()
        if self.points is None or len(self.points) == 0:
            self.pl.add_text("未加载或空点云", font_size=10)
            return
        poly = pv.PolyData(self.points)
        try:
            poly["height"] = np.asarray(poly.points)[:, 2].astype(np.float64)
        except Exception:
            pass
        try:
            print(f"点云范围 bounds: {poly.bounds}")
        except Exception:
            pass
        self.pl.add_points(
            poly,
            scalars="height",
            point_size=5,
            cmap="turbo",
            render_points_as_spheres=False,
            show_scalar_bar=False
        )
        try:
            self.pl.show_bounds(grid='front', location='outer', all_edges=True)
        except Exception:
            pass
        try:
            self.pl.add_axes()
        except Exception:
            pass
        self.pl.reset_camera()
        try:
            self.pl.reset_camera_clipping_range()
        except Exception:
            pass
        self.pl.render()
        self.pl.add_text("L:Load  B:Box  O:Export OBB  T:Export Level Transform  F:Apply Transform  Space:Level+Apply  V:Volume(DEM)", font_size=10)
        self.pl.add_text("W/A/D/X:Move Box XY  Z:Box Z+  C:Box Z-  Left/Right:Rotate Z", font_size=10, position='upper_right')

    def on_level_and_apply(self):
        """空格键：基于盒内点拟合水平面并对齐主轴到+X（包含yaw），立即应用到整幅点云，并保存 T_level_yaw。"""
        if self.points is None or len(self.points) == 0:
            print("请先加载点云 (L)")
            return
        P_sel = self._get_sel()
        if P_sel is None:
            return
        try:
            T_level, T_level_yaw = level_transforms(P_sel)
            # 应用到整幅点云
            P_h = np.c_[self.points, np.ones(len(self.points))]
            P_new = (T_level_yaw @ P_h.T).T[:, :3]
            P_new = P_new[np.isfinite(P_new).all(axis=1)]
            if P_new.size == 0:
                raise ValueError("变换后点云为空或包含非有限数值")
            self.points = P_new.astype(np.float64)

            # 清理交互状态与叠加
            self._remove_existing_box_widget()
            if "_obb" in self._actors:
                try:
                    self.pl.remove_actor(self._actors["_obb"])
                    del self._actors["_obb"]
                except Exception:
                    pass

            # 同步输出并保存 T_level_yaw
            try:
                np.savetxt("T_level_yaw.txt", T_level_yaw, fmt="%.9f")
                print("已保存: T_level_yaw.txt")
            except Exception as _e:
                print(f"保存 T_level_yaw.txt 失败: {_e}")

            print("已计算并应用水平+偏航对齐变换 (Space)")
            np.set_printoptions(precision=6, suppress=True)
            print("T_level_yaw =\n", T_level_yaw)
            self._reset_scene()
        except Exception as e:
            print(f"计算/应用水平拟合失败: {e}")

    def on_apply_transform_txt(self):
        if self.points is None or len(self.points) == 0:
            print("请先加载点云 (L)")
            return
        dlg = QFileDialog(self.pl.app_window)
        dlg.setFileMode(QFileDialog.ExistingFile)
        dlg.setNameFilter("Text Matrix (*.txt *.csv *.dat);;All (*)")
        if not dlg.exec():
            return
        path = dlg.selectedFiles()[0]
        try:
            arr = np.loadtxt(path)
            arr = np.asarray(arr, dtype=np.float64)
            if arr.size == 16:
                T = arr.reshape(4, 4)
            elif arr.shape == (4, 4):
                T = arr
            else:
                raise ValueError(f"矩阵尺寸不是 4x4: 读取形状 {arr.shape}")
            P_h = np.c_[self.points, np.ones(len(self.points))]
            P_new = (T @ P_h.T).T[:, :3]
            P_new = P_new[np.isfinite(P_new).all(axis=1)]
            if P_new.size == 0:
                raise ValueError("变换后点云为空或包含非有限数值")
            self.points = P_new.astype(np.float64)
            # 清理交互状态与叠加
            self._remove_existing_box_widget()
            if "_obb" in self._actors:
                try:
                    self.pl.remove_actor(self._actors["_obb"])
                    del self._actors["_obb"]
                except Exception:
                    pass
            print(f"已应用 4x4 变换: {path}")
            self._reset_scene()
        except Exception as e:
            print(f"加载/应用变换失败: {e}")

    def _get_box_base_z(self, default_z: float) -> float:
        """返回 box 最接近水平的两个面中更低的那个面的 z 值；失败则返回 default_z。"""
        if self.box_widget is None or vtk is None:
            return default_z
        try:
            planes = vtk.vtkPlanes()
            self.box_widget.GetPlanes(planes)
            normals_vtk = planes.GetNormals()
            points_vtk = planes.GetPoints()
            num_planes = normals_vtk.GetNumberOfTuples()
            normals = np.array([normals_vtk.GetTuple(i) for i in range(num_planes)], dtype=np.float64)
            pts_on_plane = np.array([points_vtk.GetPoint(i) for i in range(points_vtk.GetNumberOfPoints())], dtype=np.float64)
            if normals.shape[0] == 0 or pts_on_plane.shape[0] == 0:
                return default_z
            nz_abs = np.abs(normals[:, 2])
            if nz_abs.size == 0:
                return default_z
            max_val = float(nz_abs.max())
            close_idx = np.where(nz_abs >= max_val - 1e-6)[0]
            if close_idx.size == 0:
                idx = int(np.argmax(nz_abs))
                return float(pts_on_plane[idx, 2])
            # 选择 z 更低的那个面
            z_values = pts_on_plane[close_idx, 2]
            return float(z_values.min())
        except Exception:
            return default_z

    def _get_box_base_normal(self, default_n: np.ndarray) -> np.ndarray:
        """返回与 box 底面平行的法向（指向+Z方向），失败返回 default_n（会被归一化）。"""
        n_out = np.asarray(default_n, dtype=np.float64)
        if self.box_widget is None or vtk is None:
            nz = float(n_out[2])
            if nz < 0:
                n_out = -n_out
            n_out /= (np.linalg.norm(n_out) + 1e-12)
            return n_out
        try:
            planes = vtk.vtkPlanes()
            self.box_widget.GetPlanes(planes)
            normals_vtk = planes.GetNormals()
            points_vtk = planes.GetPoints()
            num_planes = normals_vtk.GetNumberOfTuples()
            if num_planes == 0 or points_vtk.GetNumberOfPoints() == 0:
                raise RuntimeError("empty planes")
            normals = np.array([normals_vtk.GetTuple(i) for i in range(num_planes)], dtype=np.float64)
            pts_on_plane = np.array([points_vtk.GetPoint(i) for i in range(points_vtk.GetNumberOfPoints())], dtype=np.float64)
            nz_abs = np.abs(normals[:, 2])
            if nz_abs.size == 0:
                raise RuntimeError("no normals")
            max_val = float(nz_abs.max())
            close_idx = np.where(nz_abs >= max_val - 1e-6)[0]
            if close_idx.size == 0:
                idx = int(np.argmax(nz_abs))
                n = normals[idx]
            else:
                # 两个面近似水平，取更低面的索引
                z_vals = pts_on_plane[close_idx, 2]
                low_i = close_idx[int(np.argmin(z_vals))]
                n = normals[low_i]
            n = np.asarray(n, dtype=np.float64)
            if n[2] < 0:
                n = -n
            n /= (np.linalg.norm(n) + 1e-12)
            return n
        except Exception:
            n_out = np.asarray(default_n, dtype=np.float64)
            if n_out[2] < 0:
                n_out = -n_out
            n_out /= (np.linalg.norm(n_out) + 1e-12)
            return n_out

    def on_volume_dem(self):
        P_sel = self._get_sel()
        if P_sel is None:
            return
        # 计算与 box 底面平行的法向（指向+Z），将其对齐到世界 +Z
        n_base = self._get_box_base_normal(default_n=np.array([0.0, 0.0, 1.0], dtype=np.float64))
        R = rot_n_to_z(n_base)  # R * n_base -> +Z
        # 旋转到“底面平行于 XY”的坐标系，再以盒内最低点作为 z=0
        P_rot = (R @ P_sel.T).T
        z0 = float(np.min(P_rot[:, 2]))
        P_rot[:, 2] = np.maximum(P_rot[:, 2] - z0, 0.0)
        vol, info, _ = estimate_volume_dem_np(
            P_rot,
            grid_resolution_x=self.grid_res_x,
            grid_resolution_y=self.grid_res_y,
            height_method=self.height_method,
            interpolate_empty_cells=self.interpolate_empty_cells,
            max_interpolation_passes=self.max_interpolation_passes
        )
        print("===============================")
        print(f"DEM 估计体积（底面∥Box底面，过盒内最低点，z0={z0:.6f}）: {vol:.6f}")
        print("—— 统计信息 ——")
        if info:
            for k in ['convex_hull_area','initial_empty_cells','final_empty_cells','filled_cells','passes_used','stop_reason','used_interpolation']:
                if k in info:
                    print(f"{k}: {info[k]}")
        print("===============================")

    def _remove_existing_box_widget(self):
        try:
            if self.box_widget is not None:
                # 尽可能多地尝试关闭/禁用旧的控件，兼容不同 VTK 版本
                try:
                    self.box_widget.SetEnabled(False)
                except Exception:
                    pass
                try:
                    self.box_widget.EnabledOff()
                except Exception:
                    pass
                try:
                    self.box_widget.Off()
                except Exception:
                    pass
                # 兼容不同 pyvista 版本的移除方式
                try:
                    self.pl.remove_box_widget(self.box_widget)
                except Exception:
                    try:
                        self.pl.remove_box_widget()
                    except Exception:
                        pass
                self.box_widget = None
            else:
                # 某些版本无需传参
                try:
                    self.pl.remove_box_widget()
                except Exception:
                    pass
        except Exception:
            pass

    def on_box(self):
        if self.points is None: return
        bounds = pv.PolyData(self.points).bounds
        # 确保同时只存在一个交互盒：先移除旧的
        self._remove_existing_box_widget()

        # Also remove the previously drawn OBB mesh if it exists
        if "_obb" in self._actors:
            try:
                self.pl.remove_actor(self._actors["_obb"])
                del self._actors["_obb"]
            except:
                pass

        def _cb(widget, _):
            try:
                # vtkBoxWidget 直接提供 GetTransform, 无需 GetRepresentation
                transform = vtk.vtkTransform()
                widget.GetTransform(transform)
                self.box_last_transform = transform
            except:
                self.box_last_transform = None

        self.box_widget = self.pl.add_box_widget(
            callback=_cb, bounds=bounds, rotation_enabled=True,
            outline_translation=True, pass_widget=True, use_planes=False, color="red"
        )

        # 缩小手柄球体，避免遮挡点云
        try:
            # 直接设置手柄大小（部分版本可用）
            if hasattr(self.box_widget, 'SetHandleSize'):
                self.box_widget.SetHandleSize(0.002)
        except Exception:
            pass
        try:
            # 通过 representation 设置（兼容路径）
            rep = getattr(self.box_widget, 'GetRepresentation', None)
            if callable(rep):
                r = rep()
                if hasattr(r, 'SetHandleSize'):
                    r.SetHandleSize(0.002)
        except Exception:
            pass

        # 初始时也尝试读取一次变换
        try:
            # 同上, 直接调用 GetTransform
            transform = vtk.vtkTransform()
            self.box_widget.GetTransform(transform)
            self.box_last_transform = transform
        except Exception:
            pass

    def _get_box_corners_from_widget(self):
        """从box widget直接读取8个角点坐标（支持旋转后的box）"""
        if self.box_widget is None or vtk is None:
            return None
        
        try:
            # 方法1：直接从polydata获取顶点坐标（可能有多于8个点，需要找到8个角点）
            box_poly_vtk = vtk.vtkPolyData()
            self.box_widget.GetPolyData(box_poly_vtk)
            
            if box_poly_vtk.GetNumberOfPoints() >= 8:
                # 获取所有点
                points = []
                for i in range(box_poly_vtk.GetNumberOfPoints()):
                    pt = box_poly_vtk.GetPoint(i)
                    points.append([pt[0], pt[1], pt[2]])
                points = np.array(points, dtype=np.float64)
                
                # 如果恰好8个点，直接返回
                if len(points) == 8:
                    # 需要对点进行排序，使其符合标准顺序
                    # 标准顺序：先z最小的4个点（按xy排序），再z最大的4个点（按xy排序）
                    return self._sort_box_corners(points)
                
                # 如果多于8个点，找到8个唯一的角点
                # 通过聚类或去重找到8个独特的顶点
                unique_points = []
                for p in points:
                    is_unique = True
                    for up in unique_points:
                        if np.linalg.norm(p - up) < 1e-6:
                            is_unique = False
                            break
                    if is_unique:
                        unique_points.append(p)
                
                if len(unique_points) == 8:
                    return self._sort_box_corners(np.array(unique_points))
                
                print(f"警告：polydata有{len(points)}个点，去重后{len(unique_points)}个唯一点")
        except Exception as e:
            print(f"从polydata获取顶点失败: {e}")
        
        try:
            # 方法2：使用变换矩阵（最可靠的方法）
            # 1. 获取初始box的bounds（未变换时）
            if self.points is not None:
                initial_bounds = pv.PolyData(self.points).bounds
                xmin, xmax, ymin, ymax, zmin, zmax = initial_bounds
                
                # 2. 构造初始box的8个角点（标准单位盒）
                initial_corners = np.array([
                    [xmin, ymin, zmin],  # 0
                    [xmax, ymin, zmin],  # 1
                    [xmin, ymax, zmin],  # 2
                    [xmax, ymax, zmin],  # 3
                    [xmin, ymin, zmax],  # 4
                    [xmax, ymin, zmax],  # 5
                    [xmin, ymax, zmax],  # 6
                    [xmax, ymax, zmax],  # 7
                ], dtype=np.float64)
                
                # 3. 获取当前box的变换矩阵
                transform = vtk.vtkTransform()
                self.box_widget.GetTransform(transform)
                M_vtk = transform.GetMatrix()
                
                # 4. 转换为numpy矩阵
                M = np.eye(4, dtype=np.float64)
                for i in range(4):
                    for j in range(4):
                        M[i, j] = float(M_vtk.GetElement(i, j))
                
                # 5. 应用变换得到旋转后的8个角点
                corners_h = np.c_[initial_corners, np.ones(8)]
                corners = (M @ corners_h.T).T[:, :3]
                
                return corners
        except Exception as e:
            print(f"使用变换矩阵计算角点失败: {e}")
        
        return None
    
    def _sort_box_corners(self, corners):
        """将8个角点按标准顺序排列：先下层4个点，再上层4个点，每层按xy排序"""
        # 按z坐标分为两层
        z_coords = corners[:, 2]
        z_mid = (z_coords.min() + z_coords.max()) / 2.0
        
        bottom = corners[z_coords <= z_mid]  # z较小的4个点
        top = corners[z_coords > z_mid]      # z较大的4个点
        
        # 对每层按 (y, x) 排序
        def sort_layer(pts):
            if len(pts) != 4:
                return pts
            # 先按y排序，再按x排序
            indices = np.lexsort((pts[:, 0], pts[:, 1]))
            return pts[indices]
        
        bottom = sort_layer(bottom)
        top = sort_layer(top)
        
        # 组合：下层4个 + 上层4个
        return np.vstack([bottom, top])

    def _get_box_corners_from_planes(self):
        """使用 box 的 6 个平面，求三元交点，精确得到 8 个角点（与红色盒一致）"""
        if self.box_widget is None or vtk is None:
            return None
        try:
            planes = vtk.vtkPlanes()
            self.box_widget.GetPlanes(planes)
            normals_vtk = planes.GetNormals()
            points_vtk = planes.GetPoints()
            num_planes = normals_vtk.GetNumberOfTuples()
            if num_planes < 6 or points_vtk.GetNumberOfPoints() < 6:
                return None
            normals = np.array([normals_vtk.GetTuple(i) for i in range(num_planes)], dtype=np.float64)
            points_on = np.array([points_vtk.GetPoint(i) for i in range(points_vtk.GetNumberOfPoints())], dtype=np.float64)
            # 归一化法向并计算平面常数 d: n·x = d
            ns = []
            ds = []
            for i in range(min(len(normals), len(points_on))):
                n = np.asarray(normals[i], dtype=np.float64)
                norm = np.linalg.norm(n)
                if norm < 1e-12:
                    continue
                n = n / norm
                d = float(n @ points_on[i])
                ns.append(n)
                ds.append(d)
            ns = np.array(ns, dtype=np.float64)
            ds = np.array(ds, dtype=np.float64)
            if ns.shape[0] < 6:
                return None
            # 将 6 个平面按相反法向配对
            used = set()
            pairs = []
            for i in range(6):
                if i in used:
                    continue
                best_j = None
                best_dot = 1.0
                for j in range(6):
                    if j == i or j in used:
                        continue
                    dot = float(ns[i] @ ns[j])
                    if dot < best_dot:
                        best_dot = dot
                        best_j = j
                if best_j is None:
                    continue
                # 期望接近 -1（相反方向）
                pairs.append((i, best_j))
                used.add(i); used.add(best_j)
            if len(pairs) != 3:
                return None
            # 组合三对，每对取一个，共 2^3 = 8 个角
            corners = []
            for s0 in [0, 1]:
                for s1 in [0, 1]:
                    for s2 in [0, 1]:
                        idxs = [pairs[0][s0], pairs[1][s1], pairs[2][s2]]
                        N = ns[idxs, :]  # 3x3
                        dvec = ds[idxs]  # 3
                        try:
                            x = np.linalg.solve(N, dvec)
                            corners.append(x)
                        except np.linalg.LinAlgError:
                            # 退化数值情况，跳过
                            continue
            if len(corners) < 8:
                # 去重后不足 8 个，尝试容差聚合
                pass
            if len(corners) == 0:
                return None
            corners = np.array(corners, dtype=np.float64)
            # 聚合去重
            uniq = []
            for p in corners:
                if not any(np.linalg.norm(p - q) < 1e-6 for q in uniq):
                    uniq.append(p)
            if len(uniq) != 8:
                # 若有数值重复/缺失，尽量返回最接近 8 个的集合
                uniq = uniq[:8]
                if len(uniq) < 8:
                    return None
            return np.array(uniq, dtype=np.float64)
        except Exception:
            return None

    def on_export_obb(self):
        """导出红色box的8个角点（直接从box widget读取，而非PCA重新计算）"""
        if self.box_widget is None:
            print("请先按 B 创建盒子")
            return
        
        # 方法优先级：
        # 1) 通过六个平面精确求交点（与红色盒完全一致）
        corners = self._get_box_corners_from_planes()
        if corners is None or len(corners) != 8:
            # 2) 回退：从 polydata 的边拓扑提取 8 个三度顶点
            corners = self._get_box_corners_from_widget()
        if corners is None:
            print("无法从box widget获取角点")
            return
        
        np.savetxt("obb_corners.txt", corners, fmt="%.9f")
        print(f"Box corners -> obb_corners.txt (共 {len(corners)} 个角点)")
        print("角点坐标:")
        np.set_printoptions(precision=6, suppress=True)
        for i, corner in enumerate(corners):
            print(f"  [{i}] {corner}")
        
        # 用黄色线框显示（自动按邻接关系连线）
        self._draw_edges(corners)

    def on_export_transform(self):
        P_sel = self._get_sel()
        if P_sel is None: return
        _, T2 = level_transforms(P_sel)
        np.savetxt("T_level_yaw.txt", T2, fmt="%.9f")
        print("Transform -> T_level_yaw.txt (level+align X)")

    def _move_box_by_transform(self, dx: float, dy: float, dz: float):
        """通过变换移动盒子"""
        if self.box_widget is None or vtk is None:
            return False
        try:
            # 获取当前变换
            transform = vtk.vtkTransform()
            self.box_widget.GetTransform(transform)

            # 应用平移
            transform.PostMultiply()
            transform.Translate(float(dx), float(dy), float(dz))

            # 设置新的变换
            self.box_widget.SetTransform(transform)

            # 触发渲染
            self.pl.render()
            return True
        except Exception as e:
            print(f"移动盒子失败: {e}")
            return False

    def on_move_box_left(self):
        """向左移动盒子 (X 负方向) - 按键 A"""
        if self._move_box_by_transform(-0.1, 0.0, 0.0):
            print("盒子左移 0.1 单位 (A)")

    def on_move_box_right(self):
        """向右移动盒子 (X 正方向) - 按键 D"""
        if self._move_box_by_transform(0.1, 0.0, 0.0):
            print("盒子右移 0.1 单位 (D)")

    def on_move_box_up(self):
        """向上移动盒子 (Y 正方向) - 按键 W"""
        if self._move_box_by_transform(0.0, 0.1, 0.0):
            print("盒子上移 0.1 单位 (W)")

    def on_move_box_down(self):
        """向下移动盒子 (Y 负方向) - 按键 X"""
        if self._move_box_by_transform(0.0, -0.1, 0.0):
            print("盒子下移 0.1 单位 (X)")

    def on_move_box_z_up(self):
        """向上移动盒子 (Z 正方向) - 按键 Z"""
        if self._move_box_by_transform(0.0, 0.0, 0.1):
            print("盒子Z轴上移 0.1 单位 (Z)")

    def on_move_box_z_down(self):
        """向下移动盒子 (Z 负方向) - 按键 C"""
        if self._move_box_by_transform(0.0, 0.0, -0.1):
            print("盒子Z轴下移 0.1 单位 (C)")

    def _rotate_box_by_transform(self, angle_degrees: float):
        """绕世界 Z 轴(即在 XY 平面)并以盒子中心为枢轴旋转 angle_degrees。"""
        if self.box_widget is None or vtk is None:
            return False
        try:
            # 1) 获取当前完整变换矩阵 M_current
            M_current_vtk = vtk.vtkTransform()
            self.box_widget.GetTransform(M_current_vtk)
            M_cur_mat = M_current_vtk.GetMatrix()
            M_current = np.eye(4, dtype=np.float64)
            for i in range(4):
                for j in range(4):
                    M_current[i, j] = float(M_cur_mat.GetElement(i, j))

            # 2) 计算旋转枢轴（世界坐标）：用当前 polydata 的 AABB 中心
            box_poly_vtk = vtk.vtkPolyData()
            self.box_widget.GetPolyData(box_poly_vtk)
            if box_poly_vtk is None or box_poly_vtk.GetNumberOfPoints() == 0:
                print("无法获取 box polydata，旋转已取消")
                return False
            b0 = box_poly_vtk.GetBounds()
            if b0 is None:
                print("无法获取 box bounds，旋转已取消")
                return False
            Cx = float((b0[0] + b0[1]) / 2.0)
            Cy = float((b0[2] + b0[3]) / 2.0)
            Cz = float((b0[4] + b0[5]) / 2.0)
            print(f"Box center (world): {Cx:.6f}, {Cy:.6f}, {Cz:.6f}")

            # 3) 构造增量旋转矩阵 delta = T(C) * Rz * T(-C)
            theta = np.deg2rad(float(angle_degrees))
            c, s = np.cos(theta), np.sin(theta)
            Rz = np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], dtype=np.float64)
            T_neg = np.array([[1,0,0,-Cx],[0,1,0,-Cy],[0,0,1,-Cz],[0,0,0,1]], dtype=np.float64)
            T_pos = np.array([[1,0,0,Cx],[0,1,0,Cy],[0,0,1,Cz],[0,0,0,1]], dtype=np.float64)
            delta = T_pos @ Rz @ T_neg

            # 4) 合成最终矩阵：M_new = delta * M_current
            M_new = delta @ M_current

            # 5) 应用最终矩阵
            M_new_vtk = vtk.vtkMatrix4x4()
            for i in range(4):
                for j in range(4):
                    M_new_vtk.SetElement(i, j, float(M_new[i, j]))
            T_new = vtk.vtkTransform()
            T_new.SetMatrix(M_new_vtk)
            self.box_widget.SetTransform(T_new)

            self.pl.render()

            # 6) 旋转后中心核验
            box_poly_vtk2 = vtk.vtkPolyData()
            self.box_widget.GetPolyData(box_poly_vtk2)
            if box_poly_vtk2 is not None and box_poly_vtk2.GetNumberOfPoints() > 0:
                b2 = box_poly_vtk2.GetBounds()
                if b2 is not None:
                    cx2 = (b2[0] + b2[1]) / 2.0
                    cy2 = (b2[2] + b2[3]) / 2.0
                    cz2 = (b2[4] + b2[5]) / 2.0
                    print(f"Box center after (world): {cx2:.6f}, {cy2:.6f}, {cz2:.6f}")
            return True
        except Exception as e:
            print(f"旋转盒子失败: {e}")
            return False

    def on_rotate_box_left(self):
        """向左旋转盒子 (绕Z轴逆时针) - 方向键左"""
        if self._rotate_box_by_transform(-3.0):  # 每次旋转3度
            print("盒子逆时针旋转 3 度 (方向键左)")

    def on_rotate_box_right(self):
        """向右旋转盒子 (绕Z轴顺时针) - 方向键右"""
        if self._rotate_box_by_transform(3.0):  # 每次旋转3度
            print("盒子顺时针旋转 3 度 (方向键右)")

    def _get_sel(self):
        """直接从当前交互盒实时计算并返回点。"""
        # 计算当前盒子内的点
        mask, P_sel = self._compute_selection_from_box()
        
        # 检查有效性
        if mask is None:
            # 区分两类情况：1) 没有盒子/无法计算；2) 有盒子但盒内无点
            if self.box_widget is None and (self.box_last_transform is None):
                print("请先 B 创建盒子")
            return None
            
        if P_sel is None or len(P_sel) == 0:
            print("当前盒子内没有点，请调整盒子")
            return None
            
        return P_sel

    def _compute_selection_from_box(self):
        # 返回 (mask, P_sel)，若无法计算则 (None, None)
        if self.points is None:
            return None, None
        # 计算数据尺度用于数值容差
        try:
            mins = self.points.min(axis=0)
            maxs = self.points.max(axis=0)
            diag = float(np.linalg.norm(maxs - mins))
        except Exception:
            diag = 1.0
        eps_plane = max(1e-9, 1e-6 * diag)
        eps_local = max(1e-9, 1e-6 * diag)

        # 优先：直接使用盒子的封闭曲面做“点在封闭体内”测试（与显示一致，最可靠）
        if self.box_widget is not None and vtk is not None:
            try:
                box_poly_vtk = vtk.vtkPolyData()
                # vtkBoxWidget 直接提供 GetPolyData
                self.box_widget.GetPolyData(box_poly_vtk)
                if box_poly_vtk.GetNumberOfPoints() > 0:
                    box_poly = pv.wrap(box_poly_vtk)
                    pts_poly = pv.PolyData(self.points)
                    enclosed = pts_poly.select_enclosed_points(box_poly, tolerance=eps_plane, check_surface=True)
                    sel_arr_name = 'SelectedPoints'
                    if sel_arr_name not in enclosed.point_data:
                        sel_arr_name = next((k for k in enclosed.point_data.keys() if 'Selected' in k), None)
                    
                    if sel_arr_name:
                        sel = enclosed[sel_arr_name].astype(bool)
                        if sel.any():
                            return sel, self.points[sel]
            except Exception:
                pass


        # 回退：基于六个平面的半空间测试（最稳健）
        if self.box_widget is not None and vtk is not None:
            try:
                planes = vtk.vtkPlanes()
                # vtkBoxWidget 直接提供 GetPlanes
                self.box_widget.GetPlanes(planes)
                normals_vtk = planes.GetNormals()
                points_vtk = planes.GetPoints()
                num_planes = normals_vtk.GetNumberOfTuples()
                if num_planes >= 6:
                    normals = np.array([normals_vtk.GetTuple(i) for i in range(num_planes)], dtype=np.float64)
                    pts_on_plane = np.array([points_vtk.GetPoint(i) for i in range(points_vtk.GetNumberOfPoints())], dtype=np.float64)
                    X = self.points.astype(np.float64)
                    diffs = X[None, :, :] - pts_on_plane[:, None, :]
                    dots = np.einsum('pni,pi->pn', diffs, normals)
                    
                    eps = eps_plane
                    inside_out = np.all(dots <= eps, axis=0)   # 法向外：内点满足 <= 0
                    inside_in  = np.all(dots >= -eps, axis=0)  # 法向内：内点满足 >= 0
                    if inside_out.sum() >= inside_in.sum():
                        mask = inside_out
                    else:
                        mask = inside_in
                    
                    if mask.any():
                        return mask, self.points[mask]
            except Exception:
                pass

        # 回退：取当前变换矩阵进行本地坐标阈值测试
        T_vtk = None
        try:
            if self.box_widget is not None:
                # vtkBoxWidget 直接提供 GetTransform
                transform = vtk.vtkTransform()
                self.box_widget.GetTransform(transform)
                T_vtk = transform.GetMatrix()
        except Exception:
            pass

        if T_vtk is None and self.box_last_transform is not None:
            try:
                # self.box_last_transform 现在是 vtkTransform 对象
                T_vtk = self.box_last_transform.GetMatrix()
            except Exception:
                pass
        
        if T_vtk is None:
            return None, None
        
        M = np.eye(4)
        for i in range(3):
            for j in range(4):
                M[i,j] = T_vtk.GetElement(i,j)
        
        try:
            Minv = np.linalg.inv(M)
        except np.linalg.LinAlgError:
            return None, None

        P_h = np.c_[self.points, np.ones(len(self.points))]
        local = (Minv @ P_h.T).T[:, :3]

        # 兼容多种局部坐标定义
        eps = eps_local
        mask_centered = np.all(np.abs(local) <= 0.5 + eps, axis=1)  # [-0.5,0.5]
        mask_unit = np.all((local >= -eps) & (local <= 1 + eps), axis=1)  # [0,1]
        mask_twounit = np.all(np.abs(local) <= 1.0 + eps, axis=1)  # [-1,1]
        
        counts = [int(mask_centered.sum()), int(mask_unit.sum()), int(mask_twounit.sum())]
        masks = [mask_centered, mask_unit, mask_twounit]
        idx_best = int(np.argmax(counts))
        mask = masks[idx_best]

        if mask.any():
            return mask, self.points[mask]

        return None, None

    def _draw_edges(self, corners):
        """根据角点自动构建 12 条边（每个角点连接最近的 3 个邻居）并绘制。"""
        corners = np.asarray(corners, dtype=np.float64)
        if corners.shape[0] < 8:
            return
        # 计算两两距离
        dists = np.linalg.norm(corners[:, None, :] - corners[None, :, :], axis=2)
        edges = set()
        for i in range(corners.shape[0]):
            idxs = np.argsort(dists[i])
            # 跳过自身，从最近的开始取 3 个
            picked = 0
            for j in idxs[1:]:
                if picked >= 3:
                    break
                # 过滤异常距离
                if dists[i, j] < 1e-9:
                    continue
                a, b = (i, j) if i < j else (j, i)
                if (a, b) not in edges:
                    edges.add((a, b))
                    picked += 1
        # 若因数值问题多于 12 条边，保留距离较短的 12 条
        if len(edges) > 12:
            edges = sorted(list(edges), key=lambda ab: dists[ab[0], ab[1]])[:12]
        else:
            edges = list(edges)
        # 生成线段点与 cells
        line_points = []
        for a, b in edges:
            line_points.append(corners[a]); line_points.append(corners[b])
        line_points = np.asarray(line_points, dtype=np.float64)
        segs = np.arange(0, len(line_points)).reshape(-1, 2)
        poly = pv.PolyData()
        poly.points = line_points
        cells = np.hstack([np.full((len(segs), 1), 2), segs]).astype(np.int64)
        poly.lines = cells
        if "_obb" in self._actors:
            try:
                self.pl.remove_actor(self._actors["_obb"])
            except:
                pass
        self._actors["_obb"] = self.pl.add_mesh(poly, line_width=2, color="yellow")

if __name__ == "__main__":
    app = App()
    # 若提供文件路径参数，则自动加载该点云
    if len(sys.argv) > 1:
        arg_path = sys.argv[1]
        if os.path.isfile(arg_path):
            print(f"从命令行加载: {arg_path}")
            app.load_from_path(arg_path)
        else:
            print(f"警告: 提供的路径不存在: {arg_path}")
    # 保证主线程进入 Qt 事件循环，避免脚本创建窗口后立即退出。
    qapp = getattr(app.pl, "app", None)
    if qapp is not None:
        if hasattr(qapp, "exec_"):
            qapp.exec_()
        else:
            qapp.exec()