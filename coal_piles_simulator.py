import math
import os
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


# =========================
# 可配置参数（无需命令行）
# =========================
OUTPUT_PCD_PATH = "coal_piles_simulation.pcd"

# 圆锥参数（3个煤堆）
# 坐标单位：米；地面在 z=0；圆锥朝上，顶点在高度处；半径为底面在 z=0 的半径
# 可直接修改下方三个圆锥参数
CONES_CONFIG = [
	{"center_x": 0.0, "center_y": 2.0, "height": 5.0, "radius": 8.0},
	{"center_x": 8.0, "center_y": 2.0, "height": 4.0, "radius": 5.0},
	{"center_x": -5.0, "center_y": -3.0, "height": 3.5, "radius": 4.5},
]

# 采样分辨率（水平网格间距，单位米）。越小点越密集，但文件更大。
GRID_RESOLUTION_M = 0.15

# 在顶面点上添加高斯噪声（单位米），模拟激光测量噪声；0 表示不加噪声
TOP_Z_NOISE_STD = 0.1

# 开关：是否对地面点云也添加相同高斯噪声（作用于 z 值）
APPLY_NOISE_TO_GROUND = True

# 在采样网格上添加轻微抖动（单位米），打破规则网格
XY_JITTER_STD = 0.1

# 地面点云开关（True 生成地面点；False 不生成）
ENABLE_GROUND = True

# 地面高度 z=0；地面采样与顶面共用 GRID_RESOLUTION_M
GROUND_Z = 0.0

# 避免在煤堆“底面”产生点的阈值（>0 即视为接近底面不出点）
BASE_Z_EPSILON = 1e-6

# 蒙特卡洛体积估计：采样数量（越大越精确，耗时也更长）
MC_NUM_SAMPLES = 300_000
MC_RANDOM_SEED = 42

# 体积估计包围盒边界外扩（米），通常保持 0 即可
BOUNDING_BOX_MARGIN = 0.0

# 全局旋转角（度）：(roll_x, pitch_y, yaw_z)
# 设为非零即可对整个点云施加旋转（先绕 x 再绕 y 再绕 z）
GLOBAL_ROTATION_DEG = (10, 10, 10)

# 顶面“空洞”区域（仅影响煤堆顶面点，不影响地面与体积估计）
# 配置简单：仅一份全局空洞列表，支持 circle 或 rect，两种形状的字段如下：
# - circle: {"type":"circle","center_x":0.0,"center_y":0.0,"radius":1.0}
# - rect:   {"type":"rect","center_x":2.0,"center_y":1.0,"size_x":3.0,"size_y":1.5}  # 轴对齐矩形
HOLES: List[dict] = [
	# 示例：默认无空洞。需要时取消注释或添加新项
	{"type":"circle","center_x":1.0,"center_y":0.0,"radius":1.2},
    {"type":"circle","center_x":5.0,"center_y":5.0,"radius":1.2},
	{"type":"rect","center_x":-1.5,"center_y":2.0,"size_x":1.0,"size_y":2.0},
]

# =========================
# 数据结构与核心几何函数
# =========================
@dataclass
class Cone:
	center_x: float
	center_y: float
	height: float
	radius: float

	def height_at(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
		"""
		给定水平坐标 (x,y)，返回圆锥顶面在该处的 z 值（z>=0）。若 (x,y) 不在圆锥投影内，返回 0。
		圆锥为直圆锥，顶点在 z=height，底面半径 radius，底面在 z=0。
		顶面高度场：z = h * (1 - rho/r)，其中 rho = sqrt((x-cx)^2 + (y-cy)^2)，rho<=r 有效，否则 z=0。
		"""
		dx = x - self.center_x
		dy = y - self.center_y
		rho = np.sqrt(dx * dx + dy * dy)
		z = self.height * (1.0 - rho / self.radius)
		z = np.where(rho <= self.radius, z, 0.0)
		# 截断到 [0, height]
		return np.clip(z, 0.0, self.height)


def build_cones(config_list: List[dict]) -> List[Cone]:
	cones: List[Cone] = []
	for cfg in config_list:
		cones.append(Cone(
			center_x=float(cfg["center_x"]),
			center_y=float(cfg["center_y"]),
			height=float(cfg["height"]),
			radius=float(cfg["radius"]),
		))
	return cones


def cones_bounding_rect(cones: List[Cone], margin: float = 0.0) -> Tuple[float, float, float, float]:
	"""
	返回包含所有圆锥底面投影的最小矩形 [xmin,xmax] x [ymin,ymax]。
	"""
	xmins = [c.center_x - c.radius for c in cones]
	xmaxs = [c.center_x + c.radius for c in cones]
	ymins = [c.center_y - c.radius for c in cones]
	ymaxs = [c.center_y + c.radius for c in cones]
	xmin = min(xmins) - margin
	xmax = max(xmaxs) + margin
	ymin = min(ymins) - margin
	ymax = max(ymaxs) + margin
	return xmin, xmax, ymin, ymax


def top_envelope_height(x: np.ndarray, y: np.ndarray, cones: List[Cone]) -> np.ndarray:
	"""
	顶层包络高度场 z_top(x,y) = max_i z_i(x,y)，仅保留从上方可见的顶面（遮挡/重叠处只取最高）。
	"""
	if len(cones) == 0:
		return np.zeros_like(x, dtype=float)
	z_top = cones[0].height_at(x, y)
	for i in range(1, len(cones)):
		z_i = cones[i].height_at(x, y)
		z_top = np.maximum(z_top, z_i)
	return z_top


# =========================
# 点云生成
# =========================
def _hole_mask_for_shapes(X: np.ndarray, Y: np.ndarray, holes: List[dict]) -> np.ndarray:
	"""
	根据 holes 规格生成布尔掩膜；True 表示位于空洞区域（应移除顶面点）
	支持：
	- circle: center_x, center_y, radius
	- rect:   center_x, center_y, size_x, size_y（轴对齐矩形）
	"""
	if holes is None or len(holes) == 0:
		return np.zeros_like(X, dtype=bool)
	mask = np.zeros_like(X, dtype=bool)
	for h in holes:
		t = h.get("type", "circle")
		cx = float(h.get("center_x", 0.0))
		cy = float(h.get("center_y", 0.0))
		if t == "circle":
			r = float(h.get("radius", 0.0))
			r2 = r * r
			dx = X - cx
			dy = Y - cy
			mask |= (dx * dx + dy * dy) <= r2
		elif t == "rect":
			sx = float(h.get("size_x", 0.0))
			sy = float(h.get("size_y", 0.0))
			mask |= (np.abs(X - cx) <= (sx * 0.5)) & (np.abs(Y - cy) <= (sy * 0.5))
		else:
			continue
	return mask


def generate_point_cloud(cones: List[Cone]) -> np.ndarray:
	"""
	生成模拟的点云：
	- 顶面：仅取顶层包络（遮挡去除），不生成底面点
	- 地面：可选，且不在煤堆投影区域生成（模拟被遮挡）
	- 空洞：通过 HOLES 配置在顶面移除点
	返回 Nx3 的 numpy 数组
	"""
	xmin, xmax, ymin, ymax = cones_bounding_rect(cones, margin=0.0)
	xs = np.arange(xmin, xmax + GRID_RESOLUTION_M * 0.5, GRID_RESOLUTION_M)
	ys = np.arange(ymin, ymax + GRID_RESOLUTION_M * 0.5, GRID_RESOLUTION_M)
	X, Y = np.meshgrid(xs, ys, indexing="xy")

	if XY_JITTER_STD > 0.0:
		jx = np.random.normal(0.0, XY_JITTER_STD, size=X.shape)
		jy = np.random.normal(0.0, XY_JITTER_STD, size=Y.shape)
		X = X + jx
		Y = Y + jy

	# 顶层包络
	Z_top = top_envelope_height(X, Y, cones)
	top_mask = Z_top > BASE_Z_EPSILON

	# 空洞掩膜（仅影响顶面点）
	holes_mask = _hole_mask_for_shapes(X, Y, HOLES)
	top_keep_mask = top_mask & (~holes_mask)

	top_points = np.stack([X[top_keep_mask], Y[top_keep_mask], Z_top[top_keep_mask]], axis=1)
	if TOP_Z_NOISE_STD > 0.0 and top_points.size > 0:
		top_points[:, 2] += np.random.normal(0.0, TOP_Z_NOISE_STD, size=top_points.shape[0])

	point_list = [top_points]

	# 地面点：仅在不被煤堆投影覆盖的位置
	if ENABLE_GROUND:
		ground_mask = Z_top <= BASE_Z_EPSILON
		if np.any(ground_mask):
			ground_points = np.stack([X[ground_mask], Y[ground_mask], np.full(np.count_nonzero(ground_mask), GROUND_Z)], axis=1)
			if APPLY_NOISE_TO_GROUND and TOP_Z_NOISE_STD > 0.0 and ground_points.size > 0:
				ground_points[:, 2] += np.random.normal(0.0, TOP_Z_NOISE_STD, size=ground_points.shape[0])
			point_list.append(ground_points)

	if len(point_list) == 0:
		return np.empty((0, 3), dtype=float)
	return np.concatenate(point_list, axis=0)


# =========================
# 蒙特卡洛体积估计（并集体积）
# =========================
def estimate_union_volume_mc(cones: List[Cone]) -> Tuple[float, float]:
	"""
	用蒙特卡洛法估计三圆锥并集体积：
	- 包围盒：水平用底面包围矩形，垂直用 [0, max(height)]
	- 在包围盒内均匀采样 (x,y,z)，若 z <= z_top(x,y) 则视为在并集内
	返回 (体积估计值, 标准误差估计)
	"""
	if len(cones) == 0:
		return 0.0, 0.0

	xmin, xmax, ymin, ymax = cones_bounding_rect(cones, margin=BOUNDING_BOX_MARGIN)
	zmin = 0.0
	zmax = max(c.height for c in cones)

	box_volume = (xmax - xmin) * (ymax - ymin) * (zmax - zmin)
	if box_volume <= 0.0:
		return 0.0, 0.0

	rng = np.random.default_rng(MC_RANDOM_SEED)
	xs = rng.uniform(xmin, xmax, MC_NUM_SAMPLES)
	ys = rng.uniform(ymin, ymax, MC_NUM_SAMPLES)
	zs = rng.uniform(zmin, zmax, MC_NUM_SAMPLES)

	# 计算顶层包络（向量化）
	z_top = np.zeros_like(xs)
	for c in cones:
		dx = xs - c.center_x
		dy = ys - c.center_y
		rho = np.sqrt(dx * dx + dy * dy)
		z_i = c.height * (1.0 - rho / c.radius)
		z_i = np.where(rho <= c.radius, z_i, 0.0)
		z_i = np.clip(z_i, 0.0, c.height)
		z_top = np.maximum(z_top, z_i)

	hits = zs <= z_top
	hit_ratio = float(np.count_nonzero(hits)) / float(MC_NUM_SAMPLES)
	volume_est = box_volume * hit_ratio

	# 二项分布标准差：sqrt(p*(1-p)/N) * box_volume
	std_err = box_volume * math.sqrt(max(hit_ratio * (1.0 - hit_ratio), 0.0) / float(MC_NUM_SAMPLES))
	return volume_est, std_err


# =========================
# PCD 写出
# =========================
def write_pcd_ascii(points_xyz: np.ndarray, save_path: str) -> None:
	"""
	将 Nx3 点写为 PCD (ASCII) 文件。字段：x y z（float）
	"""
	if points_xyz.ndim != 2 or points_xyz.shape[1] != 3:
		raise ValueError("points_xyz 需要是 Nx3 的二维数组")
	num_points = points_xyz.shape[0]
	header_lines = [
		"# .PCD v0.7 - Point Cloud Data file format",
		"VERSION 0.7",
		"FIELDS x y z",
		"SIZE 4 4 4",
		"TYPE F F F",
		"COUNT 1 1 1",
		f"WIDTH {num_points}",
		"HEIGHT 1",
		"VIEWPOINT 0 0 0 1 0 0 0",
		f"POINTS {num_points}",
		"DATA ascii",
	]
	with open(save_path, "w", encoding="utf-8") as f:
		for line in header_lines:
			f.write(line + "\n")
		# 输出坐标，限制小数位以控制文件体积
		for p in points_xyz:
			f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

def _rotation_matrix_from_euler_deg_xyz(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
	"""
	构建旋转矩阵 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)，欧拉角以度为单位。
	"""
	rx = math.radians(float(roll_deg))
	ry = math.radians(float(pitch_deg))
	rz = math.radians(float(yaw_deg))
	cx, sx = math.cos(rx), math.sin(rx)
	cy, sy = math.cos(ry), math.sin(ry)
	cz, sz = math.cos(rz), math.sin(rz)
	Rx = np.array([[1, 0, 0],
	               [0, cx, -sx],
	               [0, sx,  cx]], dtype=float)
	Ry = np.array([[ cy, 0, sy],
	               [  0, 1,  0],
	               [-sy, 0, cy]], dtype=float)
	Rz = np.array([[cz, -sz, 0],
	               [sz,  cz, 0],
	               [ 0,   0, 1]], dtype=float)
	return (Rz @ Ry @ Rx)


# =========================
# 主流程
# =========================
def main() -> None:
	# 构造圆锥
	cones = build_cones(CONES_CONFIG)

	# 生成点云
	points = generate_point_cloud(cones)
	# 全局旋转（若配置为非零角）
	r_deg, p_deg, y_deg = GLOBAL_ROTATION_DEG
	if abs(r_deg) > 1e-12 or abs(p_deg) > 1e-12 or abs(y_deg) > 1e-12:
		R = _rotation_matrix_from_euler_deg_xyz(r_deg, p_deg, y_deg)
		# points 为 Nx3 行向量，右乘 R^T 等价于左乘 R
		points = points @ R.T
	print(f"[Info] 生成点数: {points.shape[0]}")

	# 蒙特卡洛体积估计
	volume_est, std_err = estimate_union_volume_mc(cones)
	print(f"[Info] 并集体积（蒙特卡洛估计）: {volume_est:.3f} m^3  (±{std_err:.3f})")

	# 写出PCD
	save_path = os.path.abspath(OUTPUT_PCD_PATH)
	write_pcd_ascii(points, save_path)
	print(f"[Info] PCD 已保存: {save_path}")


if __name__ == "__main__":
	# 为了可复现的点云采样（抖动/噪声），设置随机种子
	np.random.seed(12345)
	main()


