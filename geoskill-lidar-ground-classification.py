#!/usr/bin/env python3
"""lidar-ground-classification — LiDAR 地面点分类

对 LiDAR 点云 (N, 3[, classification]) 执行地面 / 非地面分类。实现两种滤波：

- **PMF**（渐进形态学滤波，Zhang et al. 2003 简化版）：把点云栅格化为
  最低高程面，用逐级增大的窗口做灰度形态学开运算，高程差超过自适应
  阈值（随地物尺度放宽）的格网被视为地物并削平，得到地面估计面。
- **slope**（坡度滤波）：大窗口最低值滤波去除地物 + 坡度异常检测。

每个点按其高程与所在格网地面估计面的差值分类：差 ≤ 容差 → 地面
（ASPRS class 2），否则非地面（class 1）。输出分类点云、DTM 与点密度图。

数据源：本地点云（.npy 或 .csv/.txt xyz），或 ``--synthetic`` 生成
平滑地形 + 建筑 + 树木的模拟点云（离线，局部米制坐标）。

隐私声明 / Privacy：
- 完全离线运行，不访问任何网络服务。
- 所有处理在本地完成，不上传任何用户数据。

Usage:
    python lidar-ground-classification.py --input cloud.npy --method pmf --cell-size 1.0
    python lidar-ground-classification.py --bbox 116 39 117 40 --synthetic --output-dir ./out

License: MIT
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

VERSION = "1.0.0"
SKILL_NAME = "lidar-ground-classification"

# ---- 复用共享核心库（本地 vendored，随脚本目录一起分发）----
try:
    from _geoskill_core.errors import (
        GeoSkillError, UsageError, ValidationError, ProcessError, to_exit_code,
    )
    from _geoskill_core.manifest import OutputManifest, OutputFile
except ImportError:  # pragma: no cover - fallback minimal definitions
    class GeoSkillError(Exception):
        def __init__(self, message: str, code: int = 7, kind: str = "EGeo", **kw):
            super().__init__(message)
            self.message, self.code, self.kind = message, code, kind

    class UsageError(GeoSkillError):
        def __init__(self, m, **k): super().__init__(m, code=2, kind="EUsage", **k)

    class ValidationError(GeoSkillError):
        def __init__(self, m, **k): super().__init__(m, code=6, kind="EValidate", **k)

    class ProcessError(GeoSkillError):
        def __init__(self, m, **k): super().__init__(m, code=7, kind="EProcess", **k)

    def to_exit_code(exc):
        return getattr(exc, "code", 7)

    OutputManifest = None
    OutputFile = None


def _utc_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat().replace("+00:00", "Z")


# ---------------------------------------------------------------------------
# 输入验证
# ---------------------------------------------------------------------------
def validate_bbox(bbox) -> None:
    """Validate WGS-84 bbox: W<E, S<N, lon∈[-180,180], lat∈[-90,90], nonzero area.

    Raises ValidationError (rc=6) on failure with a human-readable message.
    Bbox that crosses the antimeridian is rejected with a hint to split.
    """
    if bbox is None:
        raise ValidationError("bbox is required (--bbox or derive from --input)")
    if len(bbox) != 4:
        raise ValidationError(f"bbox must have 4 elements [W S E N], got {len(bbox)}")
    W, S, E, N = bbox
    if not all(np.isfinite([W, S, E, N])):
        raise ValidationError(f"bbox must be finite, got {[W, S, E, N]}")
    if W >= E:
        raise ValidationError(
            f"bbox invalid: minLon (W={W}) must be < maxLon (E={E})",
            west=W, east=E,
        )
    if S >= N:
        raise ValidationError(
            f"bbox invalid: minLat (S={S}) must be < maxLat (N={N})",
            south=S, north=N,
        )
    if W < -180 or E > 180:
        raise ValidationError(
            f"bbox longitude out of range [-180, 180]: W={W}, E={E}. "
            f"If the bbox crosses the antimeridian, split into two requests.",
            west=W, east=E,
        )
    if S < -90 or N > 90:
        raise ValidationError(
            f"bbox latitude out of range [-90, 90]: S={S}, N={N}",
            south=S, north=N,
        )
    if (E - W) * (N - S) <= 0:
        raise ValidationError(f"bbox has zero area: W={W}, E={E}, S={S}, N={N}")
    return None


def validate_params(args) -> None:
    """Validate numeric parameters: --cell-size>0, --z-tolerance>=0."""
    if not np.isfinite(args.cell_size):
        raise ValidationError(f"--cell-size must be finite, got {args.cell_size}")
    if args.cell_size <= 0:
        raise ValidationError(f"--cell-size must be > 0, got {args.cell_size}")
    if not np.isfinite(args.z_tolerance):
        raise ValidationError(f"--z-tolerance must be finite, got {args.z_tolerance}")
    if args.z_tolerance < 0:
        raise ValidationError(f"--z-tolerance must be >= 0, got {args.z_tolerance}")
    return None


# ---------------------------------------------------------------------------
# 栅格化辅助
# ---------------------------------------------------------------------------
def grid_extent(points: np.ndarray, cell_size: float
                ) -> Tuple[float, float, int, int]:
    """由点云 xy 范围与 cell_size 计算格网：返回 (xmin, ymax, width, height)。

    行 0 对应最北（ymax），与 north-up GeoTIFF 一致。
    """
    xmin = float(points[:, 0].min())
    ymin = float(points[:, 1].min())
    xmax = float(points[:, 0].max())
    ymax = float(points[:, 1].max())
    w = max(1, int(np.ceil((xmax - xmin) / cell_size)))
    h = max(1, int(np.ceil((ymax - ymin) / cell_size)))
    return xmin, ymax, w, h


def cell_indices(points: np.ndarray, extent: Tuple[float, float, int, int],
                 cell_size: float) -> Tuple[np.ndarray, np.ndarray]:
    """每个点的格网 (xi 列, yi 行)。yi 行 0 = 最北。"""
    xmin, ymax, w, h = extent
    xi = np.clip(((points[:, 0] - xmin) / cell_size).astype(int), 0, w - 1)
    yi = np.clip(((ymax - points[:, 1]) / cell_size).astype(int), 0, h - 1)
    return xi, yi


def rasterize_min_surface(points: np.ndarray, extent, cell_size: float) -> np.ndarray:
    """最低高程面 (H, W)，无点格网为 NaN。"""
    xmin, ymax, w, h = extent
    xi, yi = cell_indices(points, extent, cell_size)
    surf = np.full((h, w), np.inf, dtype=np.float64)
    np.minimum.at(surf, (yi, xi), points[:, 2])
    surf[np.isinf(surf)] = np.nan
    return surf


def point_density(points: np.ndarray, extent, cell_size: float) -> np.ndarray:
    """每格网点数 (H, W)。"""
    xmin, ymax, w, h = extent
    xi, yi = cell_indices(points, extent, cell_size)
    dens = np.zeros((h, w), dtype=np.float64)
    np.add.at(dens, (yi, xi), 1.0)
    return dens


def fill_nan_nearest(grid: np.ndarray) -> np.ndarray:
    """用最近有效格网值填充 NaN。"""
    from scipy.ndimage import distance_transform_edt
    mask = np.isnan(grid)
    if not mask.any():
        return grid.copy()
    if mask.all():
        return np.zeros_like(grid)
    _, idx = distance_transform_edt(mask, return_indices=True)
    return grid[tuple(idx)]


def grey_opening(grid: np.ndarray, size: int) -> np.ndarray:
    from scipy.ndimage import grey_erosion, grey_dilation
    return grey_dilation(grey_erosion(grid, size=size), size=size)


# ---------------------------------------------------------------------------
# 核心算法：地面估计
# ---------------------------------------------------------------------------
def pmf_ground_surface(grid: np.ndarray, cell_size: float, dh_base: float = 0.5,
                       max_slope: float = 0.15,
                       max_window_cells: Optional[int] = None) -> np.ndarray:
    """渐进形态学滤波估计地面面。

    窗口逐级增大（3, 7, 15, 31...）；每级高程差阈值
    dh = dh_base + max_slope × 窗口宽度(米)，随地物尺度放宽，
    既去除小物体（树）也去除大物体（建筑）而不破坏平缓地形。
    """
    surface = grid.copy()
    if max_window_cells is None:
        max_window_cells = max(3, (min(grid.shape) // 2) * 2 + 1)
    half = 1
    while 2 * half + 1 <= max_window_cells:
        size = 2 * half + 1
        dh = dh_base + max_slope * size * cell_size
        opened = grey_opening(surface, size=size)
        diff = surface - opened
        mask = diff > dh
        surface[mask] = opened[mask]
        half = 2 * half + 1
    return surface


def slope_ground_surface(grid: np.ndarray, cell_size: float,
                         window_cells: int = 17,
                         slope_thresh_deg: float = 25.0) -> np.ndarray:
    """坡度滤波估计地面面：大窗口最低值去地物 + 坡度异常抑制。"""
    from scipy.ndimage import minimum_filter, median_filter, grey_erosion
    win = max(3, window_cells)
    min_surf = minimum_filter(grid, size=win)
    smoothed = median_filter(min_surf, size=5)
    gy, gx = np.gradient(smoothed, cell_size)
    slope_deg = np.degrees(np.arctan(np.hypot(gx, gy)))
    steep = slope_deg > slope_thresh_deg
    eroded = grey_erosion(smoothed, size=3)
    out = smoothed.copy()
    out[steep] = np.minimum(out[steep], eroded[steep])
    return out


def estimate_ground_surface(grid: np.ndarray, cell_size: float,
                            method: str = "pmf") -> np.ndarray:
    filled = fill_nan_nearest(grid)
    if method == "pmf":
        return pmf_ground_surface(filled, cell_size)
    if method == "slope":
        return slope_ground_surface(filled, cell_size)
    raise UsageError(f"unknown method '{method}'. Choose from: pmf, slope", method=method)


def classify_points(points: np.ndarray, ground_surface: np.ndarray, extent,
                    cell_size: float, z_tolerance: float = 0.6) -> np.ndarray:
    """逐点分类：与格网地面面高差 ≤ 容差 → 地面(2)，否则非地面(1)。"""
    xi, yi = cell_indices(points, extent, cell_size)
    gz = ground_surface[yi, xi]
    is_ground = (points[:, 2] - gz) <= z_tolerance
    return np.where(is_ground, 2, 1).astype(np.int32)


# ---------------------------------------------------------------------------
# 合成数据：平滑地形 + 建筑 + 树木（局部米制坐标）
# ---------------------------------------------------------------------------
def terrain_height(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """平缓地形：小坡度倾斜 + 低幅起伏（局部起伏 < 0.6 m）。"""
    return 0.02 * x + 0.25 * np.sin(x * 0.12) * np.cos(y * 0.10)


def generate_synthetic(bbox: List[float], cell_size: float = 1.0, seed: int = 42,
                       extent_m: float = 64.0, n_buildings: int = 3, n_trees: int = 8
                       ) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    """生成 (N, 3) 点云 + 真值分类 (N,) ∈ {1 非地面, 2 地面}。"""
    rng = np.random.default_rng(seed)
    pts: List[np.ndarray] = []
    labs: List[np.ndarray] = []

    # 地面点
    n_ground = 5000
    gx = rng.uniform(0.0, extent_m, n_ground)
    gy = rng.uniform(0.0, extent_m, n_ground)
    gz = terrain_height(gx, gy) + rng.normal(0.0, 0.03, n_ground)
    pts.append(np.column_stack([gx, gy, gz]))
    labs.append(np.full(n_ground, 2, dtype=np.int32))

    # 建筑：矩形平面，高 5–10 m
    placed: List[Tuple[float, float, float]] = []
    for _ in range(n_buildings):
        bw = rng.uniform(4.0, 8.0)
        bh = rng.uniform(4.0, 8.0)
        cx = rng.uniform(bw, extent_m - bw)
        cy = rng.uniform(bh, extent_m - bh)
        ht = rng.uniform(5.0, 10.0)
        placed.append((cx, cy, max(bw, bh) / 2.0 + 2.0))
        n = 160
        px = rng.uniform(cx - bw / 2, cx + bw / 2, n)
        py = rng.uniform(cy - bh / 2, cy + bh / 2, n)
        pz = terrain_height(px, py) + ht + rng.normal(0.0, 0.05, n)
        pts.append(np.column_stack([px, py, pz]))
        labs.append(np.full(n, 1, dtype=np.int32))

    # 树木：锥形冠层，高 3–8 m（避开建筑）
    trees_made = 0
    attempts = 0
    while trees_made < n_trees and attempts < 200:
        attempts += 1
        R = rng.uniform(1.5, 3.0)
        cx = rng.uniform(R + 1, extent_m - R - 1)
        cy = rng.uniform(R + 1, extent_m - R - 1)
        if any(np.hypot(cx - bx, cy - by) < br + R for bx, by, br in placed):
            continue
        ht = rng.uniform(3.0, 8.0)
        n = 120
        ang = rng.uniform(0, 2 * np.pi, n)
        rad = R * np.sqrt(rng.uniform(0, 1, n))
        px = cx + rad * np.cos(ang)
        py = cy + rad * np.sin(ang)
        pz = terrain_height(px, py) + ht * (1.0 - rad / R) + rng.normal(0.0, 0.05, n)
        pts.append(np.column_stack([px, py, pz]))
        labs.append(np.full(n, 1, dtype=np.int32))
        trees_made += 1

    points = np.vstack(pts).astype(np.float64)
    labels = np.concatenate(labs).astype(np.int32)
    info = {
        "bbox": bbox,
        "extent_m": extent_m,
        "cell_size": cell_size,
        "n_ground": int((labels == 2).sum()),
        "n_non_ground": int((labels == 1).sum()),
        "n_buildings": n_buildings,
        "n_trees": trees_made,
    }
    return points, labels, info


# ---------------------------------------------------------------------------
# 点云 I/O
# ---------------------------------------------------------------------------
def read_points(path: str) -> np.ndarray:
    """读取点云为 (N, 3)。支持 .npy（取前 3 列）与 .csv/.txt（前 3 列 xyz）。"""
    if not os.path.exists(path):
        raise UsageError(f"input point cloud not found: {path}", path=path)
    if path.lower().endswith(".npy"):
        arr = np.load(path)
    else:
        try:
            arr = np.loadtxt(path, delimiter=",", ndmin=2)
        except ValueError:
            arr = np.loadtxt(path, ndmin=2)
    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValidationError("point cloud must have at least 3 columns (x, y, z)",
                              shape=list(arr.shape))
    return arr[:, :3]


# ---------------------------------------------------------------------------
# GeoTIFF I/O（直接用 rasterio，保证离线可用）
# ---------------------------------------------------------------------------
def write_geotiff(path: str, cube: np.ndarray, bbox: List[float],
                  nodata: float = -9999.0) -> None:
    import rasterio
    from rasterio.transform import from_bounds

    if cube.ndim == 2:
        cube = cube[np.newaxis, ...]
    nb, h, w = cube.shape
    transform = from_bounds(bbox[0], bbox[1], bbox[2], bbox[3], w, h)
    profile = {
        "driver": "GTiff", "height": h, "width": w, "count": nb,
        "dtype": "float32", "crs": "EPSG:4326", "transform": transform,
        "nodata": nodata, "compress": "deflate",
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with rasterio.open(path, "w", **profile) as dst:
        for b in range(nb):
            dst.write(cube[b].astype("float32"), b + 1)


# ---------------------------------------------------------------------------
# Manifest
# ---------------------------------------------------------------------------
def write_manifest(output_dir: str, args: argparse.Namespace,
                   outputs: List[Dict[str, Any]], qa: Dict[str, Any],
                   started_at: str, exit_code: int, bbox: List[float]) -> Optional[str]:
    if OutputManifest is None:
        return None
    cmd = " ".join([SKILL_NAME] + sys.argv[1:])
    man = OutputManifest(
        skill=SKILL_NAME,
        skill_version=VERSION,
        command=cmd,
        started_at=started_at,
        finished_at=_utc_now(),
        exit_code=exit_code,
        inputs={
            "input": getattr(args, "input", None),
            "method": getattr(args, "method", None),
            "cell_size": getattr(args, "cell_size", None),
            "synthetic": bool(getattr(args, "synthetic", False)),
        },
        outputs=[OutputFile(**o) for o in outputs],
        qa=qa,
        software={"python": sys.version.split()[0], "skill": SKILL_NAME},
    )
    path = os.path.join(output_dir, "output-manifest.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(man.to_dict(), f, ensure_ascii=False, indent=2, default=str)
    return path


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def process(args: argparse.Namespace) -> int:
    started_at = _utc_now()
    output_dir = args.output_dir
    bbox = list(args.bbox) if args.bbox else None

    # ---- 1. 参数与路径验证（先于任何 makedirs / 数据读取）----
    if args.input is None and bbox is None:
        raise UsageError("provide --bbox (synthetic mode) or --input <point cloud>")
    if args.input:
        if not os.path.exists(args.input):
            raise UsageError(f"input point cloud not found: {args.input}", path=args.input)
    validate_params(args)

    # ---- 2. 读取输入（可能推导 bbox）----
    truth: Optional[np.ndarray] = None
    synth_info: Optional[Dict[str, Any]] = None
    if args.input and not args.synthetic:
        points = read_points(args.input)
        if bbox is None:
            # 用点云自身 xy 范围作为地理标注
            bbox = [float(points[:, 0].min()), float(points[:, 1].min()),
                    float(points[:, 0].max()), float(points[:, 1].max())]
        # 校验 bbox
        validate_bbox(bbox)
        source_note = args.input
    else:
        validate_bbox(bbox)
        points, truth, synth_info = generate_synthetic(bbox, cell_size=args.cell_size)
        source_note = "synthetic"

    if points.shape[0] == 0:
        raise ValidationError("point cloud is empty")
    if not np.all(np.isfinite(points)):
        raise ValidationError("point cloud contains non-finite coordinates")

    # ---- 3. 全部验证已通过，创建输出目录 ----
    os.makedirs(output_dir, exist_ok=True)

    # 栅格化 + 地面估计 + 分类
    extent = grid_extent(points, args.cell_size)
    min_surf = rasterize_min_surface(points, extent, args.cell_size)
    ground = estimate_ground_surface(min_surf, args.cell_size, method=args.method)
    classes = classify_points(points, ground, extent, args.cell_size,
                              z_tolerance=args.z_tolerance)
    density = point_density(points, extent, args.cell_size)

    n_ground = int((classes == 2).sum())
    n_non = int((classes == 1).sum())

    # 写出产物
    pts_out = np.column_stack([points, classes.astype(np.float64)])
    npy_path = os.path.join(output_dir, "classified_points.npy")
    np.save(npy_path, pts_out)

    dtm_path = os.path.join(output_dir, "dtm.tif")
    write_geotiff(dtm_path, ground.astype(np.float32), bbox)

    dens_path = os.path.join(output_dir, "density.tif")
    write_geotiff(dens_path, density.astype(np.float32), bbox, nodata=-1.0)

    stats: Dict[str, Any] = {
        "source": source_note,
        "method": args.method,
        "cell_size": args.cell_size,
        "n_points": int(points.shape[0]),
        "n_ground": n_ground,
        "n_non_ground": n_non,
        "ground_fraction": n_ground / max(points.shape[0], 1),
    }
    if truth is not None:
        acc = float(np.mean(classes == truth))
        stats["accuracy"] = acc
    stats_path = os.path.join(output_dir, "stats.json")
    with open(stats_path, "w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=2)

    h, w = ground.shape
    qa: Dict[str, Any] = {
        "source": source_note,
        "method": args.method,
        "n_points": int(points.shape[0]),
        "n_ground": n_ground,
        "n_non_ground": n_non,
        "accuracy": stats.get("accuracy"),
    }
    outputs = [
        {"path": npy_path, "kind": "table", "row_count": int(points.shape[0])},
        {"path": dtm_path, "kind": "raster", "crs_epsg": 4326,
         "bbox_wgs84": bbox, "band_count": 1},
        {"path": dens_path, "kind": "raster", "crs_epsg": 4326,
         "bbox_wgs84": bbox, "band_count": 1},
        {"path": stats_path, "kind": "json"},
    ]
    man_path = write_manifest(output_dir, args, outputs, qa, started_at, 0, bbox)

    if not args.quiet:
        print(f"[{SKILL_NAME}] source: {source_note}")
        print(f"[{SKILL_NAME}] method: {args.method}  cell_size: {args.cell_size}  grid: {(w, h)}")
        print(f"[{SKILL_NAME}] points: {points.shape[0]}  ground: {n_ground}  non-ground: {n_non}")
        if "accuracy" in stats:
            print(f"[{SKILL_NAME}] accuracy vs truth: {stats['accuracy']:.4f}")
        print(f"[{SKILL_NAME}] output: {npy_path}")
        if man_path:
            print(f"[{SKILL_NAME}] manifest: {man_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog=SKILL_NAME,
        description="LiDAR ground-point classification (progressive morphological / slope filter).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--bbox", nargs=4, type=float, metavar=("W", "S", "E", "N"),
                   help="geographic extent [minLon minLat maxLon maxLat]")
    p.add_argument("--input", help="input point cloud (.npy or .csv/.txt xyz)")
    p.add_argument("--method", default="pmf", choices=["pmf", "slope"],
                   help="ground filtering method (default: pmf)")
    p.add_argument("--cell-size", type=float, default=1.0,
                   help="rasterization cell size in point-cloud units (default: 1.0)")
    p.add_argument("--z-tolerance", type=float, default=0.6,
                   help="height above ground surface to still count as ground (default: 0.6)")
    p.add_argument("--synthetic", action="store_true",
                   help="generate a synthetic terrain + objects point cloud (offline)")
    p.add_argument("--output-dir", default="./output", help="output directory")
    p.add_argument("--quiet", action="store_true", help="suppress console output")
    p.add_argument("--version", action="version", version=f"%(prog)s {VERSION}")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return process(args)
    except GeoSkillError as exc:
        print(f"[{SKILL_NAME}] ERROR [{exc.kind}] {exc.message}", file=sys.stderr)
        return exc.code
    except KeyboardInterrupt:
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"[{SKILL_NAME}] ERROR {exc}", file=sys.stderr)
        return to_exit_code(exc)


if __name__ == "__main__":
    sys.exit(main())
