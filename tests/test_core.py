"""Core algorithm tests for lidar-ground-classification."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from conftest import mod as lg


class TestRasterize:
    def test_min_surface_and_density(self):
        # 两个点落在不同格网
        points = np.array([[0.5, 0.5, 2.0],
                           [0.5, 0.5, 5.0],   # 同一格网，min 取 2.0
                           [3.5, 3.5, 9.0]])
        extent = lg.grid_extent(points, 1.0)
        surf = lg.rasterize_min_surface(points, extent, 1.0)
        dens = lg.point_density(points, extent, 1.0)
        assert dens.sum() == 3.0
        # min surface：含两点的格网取最低
        assert np.nanmin(surf) == 2.0

    def test_fill_nan(self):
        grid = np.array([[1.0, np.nan], [np.nan, 4.0]])
        filled = lg.fill_nan_nearest(grid)
        assert not np.isnan(filled).any()

    def test_fill_all_nan(self):
        grid = np.full((3, 3), np.nan)
        filled = lg.fill_nan_nearest(grid)
        assert (filled == 0.0).all()


class TestPmf:
    def test_removes_tall_object(self):
        """平坦地面上放一个高方块，PMF 应把它削到地面附近。"""
        grid = np.zeros((32, 32), dtype=np.float64)
        grid[12:20, 12:20] = 8.0  # 8 m 高、8 格宽的建筑
        out = lg.pmf_ground_surface(grid, cell_size=1.0)
        assert out[16, 16] < 1.0  # 建筑中心被削平

    def test_preserves_slope(self):
        """缓倾斜面应基本保持。"""
        yy, xx = np.mgrid[0:32, 0:32]
        grid = 0.02 * xx.astype(np.float64)
        out = lg.pmf_ground_surface(grid, cell_size=1.0)
        assert np.abs(out - grid).max() < 1.0


class TestSlope:
    def test_removes_tall_object(self):
        grid = np.zeros((40, 40), dtype=np.float64)
        grid[15:23, 15:23] = 7.0
        out = lg.slope_ground_surface(grid, cell_size=1.0)
        assert out[19, 19] < 2.0


class TestDispatch:
    def test_unknown_method(self):
        grid = np.zeros((8, 8))
        with pytest.raises(lg.UsageError):
            lg.estimate_ground_surface(grid, 1.0, method="csmf")


class TestSyntheticAndAccuracy:
    @pytest.mark.parametrize("method", ["pmf", "slope"])
    def test_accuracy_above_08(self, method):
        """端到端：合成场景分类精度 > 0.8。"""
        points, truth, info = lg.generate_synthetic([116, 39, 117, 40],
                                                    cell_size=1.0, seed=21)
        extent = lg.grid_extent(points, 1.0)
        surf = lg.rasterize_min_surface(points, extent, 1.0)
        ground = lg.estimate_ground_surface(surf, 1.0, method=method)
        cls = lg.classify_points(points, ground, extent, 1.0)
        acc = float(np.mean(cls == truth))
        assert acc > 0.8, f"method={method} acc={acc}"

    def test_truth_labels_valid(self):
        points, truth, info = lg.generate_synthetic([116, 39, 117, 40], seed=1)
        assert set(np.unique(truth).tolist()) <= {1, 2}
        assert info["n_buildings"] == 3
        assert points.shape[0] == truth.shape[0]


class TestClassify:
    def test_ground_vs_high(self):
        ground = np.zeros((10, 10))
        points = np.array([[1.0, 1.0, 0.1],    # 接近地面 → 2
                           [5.0, 5.0, 6.0]])    # 高出地面 → 1
        extent = (0.0, 10.0, 10, 10)
        cls = lg.classify_points(points, ground, extent, 1.0, z_tolerance=0.6)
        assert cls[0] == 2 and cls[1] == 1


class TestReadPoints:
    def test_read_npy(self, tmp_path):
        arr = np.random.rand(20, 3)
        p = str(tmp_path / "c.npy")
        np.save(p, arr)
        out = lg.read_points(p)
        assert out.shape == (20, 3)

    def test_read_npy_4col(self, tmp_path):
        arr = np.random.rand(20, 4)
        p = str(tmp_path / "c.npy")
        np.save(p, arr)
        out = lg.read_points(p)
        assert out.shape == (20, 3)

    def test_read_txt(self, tmp_path):
        p = str(tmp_path / "c.txt")
        with open(p, "w", encoding="utf-8") as f:
            f.write("0 0 1\n1 1 2\n2 2 3\n")
        out = lg.read_points(p)
        assert out.shape == (3, 3)

    def test_read_csv(self, tmp_path):
        p = str(tmp_path / "c.csv")
        with open(p, "w", encoding="utf-8") as f:
            f.write("0,0,1\n1,1,2\n")
        out = lg.read_points(p)
        assert out.shape == (2, 3)

    def test_missing_raises(self):
        with pytest.raises(lg.UsageError):
            lg.read_points("/nonexistent/cloud.npy")


class TestGeoTiff:
    def test_write(self, tmp_path):
        grid = np.random.rand(8, 8).astype(np.float32)
        p = str(tmp_path / "d.tif")
        lg.write_geotiff(p, grid, [116.0, 39.0, 117.0, 40.0])
        assert os.path.exists(p)
