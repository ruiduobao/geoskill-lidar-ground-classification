---
name: geoskill-lidar-ground-classification
description: '渐进形态学滤波PMF/坡度滤波分离地面点并生成DTM'
---

# LiDAR 地面点分类 | LiDAR Ground Point Classification

Separates a LiDAR point cloud (N×3 xyz) into ground points (ASPRS class 2) and non-ground points (class 1), then interpolates a bare-earth Digital Terrain Model (DTM). Two filtering methods are implemented:

- **PMF** (Progressive Morphological Filter, a simplified version of Zhang et al. 2003): rasterizes the point cloud into a minimum-elevation surface, then applies morphological opening with progressively larger windows; protruding pixels whose elevation difference exceeds an adaptive threshold (dh = dh0 + slope × window width) are trimmed to yield the ground surface;
- **slope** (slope filtering): estimates the terrain from a large-window minimum-elevation surface, then suppresses steeply varying areas based on local slope anomalies.

Per-point classification rule: a point is classified as ground when the difference between its elevation and the gridded ground surface is ≤ z_tolerance. Synthetic mode generates a simulated point cloud of smooth terrain + buildings (rectangular boxes) + trees (Gaussian crowns), enabling offline validation of classification accuracy. Suited to DEM/DTM production, terrain analysis, and point-cloud preprocessing pipelines.

## Dependencies / 依赖

```bash
pip install numpy rasterio scipy
```

## Usage / 使用方法

### Basic usage (synthetic data, offline)

```bash
python geoskill-lidar-ground-classification.py --bbox 116.0 39.0 117.0 40.0 --synthetic --output-dir ./out
```

### Example 1: PMF with default parameters

```bash
python geoskill-lidar-ground-classification.py \
    --bbox 116.0 39.0 117.0 40.0 \
    --synthetic --method pmf --cell-size 1.0 \
    --output-dir ./pmf
```

### Example 2: slope filtering with a relaxed tolerance

```bash
python geoskill-lidar-ground-classification.py \
    --bbox 121.0 31.0 122.0 32.0 \
    --synthetic --method slope --z-tolerance 0.8 \
    --output-dir ./slope
```

### Example 3: real point cloud input (.npy / .csv / .txt xyz)

```bash
python geoskill-lidar-ground-classification.py \
    --input tile_001.npy --method pmf --cell-size 2.0 \
    --output-dir ./real_pmf
```

### Example 4: coarse-resolution quick preview

```bash
python geoskill-lidar-ground-classification.py \
    --input tile_001.csv --cell-size 5.0 --quiet \
    --output-dir ./coarse
```

## Output / 输出

| File | Format | Description |
|---|---|---|
| `classified_points.npy` | NumPy (N×4) | xyz + class code (2=ground, 1=non-ground) |
| `dtm.tif` | GeoTIFF (float32) | DTM interpolated from ground points, EPSG:4326 |
| `density.tif` | GeoTIFF (float32) | Point density map (points per grid cell) |
| `stats.json` | JSON | Ground point ratio, accuracy statistics (synthetic mode includes comparison against ground truth) |
| `output-manifest.json` | JSON | Run manifest (input/output/QA/software versions) |

## Data Source / 数据源 / Source

- **Real mode**: local point cloud files (.npy / .csv / .txt, at least 3 columns of xyz, projected or local metric coordinates recommended)
- **Synthetic mode**: locally generated point cloud of smooth terrain + buildings + trees; no external data source

## Privacy / 隐私声明 / Privacy

- Runs fully offline; accesses no network services
- All processing is done locally; no user data is uploaded

## License / License

MIT

---

<!-- ===== 中文原文 (Chinese Original) ===== -->

---
name: geoskill-lidar-ground-classification
description: '渐进形态学滤波PMF/坡度滤波分离地面点并生成DTM'
---

# LiDAR 地面点分类 | LiDAR Ground Point Classification

把 LiDAR 点云（N×3 xyz）分离为地面点（ASPRS class 2）与非地面点
（class 1），并内插生成裸地数字地形模型 DTM。实现两种滤波方法：

- **PMF**（Progressive Morphological Filter，Zhang et al. 2003 简化版）：
  把点云栅格化为最低高程面，逐级增大窗口做形态学开运算，高程差超过
  自适应阈值（dh = dh0 + slope × 窗宽）的突起像元被削平，得到地面面；
- **slope**（坡度滤波）：大窗口最低值面估计地形，再按局部坡度异常
  抑制陡变区域。

逐点分类规则：点高程与格网地面面之差 ≤ z_tolerance → 地面点。合成
模式生成平滑地形 + 建筑（长方体）+ 树木（高斯冠）的模拟点云，可离线
验证分类精度。适用于 DEM/DTM 生产、地形分析、点云预处理流水线。

## 依赖

```bash
pip install numpy rasterio scipy
```

## 使用方法

### 基本用法（合成数据，离线）

```bash
python geoskill-lidar-ground-classification.py --bbox 116.0 39.0 117.0 40.0 --synthetic --output-dir ./out
```

### 示例 1：PMF 默认参数

```bash
python geoskill-lidar-ground-classification.py \
    --bbox 116.0 39.0 117.0 40.0 \
    --synthetic --method pmf --cell-size 1.0 \
    --output-dir ./pmf
```

### 示例 2：坡度滤波 + 放宽容差

```bash
python geoskill-lidar-ground-classification.py \
    --bbox 121.0 31.0 122.0 32.0 \
    --synthetic --method slope --z-tolerance 0.8 \
    --output-dir ./slope
```

### 示例 3：真实点云输入（.npy / .csv / .txt xyz）

```bash
python geoskill-lidar-ground-classification.py \
    --input tile_001.npy --method pmf --cell-size 2.0 \
    --output-dir ./real_pmf
```

### 示例 4：粗分辨率快速预览

```bash
python geoskill-lidar-ground-classification.py \
    --input tile_001.csv --cell-size 5.0 --quiet \
    --output-dir ./coarse
```

## 输出

| 文件 | 格式 | 说明 |
|---|---|---|
| `classified_points.npy` | NumPy (N×4) | xyz + 分类码（2=地面，1=非地面） |
| `dtm.tif` | GeoTIFF (float32) | 地面点插值 DTM，EPSG:4326 |
| `density.tif` | GeoTIFF (float32) | 点密度图（点/格网） |
| `stats.json` | JSON | 地面点比例、精度统计（合成模式含真值对比） |
| `output-manifest.json` | JSON | 运行清单（输入/输出/QA/软件版本） |

## 数据源 / Source

- **真实模式**：本地点云文件（.npy / .csv / .txt，至少 3 列 xyz，
  建议投影或局部米制坐标）
- **合成模式**：本地生成平滑地形 + 建筑 + 树木点云，无外部数据源

## 隐私声明 / Privacy

- 完全离线运行，不访问任何网络服务
- 所有处理在本地完成，不上传任何用户数据

## License

MIT
