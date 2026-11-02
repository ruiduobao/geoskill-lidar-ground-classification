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

> 中文完整版：需要时再读 [references/SKILL.zh-CN.md](references/SKILL.zh-CN.md)，不要默认全量加载。
