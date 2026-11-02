# LiDAR Ground Point Classification (geoskill-lidar-ground-classification)

> Separate ground points using progressive morphological filtering (PMF) / slope filtering and generate a DTM

---

## 1. Overview

Separate a LiDAR point cloud (N×3 xyz) into ground points (ASPRS class 2) and non-ground points (class 1), and interpolate a bare-earth digital terrain model (DTM). Two filtering methods are implemented:

- **PMF** (Progressive Morphological Filter, simplified version of Zhang et al. 2003): rasterizes the point cloud into a minimum-elevation surface, then applies morphological opening with progressively larger windows; elevated pixels whose height difference exceeds an adaptive threshold (dh = dh0 + slope × window width) are flattened to yield the ground surface;
- **slope** (slope filtering): estimates the terrain from the minimum-elevation surface of a large window, then suppresses steep areas based on local slope anomalies.

Per-point classification rule: if the difference between a point's elevation and the gridded ground surface is ≤ z_tolerance, the point is classified as ground. Synthetic mode generates a simulated point cloud of smooth terrain + buildings (cuboids) + trees (Gaussian crowns), allowing offline validation of classification accuracy. Suitable for DEM/DTM production, terrain analysis, and point-cloud preprocessing pipelines.

## 3. Quick Start

```bash
pip install -r requirements.txt
python geoskill-lidar-ground-classification.py --bbox 116 39 117 40 --synthetic --output-dir ./out
```

## 4. CLI Parameters

Run `python <skill>.py --help` for the full list. Common parameters:

| Parameter | Type | Description |
|---|---|---|
| `--bbox` | `float[4]` | WGS84 bounding box `min_lon min_lat max_lon max_lat` |
| `--input` | `path` | Local input file (GeoJSON/GeoTIFF/etc.) |
| `--output-dir` | `path` | Output directory (default `./output`) |
| `--synthetic` | `flag` | Use synthetic data instead of real input |
| `--quiet` | `flag` | Suppress non-essential stdout |

## 5. Input / Output

| File | Format | Description |
|---|---|---|
| `classified_points.npy` | NumPy (N×4) | xyz + class code (2=ground, 1=non-ground) |
| `dtm.tif` | GeoTIFF (float32) | DTM interpolated from ground points, EPSG:4326 |
| `density.tif` | GeoTIFF (float32) | Point density map (points per grid cell) |
| `stats.json` | JSON | Ground point ratio, accuracy statistics (includes ground-truth comparison in synthetic mode) |
| `output-manifest.json` | JSON | Run manifest (inputs/outputs/QA/software versions) |


## 6. Technical Principle

(see SKILL.md for details)

## 8. License

MIT License. See [`LICENSE`](./LICENSE) for full text.

---

> 中文完整版：[references/SKILL.zh-CN.md](references/SKILL.zh-CN.md)
