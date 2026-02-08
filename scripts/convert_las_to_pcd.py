#!/usr/bin/env python3
"""
Convert a LAS/LAZ point cloud file to PCD (PointXYZI) format for the SUPER simulator.

Features:
  - Reads LAS via laspy
  - Optionally downsamples using a voxel grid
  - Centers the cloud at origin (or a user-specified offset)
  - Writes ASCII PCD with XYZI fields

Usage:
  python3 convert_las_to_pcd.py <input.las> <output.pcd> [--voxel 0.2] [--center] [--stats]
"""

import argparse
import sys
import numpy as np

def read_las(path):
    """Read a LAS file and return Nx3 xyz array and Nx1 intensity array."""
    try:
        import laspy
    except ImportError:
        print("ERROR: laspy not installed. Run: pip3 install laspy")
        sys.exit(1)

    print(f"Reading LAS file: {path}")
    las = laspy.read(path)

    x = np.array(las.x, dtype=np.float64)
    y = np.array(las.y, dtype=np.float64)
    z = np.array(las.z, dtype=np.float64)

    # Try to get intensity, default to zeros
    try:
        intensity = np.array(las.intensity, dtype=np.float32)
        # Normalize intensity to [0, 1] range if it's in uint16 range
        if intensity.max() > 1.0:
            intensity = intensity / 65535.0
    except Exception:
        intensity = np.zeros(len(x), dtype=np.float32)

    xyz = np.column_stack([x, y, z])
    print(f"  Raw points: {len(xyz):,}")
    print(f"  X range: [{x.min():.3f}, {x.max():.3f}] (span: {x.max()-x.min():.3f}m)")
    print(f"  Y range: [{y.min():.3f}, {y.max():.3f}] (span: {y.max()-y.min():.3f}m)")
    print(f"  Z range: [{z.min():.3f}, {z.max():.3f}] (span: {z.max()-z.min():.3f}m)")
    return xyz, intensity


def voxel_downsample(xyz, intensity, voxel_size):
    """Simple voxel grid downsample: keep one point per voxel (centroid)."""
    print(f"  Downsampling with voxel size {voxel_size}m ...")
    # Quantize to voxel indices
    voxel_idx = np.floor(xyz / voxel_size).astype(np.int32)

    # Use a dictionary to collect points per voxel
    # For speed, hash the voxel indices
    keys = voxel_idx[:, 0].astype(np.int64) * 1000000000 + \
           voxel_idx[:, 1].astype(np.int64) * 1000000 + \
           voxel_idx[:, 2].astype(np.int64)

    _, unique_indices = np.unique(keys, return_index=True)
    xyz_ds = xyz[unique_indices]
    int_ds = intensity[unique_indices]
    print(f"  After downsample: {len(xyz_ds):,} points")
    return xyz_ds, int_ds


def write_pcd(path, xyz, intensity):
    """Write an ASCII PCD file with XYZI fields."""
    n = len(xyz)
    print(f"Writing PCD file: {path} ({n:,} points)")
    with open(path, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {n}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n}\n")
        f.write("DATA ascii\n")
        for i in range(n):
            f.write(f"{xyz[i,0]:.4f} {xyz[i,1]:.4f} {xyz[i,2]:.4f} {intensity[i]:.4f}\n")
    print(f"  Done! File size: {os.path.getsize(path) / 1024 / 1024:.1f} MB")


def main():
    parser = argparse.ArgumentParser(description="Convert LAS to PCD for SUPER simulator")
    parser.add_argument("input", help="Input LAS file path")
    parser.add_argument("output", help="Output PCD file path")
    parser.add_argument("--voxel", type=float, default=0.15,
                        help="Voxel downsample size in meters (default: 0.15)")
    parser.add_argument("--center", action="store_true", default=True,
                        help="Center point cloud at origin XY, min-Z at 0 (default: True)")
    parser.add_argument("--no-center", action="store_true",
                        help="Do NOT center the point cloud")
    parser.add_argument("--stats", action="store_true",
                        help="Only print stats, don't convert")
    args = parser.parse_args()

    import os
    global os  # for write_pcd

    xyz, intensity = read_las(args.input)

    if args.stats:
        return

    # Downsample
    if args.voxel > 0:
        xyz, intensity = voxel_downsample(xyz, intensity, args.voxel)

    # Center at origin
    if not args.no_center:
        center_xy = (xyz[:, :2].max(axis=0) + xyz[:, :2].min(axis=0)) / 2.0
        min_z = xyz[:, 2].min()
        xyz[:, 0] -= center_xy[0]
        xyz[:, 1] -= center_xy[1]
        xyz[:, 2] -= min_z  # floor at Z=0
        print(f"  Centered: XY offset=[{center_xy[0]:.3f}, {center_xy[1]:.3f}], Z floor offset={min_z:.3f}")

    print(f"  Final X range: [{xyz[:,0].min():.3f}, {xyz[:,0].max():.3f}]")
    print(f"  Final Y range: [{xyz[:,1].min():.3f}, {xyz[:,1].max():.3f}]")
    print(f"  Final Z range: [{xyz[:,2].min():.3f}, {xyz[:,2].max():.3f}]")

    write_pcd(args.output, xyz, intensity)

    # Print suggested config values
    x_span = xyz[:, 0].max() - xyz[:, 0].min()
    y_span = xyz[:, 1].max() - xyz[:, 1].min()
    z_span = xyz[:, 2].max() - xyz[:, 2].min()
    print(f"\n=== Suggested SUPER config values ===")
    print(f"  Stope dimensions: {x_span:.1f} x {y_span:.1f} x {z_span:.1f} m")
    print(f"  map_size: [ {x_span+10:.0f}, {y_span+10:.0f}, {z_span+10:.0f} ]")
    print(f"  fix_map_origin: [ 0, 0, {z_span/2:.1f} ]")
    print(f"  init_position: x=0, y=0, z={min(3.0, z_span/2):.1f}")
    print(f"  virtual_ceil_height: {z_span+2:.0f}")
    print(f"  sensing_horizon: {max(30, min(x_span, y_span, z_span)/2):.0f}")


import os  # need at module level too

if __name__ == "__main__":
    main()
