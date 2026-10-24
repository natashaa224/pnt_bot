import numpy as np
import math


import random
from dataclasses import dataclass

import cv2
import os
# from scipy.spatial.transform import Rotation as R

# import pandas as pd


debug = False
BUMP_MAX_DEPTH=0.8
SUCKIN_MAX_DEPTH=0.6          # shallow suck-in (compound two-arc) cap; deeper -> two corners
SUCKIN_DEPTH_MIN=0.15         # min range-jump past a terminal edge to call it a recess (not wall-end)
LIDAR_YAW_OFFSET = -np.pi
CORNER_ARC_START_DEG = 225.0
CORNER_ARC_END_DEG = 30.0

# ------------------ depth functions
def fit_plane_from_points(p):
    # p: Nx3
    centroid = p.mean(axis=0)
    Q = p - centroid
    _, _, vh = np.linalg.svd(Q, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    d = -normal.dot(centroid)
    return normal, d


def ransac_plane(points, valid, num_iter=100, threshold=0.025, max_points=30000):
    pts = points[valid].reshape(-1, 3)

    if len(pts) > max_points:
        idx = np.random.choice(len(pts), max_points, replace=False)
        pts_sample = pts[idx]
    else:
        pts_sample = pts

    best_count = 0
    best_normal = None
    best_d = None

    N = len(pts_sample)
    if N < 100:
        raise RuntimeError("Not enough valid depth points")

    for _ in range(num_iter):
        ids = np.random.choice(N, 3, replace=False)
        p3 = pts_sample[ids]

        normal = np.cross(p3[1] - p3[0], p3[2] - p3[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue

        normal = normal / norm
        d = -normal.dot(p3[0])

        dist = np.abs(pts_sample @ normal + d)
        count = np.sum(dist < threshold)

        if count > best_count:
            best_count = count
            best_normal = normal
            best_d = d

    # Refit using inliers from best model
    dist_all = np.abs(pts @ best_normal + best_d)
    inlier_pts = pts[dist_all < threshold]

    normal, d = fit_plane_from_points(inlier_pts)

    # ensure normal roughly points toward camera
    if normal[2] > 0:
        normal = -normal
        d = -d

    return normal, d, inlier_pts


# 2) Project pixel corners to wall plane
def pixel_ray(u, v, fx, fy, cx, cy):
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float32)
    return ray / np.linalg.norm(ray)

def intersect_ray_plane(ray, normal, d):
    denom = normal.dot(ray)
    if abs(denom) < 1e-6:
        return None
    return ray * (-d / denom)

def clean_endpoint_from_fit(seg_xy, seg_a, fit):
    inliers = fit["inliers"]

    # match each inlier back to original seg_xy index
    good_idx = []
    for p in inliers:
        idx = np.argmin(np.linalg.norm(seg_xy - p, axis=1))
        good_idx.append(idx)

    good_idx = np.array(sorted(set(good_idx)))

    # last clean inlier in angular order
    last = good_idx[-1]
    return seg_xy[last], seg_a[last]


""" Corner functions
"""

def extract_combined_arc(ranges, angles, angle_min_deg=225.0, angle_max_deg=30.0,max_range=3):
    angles_deg = np.degrees(angles) % 360.0
    mask = (angles_deg >= angle_min_deg) | (angles_deg <= angle_max_deg)
    valid = mask & np.isfinite(ranges) & (ranges > 0.15) & (ranges < max_range)

    arc_ranges = ranges[valid]
    arc_angles = angles_deg[valid]

    # shift angles so 225°→359° comes first, then 0°→30°
    # remap: angles >= 225 stay as-is, angles < 225 get +360
    shifted = np.where(arc_angles < angle_min_deg, arc_angles + 360.0, arc_angles)
    order = np.argsort(shifted)

    arc_ranges = arc_ranges[order]
    arc_angles = arc_angles[order]  # keep original degrees for readability

    x = arc_ranges * np.cos(np.radians(arc_angles))
    y = arc_ranges * np.sin(np.radians(arc_angles))
    arc_xy = np.column_stack([x, y])

    return arc_ranges, arc_angles, arc_xy


# ──  mad_filter() ─────────────────────────────────────────────────
def mad_filter(arc_ranges, arc_angles, arc_xy, threshold=2.5):
    """
    Median Absolute Deviation filter on arc ranges.
    Removes beams whose range deviates more than threshold * MAD from median.
    Returns filtered (arc_ranges, arc_angles, arc_xy).
    """
    median = np.median(arc_ranges)
    mad    = np.median(np.abs(arc_ranges - median))
    if mad < 1e-6:
        return arc_ranges, arc_angles, arc_xy   # flat surface, nothing to filter

    mask = np.abs(arc_ranges - median) <= threshold * mad
    return arc_ranges[mask], arc_angles[mask], arc_xy[mask]



def segment_points(arc_ranges, arc_angles, arc_xy,
                   threshold=0.15, min_points=5):
    if len(arc_ranges) < 2:
        return []

    segments = []
    seg_start = 0

    for i in range(1, len(arc_ranges)):
        angle_gap_rad = abs(np.radians(arc_angles[i] - arc_angles[i-1]))
        if angle_gap_rad > np.pi:
            angle_gap_rad = 2*np.pi - angle_gap_rad

        # expected Cartesian spacing on flat surface
        tolerance = arc_ranges[i-1] * abs(np.sin(angle_gap_rad / 2)) * 2

        # Cartesian distance between adjacent points
        cartesian = np.linalg.norm(arc_xy[i] - arc_xy[i-1])

        # direct range difference — catches depth jumps missed by Cartesian
        range_jump = abs(arc_ranges[i] - arc_ranges[i-1])

        # print(f"  i={i:3d}  a={arc_angles[i-1]:.1f}→{arc_angles[i]:.1f}  r={arc_ranges[i]:.3f}  cart={cartesian:.4f}  tol={tolerance:.4f}  rjump={range_jump:.4f}  split={'YES' if abs(cartesian-tolerance)>threshold else 'no'}")
        # split on either signal
        if abs(cartesian - tolerance) > threshold :#or range_jump > threshold:
            if (i - seg_start) >= min_points:
                segments.append((
                    arc_ranges[seg_start:i],
                    arc_angles[seg_start:i],
                    arc_xy[seg_start:i]
                ))
            seg_start = i

    if (len(arc_ranges) - seg_start) >= min_points:
        segments.append((
            arc_ranges[seg_start:],
            arc_angles[seg_start:],
            arc_xy[seg_start:]
        ))

    return segments

def find_orientation_change(seg_xy, window=5, min_angle_deg=20.0, max_angle_deg=130.0):
    """
    Walk along a segment's Cartesian points looking for a significant
    direction change. Uses a local window to compute stable bearing vectors
    before and after each candidate point.

    Returns list of dicts:
        {
          'index'       : int,        # index into seg_xy
          'point'       : np.array,   # (x, y) of junction
          'source_deg'  : float,      # incoming bearing (degrees)
          'target_deg'  : float,      # outgoing bearing (degrees)
          'radian_diff' : float,      # signed angle change (radians)
          'type'        : str,        # 'INTERNAL' or 'EXTERNAL'
        }
    """
    results = []
    n = len(seg_xy)
    if n < window * 2 + 1:
        return results

    for i in range(window, n - window):
        # stable incoming vector — average over window points before i
        vec_in  = seg_xy[i] - seg_xy[i - window]
        # stable outgoing vector — average over window points after i
        vec_out = seg_xy[i + window] - seg_xy[i]

        source = np.arctan2(vec_in[1],  vec_in[0])
        target = np.arctan2(vec_out[1], vec_out[0])

        # normalise both to [0, 2π)
        source = source % (2 * np.pi)
        target = target % (2 * np.pi)

        radian_diff = source - target

        # normalise diff to (-π, π]
        radian_diff = (radian_diff + np.pi) % (2 * np.pi) - np.pi

        if min_angle_deg <= abs(np.degrees(radian_diff)) <= max_angle_deg:    

            corner_type = 'EXTERNAL' if radian_diff > 0 else 'INTERNAL'
            results.append({
                'index'       : i,
                'point'       : seg_xy[i],
                'source_deg'  : np.degrees(source),
                'target_deg'  : np.degrees(target),
                'radian_diff' : radian_diff,
                'type'        : corner_type,
            })
    return results



def point_line_distance(points, p0, p1):
    """
    Perpendicular distance from points to infinite line p0->p1.
    points: (N, 2)
    """
    v = p1 - p0
    denom = np.linalg.norm(v)

    if denom < 1e-9:
        return np.linalg.norm(points - p0, axis=1)

    return np.abs(np.cross(v, points - p0)) / denom


def pca_direction_angle(xy):
    """
    PCA/TLS direction angle of a 2D point cloud.
    Returns angle in radians.
    """
    if len(xy) < 2:
        return 0.0

    c = xy.mean(axis=0)
    X = xy - c

    _, _, vh = np.linalg.svd(X, full_matrices=False)
    v = vh[0]

    return np.arctan2(v[1], v[0])


def abs_line_angle_diff_deg(a, b):
    """
    Absolute difference between two line directions.
    Line has 180° ambiguity, so 0° and 180° are same.
    """
    d = (a - b + np.pi) % (2 * np.pi) - np.pi
    d = abs(np.degrees(d))
    return min(d, 180.0 - d)


def split_one_segment_by_line_deviation(
    seg_r,
    seg_a,
    seg_xy,
    deviation_threshold=0.05,
    min_points=8,
    min_angle_change_deg=12.0,
    debug=False,
):
    """
    Splits one segment when it can no longer be represented by one straight line.

    Important:
    - Rejects fake splits if both sides are still nearly collinear.
    - Good for connected L-corners.
    - Avoids splitting noisy straight walls into two pieces.
    """

    n = len(seg_xy)

    if n < 2 * min_points:
        return [(seg_r, seg_a, seg_xy)]

    out = []
    start = 0
    i = start + min_points

    while i < n:
        pts = seg_xy[start:i + 1]

        p0 = pts[0]
        p1 = pts[-1]

        dists = point_line_distance(pts, p0, p1)
        max_dev_idx_local = int(np.argmax(dists))
        max_dev = float(dists[max_dev_idx_local])

        if max_dev > deviation_threshold:
            split_idx = start + max_dev_idx_local

            left_xy = seg_xy[start:split_idx]
            right_xy = seg_xy[split_idx:i + 1]

            left_n = len(left_xy)
            right_n = len(right_xy)

            if left_n >= min_points and right_n >= min_points:
                left_angle = pca_direction_angle(left_xy)
                right_angle = pca_direction_angle(right_xy)
                angle_change = abs_line_angle_diff_deg(left_angle, right_angle)

                if debug:
                    print(
                        f"SPLIT CANDIDATE: "
                        f"start={start}, i={i}, split_idx={split_idx}, "
                        f"angle={seg_a[split_idx]:.1f}, "
                        f"max_dev={max_dev:.4f}, "
                        f"angle_change={angle_change:.1f}, "
                        f"left_n={left_n}, right_n={right_n}"
                    )

                # Reject fake split on same straight wall
                if angle_change < min_angle_change_deg:
                    if debug:
                        print("  rejected: collinear pieces")
                    i += 1
                    continue

                # Accept split
                if debug:
                    print("  accepted")

                out.append((
                    seg_r[start:split_idx],
                    seg_a[start:split_idx],
                    seg_xy[start:split_idx],
                ))

                start = split_idx
                i = start + min_points
                continue

        i += 1

    # Add final remaining piece
    if (n - start) >= min_points:
        out.append((
            seg_r[start:n],
            seg_a[start:n],
            seg_xy[start:n],
        ))

    return out


def split_segments_by_line_deviation(
    segments,
    deviation_threshold=0.05,
    min_points=5,
    min_angle_change_deg=12.0,
    debug=False,
):
    """
    Input/output format same as segment_points():
        [(seg_ranges, seg_angles, seg_xy), ...]

    Usage:
        segments = segment_points(arc_r, arc_a, arc_xy)
        segments = split_segments_by_line_deviation(segments)
    """

    refined = []

    for si, (seg_r, seg_a, seg_xy) in enumerate(segments):
        if debug:
            print(f"\nChecking segment {si}: n={len(seg_xy)} "
                  f"angle={seg_a[0]:.1f}->{seg_a[-1]:.1f}")

        pieces = split_one_segment_by_line_deviation(
            seg_r,
            seg_a,
            seg_xy,
            deviation_threshold=deviation_threshold,
            min_points=min_points,
            min_angle_change_deg=min_angle_change_deg,
            debug=debug,
        )

        refined.extend(pieces)

    return refined


def debug_segment_geometry(segments, title="Segments"):
    print(f"\n{title}")
    for si, (sr, sa, sxy) in enumerate(segments):
        p0, p1 = sxy[0], sxy[-1]
        chord = np.linalg.norm(p1 - p0)

        if len(sxy) >= 3:
            dists = point_line_distance(sxy, p0, p1)
            max_i = int(np.argmax(dists))
            max_dev = float(dists[max_i])
        else:
            max_i = -1
            max_dev = 0.0

        print(
            f"Seg {si}: n={len(sxy):3d} "
            f"angle={sa[0]:6.1f}->{sa[-1]:6.1f} "
            f"range={sr.min():.3f}-{sr.max():.3f} "
            f"chord={chord:.3f} "
            f"max_dev={max_dev:.4f} at local_idx={max_i}"
        )
# ── Test + Plot ───────────────────────────────────────────────────────────

def ransac_line(points, n_iter=100, threshold=0.05, min_inliers=8):
    """
    RANSAC line fit on 2D point cloud.
    
    Returns:
        best_mask  : boolean array, True = inlier
        best_n     : normal vector (unit, pointing toward origin)
        best_d     : perpendicular distance from origin to line
        best_count : number of inliers
    Or None if no valid line found.
    """
    n_pts = len(points)
    if n_pts < 2:
        return None

    best_mask  = None
    best_count = 0
    best_n     = None
    best_d     = 0.0

    for _ in range(n_iter):
        # 1. Sample 2 random points
        idx = random.sample(range(n_pts), 2)
        p1, p2 = points[idx[0]], points[idx[1]]

        # 2. Compute line normal
        direction = p2 - p1
        seg_len   = np.linalg.norm(direction)
        if seg_len < 1e-6:
            continue

        n = np.array([-direction[1], direction[0]]) / seg_len
        d = float(np.dot(n, p1))

        # Keep d positive (temp convention for scoring only)
        if d < 0:
            n = -n
            d = -d

        # 3. Count inliers
        dists = np.abs(points @ n - d)
        mask  = dists < threshold
        count = int(mask.sum())

        if count > best_count:
            best_count = count
            best_mask  = mask.copy()
            best_n     = n.copy()
            best_d     = d

    if best_mask is None or best_count < min_inliers:
        return None

    return best_mask, best_n, best_d, best_count

def pca_fit(arc_points):
        """
        Fit a line to arc_points using PCA.
        Returns (normal, distance, centroid, eigenvalues) or None if too few points.
        """
        if len(arc_points) < 5:
            return None

        cx, cy = arc_points.mean(axis=0)
        centroid = np.array([cx, cy])

        # Center the points
        centered = arc_points - centroid

        # Covariance matrix
        cov = (centered.T @ centered) / len(arc_points)

        # Eigendecomposition — eigh returns sorted ascending
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # smallest eigenvector = normal direction (perpendicular to wall)
        normal = eigenvectors[:, 0]

        # Flip normal to point toward robot at origin
        # If normal points same way as centroid-from-origin, flip it
        if np.dot(normal, centroid) > 0:
            normal = -normal

        # Distance from origin to line
        distance = float(abs(np.dot(normal, centroid)))

        ratio = eigenvalues[1] / (eigenvalues[0] + 1e-9)

        return normal, distance, centroid, eigenvalues, ratio

def fit_segment(sxy, n_iter=100, threshold=0.05, min_inliers=8):
    """
    RANSAC + PCA fit on a segment's Cartesian points.
    Returns dict or None if fit fails.
    """
    if len(sxy) < min_inliers:
        return None

    ransac_result = ransac_line(sxy, n_iter=n_iter,
                                threshold=threshold,
                                min_inliers=min_inliers)
    if ransac_result is None:
        return None

    mask, _, _, _ = ransac_result
    inliers = sxy[mask]

    pca_result = pca_fit(inliers)



    if pca_result is None:
        return None

    normal, distance, centroid, eigenvalues, ratio = pca_result

    # print(f"  fit_segment: distance={distance:.3f}m  ratio={ratio:.1f}  centroid=({centroid[0]:.3f},{centroid[1]:.3f})")

    if distance > 3.0:
        # print(f"  fit_segment: REJECTED — distance {distance:.3f}m > 3.0m")
        return None
    
#    print(f"  fit_segment: ACCEPTED")
    
    if ratio < 5.0:   # not a clean wall — reject
        return None

    return {
        'normal'    : normal,      # unit normal pointing toward origin
        'distance'  : distance,    # perpendicular distance from origin
        'centroid'  : centroid,    # mean of inlier points
        'eigenvalues': eigenvalues,
        'ratio'     : ratio,
        'inliers'   : inliers,
        'n_inliers' : int(mask.sum()),
        'n_total'   : len(sxy),
    }


# fits = []
# for i, (_, _, sxy) in enumerate(segments):
#     f = fit_segment(sxy)
#     if f is not None:
#         print(f"Seg {i}: ratio={f['ratio']:.1f}  "
#               f"d={f['distance']:.3f}m  "
#               f"normal=[{f['normal'][0]:.3f}, {f['normal'][1]:.3f}]  "
#               f"inliers={f['n_inliers']}/{f['n_total']}")
#     else:
#         print(f"Seg {i}: rejected")
#     fits.append(f)

# def smart_fit_segments(segments, inlier_ratio_threshold=0.80,
#                        window=25, min_angle_deg=25.0, max_angle_deg=130.0,
#                        min_points=15):
#     """
#     For each segment:
#       1. Quick RANSAC+PCA fit
#       2. If inlier_ratio < threshold → try kink split → refit each half
#       3. Returns list of (fit, sxy) tuples — only valid fits kept
#     """
#     valid_fits = []   # list of (fit_result, sxy)

#     for seg_idx, (_, _, sxy) in enumerate(segments):

#         if len(sxy) < min_points:
#             print(f"Seg {seg_idx}: skipped — too few points ({len(sxy)})")
#             continue

#         f = fit_segment(sxy)

#         if f is None:
#             print(f"Seg {seg_idx}: fit failed")
#             continue

#         inlier_ratio = f['n_inliers'] / f['n_total']

#         if inlier_ratio >= inlier_ratio_threshold:
#             # clean segment — keep as-is
#             print(f"Seg {seg_idx}: clean  "
#                   f"ratio={f['ratio']:.1f}  "
#                   f"inliers={f['n_inliers']}/{f['n_total']} ({inlier_ratio:.0%})  "
#                   f"d={f['distance']:.3f}m")
#             valid_fits.append((f, sxy))

#         else:
#             # poor fit — attempt kink split
#             print(f"Seg {seg_idx}: poor inlier ratio ({inlier_ratio:.0%}) "
#                   f"— attempting kink split")

#             kinks = find_orientation_change(sxy, window=window,
#                                             min_angle_deg=min_angle_deg,
#                                             max_angle_deg=max_angle_deg)

#             if not kinks:
#                 print(f"  → no kink found — discarding segment")
#                 continue

#             # use strongest kink (largest |radian_diff|)
#             best_kink = max(kinks, key=lambda k: abs(k['radian_diff']))
#             k_idx = best_kink['index']

#             print(f"  → kink at idx={k_idx}  "
#                   f"type={best_kink['type']}  "
#                   f"angle={np.degrees(best_kink['radian_diff']):.1f}°  "
#                   f"pt=({best_kink['point'][0]:.3f}, {best_kink['point'][1]:.3f})")

#             half_a = sxy[:k_idx]
#             half_b = sxy[k_idx:]

#             for label, half in [('A', half_a), ('B', half_b)]:
#                 if len(half) < min_points:
#                     print(f"  → half {label}: too few points ({len(half)}) — skipped")
#                     continue

#                 hf = fit_segment(half)
#                 if hf is None:
#                     print(f"  → half {label}: fit failed")
#                     continue

#                 print(f"  → half {label}: ratio={hf['ratio']:.1f}  "
#                       f"inliers={hf['n_inliers']}/{hf['n_total']}  "
#                       f"d={hf['distance']:.3f}m  "
#                       f"normal=[{hf['normal'][0]:.3f}, {hf['normal'][1]:.3f}]")
#                 valid_fits.append((hf, half))

#     return valid_fits

def ransac_fit_segments(segments, inlier_ratio_threshold=0.80,
                       window=25, min_angle_deg=25.0, max_angle_deg=130.0,
                       min_points=15):
    """
    For each segment:
      1. Quick RANSAC+PCA fit
      2. If inlier_ratio < threshold → try kink split → refit each half
      3. Stores clean fitted endpoint for terminal edge detection
      4. Returns list of (fit, sxy) tuples
    """
    valid_fits = []

    for seg_idx, (_, seg_a, sxy) in enumerate(segments):

        if len(sxy) < min_points:
            if debug:
             print(f"Seg {seg_idx}: skipped — too few points ({len(sxy)})")
            continue

        f = fit_segment(sxy, min_inliers=min_points)

        if f is None:
            # print(f"Seg {seg_idx}: fit failed")
            continue

        inlier_ratio = f['n_inliers'] / f['n_total']

        if inlier_ratio >= inlier_ratio_threshold:
            edge_pt, edge_angle = clean_endpoint_from_fit(sxy, seg_a, f)

            f["edge_point"] = edge_pt
            f["edge_angle"] = edge_angle
            f["seg_idx"] = seg_idx
            
            if debug:
                print(f"Seg {seg_idx}: clean  "
                    f"ratio={f['ratio']:.1f}  "
                    f"inliers={f['n_inliers']}/{f['n_total']} ({inlier_ratio:.0%})  "
                    f"d={f['distance']:.3f}m  "
                    f"edge=({edge_pt[0]:.3f},{edge_pt[1]:.3f}) @ {edge_angle:.1f}°")

            valid_fits.append((f, sxy))

        else:
            if debug:

                print(f"Seg {seg_idx}: poor inlier ratio ({inlier_ratio:.0%}) "
                    f"— attempting kink split")

            kinks = find_orientation_change(
                sxy,
                window=window,
                min_angle_deg=min_angle_deg,
                max_angle_deg=max_angle_deg
            )

            if not kinks:
                print("  → no kink found — discarding segment")
                continue

            best_kink = max(kinks, key=lambda k: abs(k['radian_diff']))
            k_idx = best_kink['index']

            # print(f"  → kink at idx={k_idx}  "
            #       f"type={best_kink['type']}  "
            #       f"angle={np.degrees(best_kink['radian_diff']):.1f}°  "
            #       f"pt=({best_kink['point'][0]:.3f}, {best_kink['point'][1]:.3f})")

            halves = [
                ("A", sxy[:k_idx], seg_a[:k_idx]),
                ("B", sxy[k_idx:], seg_a[k_idx:])
            ]

            for label, half_xy, half_a in halves:
                if len(half_xy) < min_points:
                    # print(f"  → half {label}: too few points ({len(half_xy)}) — skipped")
                    continue

                hf = fit_segment(half_xy, min_inliers=min_points)

                if hf is None:
                    # print(f"  → half {label}: fit failed")
                    continue

                edge_pt, edge_angle = clean_endpoint_from_fit(half_xy, half_a, hf)

                hf["edge_point"] = edge_pt
                hf["edge_angle"] = edge_angle
                hf["seg_idx"] = seg_idx

                # print(f"  → half {label}: ratio={hf['ratio']:.1f}  "
                #       f"inliers={hf['n_inliers']}/{hf['n_total']}  "
                #       f"d={hf['distance']:.3f}m  "
                #       f"normal=[{hf['normal'][0]:.3f}, {hf['normal'][1]:.3f}]  "
                #       f"edge=({edge_pt[0]:.3f},{edge_pt[1]:.3f}) @ {edge_angle:.1f}°")

                valid_fits.append((hf, half_xy))

    return valid_fits

# def merge_collinear(valid_fits, angle_tol_deg=8.0, dist_tol=0.04,
#                     gap_tol=0.15):
#     """
#     Merge adjacent valid_fits whose fitted walls are collinear.
#     Collinear criteria (all must pass):
#       - angle between normals < angle_tol_deg
#       - distance difference < dist_tol (m)
#       - gap between segment endpoints < gap_tol (m)

#     After merging, refit the combined point cloud with RANSAC+PCA.
#     Returns list of (fit, sxy) — same format as smart_fit_segments().
#     """
#     if not valid_fits:
#         return []

#     # work on a copy so we can mutate
#     groups = [[valid_fits[0]]]

#     for i in range(1, len(valid_fits)):
#         fit_prev, sxy_prev = groups[-1][-1]
#         fit_curr, sxy_curr = valid_fits[i]

#         # ── angle check ───────────────────────────────────────────────────
#         dot         = abs(np.dot(fit_prev['normal'], fit_curr['normal']))
#         dot         = min(dot, 1.0)   # clamp for arccos
#         angle_diff  = np.degrees(np.arccos(dot))

#         # ── distance check ────────────────────────────────────────────────
#         dist_diff   = abs(fit_prev['distance'] - fit_curr['distance'])

#         # ── endpoint gap check ────────────────────────────────────────────
#         gap         = np.linalg.norm(sxy_prev[-1] - sxy_curr[0])

#         angle_ok    = angle_diff  < angle_tol_deg
#         dist_ok     = dist_diff   < dist_tol
#         gap_ok      = gap         < gap_tol

#         if angle_ok and dist_ok and gap_ok:
#             # print(f"  merging fit {i-1}→{i}  "
#             #       f"angle_diff={angle_diff:.1f}°  "
#             #       f"dist_diff={dist_diff*100:.1f}cm  "
#             #       f"gap={gap*100:.1f}cm")
#             groups[-1].append(valid_fits[i])
#         else:
#             reason = []
#             if not angle_ok: reason.append(f"angle={angle_diff:.1f}°")
#             if not dist_ok:  reason.append(f"dist={dist_diff*100:.1f}cm")
#             if not gap_ok:   reason.append(f"gap={gap*100:.1f}cm")
#             # print(f"  keeping fit {i} separate — {', '.join(reason)}")
#             groups.append([valid_fits[i]])

#     # ── refit each group ──────────────────────────────────────────────────
#     merged_fits = []
#     for g_idx, group in enumerate(groups):
#         if len(group) == 1:
#             # no merge needed
#             merged_fits.append(group[0])
#             f, sxy = group[0]
#             # print(f"Wall {g_idx}: single segment  "
#             #       f"pts={len(sxy)}  d={f['distance']:.3f}m  "
#             #       f"normal=[{f['normal'][0]:.3f}, {f['normal'][1]:.3f}]")
#         else:
#             # merge point clouds and refit
#             combined_sxy = np.vstack([sxy for _, sxy in group])
#             hf = fit_segment(combined_sxy)
#             if hf is None:
#                 print(f"Wall {g_idx}: merged refit failed — discarding")
#                 continue
#             merged_fits.append((hf, combined_sxy))
#             # print(f"Wall {g_idx}: merged {len(group)} segments  "
#             #       f"pts={len(combined_sxy)}  "
#             #       f"d={hf['distance']:.3f}m  "
#             #       f"normal=[{hf['normal'][0]:.3f}, {hf['normal'][1]:.3f}]  "
#             #       f"ratio={hf['ratio']:.1f}")

#     return merged_fits

def merge_collinear(valid_fits, angle_tol_deg=8.0, dist_tol=0.04,
                    gap_tol=0.15):
    """
    Merge adjacent valid_fits whose fitted walls are collinear.
    Collinear criteria (all must pass):
      - angle between normals < angle_tol_deg
      - distance difference < dist_tol (m)
      - gap between NEAREST segment endpoints < gap_tol (m)

    After merging, refit the combined cloud and re-stamp endpoint/segment
    metadata (edge_point, edge_angle, seg_idx, seg_idx_end). A failed merged
    refit keeps the group's original fits rather than discarding them.
    Returns list of (fit, sxy) — same format as ransac_fit_segments().
    """
    if not valid_fits:
        return []

    groups = [[valid_fits[0]]]

    for i in range(1, len(valid_fits)):
        fit_prev, sxy_prev = groups[-1][-1]
        fit_curr, sxy_curr = valid_fits[i]

        dot        = min(abs(np.dot(fit_prev['normal'], fit_curr['normal'])), 1.0)
        angle_diff = np.degrees(np.arccos(dot))
        dist_diff  = abs(fit_prev['distance'] - fit_curr['distance'])

        # nearest-endpoint gap (don't assume prev[-1] abuts curr[0] in scan order)
        ep_prev = np.array([sxy_prev[0], sxy_prev[-1]])
        ep_curr = np.array([sxy_curr[0], sxy_curr[-1]])
        gap = min(np.linalg.norm(ep_prev[a] - ep_curr[b])
                  for a in (0, 1) for b in (0, 1))

        if (angle_diff < angle_tol_deg and dist_diff < dist_tol and gap < gap_tol):
            groups[-1].append(valid_fits[i])
        else:
            groups.append([valid_fits[i]])

    merged_fits = []
    for g_idx, group in enumerate(groups):
        if len(group) == 1:
            # unmerged — add seg_idx_end for a consistent downstream interface
            f, sxy = group[0]
            f.setdefault("seg_idx_end", f.get("seg_idx", g_idx))
            merged_fits.append((f, sxy))
            continue

        combined_sxy = np.vstack([sxy for _, sxy in group])
        hf = fit_segment(combined_sxy)
        if hf is None:
            # keep originals rather than drop the whole group
            print(f"Wall {g_idx}: merged refit failed — keeping {len(group)} originals")
            for f, sxy in group:
                f.setdefault("seg_idx_end", f.get("seg_idx", g_idx))
                merged_fits.append((f, sxy))
            continue

        # re-stamp metadata the refit dropped
        terminal_pt = group[-1][1][-1]                      # last pt, scan order
        inl = np.asarray(hf["inliers"], dtype=float)
        j   = int(np.argmin(np.linalg.norm(inl - terminal_pt, axis=1)))
        edge_pt = inl[j]

        seg_ids = [f.get("seg_idx", g_idx) for f, _ in group]
        hf["edge_point"]  = edge_pt
        hf["edge_angle"]  = np.degrees(np.arctan2(edge_pt[1], edge_pt[0])) % 360.0
        hf["seg_idx"]     = min(seg_ids)
        hf["seg_idx_end"] = max(seg_ids)

        merged_fits.append((hf, combined_sxy))

    return merged_fits

def oriented_wall_dir(fit, sxy):
    # direction perpendicular to normal
    d = np.array([-fit['normal'][1], fit['normal'][0]], dtype=float)
    d /= np.linalg.norm(d) + 1e-9

    # force direction to follow scan/segment order
    scan_dir = sxy[-1] - sxy[0]
    if np.dot(d, scan_dir) < 0:
        d = -d

    return d


def line_intersection(fit_a, fit_b):
    # line equation: n.x = signed_d
    da = float(np.dot(fit_a['normal'], fit_a['centroid']))
    db = float(np.dot(fit_b['normal'], fit_b['centroid']))

    A = np.array([fit_a['normal'], fit_b['normal']])
    b = np.array([da, db])

    if abs(np.linalg.det(A)) < 1e-6:
        return None

    return np.linalg.solve(A, b)


def classify_segment_pair(fit_a, sxy_a, fit_b, sxy_b,
                          min_corner_deg=20.0,
                          max_corner_deg=160.0):
    if fit_a is None or fit_b is None:
        return None

    dir_a = oriented_wall_dir(fit_a, sxy_a)
    dir_b = oriented_wall_dir(fit_b, sxy_b)

    cross = float(dir_a[0] * dir_b[1] - dir_a[1] * dir_b[0])
    dot   = float(np.dot(dir_a, dir_b))

    radian_diff = np.arctan2(cross, dot)
    angle_deg = abs(np.degrees(radian_diff))

    if not (min_corner_deg <= angle_deg <= max_corner_deg):
        return None

    vertex = line_intersection(fit_a, fit_b)
    if vertex is None:
        return None

    corner_type = 'INTERNAL' if radian_diff > 0 else 'EXTERNAL'

    return {
        'type': corner_type,
        'radian_diff': radian_diff,
        'angle_deg': angle_deg,
        'vertex': vertex,
        'cross': cross,
        'dot': dot,
        'dir_a': dir_a,
        'dir_b': dir_b,
    }


def detect_best_corner(fits):
    best_result = None
    best_score = -1

    for i in range(len(fits) - 1):
        fit_a, sxy_a = fits[i]
        fit_b, sxy_b = fits[i + 1]

        # skip nearly same wall
        normal_dot = abs(float(np.dot(fit_a['normal'], fit_b['normal'])))
        if normal_dot > 0.90:
            # print(f"Pair {i}->{i+1}: skipped same wall dot={normal_dot:.2f}")
            continue

        r = classify_segment_pair(fit_a, sxy_a, fit_b, sxy_b)
        if r is None:
            # print(f"Pair {i}->{i+1}: invalid")
            continue

        # score: prefer strong fits and nearer visible corner
        score = fit_a['n_inliers'] + fit_b['n_inliers']

        r['seg_a'] = i
        r['seg_b'] = i + 1
        r['score'] = score

        if debug:
            print(
                f"Pair {i}->{i+1}: {r['type']} "
                f"{r['angle_deg']:.1f}° "
                f"vertex=({r['vertex'][0]:.3f}, {r['vertex'][1]:.3f}) "
                f"score={score}"
            )

        if score > best_score:
            best_score = score
            best_result = r

    return best_result

def shift_angle_deg(a_deg, arc_start_deg):
    """
    Convert wrapped angle into continuous angle starting at arc_start_deg.
    Example for arc_start=180:
      180..359 stay same, 0..30 become 360..390
    """
    a = a_deg % 360.0
    return a + 360.0 if a < arc_start_deg else a


def in_wrapped_arc(a_deg, arc_start_deg, arc_end_deg):
    """
    True if angle lies in wrapped arc arc_start -> arc_end.
    Example: 180 -> 30 means [180..360] U [0..30]
    """
    a = a_deg % 360.0
    if arc_start_deg <= arc_end_deg:
        return arc_start_deg <= a <= arc_end_deg
    else:
        return a >= arc_start_deg or a <= arc_end_deg

def _beam_continuity(inliers, scan, N=None):
    inc = float(scan["angle_increment"])
    if N is None:
        N = int(round(2 * np.pi / inc))          # ~720 for the C1
    inl = np.asarray(inliers, dtype=float)
    if len(inl) < 2:
        return {"beam_fill": 1.0, "max_gap": 1, "beam_span": len(inl)}

    # beam index from each inlier's OWN bearing (no XY rematch, no cross-assign)
    ang = np.arctan2(inl[:, 1], inl[:, 0])
    bi  = np.sort(np.unique(np.round(ang / inc).astype(int) % N))
    n   = len(bi)
    if n < 2:
        return {"beam_fill": 1.0, "max_gap": 1, "beam_span": 1}

    # circular gaps around the ring, including the 0/360 seam
    diffs = np.diff(bi)
    wrap  = (bi[0] + N) - bi[-1]
    gaps  = np.append(diffs, wrap)

    largest       = int(gaps.max())              # the one empty arc = outside the wall
    occupied_span = N - largest + 1              # beams spanned by the wall (inclusive)
    within        = np.sort(gaps)[:-1]           # drop the outside arc
    max_hole      = int(within.max()) if len(within) else 1

    fill = min(n / occupied_span, 1.0)
    return {"beam_fill": round(fill, 3), "max_gap": max_hole, "beam_span": occupied_span}

def fit_metrics(fit, sxy, scan=None):
    """Geometry + (optional) beam-continuity metrics for one RANSAC+PCA fit."""
    sxy = np.asarray(sxy, dtype=float)
    normal   = np.asarray(fit["normal"],   dtype=float)
    centroid = np.asarray(fit["centroid"], dtype=float)
    wall_dir = np.array([-normal[1], normal[0]])
    inl = np.asarray(fit.get("inliers", sxy), dtype=float)
    t = (inl - centroid) @ wall_dir
    chord = float(t.max()-t.min()) if len(t) else 0.0
    perp = (inl - centroid) @ normal
    rms = float(np.sqrt(np.mean(perp**2))) if len(perp) else np.nan
    n_in = int(fit.get("n_inliers", len(inl)))
    m = {"n_in": n_in, "chord_m": chord, "rms_m": rms,
         "pts_per_m": n_in/chord if chord > 1e-6 else np.nan,
         "dist_m": float(fit.get("distance", np.nan)),
         "ratio": float(fit.get("ratio", 0.0))}
    if scan is not None:
        m.update(_beam_continuity(inl, scan))
    else:
        m.update({"beam_fill": None, "max_gap": None, "beam_span": None})
    return m

@dataclass
class WallGateCfg:
    # main wall leg (A/C) - strict wall-IDENTITY bar for corner-pair classification
    # (classify_segment_pair). It is the LENGTH floor that says "this fit is a real
    # wall whose direction we trust to declare a corner", not just "a wall exists
    # here" - edge/feature detectors run on the looser 'kept' set instead.
    #
    # Derived as 1 strip width (strip_width_m) so it scales with the paint step
    # instead of a magic 0.50: 0.203 m at a 0.203 m strip. We do NOT want to accept
    # walls shorter than one strip (noise / fragments). Set main_min_chord_m
    # explicitly (not None) to override the derivation.
    strip_width_m:    float = 0.203
    main_min_chord_m: float = 0.15     #  
    min_pts_per_m:    float = 50.0
    min_inliers:      int   = 20
    max_rms_m:        float = 0.010
    min_eigen_ratio:  float = 50.0     # NEW

    # loose continuity reject (kills bridged RANSAC-through-clutter everywhere)
    reject_fill_below: float = 0.15
    reject_gap_above:  int   = 50
    # bump connector B (relaxed chord, still dense/straight)
    conn_min_chord_m: float = 0.15
    conn_max_chord_m: float = 0.65
    # emerging sparse leg (C) rescue — ppm waived, continuity STRICT
    sparse_min_chord_m: float = 0.10
    sparse_min_inliers: int   = 5
    sparse_max_gap:     int   = 2
    sparse_min_fill:    float = 0.90


    def __post_init__(self):
        if self.main_min_chord_m is None:
            self.main_min_chord_m = self.strip_width_m


def wall_gate(m, cfg, tier="main"):
    """
    tier='main'   : strict dense wall (A/C ordinary corners)
    Continuity loose-reject applies to ALL tiers (fragmented fit = not a surface).
    Returns (passed, reason).
    """
    # universal continuity reject (only when continuity was computed)
    if m.get("max_gap") is not None:
        if m["beam_fill"] < cfg.reject_fill_below: return False, f"fill {m['beam_fill']}<{cfg.reject_fill_below}"
        if m["max_gap"]   > cfg.reject_gap_above:  return False, f"gap {m['max_gap']}>{cfg.reject_gap_above}"
    if tier == "main":
        if m["chord_m"]   < cfg.main_min_chord_m:  return False, f"chord {m['chord_m']:.2f}<{cfg.main_min_chord_m}"
        if m["pts_per_m"] < cfg.min_pts_per_m:     return False, f"ppm {m['pts_per_m']:.0f}<{cfg.min_pts_per_m}"
        if m["n_in"]      < cfg.min_inliers:       return False, f"n_in {m['n_in']}<{cfg.min_inliers}"
        if m["rms_m"]     > cfg.max_rms_m:         return False, f"rms {m['rms_m']:.3f}>{cfg.max_rms_m}"
        if m["ratio"] < cfg.min_eigen_ratio: return False, f"ratio {m['ratio']:.0f}<{cfg.min_eigen_ratio}"
        return True, "ok"
    return False, "unknown tier"


def sparse_leg_ok(m, cfg):
    return (
        m["chord_m"] >= cfg.sparse_min_chord_m
        and m["n_in"] >= cfg.sparse_min_inliers
        and m["rms_m"] <= cfg.max_rms_m
        and m["ratio"] >= cfg.min_eigen_ratio
        and m.get("max_gap") is not None
        and m["max_gap"] <= cfg.sparse_max_gap
        and m["beam_fill"] >= cfg.sparse_min_fill
    )

def _fit_of(item):
    return (item[0], item[1]) if isinstance(item, tuple) else (item, item.get("inliers"))

def _pipeline_fits(scan, min_points, yaw_offset=LIDAR_YAW_OFFSET):
    """arc -> segment -> split -> far-filter -> ransac fit -> merge.
    Returns (fit items, arc_xy, ranges, angles_full)."""
    ranges = np.asarray(scan["ranges"], dtype=np.float32)
    amin = float(scan["angle_min"]); ainc = float(scan["angle_increment"])
    af = (amin + np.arange(len(ranges))*ainc + yaw_offset) % (2*np.pi)
    ar, aa, axy = extract_combined_arc(ranges, af,
                    angle_min_deg=CORNER_ARC_START_DEG, angle_max_deg=CORNER_ARC_END_DEG)
    segs = segment_points(ar, aa, axy)
    segs = split_segments_by_line_deviation(segs, deviation_threshold=0.05,
                    min_points=min_points, min_angle_change_deg=12.0, debug=False)
    segs = [s for s in segs if np.median(s[0]) < 3.0]
    fits = ransac_fit_segments(segs, min_points=min_points)
    fits = merge_collinear(fits)          # fuse collinear fragments into their wall
    return fits, axy, ranges, af

def detect_corners(scan, cfg, yaw_offset=LIDAR_YAW_OFFSET, debug=False):
    """STRICT. Returns (corners[], fits[], dbg). corners sweep-sorted, each with
    fit_before/fit_after indices into the returned (gated) fits list."""
    raw_fits, arc_xy, ranges, angles_full = _pipeline_fits(
        scan, min_points=5, yaw_offset=yaw_offset)

    # STRICT wall gate (runs on merged walls)
    gated = []
    for item in raw_fits:
        fit, sxy = _fit_of(item)
        m = fit_metrics(fit, sxy, scan)
        ok, reason = wall_gate(m, cfg, tier="main")
        if ok:
            gated.append(item)
        elif debug:
            print(f"  GATED OUT: chord={m['chord_m']:.2f} ppm={m['pts_per_m']:.0f} "
                  f"n={m['n_in']} rms={m['rms_m']:.3f} -> {reason}")

    terminal_edges = detect_terminal_edges_from_fits(
        gated, ranges_raw=ranges, angles_full_rad=angles_full,
        max_range_interest=2.0, lookahead_beams=12, min_far_ratio=0.40, debug=False)

    corners = []
    for a, b in zip(range(len(gated)-1), range(1, len(gated))):
        fa, sa = _fit_of(gated[a]); fb, sb = _fit_of(gated[b])
        res = classify_segment_pair(fa, sa, fb, sb)
        if res is None:
            continue
        p = np.asarray(res["vertex"], float)
        sweep = (np.degrees(np.arctan2(float(p[1]), float(p[0]))) % 360 - 270.0) % 360
        corners.append({"type": res["type"], "vertex": p, "point": p,
                        "radian_diff": res["radian_diff"], "angle_deg": res["angle_deg"],
                        "turn_dir": "CCW" if res["radian_diff"] > 0 else "CW",
                        "sweep_angle": sweep, "fit_before": a, "fit_after": b,
                        "source": "line_intersection"})
    corners.sort(key=lambda c: c["sweep_angle"])

    dbg = {"arc_xy": arc_xy, "gated_fits": gated, "terminal_edges": terminal_edges}
    if debug:
        print(f"  {len(gated)} gated fits, {len(corners)} corners, "
              f"{len(terminal_edges)} terminal edges")
        for c in corners:
            print(f"    {c['type']:9s} sweep={c['sweep_angle']:6.1f} "
                  f"fits({c['fit_before']}->{c['fit_after']}) @({c['point'][0]:.2f},{c['point'][1]:.2f})")
    return corners, gated, dbg

def detect_bumpout(scan, cfg, yaw_offset=LIDAR_YAW_OFFSET, debug=False):
    """
    Bump-out = strict A -> A->B INTERNAL -> valid short connector B ->
    B->C EXTERNAL (C rescued by STRICT continuity + topology) -> right turn sense.
    Returns (event, info): BUMPOUT dict, else first strict corner, else None.
    """
    raw_fits, _,_,_ = _pipeline_fits(scan, min_points=5, yaw_offset=yaw_offset)
    # keep fits that are EITHER strict walls OR continuity-rescued sparse legs
    kept = []
    for item in raw_fits:
        fit, sxy = _fit_of(item); m = fit_metrics(fit, sxy, scan)
        if wall_gate(m, cfg, "main")[0] or sparse_leg_ok(m, cfg):
            kept.append(item)
    kept.sort(key=lambda it: _fit_of(it)[0].get("seg_idx", 0))

    # classify all adjacent pairs on this augmented set
    rc = []
    for a, b in zip(range(len(kept)-1), range(1, len(kept))):
        fa, sa = _fit_of(kept[a]); fb, sb = _fit_of(kept[b])
        res = classify_segment_pair(fa, sa, fb, sb)
        if res is None: continue
        rc.append({"type": res["type"], "vertex": np.asarray(res["vertex"], float),
                   "radian_diff": res["radian_diff"], "fit_before": a, "fit_after": b})

    # find A->B INTERNAL then B->C EXTERNAL sharing connector B
    for k in range(len(rc)-1):
        c0, c1 = rc[k], rc[k+1]
        if c0["type"] != "INTERNAL" or c1["type"] != "EXTERNAL": continue
        if c0["fit_after"] != c1["fit_before"]: continue          # shared fit B
        Afit, Asxy = _fit_of(kept[c0["fit_before"]])              # A must be strict
        if not wall_gate(fit_metrics(Afit, Asxy, scan), cfg, "main")[0]: continue
        Bfit, Bsxy = _fit_of(kept[c0["fit_after"]])               # B valid connector
        Bm = fit_metrics(Bfit, Bsxy, scan)
        if not (cfg.conn_min_chord_m <= Bm["chord_m"] <= cfg.conn_max_chord_m): continue
        V1, V2 = c0["vertex"], c1["vertex"]
        depth = float(np.linalg.norm(V1 - V2))
        turn_ok = (c0["radian_diff"] > 0) and (c1["radian_diff"] < 0)  # CCW then CW
        if turn_ok :
            sweep = (np.degrees(np.arctan2(float(V1[1]), float(V1[0]))) % 360 - 270.0) % 360
            ev = {"type": "BUMPOUT", "internal_vertex": V1, "external_vertex": V2,
                  "connector_chord": Bm["chord_m"], "depth": depth,
                  "point": V1, "sweep_angle": sweep, "source": "bumpout_pair"}
            if debug: print(f"  BUMPOUT Bchord={Bm['chord_m']:.2f} depth={depth:.2f} "
                            f"V1={V1.round(2)} V2={V2.round(2)}")
            return ev, {"reason": "bumpout"}

    corners, _, _ = detect_corners(scan, cfg, yaw_offset)
    if debug: print(f"  no bumpout (types={[c['type'] for c in rc]}); "
                    f"strict corners={[c['type'] for c in corners]}")
    return (corners[0] if corners else None), {"reason": "no bumpout"}





def detect_terminal_edges_from_fits(
    fits,
    ranges_raw,
    angles_full_rad,
    max_range_interest=2.0,
    lookahead_beams=12,
    min_far_ratio=0.50,
    depth_min=SUCKIN_DEPTH_MIN,   # min range-jump past edge to flag a recess (suck-in cue)
    recess_min_far_pts=5,         # min finite far beams to attempt the recess wall fit
    recess_max_rms=0.03,          # straightness gate for the recessed wall C
    parallel_tol_deg=10.0,        # A||C tolerance for a valid recess (suck-in)
    approach_max_range=2.00,      # A edge sanity cap (m); the latY gate is the real A filter
    recess_max_range=3.00,        # recessed wall C sanity cap (m); rejects across-room background
    debug=True,
    arc_end_deg=CORNER_ARC_END_DEG,   # true forward arc limit (30deg), not 355
    arc_end_margin_deg=5.0,           # tightened: only skip edges AT the real boundary
):
    angles_deg = np.degrees(angles_full_rad) % 360.0
    terminal_edges = []

    for fit_idx, (fit, sxy) in enumerate(fits):
        edge_angle = fit["edge_angle"]
        edge_pt = fit["edge_point"]

        # Skip only edges sitting AT the true forward arc boundary (CORNER_ARC_END_DEG),
        # where the lookahead beams would run off the usable scan. An edge well inside
        # the forward arc (e.g. a protrusion face ending at ~355deg with open space
        # beyond) has valid lookahead and must NOT be skipped - that was clipping real
        # external edges near 355deg.
        if abs(((edge_angle - arc_end_deg + 180) % 360) - 180) < arc_end_margin_deg:
            if debug:
                print(f"  Terminal check fit {fit_idx}: skipped — edge near arc boundary ({edge_angle:.1f}°)")
            continue

        idx = np.argmin(np.abs(((angles_deg - edge_angle + 180) % 360) - 180))

        ahead_idx = (idx + np.arange(1, lookahead_beams + 1)) % len(ranges_raw)
        ahead_ranges = ranges_raw[ahead_idx]

        # original 'wall ends' test: beams go far/infinite past the edge.
        far = (~np.isfinite(ahead_ranges)) | (ahead_ranges > max_range_interest)
        far_ratio = np.count_nonzero(far) / len(ahead_ranges)

        # RECESS (suck-in) test — two checks, earlier than the full EXTERNAL->INTERNAL pair:
        #   (a) discontinuity: finite beams past the edge JUMP >= depth_min beyond
        #       the edge range (near wall A ended, something recessed sits behind).
        #   (b) those far beams form a STRAIGHT wall C (SVD line fit, rms gate) —
        #       same straightness bar as a normal wall, so a doorway/clutter (no
        #       clean far wall) does NOT read as a recess.
        #   (c) C is roughly PARALLEL to approach wall A.
        # depth is the PERPENDICULAR separation of A's line and C's line — NOT the
        # raw range jump (range jump reads the far wall at an angle and badly
        # under-estimates a real recess, e.g. 0.35 for a true 0.70, which would send
        # it down the wrong maneuver branch). The raw jump is kept only as a cue.
        edge_range   = float(np.hypot(edge_pt[0], edge_pt[1]))
        finite_mask  = np.isfinite(ahead_ranges)
        recess_depth = None            # geometric A<->C perpendicular separation
        recess_jump  = None            # raw range-jump cue (diagnostic only)
        recess_C     = None
        # Gate the APPROACH wall A first: a recess is only valid off the NEAR wall
        # the robot is following, not off a steep return/drop-off wall. Two checks:
        #   - A's normal is mostly LATERAL (|ny| > |nx|): the followed standoff wall,
        #     not a wall diving away (which is a return wall, e.g. an external
        #     corner's own far face). This is what killed the scan-19 false recess.
        #   - A's edge is near standoff range (<= approach_max_range): we only read a
        #     recess off a wall we are actually tracking, not a distant one.
        nA_raw = np.asarray(fit.get("normal"), float)
        A_ok = (nA_raw.size == 2 and np.isfinite(nA_raw).all()
                and abs(nA_raw[1]) > abs(nA_raw[0])            # mostly lateral -> followed wall
                and edge_range <= approach_max_range)
        if A_ok and np.count_nonzero(finite_mask) >= max(recess_min_far_pts, 3):
            far_r   = ahead_ranges[finite_mask]
            far_ang = angles_full_rad[ahead_idx][finite_mask]
            jump = float(np.median(far_r)) - edge_range
            if jump >= depth_min:                          # (a) discontinuity
                fx = far_r * np.cos(far_ang); fy = far_r * np.sin(far_ang)
                P  = np.column_stack([fx, fy]); cC = P.mean(axis=0)
                _, _, vt = np.linalg.svd(P - cC)
                d_hat = vt[0]; nC = np.array([-d_hat[1], d_hat[0]])
                rms = float(np.sqrt(np.mean(((P - cC) @ nC) ** 2)))
                cC_range = float(np.hypot(cC[0], cC[1]))
                # (b) far rays = straight wall C, AND C within a sane range window
                # (standoff + max recess depth + margin). A C way out (e.g. 4 m
                # across a room) is background, not a feature to paint around.
                if rms <= recess_max_rms and cC_range <= recess_max_range:
                    nA = nA_raw
                    cA = np.asarray(fit.get("centroid"), float)
                    ok_A = np.linalg.norm(nA) > 1e-6
                    # (c) A || C  (normals may point opposite ways)
                    parallel = ok_A and (abs(float(np.dot(nA/np.linalg.norm(nA),
                                                          nC))) >= math.cos(math.radians(parallel_tol_deg)))
                    if parallel:
                        nAu = nA / np.linalg.norm(nA)
                        depth_geom = abs(float(np.dot(cC - cA, nAu)))   # perp A<->C
                        if depth_geom >= depth_min:
                            recess_depth = depth_geom
                            recess_jump  = jump
                            recess_C = {"centroid": cC, "normal": nC, "rms": rms,
                                        "n": int(far_r.size)}

        if debug:
            print(
                f"Terminal check fit {fit_idx}: "
                f"edge_angle={edge_angle:.1f}, "
                f"edge_pt=({edge_pt[0]:.3f},{edge_pt[1]:.3f}), "
                f"edge_range={edge_range:.2f}, far_ratio={far_ratio:.2f}, "
                f"recess_depth={recess_depth}, "
                f"ahead_ranges={np.round(ahead_ranges, 2)}"
            )

        if far_ratio >= min_far_ratio or recess_depth is not None:
            terminal_edges.append({
                "type": "EXTERNAL_EDGE",
                "fit_idx": fit_idx,
                "seg_idx": fit.get("seg_idx", None),
                "angle": edge_angle,
                "point": edge_pt,
                "far_ratio": far_ratio,
                # recess_depth None -> plain wall-end / dropout (external edge).
                # recess_depth set -> provisional SUCK-IN cue; recess_C holds the
                # fitted recessed wall C (centroid, normal, rms).
                "recess_depth": recess_depth,     # geometric A<->C perp separation (provisional d)
                "recess_jump": recess_jump,       # raw range-jump cue (diagnostic)
                "recess_C": recess_C,
            })

    return terminal_edges#  ─────────────────────────────────────────────────────────────────────────
#  detect_motion_events  — SINGLE-PASS superset detector
#  Append to geometry_utils.py (after detect_bumpout). One _pipeline_fits run,
#  classify every adjacent pair once, recognise ALL event types on that one
#  classified list, return them priority-ordered. The node picks events[0].
#
#  Event types (priority high→low): BUMPOUT > SUCKIN > INTERNAL > EXTERNAL.
#  Within a type, nearest-first by sweep angle (sweep measured CCW from right/270°,
#  so smaller sweep = closer to the right wall = encountered first).
#
#  Every event is a dict with at least:
#     type, vertex (np.array xy), point (=vertex, back-comat), sweep_angle, source
#  plus type-specific fields:
#     INTERNAL/EXTERNAL : radian_diff, angle_deg, turn_dir, fit_before, fit_after
#     BUMPOUT           : internal_vertex, external_vertex, connector_chord, depth
#  ─────────────────────────────────────────────────────────────────────────

# priority rank — lower number = higher priority
_EVENT_PRIORITY = {"BUMPOUT": 0, "SUCKIN": 1, "INTERNAL": 2, "EXTERNAL": 3,
                   "EXTERNAL_EDGE": 4}

def _sweep_of(p):
    """CCW sweep from the right side (270°). Smaller = nearer the right wall."""
    return (np.degrees(np.arctan2(float(p[1]), float(p[0]))) % 360 - 270.0) % 360

# INTERNAL corners are only actionable while AHEAD of the robot. Once turned past,
# the vertex goes behind (x < 0) and must NOT re-latch. Validated on real frames:
# legit internal latches at vertex x ≈ +1.0 and stays positive through approach;
# a just-turned corner reads x ≈ -0.27. Gate: vertex.x > this.
INTERNAL_VERTEX_AHEAD_MIN = 0.10

def _orient_toward_origin(n, centroid):
    """Return the wall normal flipped so it points toward the lidar origin — makes
    the convex/concave cross-product test independent of ransac's normal sign."""
    n = np.asarray(n, float)
    return n if np.dot(n, -np.asarray(centroid)) > 0 else -n

def detect_external_corner(scan, cfg, yaw_offset=LIDAR_YAW_OFFSET,
                           a_min_chord=0.30, c_min_chord=0.15, c_min_pts=8,
                           max_rms=0.03, perp_lo=70.0, perp_hi=110.0,
                           axis_thresh=0.85, a_behind_x=0.05, a_max_range=1.37,
                           debug=False):
    """Live A⊥C external-corner confirm, ~0.1 m PAST the vertex where the strict gate
    rejects both walls. Three physical gates (validated on real sim frames):
      (1) A⊥C perpendicular — A (approach wall, normal ~±y, chord>=0.30 relaxed) ⊥
          C (return wall, normal ~±x, connector floor chord>=0.15, n>=8), 70-110°.
          Needs BOTH walls → a doorway (edge, no C) returns None.
      (2) convex turn-sense — cross(dirA,dirC) < 0 (CW), normals canonicalised toward
          the lidar → rejects a CONCAVE (internal) perpendicular pair.
      (3) approach wall behind/abeam — A centroid x <= a_behind_x → the robot has
          passed the wall's end (is AT the corner) → rejects the pre-vertex false pair.
    Returns an EXTERNAL event dict (angle/vertex/turn) or None.
    """
    raw_fits, _, _, _ = _pipeline_fits(scan, min_points=5, yaw_offset=yaw_offset)
    info = [((it[0] if isinstance(it, tuple) else it),
             fit_metrics(*_fit_of(it), scan)) for it in raw_fits]

    # A candidate = lateral wall (normal ~ +/-y) that is BOTH:
    #   - behind/abeam the robot (centroid x <= a_behind_x): the far room wall
    #     ahead (cx ~ +0.85) is a lateral wall too and otherwise wins on chord,
    #     so we must reject it HERE, not pick-then-reject.
    #   - within reach (centroid range <= target_dist + max_suckin ~ 1.37 m):
    #     a deep recessed far wall can never be the approach wall A.
    def _rng(t):
        cx, cy = t[0]['centroid'][0], t[0]['centroid'][1]
        return (cx * cx + cy * cy) ** 0.5
    A = [t for t in info if abs(t[0]['normal'][1]) > axis_thresh
         and t[1]['chord_m'] >= a_min_chord and t[1]['rms_m'] <= max_rms
         and t[0]['centroid'][0] <= a_behind_x and _rng(t) <= a_max_range]
    C = [t for t in info if abs(t[0]['normal'][0]) > axis_thresh
         and t[1]['chord_m'] >= c_min_chord and t[0]['n_inliers'] >= c_min_pts
         and t[1]['rms_m'] <= max_rms]
    if not A or not C:
        return None
    # among the behind/abeam laterals, the real approach wall is the nearest
    # (smallest range) one -- a leftover far segment that still slips the x gate
    # loses on range; chord ties break toward the longer wall.
    a = min(A, key=lambda t: (_rng(t), -t[1]['chord_m']))
    c = max(C, key=lambda t: t[1]['chord_m'])
    na = _orient_toward_origin(a[0]['normal'], a[0]['centroid'])
    nc = _orient_toward_origin(c[0]['normal'], c[0]['centroid'])
    ang = np.degrees(np.arccos(max(-1.0, min(1.0, abs(na[0]*nc[0] + na[1]*nc[1])))))
    if not (perp_lo <= ang <= perp_hi):           # (1) perpendicular
        return None
    dA = np.array([-na[1], na[0]]); dC = np.array([-nc[1], nc[0]])
    if dA[0]*dC[1] - dA[1]*dC[0] >= 0:            # (2) convex only (CW); concave → reject
        return None

    vx = float(c[0]['centroid'][0]); vy = float(a[0]['centroid'][1])
    return {"type": "EXTERNAL", "vertex": np.array([vx, vy]), "point": np.array([vx, vy]),
            "angle_deg": 90.0, "measured_angle_deg": float(ang),
            "radian_diff": -np.pi/2, "turn_dir": "CW", "sweep_angle": _sweep_of([vx, vy]),
            "A_chord": a[1]['chord_m'], "C_chord": c[1]['chord_m'],
            "C_n": int(c[0]['n_inliers']), "source": "ext_AC_confirm"}


def detect_gapped_internal_from_fits(gated, scan, cfg,
                                     ang_tol_deg=15.0,
                                     gap_max=0.28, g_direct_max=0.35,
                                     a_axis_thresh=0.85, b_axis_thresh=0.85,
                                     a_min_chord=0.40, b_min_chord=0.30,
                                     max_rms=0.015,
                                     followed_wall_dist=None,
                                     followed_wall_tol=0.30, debug=False):
    """Gapped internal corner: the followed right wall A and the front wall B ahead
    whose INFINITE lines form a ~90deg internal corner, with the finite supports
    terminating near the virtual intersection V (within gap_max). The masonry
    between A and B may be MISSING (a protrusion separated by a gap) - that is the
    case the adjacent-pair detectors miss, because A and B are not adjacent fits.

    Consumes a fit set detect_motion_events already built (the 'kept' set = gated +
    rescued, since a short protrusion face often fails strict main-wall gating), so
    no second _pipeline_fits. Depth-agnostic (a protrusion of any height is still just
    an internal corner); V is the vertex. Returns an INTERNAL-shaped dict or None.

    A/B are chosen by a pair SCORE tied to robot context (A = followed right wall,
    near followed_wall_dist and on the right; B = front wall ahead), NOT simply the
    longest fits - a room with several long walls must not manufacture a pair.
    Emit a standard INTERNAL so env/FSM need no new state; source='gapped_internal'.
    """
    def _line(f):
        n = np.asarray(f['normal'], float)
        nn = math.hypot(n[0], n[1])
        n = n / nn if nn > 1e-9 else n
        c = np.asarray(f['centroid'], float)
        return c, np.array([-n[1], n[0]]), n          # c, along-dir, normal

    def _intersect(cA, nA, cB, nB):
        M = np.array([nA, nB]); rhs = np.array([nA @ cA, nB @ cB])
        det = M[0, 0]*M[1, 1] - M[0, 1]*M[1, 0]
        if abs(det) < 1e-9:
            return None
        return np.linalg.solve(M, rhs)

    def _ends(f, sxy):
        c, d, n = _line(f)
        P = np.asarray(sxy, float); t = (P - c) @ d
        return c + t.min()*d, c + t.max()*d, c, d, n

    cand = [(_fit_of(it), fit_metrics(*_fit_of(it), scan)) for it in gated]
    A = [(fs, m) for (fs, m) in cand
         if abs(fs[0]['normal'][1]) > a_axis_thresh
         and m['chord_m'] >= a_min_chord and m['rms_m'] <= max_rms
         and fs[0]['centroid'][1] < 0.0]                     # on the robot's right
    B = [(fs, m) for (fs, m) in cand
         if abs(fs[0]['normal'][0]) > b_axis_thresh
         and m['chord_m'] >= b_min_chord and m['rms_m'] <= max_rms]
    if not A or not B:
        return None

    best = None
    for (fA, sA), mA in A:
        cA, dA, nA = _line(fA)
        a_standoff = abs(float(nA @ cA))                     # perp lidar->A line
        if followed_wall_dist is not None and \
           abs(a_standoff - followed_wall_dist) > followed_wall_tol:
            continue
        eA0, eA1, _, _, _ = _ends(fA, sA)
        for (fB, sB), mB in B:
            cB, dB, nB = _line(fB)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, abs(nA @ nB)))))
            if abs(ang - 90.0) >= ang_tol_deg:               # (1) perpendicular
                continue
            V = _intersect(cA, nA, cB, nB)
            if V is None:
                continue
            # (2) topology: V ahead & on the right; B is the FRONT wall (ahead)
            if not (V[0] > INTERNAL_VERTEX_AHEAD_MIN and V[1] < 0.2):
                continue
            if cB[0] <= 0.0:
                continue
            # (3) gap: finite supports terminate near V.
            eB0, eB1, _, _, _ = _ends(fB, sB)
            gA = min(np.linalg.norm(eA0 - V), np.linalg.norm(eA1 - V))
            gB = min(np.linalg.norm(eB0 - V), np.linalg.norm(eB1 - V))
            g_direct = min(np.linalg.norm(x - y)
                           for x in (eA0, eA1) for y in (eB0, eB1))
            # perpendicular B->A line distance = the real missing masonry; also the
            # secondary topology guard against a long skew B being extrapolated.
            perp_B_to_A = float(np.min(np.abs((np.asarray(sB, float) - cA) @ nA)))
            # A is the FOLLOWED wall and may run PAST V. If A spans V, endpoint
            # metrics on A are meaningless - gate on gB + masonry perp. If A also
            # stops short (both truncated, meeting at V), use the full gA/gB/g_direct.
            tV = float((V - cA) @ dA)
            te = sorted([float((eA0 - cA) @ dA), float((eA1 - cA) @ dA)])
            a_spans_V = te[0] - 1e-6 <= tV <= te[1] + 1e-6
            if a_spans_V:
                gap_ok = (gB <= gap_max and perp_B_to_A <= gap_max)
                gap_report = perp_B_to_A
            else:
                gap_ok = (gA <= gap_max and gB <= gap_max
                          and g_direct <= g_direct_max and perp_B_to_A <= gap_max)
                gap_report = g_direct
            if debug:
                print('    A_so=%.2f ang=%.1f V=(%.2f,%.2f) gA=%.2f span=%s gB=%.2f '
                      'g_dir=%.2f perpBA=%.2f ok=%s'
                      % (a_standoff, ang, V[0], V[1], gA, a_spans_V, gB,
                         g_direct, perp_B_to_A, gap_ok))
            if not gap_ok:
                continue
            gA_eff = 0.0 if a_spans_V else gA
            score = gB + gA_eff + 0.5*g_direct - 0.3*mB['chord_m']
            if best is None or score < best[0]:
                best = (score, V, ang, mA['chord_m'], mB['chord_m'],
                        gap_report, perp_B_to_A)

    if best is None:
        return None
    _, V, ang, aC, bC, gap_report, perpBA = best
    return {"type": "INTERNAL", "vertex": np.asarray(V, float),
            "point": np.asarray(V, float),
            "angle_deg": 90.0, "measured_angle_deg": float(ang),
            "radian_diff": np.pi/2, "turn_dir": "CCW",
            "sweep_angle": _sweep_of(V),
            "A_chord": aC, "B_chord": bC,
            "gap": float(gap_report), "perp_B_to_A": float(perpBA),
            "source": "gapped_internal"}


def detect_motion_events(scan, cfg, yaw_offset=LIDAR_YAW_OFFSET, debug=False):
    """
    ONE pipeline pass → ALL motion events, priority-ordered.

    Returns (events, dbg):
      events : list of event dicts, sorted (priority, then sweep_angle).
               events[0] is what the node should act on. Empty if nothing.
      dbg    : {arc_xy, gated_fits, kept_fits, terminal_edges, pairs}

    Supersedes detect_corners / detect_bumpout for the node. (Those remain for
    notebook/unit use.) suckIn is stubbed — hook is here, logic deferred.
    """
    # ── one pipeline run ──────────────────────────────────────────────────
    raw_fits, arc_xy, ranges, angles_full = _pipeline_fits(
        scan, min_points=5, yaw_offset=yaw_offset)

    # metrics once per fit (with scan → beam continuity active everywhere)
    metr = [fit_metrics(_fit_of(it)[0], _fit_of(it)[1], scan) for it in raw_fits]

    # STRICT walls (A/C ordinary) and the continuity-rescued sparse legs (emerging C)
    strict = [wall_gate(m, cfg, "main")[0] for m in metr]
    rescue = [sparse_leg_ok(m, cfg)        for m in metr]
    gated  = [it for it, s in zip(raw_fits, strict) if s]            # strict-only (corners)
    kept   = [it for it, (s, r) in zip(raw_fits, zip(strict, rescue)) if (s or r)]  # +rescue (bump)
    kept.sort(key=lambda it: _fit_of(it)[0].get("seg_idx", 0))

    events = []

    # -- terminal edges (external edge dropouts) on the strict GATED set ----
    # Runs on gated (strict walls). kept was tried and gives NO benefit here: the
    # edge detection limit is physical (~0.30 m vertex-front, where the followed
    # wall has passed the lidar and left the forward view), not the chord gate, so
    # kept detects no edge closer than gated does - it only adds far-edge artifacts.
    # The real margin for acting on the edge belongs in the approach/trigger
    # distance (commit the turn while the vertex is still ~0.45-0.5 out), not in
    # widening the fit set. gated | terminal-edge + ordinary corner geometry;
    # kept | bump / sparse-leg topology only.
    terminal_edges = detect_terminal_edges_from_fits(
        gated, ranges_raw=ranges, angles_full_rad=angles_full,
        max_range_interest=2.0, lookahead_beams=12, min_far_ratio=0.40, debug=False)

    # ── classify every adjacent pair on the KEPT set (superset) ───────────
    pairs = []
    for a, b in zip(range(len(kept)-1), range(1, len(kept))):
        fa, sa = _fit_of(kept[a]); fb, sb = _fit_of(kept[b])
        res = classify_segment_pair(fa, sa, fb, sb)
        if res is None:
            continue
        pairs.append({"idx_a": a, "idx_b": b,
                      "type": res["type"], "vertex": np.asarray(res["vertex"], float),
                      "radian_diff": res["radian_diff"], "angle_deg": res["angle_deg"]})

    # ── 1. BUMPOUT: adjacent INTERNAL→EXTERNAL sharing connector B ─────────
    bump_consumed = set()          # pair indices folded into a bump (don't re-emit as corners)
    for k in range(len(pairs)-1):
        c0, c1 = pairs[k], pairs[k+1]
        if c0["type"] != "INTERNAL" or c1["type"] != "EXTERNAL":
            continue
        if c0["idx_b"] != c1["idx_a"]:                     # must share fit B
            continue
        # A must be a strict wall
        Afit, Asxy = _fit_of(kept[c0["idx_a"]])
        if not wall_gate(fit_metrics(Afit, Asxy, scan), cfg, "main")[0]:
            continue
        # B a valid short connector
        Bfit, Bsxy = _fit_of(kept[c0["idx_b"]])
        Bm = fit_metrics(Bfit, Bsxy, scan)
        if not (cfg.conn_min_chord_m <= Bm["chord_m"] <= cfg.conn_max_chord_m):
            continue
        V1, V2 = c0["vertex"], c1["vertex"]
        depth  = float(np.linalg.norm(V1 - V2))
        turn_ok = (c0["radian_diff"] > 0) and (c1["radian_diff"] < 0)   # CCW then CW
        if turn_ok and depth <= BUMP_MAX_DEPTH:
            events.append({"type": "BUMPOUT", "vertex": V1, "point": V1,
                           "internal_vertex": V1, "external_vertex": V2,
                           "connector_chord": Bm["chord_m"], "depth": depth,
                           "sweep_angle": _sweep_of(V1), "source": "bumpout_pair"})
            bump_consumed.add(k); bump_consumed.add(k+1)

    # ── 1b. EDGE-PAIRED BUMPOUT: INTERNAL corner + terminal EXTERNAL_EDGE ──────
    # The deep-bump case (depth > BUMP_MAX_DEPTH, wall C the protrusion returns to
    # is NOT yet visible): wall A (baseline) meets the protrusion FACE wall B at the
    # internal vertex V1, B runs out, and B ENDS at a terminal external edge (open
    # space beyond) -> V2. There is no external CORNER (no second wall), so the
    # corner-pair bump above cannot catch it; we pair the internal corner with the
    # terminal edge that is the FAR END OF THE SAME WALL B (shared gated fit index
    # idx_b == terminal_edge.fit_idx). depth = |V1 - V2|, UNCAPPED here (env2 owns
    # the max-depth policy). Only runs when no corner-paired bump already covers it.
    #   topology: internal corner (gated a,b) ; edge on gated fit b -> bump V1->edge.
    gi_internal = []
    for ia, ib in zip(range(len(gated)-1), range(1, len(gated))):
        gfa, gsa = _fit_of(gated[ia]); gfb, gsb = _fit_of(gated[ib])
        gres = classify_segment_pair(gfa, gsa, gfb, gsb)
        if gres is None or gres["type"] != "INTERNAL":
            continue
        gv = np.asarray(gres["vertex"], float)
        if gv[0] <= INTERNAL_VERTEX_AHEAD_MIN:        # must be ahead (not turned-past)
            continue
        gi_internal.append({"a": ia, "b": ib, "vertex": gv,
                            "radian_diff": gres["radian_diff"]})
    for ic in gi_internal:
        # terminal edge that is the far end of THIS internal corner's wall B (fit ib)
        te_match = next((te for te in terminal_edges
                         if te.get("fit_idx") == ic["b"]
                         and te.get("recess_depth") is None), None)  # plain edge, not a recess
        if te_match is None:
            continue
        V1 = ic["vertex"]
        V2 = np.asarray(te_match["point"], float)
        depth = float(np.linalg.norm(V1 - V2))
        # don't double-emit if a corner-paired bump already covers this V1
        if any(e["type"] == "BUMPOUT" and
               np.linalg.norm(V1 - np.asarray(e["internal_vertex"], float)) < 0.10
               for e in events):
            continue
        events.append({"type": "BUMPOUT", "vertex": V1, "point": V1,
                       "internal_vertex": V1, "external_vertex": V2,
                       "connector_chord": None, "depth": depth,
                       "sweep_angle": _sweep_of(V1), "source": "bumpout_edge_paired"})

    # ── 2. SUCKIN ─────────────────────────────────────────────────────────
    # A suck-in is the mirror of a bump: approach wall A ends at an EXTERNAL edge
    # V1, a short connector B drops back, and an INTERNAL vertex V2 lands on the
    # recessed baseline C. Two emission paths:
    #
    #   (2a) CONFIRMED: adjacent EXTERNAL->INTERNAL pair sharing connector B (mirror
    #        of the bump's INTERNAL->EXTERNAL). depth = |V1 - V2|. Full geometry.
    #
    #   (2b) PROVISIONAL: a terminal edge carrying recess_depth (near wall ended,
    #        a straight recessed wall C fitted behind it) BEFORE B/C resolve into a
    #        clean pair. Lets env latch V1 and pick the suck-in row early, then
    #        refine at the trigger. Only emitted when no confirmed suck-in covers it.
    for k in range(len(pairs)-1):
        c0, c1 = pairs[k], pairs[k+1]
        if c0["type"] != "EXTERNAL" or c1["type"] != "INTERNAL":
            continue
        if c0["idx_b"] != c1["idx_a"]:                     # must share connector B
            continue
        Bfit, Bsxy = _fit_of(kept[c0["idx_b"]])
        Bm = fit_metrics(Bfit, Bsxy, scan)
        if not (cfg.conn_min_chord_m <= Bm["chord_m"] <= cfg.conn_max_chord_m):
            continue
        V1, V2 = c0["vertex"], c1["vertex"]                # V1 external, V2 internal
        depth  = float(np.linalg.norm(V1 - V2))
        turn_ok = (c0["radian_diff"] < 0) and (c1["radian_diff"] > 0)   # CW then CCW
        if not turn_ok:
            continue
        # Same 0.60 branch as the provisional path: shallow -> SUCKIN (compound),
        # deep -> EXTERNAL at V1 (two-corner; V2 handled reactively as INTERNAL).
        # Without the deep branch, a confirmed >0.6 pair would emit nothing and we'd
        # be back to the walk-by failure if the provisional path missed.
        if depth <= SUCKIN_MAX_DEPTH:
            events.append({"type": "SUCKIN", "vertex": V1, "point": V1,
                           "external_vertex": V1, "internal_vertex": V2,
                           "connector_chord": Bm["chord_m"], "depth": depth,
                           "provisional": False,
                           "sweep_angle": _sweep_of(V1), "source": "suckin_pair"})
        else:
            events.append({"type": "EXTERNAL", "vertex": V1, "point": V1,
                           "radian_diff": -np.pi/2, "angle_deg": 90.0, "turn_dir": "CW",
                           "recess_depth": depth, "internal_vertex": V2,
                           "sweep_angle": _sweep_of(V1), "source": "deep_recess_pair_external"})

    # (2b) provisional recess from a terminal edge (near wall A ends, straight
    # recessed wall C behind it), BEFORE the full EXTERNAL->INTERNAL pair resolves.
    # Branch on the 0.60 cutoff (same as bump):
    #   d <= 0.60 -> provisional shallow SUCKIN (compound table), if none confirmed.
    #   d >  0.60 -> NOT a shallow suck-in: emit EXTERNAL at V1 so the bot turns the
    #                first corner now (deep recess runs as two independent corners;
    #                V2 is detected reactively as INTERNAL after the external turn).
    #                recess depth kept as metadata.
    # This is what stops the walk-by: the robot no longer drives straight past a
    # disappearing wall into the far wall.
    have_suckin = any(e["type"] == "SUCKIN" for e in events)
    for te in terminal_edges:
        d = te.get("recess_depth")
        if d is None:
            continue
        V1 = np.asarray(te["point"], float)
        if any(np.linalg.norm(V1 - e["vertex"]) < 0.15 for e in events):
            continue
        if d <= SUCKIN_MAX_DEPTH:
            if have_suckin:
                continue
            events.append({"type": "SUCKIN", "vertex": V1, "point": V1,
                           "external_vertex": V1, "internal_vertex": None,
                           "depth": float(d), "provisional": True,
                           "recess_C": te.get("recess_C"),
                           "sweep_angle": _sweep_of(V1), "source": "suckin_recess_provisional"})
            have_suckin = True   # don't emit a second provisional suck-in this scan
        else:
            # deep recess -> external corner at V1 (two-corner path). Estimate V2
            # (the internal vertex on the recessed baseline C) as the foot of V1 on
            # the C line: V2 = V1 - dot(V1 - cC, nC) * nC. Carried so env can arm the
            # expected-internal fast-commit after the external turn (B1). Falls back
            # to None if no C geometry was captured.
            rc = te.get("recess_C")
            iv2 = None
            if rc is not None:
                cC = np.asarray(rc["centroid"], float)
                nC = np.asarray(rc["normal"], float)
                nn = float(np.hypot(nC[0], nC[1]))
                if nn > 1e-9:
                    nCu = nC / nn
                    iv2 = V1 - float(np.dot(V1 - cC, nCu)) * nCu
            events.append({"type": "EXTERNAL", "vertex": V1, "point": V1,
                           "radian_diff": -np.pi/2, "angle_deg": 90.0, "turn_dir": "CW",
                           "recess_depth": float(d), "recess_C": rc,
                           "internal_vertex": (None if iv2 is None else np.asarray(iv2, float)),
                           "sweep_angle": _sweep_of(V1), "source": "deep_recess_external"})

    # ── 3/4. INTERNAL / EXTERNAL corners (strict pairs not folded into a bump) ─
    # Recompute strict corners on the GATED set so corner geometry stays strict
    # (bump uses the rescued set; ordinary corners should not ride a rescued leg).
    for a, b in zip(range(len(gated)-1), range(1, len(gated))):
        fa, sa = _fit_of(gated[a]); fb, sb = _fit_of(gated[b])
        res = classify_segment_pair(fa, sa, fb, sb)
        if res is None:
            continue
        V = np.asarray(res["vertex"], float)
        # INTERNAL only when AHEAD — a corner the robot has already turned past has a
        # behind-us vertex (x < 0) and must not re-latch (was the live re-latch bug).
        if res["type"] == "INTERNAL" and V[0] <= INTERNAL_VERTEX_AHEAD_MIN:
            continue
        # suppress a corner whose vertex coincides with a bump vertex (same feature)
        if any(np.linalg.norm(V - e["internal_vertex"]) < 0.10 or
               np.linalg.norm(V - e["external_vertex"]) < 0.10
               for e in events if e["type"] == "BUMPOUT"):
            continue
        events.append({"type": res["type"], "vertex": V, "point": V,
                       "radian_diff": res["radian_diff"], "angle_deg": res["angle_deg"],
                       "turn_dir": "CCW" if res["radian_diff"] > 0 else "CW",
                       "sweep_angle": _sweep_of(V), "fit_before": a, "fit_after": b,
                       "source": "line_intersection"})

    # -- 3b. GAPPED INTERNAL (fallback) ---------------------------------------
    # Only when the adjacent-pair logic above found NO internal corner. A bump /
    # protrusion separated from the main wall by a gap (up to ~0.25 m) leaves A and
    # B as NON-adjacent fits, so classify_segment_pair never pairs them. Here we fit
    # A (followed right wall) and B (front wall) independently and intersect their
    # infinite lines. Runs on the strict GATED set: this is INTERNAL corner geometry
    # (gated's job), and with the chord floor at 1 strip width (0.203) a real
    # protrusion face still qualifies - we do NOT want to build a corner off a wall
    # shorter than one strip. Only broadens acceptance for this one missing-corner
    # (gapped) geometry; emits a standard INTERNAL (source='gapped_internal').
    if not any(e["type"] == "INTERNAL" for e in events):
        gi = detect_gapped_internal_from_fits(gated, scan, cfg)
        if gi is not None and gi["vertex"][0] > INTERNAL_VERTEX_AHEAD_MIN and \
           not any(np.linalg.norm(gi["vertex"] - e["vertex"]) < 0.15 for e in events):
            events.append(gi)

    # ── EXTERNAL (confirmed A⊥C, ~0.1 m past vertex) ──────────────────────────
    # Runs the convex A⊥C confirm. When it fires we have a real external corner with
    # a measured angle — emit EXTERNAL (preferred over the bare edge below).
    ext = detect_external_corner(scan, cfg, yaw_offset=yaw_offset)
    if ext is not None and not any(
            np.linalg.norm(ext["vertex"] - e["vertex"]) < 0.15 for e in events):
        events.append(ext)

    # external terminal edges that aren't already covered by a corner/bump/EXTERNAL.
    # EXTERNAL_EDGE is the APPROACH-phase signal (before the vertex, C not yet seen);
    # once EXTERNAL is confirmed it supersedes the edge, so skip edges near one.
    have_external = any(e["type"] == "EXTERNAL" for e in events)
    for e in terminal_edges:
        pt = np.asarray(e["point"], float)
        if any(np.linalg.norm(pt - ev["vertex"]) < 0.15 for ev in events):
            continue
        if have_external:          # a confirmed external corner already covers this feature
            continue
        events.append({"type": "EXTERNAL_EDGE", "vertex": pt, "point": pt,
                       "radian_diff": -np.pi/2, "angle_deg": 90.0, "turn_dir": "CW",
                       "sweep_angle": _sweep_of(pt), "far_ratio": e["far_ratio"],
                       "seg_idx": e.get("seg_idx"), "source": "terminal_dropout"})

    # ── priority order: (type rank, then nearest sweep) ───────────────────
    events.sort(key=lambda e: (_EVENT_PRIORITY.get(e["type"], 9), e["sweep_angle"]))

    dbg = {"arc_xy": arc_xy, "gated_fits": gated, "kept_fits": kept,
           "terminal_edges": terminal_edges, "pairs": pairs}
    if debug:
        print(f"  motion_events: {len(gated)} strict / {len(kept)} kept fits, "
              f"{len(pairs)} pairs, {len(events)} events")
        for e in events:
            extra = (f"depth={e['depth']:.2f}" if e["type"] == "BUMPOUT"
                     else f"depth={e['depth']:.2f}{' prov' if e.get('provisional') else ''}"
                     if e["type"] == "SUCKIN"
                     else f"{e.get('angle_deg', 90):.0f}° {e.get('turn_dir','')}")
            print(f"    {e['type']:13s} sweep={e['sweep_angle']:6.1f} "
                  f"@({e['vertex'][0]:.2f},{e['vertex'][1]:.2f}) {extra}")
    return events, dbg