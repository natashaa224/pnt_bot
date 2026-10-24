import numpy as np
import matplotlib.pyplot as plt
import random
import os


# sys.path.insert(0, os.path.expanduser(
#     '~/ros_ws/painting_robot_ws/src/painting_robot_perception/painting_robot_perception'
# ))

from corner_geometry_utils import (
    extract_combined_arc,
    mad_filter,
    segment_points,
    split_segments_by_line_deviation,
    smart_fit_segments,
    merge_collinear,
    detect_best_corner,
    detect_terminal_edges_from_fits,
    ransac_line,
    pca_fit,
    fit_segment,
    point_line_distance,
    pca_direction_angle,
    abs_line_angle_diff_deg,
    oriented_wall_dir,
    line_intersection,
    classify_segment_pair,
    shift_angle_deg,
    in_wrapped_arc,
)

FORWARD_ARC_CENTER_DEG = 0.0
FORWARD_ARC_HALF_DEG   = 30.0

RIGHT_ARC_CENTER_DEG   = 270.0
RIGHT_ARC_HALF_DEG     = 45.0

CORNER_ARC_START_DEG   = 225.0   # right-rear boundary
CORNER_ARC_END_DEG     = 30.0    # forward-left boundary (wraps through 0°)

# usual turn angles, to round off 
STANDARD_ANGLES_DEG = [90.0, 120.0, 135.0, 45.0]
SNAP_TOLERANCE_DEG  = 15.0   # within 15° of a standard → snap to it


MIN_EXTERNAL_REVEAL_DIST = 0.05   # or 0.30 m
MIN_EXTERNAL_SCORE = 25
MIN_EXTERNAL_ANGLE_DEG = 70.0

TARGET_DIST = 0.549   # lidar to wall target distance — keep in sync with wall_motion_params


def _detect_corner_event(scan):


    ranges    = np.array(scan['ranges'], dtype=np.float32)
    angle_min = scan['angle_min']
    angle_inc = scan['angle_increment']
    angles_full = angle_min + np.arange(len(ranges)) * angle_inc

    arc_r, arc_a, arc_xy = extract_combined_arc(ranges, angles_full, angle_min_deg=CORNER_ARC_START_DEG, angle_max_deg=CORNER_ARC_END_DEG)

    segments = segment_points(arc_r, arc_a, arc_xy)

    segments = split_segments_by_line_deviation(
        segments,
        deviation_threshold=0.05,
        min_points=8,
        min_angle_change_deg=12.0,
        debug=False,
    )

    # filter segments from far edge out
    segments = [
        seg for seg in segments
        if np.median(seg[0]) < 3.0
    ]  
    
    fits = smart_fit_segments(segments, min_points=8)

    # terminal_edges = detect_terminal_edges_from_fits(
    #     fits,
    #     ranges_raw=ranges,
    #     angles_full_rad=angles_full,
    #     max_range_interest=2.0,
    #     lookahead_beams=12,
    #     min_far_ratio=0.40,
    #     debug=False,
    # )

    fits = merge_collinear(fits)

    result = detect_best_corner(fits)
    events = []

    if result is not None:
        events.append({
            "type": result["type"],
            "point": result["vertex"],
            "radian_diff": result["radian_diff"],
            "angle_deg": result["angle_deg"],
            "turn_dir": "CCW" if result["radian_diff"] > 0 else "CW",
            "source": "line_intersection",
        })

    # for e in terminal_edges:
    #     events.append({
    #         "type": "EXTERNAL_EDGE",
    #         "point": e["point"],
    #         "radian_diff": -np.pi / 2,   # CW 90° default
    #         "angle_deg": 90.0,
    #         "turn_dir": "CW",
    #         "source": "terminal_dropout",
    #         "seg_idx": e["seg_idx"],
    #         "far_ratio": e["far_ratio"],
    #     })

    def event_sort_key(ev):
        d = np.linalg.norm(ev["point"])
        return d

    events.sort(key=event_sort_key)

    # after distance sort, only swap INTERNAL ahead if comparable
    COMPARABLE_DIST = 0.40  # meters

    for i, ev in enumerate(events):
        if ev["type"] == "INTERNAL":
            internal = ev
            break
    else:
        internal = None

    if internal is not None:
        d_int = np.linalg.norm(internal["point"])

        closer_ext = [
            ev for ev in events
            if ev["type"] in ("EXTERNAL", "EXTERNAL_EDGE")
            and np.linalg.norm(ev["point"]) < d_int
        ]

        if closer_ext:
            d_ext = min(np.linalg.norm(ev["point"]) for ev in closer_ext)

            if abs(d_int - d_ext) < COMPARABLE_DIST:
                events.remove(internal)
                events.insert(0, internal)

    for ev in events:
        p = ev["point"]
        print(
            f"{ev['type']} source={ev['source']} "
            f"turn={ev['turn_dir']} {ev['angle_deg']:.1f}° "
            f"pt=({p[0]:.3f},{p[1]:.3f})"
        )

    return events, segments, fits,

def plot_events(scan, events, segments, fits, out_dir, scan_name):
    fig, ax = plt.subplots(figsize=(6, 6))

    # Full scan points (within 5 m)
    ranges = np.array(scan['ranges'], dtype=np.float32)
    valid = np.isfinite(ranges) & (ranges > 0.15) & (ranges < 15.0)
    r = ranges[valid]
    angle_min = scan['angle_min']
    angle_inc = scan['angle_increment']
    angles_full = angle_min + np.arange(len(ranges)) * angle_inc
    a = angles_full[valid]
    x = r * np.cos(a)
    y = r * np.sin(a)

    mask = r < 5.0
    ax.scatter(x[mask], y[mask], s=8, c='steelblue', alpha=0.4, label='scan points')

    # Segments
    colors = plt.cm.tab10.colors
    for i, (_, _, sxy) in enumerate(segments):
        ax.scatter(sxy[:, 0], sxy[:, 1], s=10,
                   color=colors[i % len(colors)], alpha=0.9, label=f'Segment {i}')

    # Fits
    for j, (f, sxy) in enumerate(fits):
        if f is None:
            continue
        n, c = f['normal'], f['centroid']
        wall_dir = np.array([-n[1], n[0]])
        p1, p2 = c + wall_dir * 2.0, c - wall_dir * 2.0
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                color=colors[j % len(colors)], lw=2)

    # Events
    for ev in events:
        px, py = ev["point"]
        if ev["type"] in ["INTERNAL", "EXTERNAL"]:
            ax.scatter(px, py, c='magenta', s=120, marker='o', zorder=6)
            ax.text(px, py - 0.1,
                    f"{ev['type']} {ev['angle_deg']:.1f}°",
                    color='magenta', fontsize=9, ha='center')
        elif ev["type"] == "EXTERNAL_EDGE":
            ax.scatter(px, py, c='green', s=100, marker='x', zorder=6)
            ax.text(px, py - 0.1,
                    f"{ev['type']} seg={ev['seg_idx']}",
                    color='green', fontsize=9, ha='center')

    # Robot marker + heading
    robot_marker = plt.matplotlib.markers.MarkerStyle('^')
    robot_marker._transform.rotate_deg(0)
    ax.scatter(0, 0, c='red', s=120, marker=robot_marker, zorder=5, label='robot')
    ax.annotate('', xy=(0.4, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle='->', color='red', lw=2))

    ax.set_xlim(-1.5, 2.5)
    ax.set_ylim(-1, 1.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("Pipeline 2 — Events (corners + edges)")
    save_path = os.path.join(out_dir, f"{scan_name}_events.png")
    plt.tight_layout()
    plt.savefig(save_path, bbox_inches='tight')
    plt.close() # Clean up memory for the next loop iteration







# Define the base directory where all experiments live
BASE_DIR = os.path.expanduser('~/ros_ws/painting_robot_ws/src/test/corner_detect/all_test_data/lidar_scans')

# 1. Automatically get all experiment folder names (run_1, run_2, etc.)
if not os.path.exists(BASE_DIR):
    raise FileNotFoundError(f"Base directory not found: {BASE_DIR}")

# expt_names = [d for d in os.listdir(BASE_DIR) if os.path.isdir(os.path.join(BASE_DIR, d))]
expt_names=['run_30June_1']
# Loop through each experiment folder
for name in expt_names:
    EXPT_DIR = os.path.join(BASE_DIR, name)
    DATA_DIR = os.path.join(EXPT_DIR, 'scans')
    OUT_DIR  = os.path.join(EXPT_DIR, "outputs")
    
    print("-" * 50)
    print(f"Processing Experiment: {name}")
    
    if not os.path.exists(DATA_DIR):
        print(f"  --> [SKIPPED] 'scans' directory not found in {name}")
        continue
        
    os.makedirs(OUT_DIR, exist_ok=True)
    
    # 2. Find all .npy files inside the scans folder
    all_files = os.listdir(DATA_DIR)
    npy_files = [f for f in all_files if f.endswith('.npy')]
    
    print(f"  --> Found {len(npy_files)} .npy scan(s) to process.")
    
    # Loop through each scan file
    for scan_file in sorted(npy_files):
        data_path = os.path.join(DATA_DIR, scan_file)
        
        try:
            # Load the dictionary data from the .npy file
            data = np.load(data_path, allow_pickle=True).item()
            
            scan_name = os.path.splitext(scan_file)[0]

            # Run your detection pipeline
            events, segments, fits = _detect_corner_event(data)
            
            # Save visual outputs into the specific experiment's output folder
            plot_events(data, events, segments, fits, OUT_DIR,scan_name)
            print(f"    + Successfully processed: {scan_file}")
            
        except Exception as e:
            print(f"    x Failed to process {scan_file}: {e}")

print("-" * 50)
print("All experiments completed!")
