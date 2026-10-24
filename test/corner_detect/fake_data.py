import numpy as np

def generate_fake_scan(scenario='internal_corner', noise_std=0.005,
                       n_beams=360, range_min=0.15, range_max=15.0,
                       save_path=None):
    """
    Generate a fake RPLidar-style scan as a dict matching real .npy format.
    
    Scenarios:
      'internal_corner'  — two walls meeting, robot in the corner
      'external_corner'  — wall ends, robot approaching the edge
      'straight_wall'    — single wall on the right, nothing ahead
      'stub_pillar'      — projecting stub on the right wall
    
    Robot at origin, +x = forward, 270° = right wall.
    """
    angle_inc  = 2 * np.pi / n_beams
    angles     = np.arange(n_beams) * angle_inc   # 0 → 2π
    # ranges     = np.full(n_beams, range_max)       # default — open space
    ranges = np.full(n_beams, np.inf)   # no-hit = inf, not range_max
    if scenario == 'internal_corner':
        # right wall at y = -0.30, x from -2 to 0.40
        # front wall at x =  0.40, y from -0.30 to 2
        right_wall_x  =  0.40   # corner x
        right_wall_y  = -0.30   # right wall y
        front_wall_x  =  0.40   # front wall x

        for i, a in enumerate(angles):
            x_dir = np.cos(a)
            y_dir = np.sin(a)

            hits = []

            # right wall: y = right_wall_y, only where x <= right_wall_x
            if abs(y_dir) > 1e-6:
                t = right_wall_y / y_dir
                if t > 0:
                    x_hit = x_dir * t
                    if x_hit <= right_wall_x:
                        hits.append(t)

            # front wall: x = front_wall_x, only where y >= right_wall_y
            if abs(x_dir) > 1e-6:
                t = front_wall_x / x_dir
                if t > 0:
                    y_hit = y_dir * t
                    if y_hit >= right_wall_y:
                        hits.append(t)

            if hits:
                ranges[i] = min(hits)

    elif scenario == 'external_corner':
        # right wall at y = -0.30, only for x < 0.35 (wall ends)
        # after x=0.35 — open space on the right
        # no front wall visible yet
        wall_end_x   =  0.35
        right_wall_y = -0.30

        for i, a in enumerate(angles):
            x_dir = np.cos(a)
            y_dir = np.sin(a)

            if abs(y_dir) > 1e-6:
                t = right_wall_y / y_dir
                if t > 0:
                    x_hit = x_dir * t
                    if x_hit <= wall_end_x:
                        ranges[i] = t

    elif scenario == 'straight_wall':
        # right wall at y = -0.30, full length
        right_wall_y = -0.30
        for i, a in enumerate(angles):
            y_dir = np.sin(a)
            if abs(y_dir) > 1e-6:
                t = right_wall_y / y_dir
                if t > 0:
                    ranges[i] = t

    elif scenario == 'stub_pillar':
        right_wall_y = -0.30
        stub_x_min, stub_x_max =  0.10,  0.25
        stub_y_min, stub_y_max = -0.30, -0.10

        for i, a in enumerate(angles):
            x_dir = np.cos(a)
            y_dir = np.sin(a)
            hits  = []

            # right wall — only where stub doesn't block
            if abs(y_dir) > 1e-6:
                t = right_wall_y / y_dir
                if t > 0:
                    x_hit = x_dir * t
                    # blocked by stub if ray passes through stub x range
                    if not (stub_x_min <= x_hit <= stub_x_max):
                        hits.append(t)

            # stub left face: x = stub_x_min, strict y bounds
            if abs(x_dir) > 1e-6:
                t = stub_x_min / x_dir
                if t > 0:
                    y_hit = y_dir * t
                    if stub_y_min < y_hit < stub_y_max:
                        hits.append(t)

            # stub right face: x = stub_x_max
            if abs(x_dir) > 1e-6:
                t = stub_x_max / x_dir
                if t > 0:
                    y_hit = y_dir * t
                    if stub_y_min < y_hit < stub_y_max:
                        hits.append(t)

            # stub front face: y = stub_y_max
            if abs(y_dir) > 1e-6:
                t = stub_y_max / y_dir
                if t > 0:
                    x_hit = x_dir * t
                    if stub_x_min < x_hit < stub_x_max:
                        hits.append(t)

            # take closest hit only
            ranges[i] = min(hits) if hits else np.inf

    # ── clip and add noise ────────────────────────────────────────────────
    ranges = np.clip(ranges, range_min, range_max)
    noise  = np.random.normal(0, noise_std, n_beams)
    ranges = np.clip(ranges + noise, range_min, range_max).astype(np.float32)

    scan = {
        'ranges'        : ranges.tolist(),
        'angle_min'     : 0.0,
        'angle_max'     : 2 * np.pi,
        'angle_increment': angle_inc,
        'range_min'     : range_min,
        'range_max'     : range_max,
    }

    if save_path:
        np.save(save_path, scan)
        print(f"Saved: {save_path}")

    return scan


# ── Generate all four scenarios ───────────────────────────────────────────
import os
os.makedirs('./test/corner_detect/fake_data', exist_ok=True)

for scenario in ['internal_corner', 'external_corner',
                 'straight_wall', 'stub_pillar']:
    generate_fake_scan(scenario=scenario, noise_std=0.005,
                       save_path=f'./data/fake_{scenario}.npy')

print("Done — 4 fake scans saved to ./data/")