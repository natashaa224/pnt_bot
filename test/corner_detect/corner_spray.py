#!/usr/bin/env python3
"""
corner_spray.py — internal corner arc, SPRAYHEAD-primary. Standalone (no ROS).

Objective: the SPRAYHEAD holds its target distance to the wall along the trace
(paint quality). The sprayhead traces the clean tangent arc. The turn axis arc
is concentric and is what the diff-drive base actually executes.

Frame (internal / concave corner, CCW left turn):
  right wall : y = 0   (robot side y>0)   -- entry wall, followed
  front wall : x = 0   (robot side x<0)   -- exit wall
  vertex     : (0,0)
  arc center C : on +y side (left / inside the corner)

Radii (all measured from arc center C):
  sprayhead is at base-frame (0, -SPRAY_D_ORIGIN); it sits on the -y side
  (toward the wall, AWAY from C) so it traces the LARGER arc:
        R_spray = R_axis + SPRAY_D_ORIGIN
  Parametrize by R_SPRAY (the objective). Derive R_axis = R_spray - SPRAY_D_ORIGIN.
  Sprayhead arc is tangent to the sprayhead-standoff lines (SPRAY_STANDOFF inside walls).

Signed clearance (positive = safe side of wall, negative = penetrated):
  right wall:  c = y
  front wall:  c = -x

Sweep R_SPRAY values at top. R_axis<=0 => degenerate (treated as spin strawman).

Run:  python3 corner_spray.py
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly
import math, os

HERE   = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(HERE, 'outputs')
os.makedirs(OUTDIR, exist_ok=True)

# ═══════════════════════════════════════════════════════════════════
#  TUNABLES
# ═══════════════════════════════════════════════════════════════════
R_AXIS_VALUES  = [0.0, 0.40, 0.80]    # <-- TURN-AXIS radii to sweep. 0.0 = zero-radius turn
SPRAY_STANDOFF = 0.304                 # sprayhead target distance to wall (12 in)
SPRAY_D_ORIGIN = 0.600                 # distance origin(turn axis) -> sprayhead
DELTA          = 90.0                  # corner turn angle (deg)

# mounts in base frame (x fwd, y left), relative to turn axis
SPRAY_OFFSET = (0.0, -SPRAY_D_ORIGIN)  # sprayhead position (on axis x-line)
LIDAR_OFFSET = (0.195, -0.1475)        # lidar position (derived, clearance-check only)

# ── body geometry ──
# CHASSIS = the real 0.7-wide box (drawn in plots)
CHASSIS_FRONT =  0.500
CHASSIS_REAR  = -0.500      # 1.0 m long
CHASSIS_LEFT  =  0.350
CHASSIS_RIGHT = -0.350      # 0.7 m wide

# SPRAYHEAD PROTRUSION TAB (drawn in plots): sticks out right side to -0.600 around x=0
SPRAY_TAB_XHALF = 0.10      # tab half-length along x (front-back) around sprayhead
SPRAY_TAB_OUT   = -0.600    # tab reaches to sprayhead y

# CLEARANCE body = conservative whole-right-side-at-0.600 (used for the math only)
BODY_FRONT =  0.500
BODY_REAR  = -0.500
BODY_LEFT  =  0.350
BODY_RIGHT = -0.600      # conservative: whole right side at protrusion depth

LEAD = 0.6               # straight lead-in / lead-out (m)
STEP = 0.01              # sample spacing (m)
SHOW = False             # True => plt.show() each figure

# ── plot cosmetics ──
N_FOOTPRINTS = 8         # how many robot footprints to draw along the path (fewer = cleaner)
TIP_MARKERS  = 8         # how many sprayhead-tip dots to draw
# ═══════════════════════════════════════════════════════════════════

d = math.radians(DELTA)

# ── rigid transforms ──
def base_to_world(ax, ay, th, bx, by):
    c, s = math.cos(th), math.sin(th)
    return (ax + bx*c - by*s, ay + bx*s + by*c)

def spray_world(axpose):
    return base_to_world(*axpose, *SPRAY_OFFSET)
def lidar_world(axpose):
    return base_to_world(*axpose, *LIDAR_OFFSET)

# ── signed clearance to the two walls (infinite-line approx) ──
def clr_right(x, y):  return y          # right wall y=0, safe when y>0
def clr_front(x, y):  return -x         # front wall x=0, safe when x<0
def clr_min_signed(x, y):
    return min(clr_right(x, y), clr_front(x, y))

# ── body corners in world for an axis pose (CLEARANCE: conservative box) ──
def body_corners(axpose):
    pts = [(BODY_REAR, BODY_RIGHT), (BODY_FRONT, BODY_RIGHT),
           (BODY_FRONT, BODY_LEFT),  (BODY_REAR, BODY_LEFT)]
    return [base_to_world(*axpose, px, py) for px, py in pts]

# ── VISUAL: chassis rectangle (light) — plotting only ──
def chassis_outline(axpose):
    pts = [
        (CHASSIS_REAR,  CHASSIS_LEFT),
        (CHASSIS_FRONT, CHASSIS_LEFT),
        (CHASSIS_FRONT, CHASSIS_RIGHT),
        (CHASSIS_REAR,  CHASSIS_RIGHT),
    ]
    return [base_to_world(*axpose, px, py) for px, py in pts]

# ── VISUAL: sprayhead as a T-shaped nozzle (bold black) — plotting only ──
# crossbar (flat top of T) attaches to the chassis; the stem points OUTWARD (-y)
# to the wall, with the spray tip at the outer end. SPRAY_OFFSET (0,-0.600) = tip.
def spray_tab_outline(axpose):
    stem_half = 0.045                      # stem half-width (along x)
    bar_half  = SPRAY_TAB_XHALF            # crossbar half-length (along x)
    bar_root  = CHASSIS_RIGHT              # crossbar outer face at chassis (y=-0.35)
    bar_out   = CHASSIS_RIGHT - 0.06       # crossbar inner face (toward tip)
    tip_y     = SPRAY_TAB_OUT              # stem tip (y=-0.600)
    # trace: crossbar across the chassis edge, then down the stem to the tip and back
    pts = [
        ( bar_half,  bar_root),            # crossbar front-root
        ( bar_half,  bar_out),             # crossbar front-inner
        ( stem_half, bar_out),             # into stem (front)
        ( stem_half, tip_y),               # down stem to tip (front)
        (-stem_half, tip_y),               # across the tip
        (-stem_half, bar_out),             # up stem (rear)
        (-bar_half,  bar_out),             # crossbar rear-inner
        (-bar_half,  bar_root),            # crossbar rear-root
    ]
    return [base_to_world(*axpose, px, py) for px, py in pts]

def build(R_axis):
    """Build TURN-AXIS poses for a turn of axis-radius R_axis.
       Sprayhead is concentric at R_spray = R_axis + SPRAY_D_ORIGIN and (for R_axis>0)
       traces the arc tangent to its standoff lines (y=s, x=-s).
       Arc center C shared: C = (-(s+R_spray), (s+R_spray)).
       R_axis = 0  => zero-radius turn: axis pinned, spins in place through DELTA.
    """
    s = SPRAY_STANDOFF
    R_spray = R_axis + SPRAY_D_ORIGIN
    Cx, Cy = -(s + R_spray), (s + R_spray)         # shared arc center

    Ts_in  = (Cx, s)          # sprayhead tangent on right standoff (y=s)
    Ts_out = (-s, Cy)         # sprayhead tangent on front standoff (x=-s)

    axposes = []

    # ---- straight IN: sprayhead rides right standoff y=s, heading +x ----
    n_in = max(1, int(LEAD/STEP))
    for i in range(n_in):
        sx = Ts_in[0] - LEAD + i*STEP
        ax = sx - SPRAY_OFFSET[0]
        ay = s  - SPRAY_OFFSET[1]     # = s + SPRAY_D_ORIGIN
        axposes.append((ax, ay, 0.0))

    # ---- ARC (R_axis>0) or ZERO-RADIUS TURN (R_axis==0) ----
    if R_axis > 1e-6:
        a0 = math.atan2(Ts_in[1]-Cy,  Ts_in[0]-Cx)     # entry (-y => -90)
        a1 = math.atan2(Ts_out[1]-Cy, Ts_out[0]-Cx)    # exit (+x => 0)
        sweep = a1 - a0
        if sweep < 0: sweep += 2*math.pi
        n_arc = max(2, int(sweep*R_spray/STEP))
        for i in range(n_arc+1):
            a = a0 + sweep*i/n_arc
            sx = Cx + R_spray*math.cos(a)
            sy = Cy + R_spray*math.sin(a)
            th = a + math.pi/2                          # heading tangent (CCW)
            c, si = math.cos(th), math.sin(th)
            ax = sx - (SPRAY_OFFSET[0]*c - SPRAY_OFFSET[1]*si)
            ay = sy - (SPRAY_OFFSET[0]*si + SPRAY_OFFSET[1]*c)
            axposes.append((ax, ay, th))
    else:
        # zero-radius: axis fixed at its in-segment end, sweep heading 0..DELTA
        ax, ay, _ = axposes[-1]
        n_spin = 60
        for i in range(n_spin+1):
            th = d * i/n_spin
            axposes.append((ax, ay, th))

    # ---- straight OUT: sprayhead rides front standoff x=-s, heading +DELTA ----
    n_out = max(1, int(LEAD/STEP))
    for i in range(n_out):
        sy = Ts_out[1] + i*STEP
        th = math.pi/2
        c, si = math.cos(th), math.sin(th)
        sx = -s
        ax = sx - (SPRAY_OFFSET[0]*c - SPRAY_OFFSET[1]*si)
        ay = sy - (SPRAY_OFFSET[0]*si + SPRAY_OFFSET[1]*c)
        axposes.append((ax, ay, th))

    return axposes, (Cx, Cy), R_spray, n_in

def analyze(R_axis):
    axposes, C, R_spray, i_start = build(R_axis)
    spray = [spray_world(p) for p in axposes]
    lidar = [lidar_world(p) for p in axposes]
    spray_clr = [clr_min_signed(*p) for p in spray]     # signed dist to nearest wall
    lidar_clr = [clr_min_signed(*p) for p in lidar]
    body_clr  = [min(clr_min_signed(cx,cy) for cx,cy in body_corners(p)) for p in axposes]

    # TRIGGER_DIST: lidar distance to the FRONT wall (x=0) at arc-start.
    # On the straight approach the front-wall distance SHRINKS as we advance, so it
    # crosses a threshold at the right moment (unlike right-wall dist, which is ~constant).
    # This is the live wall_detection front-wall reading that gates the start of the turn.
    lx_start, ly_start = lidar[i_start]
    trigger_dist = abs(lx_start)                         # front wall is x=0
    # right-wall standoff at start (for reference; ~constant during follow)
    trigger_right = abs(ly_start)
    ax_start = axposes[i_start][0]

    return dict(R_spray=R_spray, R_axis=R_axis, C=C, axposes=axposes,
                spray=spray, lidar=lidar,
                spray_clr=spray_clr, lidar_clr=lidar_clr, body_clr=body_clr,
                worst_body=min(body_clr),
                spray_dev=(min(spray_clr), max(spray_clr)),
                trigger_dist=trigger_dist, trigger_right=trigger_right,
                i_start=i_start, lidar_start=(lx_start, ly_start), ax_start=ax_start,
                ang_over_lin=(0.0 if R_axis<=1e-6 else 1.0/R_axis))

def plot_one(res):
    Rs = res['R_spray']; Ra = res['R_axis']; C = res['C']
    fig,(ax,ax2)=plt.subplots(1,2,figsize=(15,7),gridspec_kw={'width_ratios':[1.5,1]})

    # walls + standoff line for sprayhead
    ax.plot([-3,0],[0,0],'-',color='#A32D2D',lw=2.5)
    ax.plot([0,0],[0,3],'-',color='#A32D2D',lw=2.5)
    ax.plot(0,0,'o',color='#A32D2D'); ax.annotate('vertex',(0.05,0.05))
    s = SPRAY_STANDOFF
    ax.plot([-3,0],[s,s],'--',color='#E08A2B',lw=0.7)
    ax.plot([-s,-s],[0,3],'--',color='#E08A2B',lw=0.7, label='spray standoff')
    ax.plot(C[0],C[1],'o',color='#534AB7'); ax.annotate('C',(C[0]+0.04,C[1]+0.04),color='#534AB7')

    sp=np.array(res['spray']); li=np.array(res['lidar'])
    pp=np.array([(p[0],p[1]) for p in res['axposes']])
    # draw chassis (light) + sprayhead tab (bold black) at N_FOOTPRINTS poses
    for k in np.linspace(0,len(res['axposes'])-1,N_FOOTPRINTS).astype(int):
        ax.add_patch(MplPoly(chassis_outline(res['axposes'][k]),closed=True,
                     fill=False,edgecolor='#2ca02c',lw=1.0,alpha=0.45))
        ax.add_patch(MplPoly(spray_tab_outline(res['axposes'][k]),closed=True,
                     fill=True,facecolor='#222',edgecolor='#000',lw=1.4,alpha=0.85,zorder=4))
    # sprayhead tip path + a few markers
    ax.plot(sp[:,0],sp[:,1],'-',color='#E0542B',lw=2.0,label='sprayhead TIP path',zorder=5)
    for k in np.linspace(0,len(res['axposes'])-1,TIP_MARKERS).astype(int):
        ax.plot(sp[k,0],sp[k,1],'o',color='#E0542B',ms=4,zorder=6)
    ax.plot(pp[:,0],pp[:,1],'-',color='#d4a017',lw=1.3,label='turn-axis path')

    # TRIGGER distance: lidar-to-front-wall at arc start (runtime gate value)
    lxs, lys = res['lidar_start']
    ax.plot([lxs, 0], [lys, lys], '-', color='#3A9BD4', lw=1.0, alpha=0.7)
    ax.annotate(f'TRIGGER Distance (Lidar front)={res["trigger_dist"]:.3f}',
                (lxs/2+0.5, lys+0.05), color='#1B6FA8', fontsize=10)

    kind = 'Zero-Radius Turn' if Ra<=1e-6 else f'Arc Turn  Radius={Ra:.2f} m'
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_title(f'Sprayhead & Bot Path — Internal {kind}')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.legend(loc='upper right',fontsize=8); ax.set_xlim(-2.6,1.0); ax.set_ylim(-0.8,2.6)

    prog=np.linspace(0,100,len(res['axposes']))
    ax2.axhline(SPRAY_STANDOFF,color='#E08A2B',ls='--',lw=0.9,label=f'target standoff {s:.3f} m')
    ax2.plot(prog,res['spray_clr'],'-',color='#E0542B',lw=1.8,label='sprayhead → wall')
    ax2.plot(prog,res['body_clr'],'-',color='#2ca02c',lw=1.4,label='nearest body edge → wall')
    ax2.axhline(0,color='#A32D2D',lw=1,label='wall (0 = contact)')
    ax2.set_xlabel('turn progress (%)'); ax2.set_ylabel('clearance (m)')
    ax2.set_title(f'Clearance — Internal {kind}')
    ax2.grid(alpha=0.3); ax2.legend(fontsize=8); ax2.set_ylim(-0.1, SPRAY_STANDOFF+0.9)

    plt.tight_layout()
    fn=os.path.join(OUTDIR,f'spray_Raxis{Ra:.2f}.png')
    plt.savefig(fn,dpi=110,bbox_inches='tight'); print('saved',fn)
    if SHOW: plt.show()
    plt.close(fig)

print(f'{"R_axis":>7} {"R_spray":>8} {"ang/lin":>8} {"TRIGGER":>8} {"spray[min,max]":>18} {"worst body":>11}')
for Ra in R_AXIS_VALUES:
    r = analyze(Ra)
    dmin,dmax = r['spray_dev']
    print(f'{Ra:7.2f} {r["R_spray"]:8.2f} {r["ang_over_lin"]:8.3f} {r["trigger_dist"]:8.3f}'
          f'   [{dmin:.3f},{dmax:.3f}]     {r["worst_body"]:8.3f}')
    plot_one(r)

print("\nRuntime use per corner (internal, 90°):")
print("  1. WALL_FOLLOW: crawl until wall_detection right-wall dist == TRIGGER")
print("  2. CORNER_TURN: command lin=V_turn, ang = V_turn * (ang/lin) = V_turn / R_axis")
print("     integrate odom yaw until swept == corner angle (90°)")
print("  3. exit -> WALL_ALIGN / resume follow")