#!/usr/bin/env python3
"""
corner_spray_external.py — EXTERNAL (convex) 90 deg corner. Standalone, no ROS.

Honest forward simulation + solver. Wall on the robot's RIGHT throughout; robot
follows the entry wall, rounds the convex vertex (CW turn), re-aligns on the new
(exit) wall. Wall orientation is arbitrary; canonical frame (approach +x, exit -y).

TWO MODES, both triggered on ODOM DISTANCE PAST THE VERTEX (only measurable anchor
once wall drops from view — env_monitoring latches the vertex):

  MODE A ZERO-RADIUS spin: knob = pivot distance past vertex; output = measured
     sprayhead landing on new wall; solve pivot that lands at SPRAY_STANDOFF.
  MODE B ARC radius R_axis: knob = trigger distance past vertex; solve trigger
     (per R_axis) that lands sprayhead at SPRAY_STANDOFF.

Sprayhead standoff is MEASURED on exit, never imposed — bad params visibly miss.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly
import math, os

HERE   = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(HERE, 'outputs')
os.makedirs(OUTDIR, exist_ok=True)

# ═══ TUNABLES ═══
R_AXIS_VALUES  = [0.40, 0.80]
SPRAY_STANDOFF = 0.304
SPRAY_D_ORIGIN = 0.600
DELTA          = 90.0

# ZERO-RADIUS spin: set a value to TEST your own tested pivot distance past vertex
# (leave as None to let the solver find the ideal). Compare the honest landing to 0.30.
MANUAL_PIVOT_PAST_VERTEX = None   # e.g. 0.30 to test; None = solve
SPRAY_OFFSET = (0.0, -SPRAY_D_ORIGIN)
LIDAR_OFFSET = (0.195, -0.1475)
LIDAR_OFFSET = (0.28, -0.1645)

CHASSIS_FRONT=0.500; CHASSIS_REAR=-0.500; CHASSIS_LEFT=0.350; CHASSIS_RIGHT=-0.350
SPRAY_TAB_XHALF=0.10; SPRAY_TAB_OUT=-0.600
BODY_FRONT=0.500; BODY_REAR=-0.500; BODY_LEFT=0.350; BODY_RIGHT=-0.600
V=0.12; DT=0.02
LEAD_IN=0.8; LEAD_OUT=0.9
SHOW=False
N_FOOTPRINTS=8; TIP_MARKERS=8    # plot subsample (fewer = cleaner)
# ════════════════

d=math.radians(DELTA)
AXIS_APPROACH_Y = SPRAY_STANDOFF + SPRAY_D_ORIGIN

def base_to_world(ax,ay,th,bx,by):
    c,s=math.cos(th),math.sin(th)
    return (ax+bx*c-by*s, ay+bx*s+by*c)
def spray_world(p): return base_to_world(*p,*SPRAY_OFFSET)
def lidar_world(p): return base_to_world(*p,*LIDAR_OFFSET)

def clr_entry(x,y):
    if x<=0: return y
    return math.hypot(x,y)
def clr_exit(x,y):
    if y<=0: return x
    return math.hypot(x,y)
def clr_min_signed(x,y): return min(clr_entry(x,y),clr_exit(x,y))
def dist_vertex(x,y): return math.hypot(x,y)

def body_corners(p):
    pts=[(BODY_REAR,BODY_RIGHT),(BODY_FRONT,BODY_RIGHT),(BODY_FRONT,BODY_LEFT),(BODY_REAR,BODY_LEFT)]
    return [base_to_world(*p,px,py) for px,py in pts]
def chassis_outline(p):
    pts=[(CHASSIS_REAR,CHASSIS_LEFT),(CHASSIS_FRONT,CHASSIS_LEFT),
         (CHASSIS_FRONT,CHASSIS_RIGHT),(CHASSIS_REAR,CHASSIS_RIGHT)]
    return [base_to_world(*p,px,py) for px,py in pts]
def spray_tab_outline(p):
    # T-nozzle: crossbar (flat top) at chassis, stem points OUT (-y) to the wall, tip at end
    stem_half=0.045; bar_half=SPRAY_TAB_XHALF
    bar_root=CHASSIS_RIGHT; bar_out=CHASSIS_RIGHT-0.06; tip_y=SPRAY_TAB_OUT
    pts=[( bar_half,bar_root),( bar_half,bar_out),( stem_half,bar_out),( stem_half,tip_y),
         (-stem_half,tip_y),(-stem_half,bar_out),(-bar_half,bar_out),(-bar_half,bar_root)]
    return [base_to_world(*p,px,py) for px,py in pts]

def forward_sim(mode, knob, R_axis=None):
    poses=[]
    yA=AXIS_APPROACH_Y
    n_in=max(1,int((knob-(-LEAD_IN))/(V*DT)))
    for i in range(n_in+1):
        xi=-LEAD_IN+(knob-(-LEAD_IN))*i/n_in
        poses.append((xi,yA,0.0))
    ax,ay,th=poses[-1]
    if mode=='spin':
        n=60
        for i in range(1,n+1):
            poses.append((ax,ay,-d*i/n))
        ax,ay,th=poses[-1]
        n_out=max(1,int(LEAD_OUT/(V*DT)))
        for i in range(1,n_out+1):
            poses.append((ax+V*DT*i*math.cos(th), ay+V*DT*i*math.sin(th), th))
    else:
        cx,cy=ax,ay-R_axis
        a0=math.atan2(ay-cy,ax-cx)
        sweep=-d
        n=max(2,int(abs(sweep)*R_axis/(V*DT)))
        for i in range(1,n+1):
            a=a0+sweep*i/n
            poses.append((cx+R_axis*math.cos(a), cy+R_axis*math.sin(a), 0.0+sweep*i/n))
        ax,ay,th=poses[-1]
        n_out=max(1,int(LEAD_OUT/(V*DT)))
        for i in range(1,n_out+1):
            poses.append((ax+V*DT*i*math.cos(th), ay+V*DT*i*math.sin(th), th))
    return poses

def exit_landing(poses):
    # exit wall is the half-line x=0 (y<0); robot's valid side is x>0.
    # return SIGNED sprayhead x so the solver targets +SPRAY_STANDOFF (correct side).
    # abs() would admit a false root at x=-standoff (sprayhead through the wall).
    sx,sy=spray_world(poses[-1])
    return sx

def solve_knob(mode, target=SPRAY_STANDOFF, R_axis=None, lo=-0.5, hi=1.5):
    def f(k): return exit_landing(forward_sim(mode,k,R_axis))-target
    if f(lo)*f(hi)>0:
        for hi2 in np.linspace(hi,4.0,40):
            if f(lo)*f(hi2)<0: hi=hi2; break
    for _ in range(60):
        mid=(lo+hi)/2; fm=f(mid)
        if abs(fm)<1e-4: return mid
        if f(lo)*fm<0: hi=mid
        else: lo=mid
    return (lo+hi)/2

def analyze(mode, knob, R_axis=None):
    poses=forward_sim(mode,knob,R_axis)
    spray=[spray_world(p) for p in poses]
    lidar=[lidar_world(p) for p in poses]
    spray_clr=[clr_min_signed(*p) for p in spray]
    body_clr=[min(clr_min_signed(cx,cy) for cx,cy in body_corners(p)) for p in poses]
    spray_vtx=[dist_vertex(*p) for p in spray]
    return dict(mode=mode,knob=knob,R_axis=R_axis,poses=poses,spray=spray,lidar=lidar,
                spray_clr=spray_clr,body_clr=body_clr,spray_vtx=spray_vtx,
                landing=exit_landing(poses),worst_body=min(body_clr),
                spray_vtx_min=min(spray_vtx),
                ang_over_lin=(0.0 if (R_axis is None or R_axis<=1e-6) else 1.0/R_axis))

def plot_one(res, tag):
    mode=res['mode']; knob=res['knob']; Ra=res['R_axis']; s=SPRAY_STANDOFF
    fig,(ax,ax2)=plt.subplots(1,2,figsize=(15,7),gridspec_kw={'width_ratios':[1.5,1]})
    ax.plot([-3,0],[0,0],'-',color='#A32D2D',lw=2.5)
    ax.plot([0,0],[-3,0],'-',color='#A32D2D',lw=2.5)
    ax.plot(0,0,'o',color='#A32D2D'); ax.annotate('vertex',(0.05,0.08))
    ax.plot([-3,0],[s,s],'--',color='#E08A2B',lw=0.7)
    ax.plot([s,s],[-3,0],'--',color='#E08A2B',lw=0.7,label='spray standoff')
    sp=np.array(res['spray'])
    pp=np.array([(p[0],p[1]) for p in res['poses']])
    for k in np.linspace(0,len(res['poses'])-1,N_FOOTPRINTS).astype(int):
        ax.add_patch(MplPoly(chassis_outline(res['poses'][k]),closed=True,
                     fill=False,edgecolor='#2ca02c',lw=1.0,alpha=0.45))
        ax.add_patch(MplPoly(spray_tab_outline(res['poses'][k]),closed=True,
                     fill=True,facecolor='#222',edgecolor='#000',lw=1.4,alpha=0.85,zorder=4))
    ax.plot(sp[:,0],sp[:,1],'-',color='#E0542B',lw=2.0,label='sprayhead TIP',zorder=5)
    for k in np.linspace(0,len(res['poses'])-1,TIP_MARKERS).astype(int):
        ax.plot(sp[k,0],sp[k,1],'o',color='#E0542B',ms=4,zorder=6)
    ax.plot(pp[:,0],pp[:,1],'-',color='#d4a017',lw=1.3,label='turn-axis')
    ax.axvline(knob,color='#888',ls=':',lw=0.8)
    ax.annotate(f'turn start @ {knob:+.3f} past vertex',(knob+0.03,-2.2),color='#555',fontsize=8)
    title=('Zero-Radius Spin' if mode=='spin' else f'Arc Turn  Radius={Ra:.2f} m')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_title(f'Sprayhead & Bot Path — External {title}')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.legend(loc='upper right',fontsize=8); ax.set_xlim(-1.6,1.8); ax.set_ylim(-2.4,1.4)
    prog=np.linspace(0,100,len(res['poses']))
    ax2.axhline(s,color='#E08A2B',ls='--',lw=0.9,label=f'target standoff {s:.3f} m')
    ax2.plot(prog,res['spray_clr'],'-',color='#E0542B',lw=1.8,label='sprayhead → wall')
    ax2.plot(prog,res['spray_vtx'],'-',color='#B03A6E',lw=1.2,label='sprayhead → corner point')
    ax2.plot(prog,res['body_clr'],'-',color='#2ca02c',lw=1.4,label='nearest body edge → wall')
    ax2.axhline(0,color='#A32D2D',lw=1,label='wall (0 = contact)')
    ax2.set_xlabel('turn progress (%)'); ax2.set_ylabel('clearance (m)')
    ax2.set_title(f'Clearance — External {title}')
    ax2.grid(alpha=0.3); ax2.legend(fontsize=8); ax2.set_ylim(-0.3,1.4)
    plt.tight_layout()
    fn=os.path.join(OUTDIR,f'ext_{tag}.png')
    plt.savefig(fn,dpi=110,bbox_inches='tight'); print('saved',fn)
    if SHOW: plt.show()
    plt.close(fig)

print("MODE A - ZERO-RADIUS spin")
if MANUAL_PIVOT_PAST_VERTEX is not None:
    k_spin = MANUAL_PIVOT_PAST_VERTEX
    print(f"  MANUAL pivot_past_vertex = {k_spin:.3f} m (your tested value)")
else:
    k_spin = solve_knob('spin')
    print(f"  SOLVED pivot_past_vertex = {k_spin:.3f} m (for 0.30 landing)")
r=analyze('spin',k_spin)
print(f"  -> landing {r['landing']:.3f}  (target {SPRAY_STANDOFF})  "
      f"worst_body {r['worst_body']:.3f}  vtx_min {r['spray_vtx_min']:.3f}")
plot_one(r,'spin')

print("\nMODE B - ARC: solve trigger distance past vertex per R_axis for 0.30 landing")
print(f'{"R_axis":>7} {"ang/lin":>8} {"trigger_past_vtx":>17} {"landing":>8} {"vtx_min":>8} {"worst_body":>11}')
for Ra in R_AXIS_VALUES:
    k=solve_knob('arc',R_axis=Ra)
    r=analyze('arc',k,R_axis=Ra)
    print(f'{Ra:7.2f} {r["ang_over_lin"]:8.3f} {k:17.3f} {r["landing"]:8.3f} '
          f'{r["spray_vtx_min"]:8.3f} {r["worst_body"]:11.3f}')
    plot_one(r,f'arc_R{Ra:.2f}')

print("\nAll 'past vertex' distances are odom-measurable: latch vertex, count distance past it.")