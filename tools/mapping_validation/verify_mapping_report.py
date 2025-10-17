# examples/verify_mapping_report.py
# Verify Scenic (XODR) world (x,y) → RD world (x,y) → (s,t)
# - Auto-selects the XODR that best matches the RD file
# - Samples points along the XODR centerline, transforms to RD, projects to (s,t)
# - Prints errors and saves a CSV report
# last modified 10/16/2025 5:02pm Jeffrey, Yikang

import os, math, csv
from statistics import mean

from scenic.simulators.dspace.coordinate_transform import (
    build_coordinate_transform, apply_coordinate_transform
)
from scenic.simulators.dspace.rd_geometry import parse_rd_geometry
from scenic.simulators.dspace import utils as dutils

# ------------------------------------------------------------------------------------
# PATHS
# ------------------------------------------------------------------------------------
HERE   = os.path.dirname(__file__)
ASSETS = os.path.join(HERE, '..', 'assets', 'maps', 'dSPACE')
RD_PATH = os.path.join(ASSETS, 'Laguna_Seca.rd')  # <-- this is the RD used by AURELION

# Candidate XODR files in your repo (adjust list if you add more variants)
XODR_CANDIDATES = [
    'LagunaSeca.xodr',
    'Laguna_Seca_Comprehensive.xodr',
    'Laguna_Seca_OuterLoop_Optimized.xodr',
    'Laguna_Seca_PitLane_Optimized.xodr',
]

# Report output
OUT_CSV = os.path.join(HERE, 'verify_mapping_report.csv')

# Tolerances for "PASS"
TOL_T   = 0.20   # |t| should be near 0 on centerline (m)
TOL_SRT = 0.20   # |s - s_target| (m)

# ------------------------------------------------------------------------------------
# HELPERS
# ------------------------------------------------------------------------------------
def eval_transform_quality(transform):
    """Mean XY error on the transform's stored calibration pairs."""
    cps = transform.get('calibration_points', [])
    if not cps:
        return float('inf')
    errs = []
    for _, x_xodr, y_xodr, x_rd, y_rd in cps[:: max(1, len(cps)//50) ]:
        px, py = apply_coordinate_transform(transform, (x_xodr, y_xodr))
        errs.append(math.hypot(px - x_rd, py - y_rd))
    return mean(errs)

def load_rd_reference(rd_path):
    """Load RD geometry and return list of (x, y, s) along the main reference."""
    rd_roads = parse_rd_geometry(rd_path, step=0.2)

    rd_main  = max(rd_roads, key=lambda r: r['total_length'])
    return rd_main['sec_points'][0], rd_main['total_length']  # (points, length)

def xy_to_st_rd(x, y, RD_PTS):
    """Project (x,y) (RD frame) onto the centerline segment for precise s,t."""
    best = None
    # search all segments; for speed you could stride (e.g., range(0, len-1, 2))
    for i in range(len(RD_PTS) - 1):
        x0, y0, s0 = RD_PTS[i]
        x1, y1, s1 = RD_PTS[i + 1]

        vx, vy = x1 - x0, y1 - y0
        wx, wy = x - x0, y - y0
        seg_len2 = vx * vx + vy * vy
        if seg_len2 == 0:
            continue

        # param of projection onto segment [0,1]
        tparam = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len2))

        # closest point on segment
        cx = x0 + tparam * vx
        cy = y0 + tparam * vy

        # squared distance to segment
        d2 = (x - cx) ** 2 + (y - cy) ** 2

        if best is None or d2 < best[0]:
            # unit tangent at segment
            seg_len = math.sqrt(seg_len2)
            tx, ty = vx / seg_len, vy / seg_len
            # left-hand normal
            nx, ny = -ty, tx
            # lateral offset t = distance from (cx,cy) to (x,y) along normal
            t_lat = (x - cx) * nx + (y - cy) * ny
            # along-track s = s0 + tparam * segment_length
            s_proj = s0 + tparam * (s1 - s0)
            best = (d2, s_proj, t_lat)

    # fall back if something odd
    if best is None:
        # nearest-point fallback
        best_i = min(range(len(RD_PTS)), key=lambda i: (x - RD_PTS[i][0]) ** 2 + (y - RD_PTS[i][1]) ** 2)
        i0 = max(0, best_i - 1); i1 = min(len(RD_PTS) - 1, best_i + 1)
        tx = RD_PTS[i1][0] - RD_PTS[i0][0]; ty = RD_PTS[i1][1] - RD_PTS[i0][1]
        n = math.hypot(tx, ty) or 1.0; tx/=n; ty/=n
        nx, ny = -ty, tx
        rx = x - RD_PTS[best_i][0]; ry = y - RD_PTS[best_i][1]
        return RD_PTS[best_i][2], rx * nx + ry * ny

    _, s_val, t_val = best
    return s_val, t_val

def xodr_xy_at_s(xodr_pts, target_s):
    """Pick the XODR (x,y) whose s is closest to target_s."""
    px, py, ps = min(xodr_pts, key=lambda p: abs(p[2] - target_s))
    return px, py, ps

# ------------------------------------------------------------------------------------
# 1) AUTO-PICK THE BEST-MATCHING XODR
# ------------------------------------------------------------------------------------
best = None
for fname in XODR_CANDIDATES:
    xodr_path = os.path.join(ASSETS, fname)
    if not os.path.exists(xodr_path):
        continue
    print(f"\n>>> Trying XODR: {fname}")
    T = build_coordinate_transform(xodr_path, RD_PATH, num_samples=200)
    m = eval_transform_quality(T)
    print(f"Mean error vs RD: {m:.3f} m")
    if best is None or m < best[0]:
        best = (m, fname, T)

if best is None:
    raise SystemExit("No XODR candidate found. Check ASSETS path / filenames.")

mean_err, best_name, transform = best
print(f"\n### Selected XODR: {best_name} (mean error {mean_err:.3f} m)")

# ------------------------------------------------------------------------------------
# 2) LOAD GEOMETRY FOR PROJECTION
# ------------------------------------------------------------------------------------
RD_PTS, RD_LEN = load_rd_reference(RD_PATH)
xodr_index     = dutils.build_xodr_sec_points(os.path.join(ASSETS, best_name))
xodr_main      = xodr_index['roads']['main_road']
XODR_PTS       = xodr_main['sec_points'][0]  # (x, y, s)

# Choose several s targets along the lap (end minus a bit to avoid wrap)
target_s_values = [0, 250, 500, 1000, 1500, max(0.0, RD_LEN - 1.0)]

# ------------------------------------------------------------------------------------
# 3) SAMPLE CENTERLINE POINTS, TRANSFORM, AND PROJECT TO (s,t)
# ------------------------------------------------------------------------------------
rows = []
for s0 in target_s_values:
    x_sc, y_sc, s_true = xodr_xy_at_s(XODR_PTS, s0)        # Scenic/XODR world XY at s
    x_rd, y_rd = apply_coordinate_transform(transform, (x_sc, y_sc))  # -> RD XY
    s_rd, t_rd = xy_to_st_rd(x_rd, y_rd, RD_PTS)           # -> (s,t)

    rows.append(dict(
        s_target=s_true,
        x_sc=x_sc, y_sc=y_sc,
        x_rd=x_rd, y_rd=y_rd,
        s=s_rd, t=t_rd,
        abs_s_err=abs(s_rd - s_true),
        abs_t=abs(t_rd),
    ))

# ------------------------------------------------------------------------------------
# 4) PRINT SUMMARY AND SAVE CSV
# ------------------------------------------------------------------------------------
print("\nRESULTS (centerline samples):")
print(f"{'s_target':>9} | {'x_sc':>9} {'y_sc':>9} | {'x_rd':>9} {'y_rd':>9} | {'s_rd':>9} {'t_rd':>9} | {'|Δs|':>7} {'|t|':>7}")
for r in rows:
    print(f"{r['s_target']:9.3f} | {r['x_sc']:9.3f} {r['y_sc']:9.3f} | {r['x_rd']:9.3f} {r['y_rd']:9.3f} | "
          f"{r['s']:9.3f} {r['t']:9.3f} | {r['abs_s_err']:7.3f} {r['abs_t']:7.3f}")

mean_abs_s = mean(r['abs_s_err'] for r in rows)
mean_abs_t = mean(r['abs_t'] for r in rows)
print(f"\nMEAN |Δs| = {mean_abs_s:.3f} m   MEAN |t| = {mean_abs_t:.3f} m")
pass_s = all(r['abs_s_err'] <= TOL_SRT for r in rows)
pass_t = all(r['abs_t']     <= TOL_T   for r in rows)
print(f"PASS |Δs|≤{TOL_SRT} m? {'YES' if pass_s else 'NO'}")
print(f"PASS |t| ≤{TOL_T} m?   {'YES' if pass_t else 'NO'}")

with open(OUT_CSV, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=[
        "s_target","x_sc","y_sc","x_rd","y_rd","s","t","abs_s_err","abs_t",
        "chosen_xodr","rd_path","mean_xodr_rd_error"
    ])
    w.writeheader()
    for r in rows:
        r = dict(r)
        r["chosen_xodr"] = best_name
        r["rd_path"] = RD_PATH
        r["mean_xodr_rd_error"] = round(mean_err, 3)
        w.writerow(r)

print(f"\nSaved CSV: {OUT_CSV}")
