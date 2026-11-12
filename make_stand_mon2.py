"""
Second stand: 11.6" cased monitor on a VESA 75x75 plate.  Reuses the base / carriage / turntable / hinge /
stop-screw / wing-nut parts of make_stand.py (same tilt 0..45, swivel +-45, slide +-15 mechanism, M5 wing-nut hinge)
and replaces the PCB frame by a flat plate that bolts to the 75 x 75 hole pattern on the monitor's control box.

Run:  python3 -I make_stand_mon2.py --out ./out2
Units: mm.  Frame coords: back (flat) face z=0, tilt axis at y=0, z=lug_r.  +y runs toward the top of the screen.
"""
import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bpy, bmesh
from mathutils import Matrix, Vector
import make_stand as ms
from make_stand import (prism, circle, rrect, hull, sector, union, cut, hexagon,
                        _bvh, hits, nonmanifold, export_stl, reset_scene)

def hinge_layout(C):
    """[(kind, x0, x1)] left->right: lug, ear, lug, ..., lug.  The FIRST and LAST knuckles belong to the monitor holder (frame),
    so the frame is a clevis straddling the turntable ears and the bolt / wing-nut clamp load goes through the frame, not the ears."""
    seq = []
    for i in range(C['n_lugs']):
        seq.append(('lug', C['lug_w_outer'] if i in (0, C['n_lugs'] - 1) else C['lug_w']))
        if i < C['n_lugs'] - 1: seq.append(('ear', C['ear_t']))
    total = sum(w for _, w in seq) + C['ear_gap'] * (len(seq) - 1)
    x, out = -total / 2, []
    for k, w in seq:
        out.append((k, x, x + w)); x += w + C['ear_gap']
    return out


ms.hinge_layout = hinge_layout

CFG2 = dict(ms.CFG_MONITOR1)
CFG2.update(
    name="mon2_11in6_v21_m4ends",
    # --- base / turntable scaled up for the heavier monitor
    spacer_lift=0.2, left_rib=False, base_monitor_margin=10.0, case_hole_n=3, case_hole_x=135.0, case_hole_span=83.8, case_hole_d=4.5, case_cb_d=8.2, case_cb_depth=4.6, case_end_back=7.7, case_end_front=6.5, seat_rib_len=30.0, seat_rib_w=6.0, seat_rib_h=10.0, seat_rib_gap=1.0, seat_rib_y0=62.0, seat_rib_ch=4.0, lock_rib_h=7.0, lock_rib_w=4.0, lock_rib_bx=16.0, base_l=290.0, tt_r=57.0, base_back_wall=28.0, foot_x=128.0, slide_travel=40.0, swivel_m=6, m5_hole=6.6, m5_cb_d=13.0, m5_cb_depth=4.6, m5_nut_af=10.4, m5_nut_depth=5.6,
    # --- VESA plate (holes are on the top face of the control box, 23 mm above the monitor back panel)
    plate_t=8.0, y0=8.5, lug_ramp_y=30.0,        # plate 8 mm (3.6 mm floor under the M4 heads); each lug ramps into the plate over 30 mm
   
    vesa=75.0, vesa_margin=10.0, vesa_yc=96.0,          # hole pitch, plate margin beyond the holes, hole-centre height above the tilt axis
    vesa_hole_d=4.5, vesa_head_d=7.6, vesa_head_depth=4.4,   # M4 clearance hole + socket-head counterbore (head hidden on the back)
    # --- monitor (from the drawing): body 305 wide incl. ears x 166.3 tall x 15 thick, control box 160 x 93 x 23
    body_w=305.0, body_t=15.0, body_below=99.5, body_above=66.8,     # body edges relative to the hole centre (62+37.5 below, 29.3+37.5 above)
    box_w=160.0, box_h=23.0, box_y0=-42.5, box_y1=50.5,
    # --- wiring bay: the box is lifted off the plate on two rails so the connector side (faces the hinge) is above the hinge discs
    standoff=25.0, rail_w=12.0, plug_len=60.0, plug_x=72.0,   # rail height / width; straight-plug + cable-bend envelope in front of the box
   
    # --- v2: 10 ears + 9 lugs (18 friction faces) and an M6 tilt hinge (swivel bolt stays M5)
    n_lugs=10, lug_w_outer=8.0, ear_t=5.0, lug_w=5.5, lug_r=14.0, stop_r=9.5, stop_groove_d=2.5, stop_screw_len=10.0,
    bolt_hinge_d=6.4, m4_nut_af=10.4, m4_nut_depth=4.5,                       # M6 hex head (10 AF, 4 tall) pocket on the right ear
    wn_span=24.3, wn_h=13.9, wn_boss_d=12.1, wn_boss_h=8.3, wn_wing_t=5.6, washer_t=3.2, washer_d=12.0, wn_m=6.0,   # M6 wing nut + 2 M6 washers
    tilt_max=90.0, base_front_margin=-33.0,             # 90 deg tilt: monitor moved 20 mm up the plate; base stops at y=126.5 as before (plate overhangs it)
    # --- swivel + slide lock: 2 x M6 knurled thumb screws push a long rectangular choke block against the turntable rim
    lock_z=14.0, lock_hole_d=6.6, lock_nut_af=10.0, lock_nut_t=5.2, lock_nut_y=-76.5,
    lock_block_len=96.0, lock_block_t=10.0, lock_block_h=6.4, lock_screw_x=34.0, lock_stroke=1.0,
    lock_screw_len=25.0, lock_head_d=22.0, lock_head_h=8.0,
    mon_mass=1.5, dyn=3.0,                               # kg: net 1.15 (datasheet) + cables/bracket margin; dyn = knock/bump factor on top of static weight
)


def frame_dims2(C):
    top = C['vesa_yc'] + C['vesa'] / 2 + C['vesa_margin']
    ph = top - C['y0']
    lay = hinge_layout(C)
    return dict(plate_w=max(C['vesa'] + 2 * C['vesa_margin'], lay[-1][2] - lay[0][1] + 12.0), plate_h=ph, yc=C['y0'] + ph / 2, y_top=top, top=C['plate_t'], xs=0.0)


ms.frame_dims = frame_dims2          # make_base() sizes itself from this


def lug_profile(C, outer=False):
    """Frame lug (YZ): hinge disc plus a long ramp (rib) that blends into the plate: carries the weight moment into the plate.
    outer=True (first and last knuckle): a full-height WEB from the disc up to the underside of the control box, so the end
    knuckles are solidly tied to the monitor holder (the gap between disc, plate and rail is filled)."""
    Rl, pt = C['lug_r'], C['plate_t']
    if outer:
        y_end = C['vesa_yc'] + C['box_y0'] - 3.0
        return hull(circle(0, Rl, Rl - C['lug_clear'], 64) + [(y_end, 0.0), (y_end, pt + C['standoff'])])
    return hull(circle(0, Rl, Rl - C['lug_clear'], 64) + [(C['lug_ramp_y'], 0.0), (C['lug_ramp_y'], pt)])


def chord(poly, y):
    zs = []
    for i in range(len(poly)):
        (y0, z0), (y1, z1) = poly[i], poly[(i + 1) % len(poly)]
        if (y0 - y) * (y1 - y) <= 0 and y0 != y1:
            zs.append(z0 + (z1 - z0) * (y - y0) / (y1 - y0))
    return (min(zs), max(zs)) if zs else (0.0, 0.0)


def stress_report(C):
    """Hand calculation (not FEA) of the load path with the real weight."""
    W = C['mon_mass'] * 9.81 * C['dyn']
    arm = 64.0                                              # mm, CG distance from the tilt axis when flat (worst case, from the pose sweep)
    M = W * arm
    pt, nl, lw = C['plate_t'], C['n_lugs'], C['lug_w']
    widths = [l[2] - l[1] for l in hinge_layout(C) if l[0] == 'lug']
    print("\n=== load path with the real weight (hand calc, ASA printed in XY: ~40 MPa nominal; planning allowable 12 MPa) ===")
    print(f"design load: {C['mon_mass']:.2f} kg x {C['dyn']:.0f} (knocks) = {W:.0f} N at {arm:.0f} mm from the hinge -> bending moment {M / 1000:.2f} N*m")
    prof = lug_profile(C)
    lo, hi = chord(prof, 16.5)
    h = hi - lo
    I_new = 0.0
    for i, w in enumerate(widths):
        lo, hi = chord(lug_profile(C, outer=(i in (0, len(widths) - 1))), 16.5)
        I_new += w * (hi - lo) ** 3 / 12.0
    I_old = 49.5 * 7.0 ** 3 / 12.0
    print(f"weakest section (end of the ear notches, y=16.5): {nl} lug ribs, {sum(widths):.1f} mm total width x {h:.1f} tall -> sigma = {M * (h / 2) / I_new:.1f} MPa (outer web {chord(lug_profile(C, outer=True), 16.5)[1] - chord(lug_profile(C, outer=True), 16.5)[0]:.0f} mm tall); "
          f"the v2 design had a 7 mm-thick finger there: {M * 3.5 / I_old:.1f} MPa, ~{3 * M * 3.5 / I_old:.0f} MPa at the sharp notch corner")
    PW = frame_dims2(C)['plate_w']
    Ip = PW * pt ** 3 / 12.0
    print(f"plate away from the notches: {PW:.0f} x {pt:.0f} mm -> sigma = {M * (pt / 2) / Ip:.2f} MPa")
    Ahead = math.pi / 4 * (C['vesa_head_d'] ** 2 - C['vesa_hole_d'] ** 2)
    T = (W * 0.7 * 35.0 / C['vesa'] + W * 0.7) / 2          # tilt 45: pull-off + moment couple, shared by 2 screws (conservative)
    print(f"VESA screw head bearing: floor {pt - C['vesa_head_depth']:.1f} mm thick, {T:.0f} N per screw on {Ahead:.0f} mm2 -> {T / Ahead:.2f} MPa; "
          f"punch-through shear {T / (math.pi * C['vesa_head_d'] * (pt - C['vesa_head_depth'])):.2f} MPa")
    I_b = math.pi * 6 ** 4 / 64
    S = C['standoff']
    print(f"rails: 2 x {C['rail_w']:.0f} wide x {S:.0f} tall; sideways 20 N push at the monitor CG -> rail root sigma = {20 * (S + 30) / 2 * 6 / (87 * C['rail_w'] ** 2):.2f} MPa (per rail, across layers)")
    print(f"M6 bolt: shank bending is negligible (the load is shared by {nl} lugs, the two 8 mm outer ones carry the clamp); clamp from the wing nut is what holds the angle")


def make_frame2(C):
    Rl, pt = C['lug_r'], C['plate_t']
    D = frame_dims2(C)
    fr = prism('frame', rrect(0, D['yc'], D['plate_w'], D['plate_h'], 6.0), 0, pt)
    lay = hinge_layout(C)
    ny, notches = 16.5, []
    for i, (k, x0, x1) in enumerate(lay):
        if k != 'ear': continue
        a = lay[i - 1][2] if i > 0 else x0 - 0.9
        b = lay[i + 1][1] if i < len(lay) - 1 else x1 + 0.9
        notches.append(prism('notch', rrect((a + b) / 2, (C['y0'] - 2 + ny) / 2, b - a, ny - (C['y0'] - 2), 1.5, 6), -1, pt + 1))   # rounded corners: no stress raiser
    # the M5 wing nut sweeps a circle around the tilt axis on the left face of the stack: keep plate material
    # out of that circle (every plate point must be > wing radius from the axis) by notching both lower corners
    xl0, xr0 = lay[0][1], lay[-1][2]
    ny2 = 15.0
    for a, b in ((-D['plate_w'] / 2 - 1, xl0), (xr0, D['plate_w'] / 2 + 1)):
        notches.append(prism('cornernotch', [(a, C['y0'] - 2), (b, C['y0'] - 2), (b, ny2), (a, ny2)], -1, pt + 1))
    cut(fr, *notches)
    lugs = [l for l in lay if l[0] == 'lug']
    union(fr, *[prism('lug', lug_profile(C, outer=(i in (0, len(lugs) - 1))), x0, x1, 'YZ') for i, (_, x0, x1) in enumerate(lugs)])
    xl, xr = lugs[0][1], lugs[-1][2]
    h = C['vesa'] / 2
    rl = C['vesa'] + 2 * (C['rail_w'] / 2)
    union(fr, *[prism('rail', rrect(sx * h, C['vesa_yc'], C['rail_w'], rl, 3.0, 6), pt - 0.5, pt + C['standoff']) for sx in (1, -1)])
    # cross walls: tie the end of each outer hinge web into the front end of the monitor rail (webs, walls and rails overlap = one solid)
    yb = C['vesa_yc'] + C['box_y0']
    for xa, xb in ((xl, -h), (h, xr)):
        union(fr, prism('xwall', [(xa, yb - 11.5), (xb, yb - 11.5), (xb, yb + 2.5), (xa, yb + 2.5)], pt - 0.5, pt + C['standoff']))
    cuts = []
    for sx in (1, -1):
        for sy in (1, -1):
            cx, cy = sx * h, C['vesa_yc'] + sy * h
            cuts.append(prism('thru', circle(cx, cy, C['vesa_hole_d'] / 2, 32, 4.5), -1, pt + C['standoff'] + 1))
            cuts.append(prism('cbore', circle(cx, cy, C['vesa_head_d'] / 2, 48, 3.5), -1, C['vesa_head_depth']))
    cuts.append(prism('hinge', circle(0, Rl, C['bolt_hinge_d'] / 2, 40, 4.5), xl - 1, xr + 1, 'YZ'))
    lugs_x = [(l[1], l[2]) for l in lay if l[0] == 'lug']
    for xa, xb in (lugs_x[0], lugs_x[-1]):                               # thread-forming holes for the M3 stop screws (they live in the frame now)
        cuts.append(prism('stophole', circle(C['stop_r'], Rl, C['stop_hole_d'] / 2, 32, 5.5), xa - 1, xb + 1, 'YZ'))
    cuts.append(prism('nutL', hexagon(0, Rl, C['m4_nut_af'], 90), xl - 1, xl + C['m4_nut_depth'], 'YZ'))   # M6 hex head pocket in the LEFT outer lug (wing nut is on the right)
    cut(fr, *cuts)
    fr['yc'] = D['yc']
    return fr


def ms_plate_w2(C):
    return frame_dims2(C)['plate_w']


def make_base2(C):
    """Base + lock features in the back wall: a rectangular tunnel for the choke block (opens into the turntable recess),
    two clearance holes for the thumb screws and two open-top slots that capture the M6 nuts.  Rib grid kept solid around them."""
    R = C['tt_r'] + C['pocket_clear'] / 2
    ymin = -(R + C['base_back_wall'])
    hx = C['lock_block_len'] / 2 + 6.0
    # the base is also the cover of the computer case: its front edge reaches the TOP EDGE of the monitor at rest (tilt 0) plus base_monitor_margin,
    # so the monitor sits behind that line and the case edge protects it
    C = dict(C, base_front_margin=C['vesa_yc'] + C['body_above'] + C['base_monitor_margin'] - frame_dims2(C)['y_top'])
    # M4 socket-head screw holes (case fastening): case_hole_n per long side, evenly spread about the middle of the base, solid boss kept round each
    yb1 = C['vesa_yc'] + C['body_above'] + C['base_monitor_margin']
    yc_b = (ymin + yb1) / 2
    hy = [yc_b + (i - (C['case_hole_n'] - 1) / 2) * 2 * C['case_hole_span'] / max(1, C['case_hole_n'] - 1) for i in range(C['case_hole_n'])]
    hpts = [(sx * C['case_hole_x'], y) for sx in (-1, 1) for y in hy]
    # + one hole on the centre line at each end: back wall between the two lock thumb screws, and the front edge opposite them
    hpts += [(0.0, ymin + C['case_end_back']), (0.0, yb1 - C['case_end_front'])]
    keep = [(-hx - 2.0, hx + 2.0, ymin - 1.0, -R + 4.0)] + [(x - 7.0, x + 7.0, y - 7.0, y + 7.0) for x, y in hpts]
    b = ms.make_base(dict(C, keep_solid=keep))
    zc, yp, bt = C['lock_z'], -R, C['lock_block_t']
    ch0 = yp - bt - 1.5
    def xz(x0, x1, z0, z1): return [(x0, z0), (x1, z0), (x1, z1), (x0, z1)]
    bw, bh = C['lock_block_len'] / 2 + 0.3, C['lock_block_h'] / 2 + 0.3
    nx, nz = C['lock_nut_af'] / 2 + 0.25, C['lock_nut_af'] / math.sqrt(3) + 0.2        # nut slot half-width, depth below the axis (corner radius)
    yn, nt = C['lock_nut_y'], C['lock_nut_t'] + 0.4
    cuts = [prism('lk_tunnel', xz(-bw, bw, zc - bh, zc + bh), ch0, yp + 3.0, 'XZ')]       # +3: also opens through the curved ends of the recess
    for sx in (-1, 1):
        x = sx * C['lock_screw_x']
        cuts.append(prism('lk_hole', circle(x, zc, C['lock_hole_d'] / 2, 32, 4.5), ymin - 1.0, ch0 + 0.5, 'XZ'))
        cuts.append(prism('lk_nutslot', xz(x - nx, x + nx, zc - nz, C['base_t'] + C['lock_rib_h'] + 2.0), yn - nt / 2, yn + nt / 2, 'XZ'))
    # raised U-shaped ribs round each nut slot (back bar over the screw hole + a side bar each side): the wall behind the slot is only ~6 mm thick
    # and has ~2 mm of material over the hole, so this carries the screw clamp load into the 28 mm wall
    hr, rw, bx = C['lock_rib_h'], C['lock_rib_w'], C['lock_rib_bx']
    y_back, y_fwd = yn - nt / 2, yn + nt / 2 + 4.0
    z0 = C['base_t'] - 0.5
    ribs = []
    for sx in (-1, 1):
        x = sx * C['lock_screw_x']
        ztop = C['base_t'] + hr
        d = ztop - z0                                                   # 45 deg ramps: the rib starts at base-top level and rises gradually
        ribs.append(prism('lk_rib_back', [(x - bx, z0), (x + bx, z0), (x + bx - d, ztop), (x - bx + d, ztop)], ymin, y_back, 'XZ'))
        for sg in (-1, 1):
            xa, xb = sorted((x + sg * nx, x + sg * (nx + rw)))
            ribs.append(prism('lk_rib_side', [(ymin, z0), (y_fwd, z0), (y_fwd - d, ztop), (ymin, ztop)], xa, xb, 'YZ'))
    # seat ribs: when the frame is tilted down flat it drops BETWEEN these two ribs (plate half width + gap), which centres it and locks slide + swivel
    pw = ms_plate_w2(C) / 2 + C['seat_rib_gap']
    sy0, sl, sh, sw_, sc = C['seat_rib_y0'], C['seat_rib_len'], C['seat_rib_h'], C['seat_rib_w'], C['seat_rib_ch']
    zs, ztop = C['base_t'] - 0.5, C['base_t'] + sh
    dd = ztop - zs
    for sg in (-1, 1):
        xin, xout = sg * pw, sg * (pw + sw_)
        xa, xb = min(xin, xout), max(xin, xout)
        r = prism('seat_rib', [(sy0, zs), (sy0 + sl, zs), (sy0 + sl - dd, ztop), (sy0 + dd, ztop)], xa, xb, 'YZ')    # 45 deg ramp at both ends
        # 45 deg lead-in on the inner top edge so the plate is funnelled to the middle
        tri = [(xin - sg * 1.0, ztop - sc - 1.0), (xin + sg * (sc + 1.0), ztop + 1.0), (xin - sg * 1.0, ztop + 1.0)]
        cut(r, prism('seat_lead', tri, sy0 - 1.0, sy0 + sl + 1.0, 'XZ'))
        ribs.append(r)
    union(b, *ribs)
    for x, y in hpts:
        cuts.append(prism('case_hole', circle(x, y, C['case_hole_d'] / 2, 32), -1, C['base_t'] + 1))
        cuts.append(prism('case_cb', circle(x, y, C['case_cb_d'] / 2, 32), C['base_t'] - C['case_cb_depth'], C['base_t'] + 1))
    cut(b, *cuts)
    b['case_holes'] = hpts
    return b


def make_lockblock(C):
    """Choke block: 48 x 10 x 6.4 mm bar with a flat face.  It slides in the tunnel; the disc's rim is tangent to the face
    at (slide position, -R) for every slide position, so a flat face clamps it anywhere along the slide."""
    R = C['tt_r'] + C['pocket_clear'] / 2
    yp, zc, w, h, t = -R, C['lock_z'], C['lock_block_len'], C['lock_block_h'], C['lock_block_t']
    return prism('lockblock', [(-w / 2, yp - t), (w / 2, yp - t), (w / 2, yp), (-w / 2, yp)], zc - h / 2, zc + h / 2)


def make_lockscrew(C):
    """Clearance dummy of the two knurled thumb screws (head + shank), tips just touching the block."""
    R = C['tt_r'] + C['pocket_clear'] / 2
    zc, L = C['lock_z'], C['lock_screw_len']
    tip = -R - C['lock_block_t'] - 0.2
    yh = tip - L
    out = None
    for sx in (-1, 1):
        x = sx * C['lock_screw_x']
        sh = prism('lockscrew', circle(x, zc, 3.0, 24, 2.0), yh, tip, 'XZ')
        hd = prism('lockhead', circle(x, zc, C['lock_head_d'] / 2, 64, 1.0), yh - C['lock_head_h'], yh, 'XZ')
        union(sh, hd)
        if out is None: out = sh
        else: union(out, sh)
    return out


def make_turntable2(C):
    T, Rl = C['tt_t'], C['lug_r']
    za = T + Rl
    tt = prism('turntable', circle(0, 0, C['tt_r'], 128), 0, T)
    ears = [l for l in hinge_layout(C) if l[0] == 'ear']
    mid = len(ears) // 2                                              # the ear over the swivel nut is left out: the nut drops in from above here, then make_spacer() fills the gap
    union(tt, *[prism('ear', ms._ear_profile(C, za), e[1], e[2], 'YZ') for i, e in enumerate(ears) if i != mid])
    xl, xr = ears[0][1], ears[-1][2]
    ha = ms.swivel_groove_half_angle(C)
    off = math.degrees(math.asin((C['stop_screw_d'] / 2) / C['stop_r']))
    a0, a1 = -(off + 2.5), C['tilt_max'] + off                       # static angles swept by the frame-mounted stop screw (0 = ahead, 90 = up)
    gd, r0, r1 = C['stop_groove_d'], C['stop_r'] - C['stop_groove_w'] / 2, C['stop_r'] + C['stop_groove_w'] / 2
    cut(tt,
        prism('bolt', circle(0, 0, C['m5_hole'] / 2, 48), -1, T + 1),
        prism('nutpocket', hexagon(0, 0, C['m5_nut_af'], 90), T - C['m5_nut_depth'], T + 1),
        prism('groove', sector(0, 0, C['swivel_stop_r'] - C['groove_w'] / 2, C['swivel_stop_r'] + C['groove_w'] / 2, -90 - ha, -90 + ha, 64), -1, C['groove_d']),
        prism('hinge', circle(0, za, C['bolt_hinge_d'] / 2, 40, 4.5), xl - 1, xr + 1, 'YZ'),
        prism('stopgrooveL', sector(0, za, r0, r1, a0, a1, 64), xl - 1, xl + gd, 'YZ'),
        prism('stopgrooveR', sector(0, za, r0, r1, a0, a1, 64), xr - gd, xr + 1, 'YZ'))
    return tt


def make_spacer(C):
    """Cylinder that takes the place of the removed middle turntable ear (on the M6 hinge bolt, between the two frame lugs next to it).
    Fitted AFTER the M5/M6 swivel nut is in its pocket; frame-local-to-turntable coords (z=0 = turntable underside)."""
    ears = [l for l in hinge_layout(C) if l[0] == 'ear']
    e = ears[len(ears) // 2]
    za, r = C['tt_t'] + C['lug_r'], C['lug_r'] - C['spacer_lift']
    sp = prism('spacer', circle(0, za, r, 96), e[1], e[2], 'YZ')
    cut(sp, prism('spacer_bore', circle(0, za, C['bolt_hinge_d'] / 2, 40), e[1] - 1, e[2] + 1, 'YZ'))
    return sp


def make_stop_screw2(C, side=1):
    """M3 stop screw, threaded into the outer frame lug, tip pokes into the arc groove of the first/last turntable ear (frame-local coords)."""
    lugs = [l for l in hinge_layout(C) if l[0] == 'lug']
    xo = lugs[-1][2] if side > 0 else lugs[0][1]
    sg = 1 if side > 0 else -1
    L = C['stop_screw_len']
    shaft = prism('stop_screw', circle(C['stop_r'], C['lug_r'], C['stop_screw_d'] / 2, 24), min(xo, xo - sg * L), max(xo, xo - sg * L), 'YZ')
    head = prism('head', circle(C['stop_r'], C['lug_r'], 2.85, 24), min(xo, xo + sg * 1.9), max(xo, xo + sg * 1.9), 'YZ')
    union(shaft, head)
    return shaft


def make_wingnut2(C):
    """Wing nut + washers on the RIGHT face of the stack (make_stand builds it on the left: mirror it about x=0)."""
    wn = ms.make_wingnut(C)
    wn.data.transform(Matrix.Scale(-1, 4, Vector((1, 0, 0))))
    bm = bmesh.new(); bm.from_mesh(wn.data)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(wn.data); bm.free(); wn.data.update()
    return wn


def make_monitor(C):
    """Collision dummy (not exported): control box + body, 0.2 mm off the plate so the mating face is not flagged."""
    pt, yv = C['plate_t'], C['vesa_yc']
    S = C['standoff']
    z0, z1, z2 = pt + S + 0.2, pt + S + C['box_h'], pt + S + C['box_h'] + C['body_t']
    def box(x0, x1, y0, y1, za, zb): return prism('m', [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], za, zb)
    mon = box(-C['box_w'] / 2, C['box_w'] / 2, yv + C['box_y0'], yv + C['box_y1'], z0, z1)
    union(mon, box(-C['body_w'] / 2, C['body_w'] / 2, yv - C['body_below'], yv + C['body_above'], z1 - 0.5, z2))
    mon.name = 'monitor'
    yb = yv + C['box_y0']                                  # connector face of the box (toward the hinge)
    plugs = box(-C['plug_x'], C['plug_x'], yb - C['plug_len'], yb + 0.5, pt + S + 1.0, pt + S + C['box_h'] - 1.0)
    plugs.name = 'plugs'
    return mon, plugs


def build_assembly2(C):
    D = frame_dims2(C)
    base, car, tt, fr = make_base2(C), ms.make_carriage(C), make_turntable2(C), make_frame2(C)
    shoe, lock = make_lockblock(C), make_lockscrew(C)
    sp = make_spacer(C)
    mon, plugs = make_monitor(C)
    screw, screwL, wn = make_stop_screw2(C, 1), make_stop_screw2(C, -1), make_wingnut2(C)
    for o in [o for o in bpy.data.objects if o.name.startswith('wing') and o.name != 'wingnut']:   # stray unused primitive made inside make_wingnut()
        bpy.data.objects.remove(o, do_unlink=True)
    sc = bpy.context.scene
    def empty(n):
        e = bpy.data.objects.new(n, None); sc.collection.objects.link(e); return e
    slide, swivel, tilt, lidj, cg = empty("SLIDE"), empty("SWIVEL"), empty("TILT"), empty("LID_HINGE"), empty("CG")
    swivel.parent = slide; tilt.parent = swivel
    F = C['floor_t']
    tilt.location = (0, 0, F + C['tt_t'] + C['lug_r'])
    tt.parent = swivel; tt.location = (0, 0, F)
    car.parent = slide; car.location = (0, 0, F - C['car_t'])
    sp.parent = swivel; sp.location = (0, 0, F)
    wn.parent = swivel; wn.location = (0, 0, F + C['tt_t'] + C['lug_r'])
    screw.parent = fr; screwL.parent = fr                  # stop screws ride on the frame
    fr.parent = tilt; fr.location = (0, 0, -C['lug_r'])
    mon.parent = fr
    plugs.parent = fr
    lidj.parent = fr
    cg.parent = fr
    cg.location = (0, C['vesa_yc'] + (C['body_above'] - C['body_below']) / 2, C['plate_t'] + C['standoff'] + C['box_h'] + C['body_t'] / 2)   # body centre
    c = slide.constraints.new('LIMIT_LOCATION'); c.use_min_x = c.use_max_x = True; c.owner_space = 'LOCAL'
    c.min_x, c.max_x = -C['slide_travel'], C['slide_travel']
    c = swivel.constraints.new('LIMIT_ROTATION'); c.use_limit_z = True; c.owner_space = 'LOCAL'
    c.min_z, c.max_z = -math.radians(C['swivel_max']), math.radians(C['swivel_max'])
    c = tilt.constraints.new('LIMIT_ROTATION'); c.use_limit_x = True; c.owner_space = 'LOCAL'
    c.min_x, c.max_x = 0, math.radians(C['tilt_max'])
    base.color = (0.18, 0.18, 0.2, 1); tt.color = (0.85, 0.35, 0.1, 1); fr.color = (0.15, 0.45, 0.8, 1)
    car.color = (0.2, 0.75, 0.3, 1); mon.color = (0.08, 0.08, 0.09, 1); plugs.color = (0.85, 0.15, 0.15, 1)
    screw.color = screwL.color = (0.8, 0.8, 0.8, 1); wn.color = (0.9, 0.8, 0.2, 1)
    sp.color = (0.95, 0.55, 0.2, 1); shoe.color = (0.9, 0.9, 0.2, 1); lock.color = (0.55, 0.55, 0.6, 1)
    return dict(sp=sp, base=base, car=car, tt=tt, fr=fr, mon=mon, plugs=plugs, shoe=shoe, lock=lock, screw=screw, screwL=screwL, wn=wn,
                slide=slide, swivel=swivel, tilt=tilt, lidj=lidj, cg=cg, D=D)


def pose(A, tilt_deg, swivel_deg, slide_mm=0.0):
    ms.pose(A, tilt_deg, swivel_deg, slide_mm, 0.0)


def run_checks2(A, C):
    tv, F = C['slide_travel'], C['floor_t']
    for k in ('slide', 'swivel', 'tilt'):
        for c in A[k].constraints: c.enabled = False
    ok = True
    print("\n=== interference check (triangle overlaps; 0 = clear) ===")
    pairs = [('sp', 'fr'), ('sp', 'tt'), ('sp', 'mon'), ('sp', 'wn'), ('sp', 'base'), ('fr', 'tt'), ('fr', 'base'), ('fr', 'car'), ('tt', 'screw'), ('tt', 'screwL'), ('base', 'screw'), ('base', 'screwL'),
             ('mon', 'tt'), ('mon', 'base'), ('mon', 'car'), ('mon', 'screw'), ('mon', 'screwL'), ('mon', 'wn'),
             ('sp', 'plugs'), ('lock', 'mon'), ('lock', 'plugs'), ('lock', 'fr'), ('lock', 'wn'), ('lock', 'tt'), ('lock', 'base'), ('shoe', 'tt'), ('shoe', 'base'), ('shoe', 'car'),
             ('plugs', 'fr'), ('plugs', 'tt'), ('plugs', 'base'), ('plugs', 'car'), ('plugs', 'screw'), ('plugs', 'screwL'), ('plugs', 'wn')]
    base_poly = (-C['base_l'] / 2, C['base_l'] / 2, A['base']['y_min'], A['base']['y_max'])
    worst_margin, worst_at = 1e9, None
    min_clear, min_at = 1e9, None
    for dx in (-tv, 0, tv):
        for sw in (-C['swivel_max'], 0, C['swivel_max']):
            line, bad_any = [], False
            ths = (0.3, 10, 20, 30, 45, 60, 75, 85, C['tilt_max']) if (dx == 0 and sw == 0) else (20, 30, 45, 60, 75, 85, C['tilt_max'])
            for th in ths:
                pose(A, th, sw, dx)
                bad = {f"{a}-{b}": n for a, b in pairs if (n := hits(A[a], A[b]))}
                bad_any |= bool(bad); ok &= not bad
                if bad: line.append(f"tilt {th}: {bad}")
                p = A['cg'].matrix_world.translation                  # tipping: CG projection vs base footprint
                m = min(p.x - base_poly[0], base_poly[1] - p.x, p.y - base_poly[2], base_poly[3] - p.y)
                if m < worst_margin: worst_margin, worst_at = m, (dx, sw, th, round(p.x, 1), round(p.y, 1), round(p.z, 1))
                zmin = min((A['mon'].matrix_world @ v.co).z for v in A['mon'].data.vertices) - C['base_t']
                if zmin < min_clear: min_clear, min_at = zmin, (dx, sw, th)
            print(f"slide {dx:+5.1f}  swivel {sw:+5.1f}  tilt 0.3..45 ->", "OK" if not bad_any else "COLLISION " + "; ".join(line))
    print("\nseat ribs: lowest tilt (deg) at which the frame clears the base ribs; the plate lies flat only when centred between them:")
    def lowest_clear(dx, sw):
        for k in range(1, 120):
            th = k * 0.5
            pose(A, th, sw, dx)
            if not hits(A['fr'], A['base']):
                return th
        return None
    for dx in (-tv, -10, -2, 0, 2, 10, tv):
        cells = [(sw, lowest_clear(dx, sw)) for sw in (-C['swivel_max'], -10, -2, 0, 2, 10, C['swivel_max'])]
        print(f"  slide {dx:+5.1f}: " + "  ".join(f"sw{sw:+3.0f}:{'--' if lo is None else f'{lo:.1f}'}" for sw, lo in cells))
    print(f"\nlowest point of the monitor above the base top over all poses: {min_clear:.1f} mm  (slide/swivel/tilt = {min_at})")
    print(f"tipping: monitor CG stays inside the base footprint by at least {worst_margin:.1f} mm  "
          f"(worst pose slide/swivel/tilt = {worst_at[:3]}, CG xyz = {worst_at[3:]})")
    ok &= worst_margin > 10 and min_clear > 2

    print("\n=== 45 deg tilt stop test ===")
    first = None
    for th in [x * 0.5 for x in range(-12, 2 * int(C['tilt_max']) + 20)]:
        pose(A, th, 0)
        h = hits(A['tt'], A['screw']) + hits(A['tt'], A['screwL'])
        if h and first is None and th > 20: first = th
        if th in (-2.0, 0.0, C['tilt_max'], C['tilt_max'] + 1.0): print(f"tilt {th:5.1f}: screw/frame overlaps = {h}")
    print(f"first tilt angle (above 20 deg) where a screw hits its groove wall: {first}")

    print("\n=== swivel stop test ===")
    tp = prism('test_post', circle(0, -C['swivel_stop_r'], C['post_d'] / 2, 32, 3.5), F, F + C['post_h'])
    tp.parent = A['slide']
    for dx in (-tv, 0, tv):
        res = []
        for sw in (-47, -45, 0, 45, 47):
            pose(A, 0, sw, dx); res.append(f"{sw:+d}deg:{hits(tp, A['tt']):3d}")
        print(f"slide {dx:+5.1f}:  " + "  ".join(res) + "   (expect 0 up to +-45, >0 at +-47)")
    bpy.data.objects.remove(tp, do_unlink=True)

    print("\n=== M6 star knob D25 (RIGHT of the hinge): clearance at every wing angle ===")
    worst = {}
    for dx in (-tv, 0, tv):
        for sw, th in ((0, 0.3), (0, 20), (0, C['tilt_max']), (-C['swivel_max'], C['tilt_max']), (C['swivel_max'], C['tilt_max']), (C['swivel_max'], 0.3), (-C['swivel_max'], 0.3)):
            pose(A, th, sw, dx)
            for ang in range(0, 180, 10):
                A['wn'].rotation_euler = (math.radians(ang), 0, 0); bpy.context.view_layer.update()
                for k in ('fr', 'tt', 'base', 'car', 'screw', 'screwL', 'mon', 'plugs'):
                    if hits(A['wn'], A[k]): worst.setdefault(k, []).append((dx, sw, th, ang))
    A['wn'].rotation_euler = (0, 0, 0)
    if not worst: print("wing nut clear of everything at every angle tested")
    for k, v in worst.items():
        ok = False; print(f"wing nut touches '{k}' in {len(v)} cases, e.g. {v[:3]}")

    print("\n=== swivel + slide lock (2 thumb screws -> choke block -> turntable rim) ===")
    for dx in (-tv, -tv / 2, 0, tv / 2, tv):
        res = []
        for sw in (-45, 0, 45):
            pose(A, 0, sw, dx)
            A['shoe'].location = (0, 0, 0); bpy.context.view_layer.update()
            rest = hits(A['shoe'], A['tt'])
            A['shoe'].location = (0, 0.6, 0); bpy.context.view_layer.update()
            clamp = hits(A['shoe'], A['tt'])
            A['shoe'].location = (0, 2.0, 0); bpy.context.view_layer.update()
            res.append((rest, clamp, hits(A['shoe'], A['base'])))
        A['shoe'].location = (0, 0, 0)
        print(f"slide {dx:+5.1f}: (rest-touch, clamped-touch, block/base at 2 mm) at swivel -45/0/+45 = {res}   (expect rest 0, clamped >0, base 0)")
        ok &= all(r[0] == 0 and r[1] > 0 and r[2] == 0 for r in res)
    pose(A, 0, 0, 0)
    N, mu, R_ = 2 * 150.0, 0.3, C['tt_r'] / 1000.0
    print(f"lock holding force estimate: 2 screws x 150 N -> slide hold ~{2 * mu * N:.0f} N, swivel hold ~{2 * mu * N * R_:.1f} N*m (mu={mu}, block + front wall contact)")
    print("\n=== slide tests ===")
    A['tt'].location = (0, 0, F + 0.3); A['car'].location = (0, 0, F - C['car_t'] + 0.3)
    bolt = prism('test_bolt', circle(0, 0, 2.5, 24, 3.5), 0, C['base_t'] + 5); bolt.parent = A['slide']
    for dx in (-tv - 0.5, -tv, 0, tv, tv + 0.5):
        res = []
        for sw in (-45, 0, 45):
            pose(A, 0, sw, dx); res.append(f"sw{sw:+d}: turntable/base {hits(A['base'], A['tt']):3d}")
        pose(A, 0, 0, dx)
        print(f"slide {dx:+5.1f}:  " + "   ".join(res) + f"   carriage/base {hits(A['base'], A['car']):3d}   bolt/slot {hits(A['base'], bolt):3d}")
    bpy.data.objects.remove(bolt, do_unlink=True)
    A['tt'].location = (0, 0, F); A['car'].location = (0, 0, F - C['car_t'])
    pose(A, 0, 0)

    print("\n=== static estimates (assumptions, not measurements) ===")
    W = C['mon_mass'] * 9.81
    cgp = A['cg'].location
    zc = C['lug_r']
    for th in (0, 20, 45, 70, 90):
        t = math.radians(th)
        arm = cgp.y * math.cos(t) - (cgp.z - zc) * math.sin(t)
        print(f"tilt {th:2d} deg: monitor weight {W:.1f} N, CG {arm:5.1f} mm from the hinge axis -> torque {W * arm / 1000:.2f} N*m")
    r_f, n_f, mu = 8.0, 2 * (C['n_lugs'] - 1) - 2, 0.3   # the loose spacer turns with the frame lugs: its 2 faces give no friction
    for N in (100, 200):
        print(f"hinge friction if the wing nut gives {N} N clamp: ~{mu * N * r_f * n_f / 1000:.1f} N*m  (mu={mu}, {n_f} faces, r={r_f} mm)")
    for k in ('slide', 'swivel', 'tilt'):
        for c in A[k].constraints: c.enabled = True
    return ok


def bom2(C):
    lay = hinge_layout(C)
    xl, xr = lay[0][1], lay[-1][2]
    bear = xl + C['m4_nut_depth']                              # underside of the hex head seats here (left outer lug)
    need = (xr - bear) + C['washer_t'] + C['wn_m']             # wn_m = minimum thread engagement in the knob (through-thread knob: the bolt may protrude)
    L = math.ceil(need / 5) * 5
    eng = L - ((xr - bear) + C['washer_t'])
    print("\n=== hardware ===")
    print(f"tilt hinge: M6x{L} hex-head bolt (head in the pocket of the LEFT outer frame lug) + 2 M6 washers + M6 star knob D25 (female thread) on the RIGHT face (min {need:.1f} mm; the {L} gives {eng:.1f} mm of engagement; the knob is a through-thread type so M6x130 also works: tip pokes ~3 mm past the knob)")
    print("VESA plate: 4x M4 socket-head screws, heads sunk in the back of the plate "
          f"({C['vesa_head_d']:.1f} dia x {C['vesa_head_depth']:.1f} deep, {C['plate_t'] - C['vesa_head_depth']:.1f} mm floor); "
          f"use M4x{math.ceil((C['plate_t'] - C['vesa_head_depth'] + C['standoff'] + 8) / 5) * 5} (head sunk in the plate, through the rail, 8 mm into the box) and check how deep the box threads are (plate floor {C['plate_t'] - C['vesa_head_depth']:.1f} mm + thread depth)")
    print(f"lock: 2x M6x{C['lock_screw_len']:.0f} knurled thumb screw + 2x M6 hex nut (drop into the open-top slots in the back wall) + 1 printed choke block (..._lockblock.stl)")
    print(f"swivel: M6x14 hex bolt + captured M6 nut (10 mm AF), nut dropped in from above through the gap of the removed middle ear, then 1 printed spacer cylinder (..._spacer.stl) on the M6 hinge bolt; 2x M3x10 tilt-stop screws (frame outer lugs); 4 rubber feet")
    print(f"base {C['base_l']:.0f} wide, turntable dia {2 * C['tt_r']:.0f}, slide +-{C['slide_travel']:.0f} mm")


def render2(A, C, out):
    sc = bpy.context.scene
    sc.render.engine = 'BLENDER_WORKBENCH'
    sc.render.resolution_x, sc.render.resolution_y = 1400, 900
    sc.display.shading.light = 'STUDIO'; sc.display.shading.color_type = 'OBJECT'; sc.display.shading.show_cavity = True
    sc.world = bpy.data.worlds.new("w"); sc.world.color = (0.93, 0.93, 0.95)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam")); sc.collection.objects.link(cam)
    cam.data.type = 'ORTHO'
    tgt = bpy.data.objects.new("tgt", None); sc.collection.objects.link(tgt)
    t = cam.constraints.new('TRACK_TO'); t.target = tgt; t.track_axis = 'TRACK_NEGATIVE_Z'; t.up_axis = 'UP_Y'
    sc.camera = cam
    def shot(name, th, sw, dx, loc, target, scale):
        pose(A, th, sw, dx)
        cam.data.ortho_scale = scale; cam.location = loc; tgt.location = target
        sc.render.filepath = os.path.join(out, f"preview_{name}.png")
        bpy.ops.render.render(write_still=True)
    shot("flat", 0, 0, 0, (470, -420, 360), (0, 50, 25), 560)
    shot("tilt90", 90, 0, 0, (470, -420, 360), (0, 0, 80), 560)
    shot("tilt90_side", 90, 0, 0, (600, 30, 120), (0, 20, 90), 420)
    shot("lock_back", 0, 0, 0, (260, -420, 200), (0, -80, 14), 220)
    shot("tilt45", 45, 0, 0, (470, -420, 360), (0, 30, 50), 560)
    shot("tilt45_swivel30_slide", 45, 30, 15, (470, -420, 360), (0, 30, 50), 560)
    shot("tilt20_back", 20, -20, 0, (-300, 450, 260), (0, 20, 50), 560)
    A['mon'].hide_render = True; A['plugs'].hide_render = True
    shot("no_monitor_tilt30", 30, 0, 0, (260, -300, 220), (0, 40, 30), 260)
    for k in ('base', 'tt', 'car', 'screw', 'screwL', 'wn'): A[k].hide_render = True
    shot("frame_back", 0, 0, 0, (60, 60, -380), (0, 55, 3), 190)
    shot("frame_front", 0, 0, 0, (60, -100, 380), (0, 55, 3), 190)
    for k in ('base', 'tt', 'car', 'screw', 'screwL', 'wn', 'mon', 'plugs'): A[k].hide_render = False


def main(C, out):
    os.makedirs(out, exist_ok=True)
    reset_scene()
    A = build_assembly2(C)
    parts = (('base', 'base'), ('car', 'carriage'), ('tt', 'turntable'), ('fr', 'frame'), ('shoe', 'lockblock'), ('sp', 'spacer'))
    for key, nm in parts: print(f"{nm:10s} non-manifold edges: {nonmanifold(A[key])}")
    b = A['base']
    print(f"base footprint {C['base_l']:.0f} x {b['len']:.1f} mm (y {b['y_min']:.1f} .. {b['y_max']:.1f}); plate {A['D']['plate_w']:.0f} x {A['D']['plate_h']:.0f} x {C['plate_t']:.0f} mm")
    if C['cavity']:
        print(f"base volume {b['vol_solid']:.0f} cm3 solid -> {b['vol_final']:.0f} cm3 with {b['cells']} underside pockets")
    stress_report(C)
    ok = run_checks2(A, C)
    pose(A, 0, 0)
    bom2(C)
    for key, nm in parts:
        export_stl(A[key], os.path.join(out, f"{C['name']}_{nm}.stl"))
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out, f"{C['name']}_assembly.blend"))
    render2(A, C, out)
    print("\nALL CLEAR" if ok else "\nCHECK ABOVE")


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    out = argv[argv.index("--out") + 1] if "--out" in argv else os.path.join(os.path.dirname(os.path.abspath(__file__)), "out2")
    main(CFG2, out)
