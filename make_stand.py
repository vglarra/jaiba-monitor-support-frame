"""
Parametric 2-axis display stand (tilt 0..45 deg, swivel +-45 deg) for 3D printing in ASA.

Run inside Blender:   blender -b -P make_stand.py -- --out ./out
or with the pip module: python make_stand.py --out ./out

Units: 1 Blender unit = 1 mm (STL files are exported in mm).

Parts (each is exported lying in its print orientation, nothing needs supports
except the bolt holes, which are small):
  base       - 150 x 144 mm block with a stadium-shaped recess; the turntable sits SUNK in it so its top
               is flush with the base top (frame lies flat on the base) and can slide +-15 mm left/right
  carriage   - small plate in a channel under the turntable: slides with the bolt, cannot rotate, carries
               the swivel stop post (so the +-45 deg swivel limit works at every slide position)
  turntable  - round, rotates on the base, carries the hinge ears
  lid        - foldable screen cap, hinged on the top edge of the frame with three M3 screws; opens 180 deg to a flat
               back surface (flush with the frame back) and keeps rotating when the screen is tilted
  frame      - PCB tray with 4 M3 standoffs, hinge lug with 45 deg stop groove

Hardware (per stand):
  1x M5 bolt + 1x M5 nut      swivel axis (nut is captured in the turntable)
  1x M4 bolt + 1x M4 nut      tilt hinge  (+ 1 washer, nyloc/lock nut recommended)
  2x M3x8 button-head screw   45 deg tilt stops, left+right outer ears (thread-forming)
  4x M3 socket-cap screws M3x14..16 + 4x M3 nuts   PCB mounting: head hidden in a counterbore on
                              the BACK of the frame, nut on the PCB front (frame stays flat)
  3x M3x16 screw + 3x M3 (nyloc) nut   cap hinge pins (left, middle, right)
  4x self-adhesive rubber feet
"""
import bpy, bmesh, math, os, sys
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

# --------------------------------------------------------------------------
# CONFIG - change these for the second monitor
# --------------------------------------------------------------------------
CFG_MONITOR1 = dict(
    name="mon1_7in_v14",
    # PCB (landscape: length along X, width along Y)
    pcb_l=181.0, pcb_w=104.0, pcb_t=1.6,
    hole_d=3.0, hole_off=3.5,          # hole centre is hole_off from each PCB edge
    glass_l=165.0, glass_w=100.0, glass_t=5.0,   # display stack height above the PCB front: MEASURE IT (sets cap clearance)
    pcb_clear=0.5,                      # gap between PCB edge and corner guards
    # frame
    plate_t=5.0, standoff_h=8.0, standoff_d=9.0,
    screw_clear_d=3.4, screw_head_d=5.0, screw_head_h=3.0,   # M3 socket-cap screw inserted from the BACK
    cb_clear=1.2, cb_extra=0.4, nut_h=2.4,                     # counterbore = head + clearance
    guard_t=2.4, guard_len=16.0, margin_x=6.5, margin_y=4.5,   # plate overhang past guards
    window_l=150.0, window_w=66.0,      # open back window for components / cables
    # hinge
    lug_r=11.6, lug_clear=0.6, lug_w=6.0, n_lugs=5, ear_gap=0.5, ear_t=5.0, ear_base=15.0,   # hinge stack (65 mm) sits clear of the corner supports
    frame_edge_margin=2.0,             # frame left edge stays this far inside the base's left border even at full left slide
    bolt_hinge_d=5.4, stop_r=7.8, stop_groove_w=3.6, stop_groove_d=4.2,
    stop_screw_d=3.0, stop_hole_d=2.6,
    tilt_max=45.0,
    # swivel
    base_l=150.0, base_corner=10.0,                    # base width (X, slide direction); its depth (Y) is computed from the frame
    base_back_wall=14.0, base_front_margin=3.0,        # wall behind the turntable recess / overhang past the frame top edge
    cavity=True, cav_wall=4.0, cav_rib=3.0, cav_cell=21.0, cav_deck=4.0, cav_pocket_top=4.0,   # hollow underside (rib grid)
    floor_t=9.0, tt_r=50.0, tt_t=10.0, base_t=19.0,    # turntable is SUNK into the base: base_t = floor_t + tt_t
    slide_travel=15.0, pocket_clear=0.4,               # turntable slides +-travel along X in a stadium recess
    car_x=20.0, car_y0=-18.0, car_y1=10.0, car_t=3.0, car_clear=0.2,   # sliding stop carriage (rides in a channel)
    m5_hole=5.4, m5_cb_d=10.6, m5_cb_depth=3.6, m5_nut_af=8.4, m5_nut_depth=5.0,
    m4_head_d=9.4, m4_head_depth=2.6, m4_nut_af=8.4, m4_nut_depth=4.0,   # M5 hex HEAD (8 AF) sits in the pocket on the right ear; wing nut on the left
    wn_span=21.0, wn_h=13.0, wn_boss_d=10.0, wn_boss_h=4.0, wn_wing_t=2.0, washer_t=2.0, washer_d=9.6, wn_m=4.7,   # M5 wing nut (DIN 315, ~21 mm span) + 2 stacked M5 washers (clear the left stop-screw head)
    swivel_stop_r=15.0, post_d=3.6, post_h=2.8, groove_w=4.4, groove_d=3.2,
    swivel_max=45.0,
    y0=8.5,                              # frame plate starts this far in front of hinge axis
    # foldable screen cap (lid): 3 hinge stations (left, MIDDLE, right) at the top edge of the frame, one M3 screw each
    lid_t=3.0, lid_clear=0.8, lid_hinge_x=78.0, lid_r=6.0, lid_ear_w=6.0, lid_lug_w=6.0, lid_gap=0.5,
    lid_axis_off=12.0, lid_edge_gap=5.5, lid_hole_d=3.4, lid_head_d=5.6, lid_head_depth=3.2,
    lid_nut_af=5.9, lid_nut_depth=2.6, lid_notch_r=9.0, lid_max=270.0,
    # snap latches: spring hooks hang from the lid sides; the nub on each hook clicks into a groove in the frame plate edge
    lid_rib_d=3.0, lid_rib_t=1.6, lid_rib_h=5.0,      # gusset webs on the outside of each lid hinge ear (depth outward, thickness, max height)
    snap_rib_h=6.0, snap_rib_d=3.0, snap_rib_t=1.6,   # outer gusset ribs on each hook: height down the hook, depth outward, thickness
    snap_y=(32.0, 80.0), snap_w=14.0, snap_t=1.8, snap_gap=0.3, snap_bite=0.9,   # hook y (from frame front edge), width, wall, gap to plate, nub protrusion
    snap_z0=1.2, snap_z1=2.2, snap_z2=3.0, snap_z3=4.0, groove_z0=1.5, groove_z1=3.8, snap_groove_d=0.8,   # nub profile / groove heights above the frame back plane
)

# --------------------------------------------------------------------------
# small geometry toolkit (bmesh + boolean modifiers)
# --------------------------------------------------------------------------
def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = 'METRIC'
    sc.unit_settings.scale_length = 0.001
    sc.unit_settings.length_unit = 'MILLIMETERS'


def _obj_from_bm(name, bm):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def prism(name, pts, d0, d1, plane='XY'):
    """Extrude polygon pts [(u,v)] between d0..d1.
    plane XY -> extrude along Z, YZ -> along X, XZ -> along Y."""
    def P(u, v, d):
        return {'XY': (u, v, d), 'YZ': (d, u, v), 'XZ': (u, d, v)}[plane]
    bm = bmesh.new()
    a = [bm.verts.new(P(u, v, d0)) for u, v in pts]
    b = [bm.verts.new(P(u, v, d1)) for u, v in pts]
    n = len(pts)
    bm.faces.new(a)
    bm.faces.new(b)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((a[i], a[j], b[j], b[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _obj_from_bm(name, bm)


def circle(cx, cy, r, n=64, a0=0.0):
    return [(cx + r * math.cos(math.radians(a0) + 2 * math.pi * i / n),
             cy + r * math.sin(math.radians(a0) + 2 * math.pi * i / n)) for i in range(n)]


def hexagon(cx, cy, af, a0=90.0):
    return circle(cx, cy, af / math.sqrt(3), 6, a0)


def rrect(cx, cy, w, h, r, n=10):
    pts = []
    for sx, sy, a in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
        ccx, ccy = cx + sx * (w / 2 - r), cy + sy * (h / 2 - r)
        for i in range(n + 1):
            t = math.radians(a + 90 * i / n)
            pts.append((ccx + r * math.cos(t), ccy + r * math.sin(t)))
    return pts


def sector(cx, cy, r0, r1, a0, a1, n=48):
    out = [(cx + r1 * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
            cy + r1 * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]
    inn = [(cx + r0 * math.cos(math.radians(a1 + (a0 - a1) * i / n)),
            cy + r0 * math.sin(math.radians(a1 + (a0 - a1) * i / n))) for i in range(n + 1)]
    return out + inn


def hull(pts):
    pts = sorted(set((round(x, 6), round(y, 6)) for x, y in pts))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def _bool(a, b, op):
    bpy.context.view_layer.objects.active = a
    m = a.modifiers.new("B", 'BOOLEAN')
    m.operation = op
    m.solver = 'EXACT'
    m.object = b
    bpy.ops.object.modifier_apply(modifier=m.name)
    bpy.data.objects.remove(b, do_unlink=True)


def union(a, *bs):
    for b in bs: _bool(a, b, 'UNION')
    return a


def cut(a, *bs):
    for b in bs: _bool(a, b, 'DIFFERENCE')
    return a


def mirror_x(pts_obj_fn, *args):
    return pts_obj_fn(*args)

# --------------------------------------------------------------------------
# PARTS
# --------------------------------------------------------------------------
def swivel_groove_half_angle(C):
    return C['swivel_max'] + math.degrees(math.asin((C['post_d'] / 2) / C['swivel_stop_r']))


def prisms(name, items):
    """Several disjoint XY-plane prisms [(pts, d0, d1)] in one mesh (one boolean instead of many)."""
    bm = bmesh.new()
    for pts, d0, d1 in items:
        a = [bm.verts.new((u, v, d0)) for u, v in pts]
        b = [bm.verts.new((u, v, d1)) for u, v in pts]
        bm.faces.new(a); bm.faces.new(b)
        for i in range(len(pts)):
            j = (i + 1) % len(pts)
            bm.faces.new((a[i], a[j], b[j], b[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _obj_from_bm(name, bm)


def frame_dims(C):
    """Shared frame / lid / base dimensions (frame-local coords: back plane z=0, hinge axis at y=0)."""
    px, py = C['pcb_l'] / 2 + C['pcb_clear'], C['pcb_w'] / 2 + C['pcb_clear']
    plate_w = 2 * (px + C['guard_t'] + C['margin_x'] - 6.0)
    plate_h = 2 * (py + C['guard_t'] + C['margin_y'])
    yc = C['y0'] + plate_h / 2
    top = C['plate_t'] + C['standoff_h']
    z_c0 = top + C['pcb_t'] + C['glass_t'] + C['lid_clear']     # lid underside when closed
    z_c1 = z_c0 + C['lid_t']
    return dict(plate_w=plate_w, plate_h=plate_h, yc=yc, top=top, y_top=yc + plate_h / 2,
                z_c0=z_c0, z_c1=z_c1, zh=z_c1 / 2,             # zh: lid axis height -> open lid is flush with frame back
                y_la=yc + plate_h / 2 + C['lid_axis_off'],
                xs=-(C['base_l'] / 2 - C['slide_travel'] - C['frame_edge_margin']) + plate_w / 2)   # frame centre x (hinge stays at x=0)


def lid_xs(C):
    return [-C['lid_hinge_x'], 0.0, C['lid_hinge_x']]       # cap hinge stations relative to the frame centre


def stadium(r, half_len, cx=0.0, cy=0.0, n=48):
    """Slot / stadium outline: two semicircles of radius r whose centres are 2*half_len apart along X."""
    pts = [(cx + half_len + r * math.cos(math.radians(-90 + 180 * i / n)),
            cy + r * math.sin(math.radians(-90 + 180 * i / n))) for i in range(n + 1)]
    pts += [(cx - half_len + r * math.cos(math.radians(90 + 180 * i / n)),
             cy + r * math.sin(math.radians(90 + 180 * i / n))) for i in range(n + 1)]
    return pts


def _overlap(a, b):
    return a[0] < b[1] and b[0] < a[1] and a[2] < b[3] and b[2] < a[3]


def make_base(C):
    D = frame_dims(C)
    T, F, tv = C['base_t'], C['floor_t'], C['slide_travel']
    assert abs(T - (F + C['tt_t'])) < 1e-6, "base_t must equal floor_t + tt_t (turntable top flush with base top)"
    R = C['tt_r'] + C['pocket_clear'] / 2
    y_min = -(R + C['base_back_wall'])
    y_max = D['y_top'] + C['base_front_margin']            # base reaches the top edge of the frame
    L, ybc, bl = y_max - y_min, (y_max + y_min) / 2, C['base_l']
    b = prism('base', rrect(0, ybc, bl, L, C['base_corner'], 12), 0, T)
    car_hx = C['car_x'] + tv + 1.5 + C['car_clear']
    cc = C['car_clear']
    cut(b,
        prism('pocket', stadium(R, tv, n=64), F, T + 1),                                  # stadium recess for the turntable
        prism('channel', [(-car_hx, C['car_y0'] - cc), (car_hx, C['car_y0'] - cc),
                          (car_hx, C['car_y1'] + cc), (-car_hx, C['car_y1'] + cc)],
              F - C['car_t'], F + 1),                                                      # channel for the stop carriage
        prism('slot', stadium(C['m5_hole'] / 2, tv + 1.0, n=24), -1, F + 1),               # M5 bolt slot
        prism('hslot', stadium(C['m5_cb_d'] / 2, tv + 1.0, n=24), -1, C['m5_cb_depth']))   # bolt-head slot underneath
    fx, fy = C.get('foot_x', 58.0), L / 2 - 16.0
    feet = [(sx * fx, ybc + sy * fy) for sx in (1, -1) for sy in (1, -1)]
    for fxx, fyy in feet:                                                                  # recesses for adhesive feet
        cut(b, prism('foot', circle(fxx, fyy, 7.2, 40), -1, 0.8))
    b['y_min'], b['y_max'], b['len'] = y_min, y_max, L
    b['vol_solid'] = _volume(b)
    if C['cavity']:
        w, rb = C['cav_wall'], C['cav_rib']
        gx0, gx1, gy0, gy1 = -bl / 2 + w, bl / 2 - w, y_min + w, y_max - w
        nx = max(1, round((gx1 - gx0 + rb) / (C['cav_cell'] + rb)))
        ny = max(1, round((gy1 - gy0 + rb) / (C['cav_cell'] + rb)))
        cw, ch = ((gx1 - gx0) - (nx - 1) * rb) / nx, ((gy1 - gy0) - (ny - 1) * rb) / ny
        pocket_bb = (-(tv + R), tv + R, -R, R)
        slot_bb = (-(tv + 1 + C['m5_cb_d'] / 2 + 1.0), tv + 1 + C['m5_cb_d'] / 2 + 1.0, -(C['m5_cb_d'] / 2 + 1.0), C['m5_cb_d'] / 2 + 1.0)
        items, kept = [], 0
        for i in range(nx):
            for j in range(ny):
                cx0, cy0 = gx0 + i * (cw + rb), gy0 + j * (ch + rb)
                bb = (cx0, cx0 + cw, cy0, cy0 + ch)
                if _overlap(bb, slot_bb): continue                                           # keep the bolt-slot area solid
                if any(_overlap(bb, k) for k in C.get('keep_solid', ())): continue          # caller-defined solid regions (lock screw)
                if any(_overlap(bb, (fxx - 8.7, fxx + 8.7, fyy - 8.7, fyy + 8.7)) for fxx, fyy in feet): continue
                top = C['cav_pocket_top'] if _overlap(bb, pocket_bb) else T - C['cav_deck']
                items.append((rrect(cx0 + cw / 2, cy0 + ch / 2, cw, ch, 3.0, 4), -1, top))
        cut(b, prisms('cavity', items))
        b['cells'] = len(items)
    b['vol_final'] = _volume(b)
    return b


def _volume(ob):
    bm = bmesh.new(); bm.from_mesh(ob.data)
    v = bm.calc_volume(); bm.free()
    return abs(v) / 1000.0        # cm^3


def make_carriage(C):
    ct = C['car_t']
    c = prism('carriage', rrect(0, (C['car_y0'] + C['car_y1']) / 2, 2 * C['car_x'], C['car_y1'] - C['car_y0'], 3.0, 6), 0, ct)
    post = prism('post', circle(0, -C['swivel_stop_r'], C['post_d'] / 2, 32, 3.5), ct - 0.5, ct + C['post_h'])
    union(c, post)
    cut(c, prism('bolt', circle(0, 0, C['m5_hole'] / 2, 48, 2.5), -1, ct + 1))
    return c


def _ear_profile(C, za):
    return hull(circle(0, za, C['lug_r'], 64) + [(-C['ear_base'], C['tt_t'] - 1), (C['ear_base'], C['tt_t'] - 1)])


def hinge_layout(C):
    """[(kind, x0, x1)] left->right: ear, lug, ear, ..., ear  (n_lugs lugs, n_lugs+1 ears)."""
    seq = []
    for _ in range(C['n_lugs']):
        seq += [('ear', C['ear_t']), ('lug', C['lug_w'])]
    seq.append(('ear', C['ear_t']))
    total = sum(w for _, w in seq) + C['ear_gap'] * (len(seq) - 1)
    x, out = -total / 2, []
    for k, w in seq:
        out.append((k, x, x + w)); x += w + C['ear_gap']
    return out


def make_turntable(C):
    T, Rl = C['tt_t'], C['lug_r']
    za = T + Rl                                              # hinge axis height (local)
    tt = prism('turntable', circle(0, 0, C['tt_r'], 128), 0, T)
    ears = [l for l in hinge_layout(C) if l[0] == 'ear']
    prof = _ear_profile(C, za)
    union(tt, *[prism('ear', prof, e[1], e[2], 'YZ') for e in ears])
    xl, xr, xi = ears[0][1], ears[-1][2], ears[-1][1]
    # --- swivel features
    ha = swivel_groove_half_angle(C)
    cut(tt,
        prism('bolt', circle(0, 0, C['m5_hole'] / 2, 48), -1, T + 1),
        prism('nutpocket', hexagon(0, 0, C['m5_nut_af'], 0), T - C['m5_nut_depth'], T + 1),
        prism('groove', sector(0, 0, C['swivel_stop_r'] - C['groove_w'] / 2, C['swivel_stop_r'] + C['groove_w'] / 2,
                               -90 - ha, -90 + ha, 64), -1, C['groove_d']))
    # --- hinge features
    cut(tt,
        prism('hinge', circle(0, za, C['bolt_hinge_d'] / 2, 40, 4.5), xl - 1, xr + 1, 'YZ'),
        prism('nutR', hexagon(0, za, C['m4_nut_af'], 90), xr - C['m4_nut_depth'], xr + 1, 'YZ'),
        prism('stophole', circle(0, za + C['stop_r'], C['stop_hole_d'] / 2, 32, 5.5), xi - 1, xr + 1, 'YZ'),
        prism('stopholeL', circle(0, za + C['stop_r'], C['stop_hole_d'] / 2, 32, 5.5), xl - 1, ears[0][2] + 1, 'YZ'))
    return tt


def groove_angles(C):
    off = math.degrees(math.asin((C['stop_screw_d'] / 2) / C['stop_r']))
    a_lo = 90 - C['tilt_max'] - off                          # wall that stops the 45 deg end
    a_hi = 90 + off + 2.5                                    # 2.5 deg slack: the platform defines 0 deg
    return a_lo, a_hi


def make_frame(C):
    Rl, pt = C['lug_r'], C['plate_t']
    hx = C['pcb_l'] / 2 - C['hole_off']
    hy = C['pcb_w'] / 2 - C['hole_off']
    px, py = C['pcb_l'] / 2 + C['pcb_clear'], C['pcb_w'] / 2 + C['pcb_clear']   # pocket half sizes
    D = frame_dims(C)
    plate_w, plate_h, yc, top = D['plate_w'], D['plate_h'], D['yc'], D['top']
    fr = prism('frame', rrect(0, yc, plate_w, plate_h, 6.0), 0, pt)
    # notches that give the hinge ears room (cut first, lug is added afterwards)
    xs = D['xs']
    lay = [(k, a - xs, b - xs) for k, a, b in hinge_layout(C)]   # hinge sits at x=0 in world: -xs in frame-centred coords
    ny = 16.5
    notches = []
    for i, (k, x0, x1) in enumerate(lay):
        if k != 'ear': continue
        a = lay[i - 1][2] if i > 0 else x0 - 0.9           # from previous lug edge
        b = lay[i + 1][1] if i < len(lay) - 1 else x1 + 0.9  # to next lug edge
        notches.append(prism('notch', [(a, C['y0'] - 2), (b, C['y0'] - 2), (b, ny), (a, ny)], -1, pt + 1))
    for x0 in lid_xs(C):                                     # relief for the cap hinge arms
        lw = C['lid_lug_w']
        ear_R = math.hypot(C['lid_edge_gap'] + 4.0, D['z_c1'] - D['zh'])
        reach = max(2.0, ear_R + 0.8 - C['lid_axis_off'])
        for a, b in ((x0 - lw / 2 - C['lid_gap'] - C['lid_ear_w'] - 0.9, x0 - lw / 2),
                     (x0 + lw / 2, x0 + lw / 2 + C['lid_gap'] + C['lid_ear_w'] + 0.9)):
            notches.append(prism('lidnotch', [(a, D['y_top'] - reach), (b, D['y_top'] - reach), (b, D['y_top'] + 2), (a, D['y_top'] + 2)], -1, pt + 1))
    cut(fr, *notches, prism('window', rrect(0, yc, C['window_l'], C['window_w'], 4.0), -1, pt + 1))
    parts = []
    for sx in (1, -1):
        for sy in (1, -1):
            parts.append(prism('so', circle(sx * hx, yc + sy * hy, C['standoff_d'] / 2, 48), pt - 0.5, top))
            gh = D['z_c0'] - 0.2                                  # guards carry the closed cap
            g = C['guard_t']
            L = C['guard_len']
            # L-shaped corner guard (built as union of two bars)
            def bar(x0, x1, y0, y1):
                xa, xb = sorted((sx * x0, sx * x1)); ya, yb = sorted((yc + sy * y0, yc + sy * y1))
                return prism('g', [(xa, ya), (xb, ya), (xb, yb), (xa, yb)], pt - 0.5, gh)
            parts.append(bar(px, px + g, py - L, py + g))
            parts.append(bar(px - L, px + g, py, py + g))
    # hinge lug: disc + gusset toward the plate, 4.5 mm high so it stays clear of the PCB
    lug_prof = hull(circle(0, Rl, Rl - C['lug_clear'], 64) + [(14.0, 0.0), (14.0, 4.5)])
    lugs = [l for l in lay if l[0] == 'lug']
    for _, x0, x1 in lugs:
        parts.append(prism('lug', lug_prof, x0, x1, 'YZ'))
    xl, xr = lugs[0][1], lugs[-1][2]
    # foldable-cap hinge lugs on the top edge (one per side, M3 pin)
    lid_lug = hull(circle(D['y_la'], D['zh'], C['lid_r'], 48) + [(D['y_top'] - 6.0, 0.0), (D['y_top'] - 6.0, pt)])
    for x0 in lid_xs(C):
        parts.append(prism('lidlug', lid_lug, x0 - C['lid_lug_w'] / 2, x0 + C['lid_lug_w'] / 2, 'YZ'))
    union(fr, *parts)
    # standoff insert holes + hinge hole + 45 deg stop groove (on +X face of the lug)
    cuts = []
    for sx in (1, -1):
        for sy in (1, -1):
            cx, cy = sx * hx, yc + sy * hy
            cuts.append(prism('thru', circle(cx, cy, C['screw_clear_d'] / 2, 32, 4.5), -1, top + 1))
            cuts.append(prism('cbore', circle(cx, cy, (C['screw_head_d'] + C['cb_clear']) / 2, 48, 3.5), -1,
                              C['screw_head_h'] + C['cb_extra']))
    cuts.append(prism('hinge', circle(0, Rl, C['bolt_hinge_d'] / 2, 40, 4.5), xl - 1, xr + 1, 'YZ'))
    a_lo, a_hi = groove_angles(C)
    cuts.append(prism('stopgroove', sector(0, Rl, C['stop_r'] - C['stop_groove_w'] / 2,
                                           C['stop_r'] + C['stop_groove_w'] / 2, a_lo, a_hi, 64),
                      xr - C['stop_groove_d'], xr + 1, 'YZ'))
    cuts.append(prism('stopgrooveL', sector(0, Rl, C['stop_r'] - C['stop_groove_w'] / 2,
                                            C['stop_r'] + C['stop_groove_w'] / 2, a_lo, a_hi, 64),
                      xl - 1, xl + C['stop_groove_d'], 'YZ'))
    for x0 in lid_xs(C):
        cuts.append(prism('lidhole', circle(D['y_la'], D['zh'], C['lid_hole_d'] / 2, 32, 6.5),
                          x0 - C['lid_lug_w'] / 2 - 1, x0 + C['lid_lug_w'] / 2 + 1, 'YZ'))
    hw = plate_w / 2
    for sx in (1, -1):
        for yo in C['snap_y']:
            yf = C['y0'] + yo
            cuts.append(prism('snapgroove', [(yf - C['snap_w'] / 2 - 1.0, C['groove_z0']), (yf + C['snap_w'] / 2 + 1.0, C['groove_z0']),
                                           (yf + C['snap_w'] / 2 + 1.0, C['groove_z1']), (yf - C['snap_w'] / 2 - 1.0, C['groove_z1'])],
                              sx * (hw - C["snap_groove_d"]) if sx > 0 else -hw - 1, hw + 1 if sx > 0 else -(hw - C["snap_groove_d"]), 'YZ'))
    cut(fr, *cuts)
    fr.data.transform(Matrix.Translation((xs, 0, 0)))          # move the frame to the right of the hinge
    fr['yc'] = yc
    return fr


def make_lid(C):
    """Cap, modelled in HINGE-AXIS-LOCAL coordinates (origin on the lid hinge axis), closed orientation."""
    D = frame_dims(C)
    yla, zh = D['y_la'], D['zh']
    hi, lo = D['y_la'] - C['lid_edge_gap'], C['y0']
    z0, z1 = D['z_c0'] - zh, D['z_c1'] - zh
    plate = prism('lid', rrect(0, (lo + hi) / 2 - yla, D['plate_w'], hi - lo, 6.0), z0, z1)
    h = hi - yla
    ear = hull(circle(0, 0, C['lid_r'], 48) + [(h - 4.0, z0), (h, z0), (h, z1), (h - 4.0, z1)])
    ew, lw, g = C['lid_ear_w'], C['lid_lug_w'], C['lid_gap']
    ears, cuts = [], []
    arm = C['lid_axis_off'] + 6.0 + 1.0                                          # frame hinge arm length (+clearance)
    for x0 in lid_xs(C):
        xa, xb = x0 - lw / 2 - g - ew, x0 + lw / 2 + g + ew                  # outer faces of the two ears
        ears += [prism('ear', ear, xa, xa + ew, 'YZ'), prism('ear', ear, xb - ew, xb, 'YZ')]
        cuts += [prism('hole', circle(0, 0, C['lid_hole_d'] / 2, 32, 6.5), xa - 1, xb + 1, 'YZ'),
                 prism('head', circle(0, 0, C['lid_head_d'] / 2, 48, 3.5), xa - 1, xa + C['lid_head_depth'], 'YZ'),
                 prism('nut', hexagon(0, 0, C['lid_nut_af'], 90), xb - C['lid_nut_depth'], xb + 1, 'YZ'),
                 prism('armslot', [(x0 - lw / 2 - 0.7, h - arm), (x0 + lw / 2 + 0.7, h - arm), (x0 + lw / 2 + 0.7, h + 1), (x0 - lw / 2 - 0.7, h + 1)], z0 - 1, z1 + 1)]
    hw = D['plate_w'] / 2
    hooks = []
    yr_ = h - 2.0                                                               # middle of the ear arm (where the ear joins the slab)
    zb = min(z for (y1, z1_), (y2, z2_) in zip(ear, ear[1:] + ear[:1]) if min(y1, y2) <= yr_ <= max(y1, y2) and y1 != y2
             for z in [z1_ + (z2_ - z1_) * (yr_ - y1) / (y2 - y1)])            # lowest point of the ear profile at that y
    rh = min(C['lid_rib_h'], z0 - zb - 0.4)
    for x0 in lid_xs(C):                                                        # gusset webs: ear outer face <-> slab underside
        xa, xb = x0 - lw / 2 - g - ew, x0 + lw / 2 + g + ew
        for xf, sg in ((xa, -1), (xb, 1)):
            web = [(xf - sg * 0.05, z0 - rh), (xf - sg * 0.05, z1), (xf + sg * C['lid_rib_d'], z1), (xf + sg * C['lid_rib_d'], z0)]
            hooks.append(prism('earrib', web, yr_ - C['lid_rib_t'] / 2, yr_ + C['lid_rib_t'] / 2, 'XZ'))
    plate['ear_rib_h'] = rh
    print(f"lid hinge ear ribs: {C['lid_rib_t']:.1f} mm thick, {C['lid_rib_d']:.1f} mm out, {rh:.1f} mm high (ear profile bottom at z={zb:.1f}, slab underside z={z0:.1f})")
    xin = hw + C['snap_gap']
    for sx in (1, -1):
        for yo in C['snap_y']:
            ya, yb = C['y0'] + yo - C['snap_w'] / 2 - yla, C['y0'] + yo + C['snap_w'] / 2 - yla
            xa, xb = sorted((sx * (hw - 1.0), sx * (xin + C['snap_t'])))
            hooks.append(prism('flange', [(xa, ya), (xb, ya), (xb, yb), (xa, yb)], z0, z1))
            xa, xb = sorted((sx * xin, sx * (xin + C['snap_t'])))
            hooks.append(prism('hook', [(xa, ya), (xb, ya), (xb, yb), (xa, yb)], C['snap_z0'] - zh, z1))
            nub = [(sx * (xin + 0.05), C['snap_z0'] - zh), (sx * (xin - C['snap_bite']), C['snap_z1'] - zh),
                   (sx * (xin - C['snap_bite']), C['snap_z2'] - zh), (sx * (xin + 0.05), C['snap_z3'] - zh)]
            hooks.append(prism('nub', nub, ya + 2.0, yb - 2.0, 'XZ'))
            xo = xin + C['snap_t']
            rib = [(sx * (xo - 0.05), z0 - C['snap_rib_h']), (sx * (xo - 0.05), z1), (sx * (xo + C['snap_rib_d']), z1), (sx * (xo + C['snap_rib_d']), z0)]
            off = C['snap_w'] / 2 - C['snap_rib_t'] / 2 - 0.5
            for yr in (-1, 1):
                yrc = (ya + yb) / 2 + yr * off
                hooks.append(prism('rib', rib, yrc - C['snap_rib_t'] / 2, yrc + C['snap_rib_t'] / 2, 'XZ'))
    union(plate, *ears, *hooks)
    cuts.append(prism('notch', circle(0, lo - yla, C['lid_notch_r'], 48), z0 - 1, z1 + 1))   # thumb notch
    cut(plate, *cuts)
    plate['flip'] = z1                                                          # print: outer face down
    return plate


def make_dummies(C, yc):
    """Visual-only PCB, glass and stop screw (not exported)."""
    pt, so = C['plate_t'], C['standoff_h']
    pcb = prism('pcb_dummy', rrect(0, yc, C['pcb_l'], C['pcb_w'], 2.0, 4), pt + so, pt + so + C['pcb_t'])
    hx = C['pcb_l'] / 2 - C['hole_off']; hy = C['pcb_w'] / 2 - C['hole_off']
    for sx in (1, -1):
        for sy in (1, -1):
            cut(pcb, prism('h', circle(sx * hx, yc + sy * hy, C['hole_d'] / 2, 24), 0, 100))
    z0 = pt + so + C['pcb_t']
    glass = prism('glass_dummy', [(-C['glass_l'] / 2, yc - C['glass_w'] / 2), (C['glass_l'] / 2, yc - C['glass_w'] / 2),
                                  (C['glass_l'] / 2, yc + C['glass_w'] / 2), (-C['glass_l'] / 2, yc + C['glass_w'] / 2)],
                  z0, z0 + C['glass_t'])
    xs = frame_dims(C)['xs']
    for o in (pcb, glass): o.data.transform(Matrix.Translation((xs, 0, 0)))
    return pcb, glass


def make_wingnut(C):
    """Clearance model of the M4 wing nut + washer on the LEFT face of the hinge stack (axis along X through the origin)."""
    xl = hinge_layout(C)[0][1] - 0.2                          # 0.2 mm model gap so contact with the ear face is not flagged
    x0, wt = xl - C['washer_t'], C['wn_wing_t']
    wash = prism('washer', circle(0, 0, C['washer_d'] / 2, 32, 3.5), xl - C['washer_t'], xl, 'YZ')
    boss = prism('boss', circle(0, 0, C['wn_boss_d'] / 2, 32, 3.5), x0 - C['wn_boss_h'], x0, 'YZ')
    r = C['wn_span'] / 2
    wings = [prism('wing', [(-r, -wt / 2), (r, -wt / 2), (r, wt / 2), (-r, wt / 2)], x0 - C['wn_h'], x0 - C['wn_boss_h'] + 1.0, 'XZ')]
    # (u,v) of an 'XZ' prism are (x, z): build the wing as a plate in the X-Z plane through the axis -> rotate: plate spans local Y (radial), thin in Z
    wings = [prism('wing', [(x0 - C['wn_h'], -r), (x0 - C['wn_boss_h'] + 1.0, -r), (x0 - C['wn_boss_h'] + 1.0, r), (x0 - C['wn_h'], r)], -wt / 2, wt / 2, 'XZ')]
    wn = prism('wingnut', circle(0, 0, 0.5, 8), 0, 0.5)
    union(wn, wash, boss, *wings)
    return wn


def make_stop_screw(C, side=1):
    ears = [l for l in hinge_layout(C) if l[0] == 'ear']
    xo = ears[-1][2] if side > 0 else ears[0][1]
    za = C['tt_t'] + C['lug_r']
    sg = 1 if side > 0 else -1
    shaft = prism('stop_screw', circle(0, za + C['stop_r'], C['stop_screw_d'] / 2, 24), min(xo, xo - sg * 8.0), max(xo, xo - sg * 8.0), 'YZ')
    head = prism('head', circle(0, za + C['stop_r'], 2.85, 24), min(xo, xo + sg * 1.9), max(xo, xo + sg * 1.9), 'YZ')
    union(shaft, head)
    return shaft

# --------------------------------------------------------------------------
# assembly rig + checks
# --------------------------------------------------------------------------
def build_assembly(C):
    D = frame_dims(C)
    base = make_base(C)
    car = make_carriage(C)
    tt = make_turntable(C)
    fr = make_frame(C)
    lid = make_lid(C)
    yc = fr['yc']
    pcb, glass = make_dummies(C, yc)
    screw = make_stop_screw(C, 1)
    screwL = make_stop_screw(C, -1)
    wn = make_wingnut(C)
    sc = bpy.context.scene
    def empty(n):
        e = bpy.data.objects.new(n, None); sc.collection.objects.link(e); return e
    slide, swivel, tilt, lidj = empty("SLIDE"), empty("SWIVEL"), empty("TILT"), empty("LID_HINGE")
    swivel.parent = slide
    tilt.parent = swivel
    F = C['floor_t']
    tilt.location = (0, 0, F + C['tt_t'] + C['lug_r'])
    tt.parent = swivel; tt.location = (0, 0, F)
    car.parent = slide; car.location = (0, 0, F - C['car_t'])
    wn.parent = swivel; wn.location = (0, 0, F + C['tt_t'] + C['lug_r'])
    screw.parent = swivel; screw.location = (0, 0, F)
    screwL.parent = swivel; screwL.location = (0, 0, F)
    fr.parent = tilt; fr.location = (0, 0, -C['lug_r'])
    pcb.parent = fr; glass.parent = fr
    lidj.parent = fr; lidj.location = (D['xs'], D['y_la'], D['zh'])
    lid.parent = lidj
    c = slide.constraints.new('LIMIT_LOCATION'); c.use_min_x = c.use_max_x = True; c.owner_space = 'LOCAL'
    c.min_x = -C['slide_travel']; c.max_x = C['slide_travel']
    c = swivel.constraints.new('LIMIT_ROTATION'); c.use_limit_z = True; c.owner_space = 'LOCAL'
    c.min_z = -math.radians(C['swivel_max']); c.max_z = math.radians(C['swivel_max'])
    c = tilt.constraints.new('LIMIT_ROTATION'); c.use_limit_x = True; c.owner_space = 'LOCAL'
    c.min_x = 0; c.max_x = math.radians(C['tilt_max'])
    c = lidj.constraints.new('LIMIT_ROTATION'); c.use_limit_x = True; c.owner_space = 'LOCAL'
    c.min_x = -math.radians(C['lid_max']); c.max_x = 0
    base.color = (0.18, 0.18, 0.2, 1); tt.color = (0.85, 0.35, 0.1, 1); fr.color = (0.15, 0.45, 0.8, 1)
    car.color = (0.2, 0.75, 0.3, 1); lid.color = (0.9, 0.9, 0.92, 1)
    pcb.color = (0.05, 0.4, 0.15, 1); glass.color = (0.02, 0.02, 0.03, 1)
    screw.color = (0.8, 0.8, 0.8, 1); screwL.color = (0.8, 0.8, 0.8, 1)
    return dict(base=base, car=car, tt=tt, fr=fr, lid=lid, pcb=pcb, glass=glass, screw=screw, screwL=screwL, wn=wn,
                slide=slide, swivel=swivel, tilt=tilt, lidj=lidj, D=D)


def pose(A, tilt_deg, swivel_deg, slide_mm=0.0, lid_deg=0.0):
    A['tilt'].rotation_euler = (math.radians(tilt_deg), 0, 0)
    A['swivel'].rotation_euler = (0, 0, math.radians(swivel_deg))
    A['slide'].location = (slide_mm, 0, 0)
    A['lidj'].rotation_euler = (-math.radians(lid_deg), 0, 0)      # positive = cap opens up and over the top edge
    bpy.context.view_layer.update()


def _bvh(ob):
    me = ob.data
    me.calc_loop_triangles()
    mw = ob.matrix_world
    verts = [mw @ v.co for v in me.vertices]
    tris = [tuple(t.vertices) for t in me.loop_triangles]
    return BVHTree.FromPolygons(verts, tris, epsilon=1e-5)


def hits(a, b):
    return len(_bvh(a).overlap(_bvh(b)))


def run_checks(A, C):
    """Interference checks; constraints are bypassed (we set the angles explicitly)."""
    tv, F = C['slide_travel'], C['floor_t']
    for k in ('slide', 'swivel', 'tilt', 'lidj'):
        for c in A[k].constraints: c.enabled = False
    print("\n=== interference check (triangle overlaps; 0 = clear) ===")
    ok = True
    for dx in (-tv, 0, tv):
        for sw in (-C['swivel_max'], 0, C['swivel_max']):
            line, bad_any = [], False
            for th in (0.3, 10, 20, 30, 40, C['tilt_max']):
                pose(A, th, sw, dx)
                r = dict(frame_turntable=hits(A['fr'], A['tt']), frame_base=hits(A['fr'], A['base']),
                         frame_carriage=hits(A['fr'], A['car']),
                         pcb_turntable=hits(A['pcb'], A['tt']), pcb_base=hits(A['pcb'], A['base']),
                         frame_screw=hits(A['fr'], A['screw']) + hits(A['fr'], A['screwL']),
                         lid_frame=hits(A['lid'], A['fr']), lid_pcb=hits(A['lid'], A['pcb']), lid_glass=hits(A['lid'], A['glass']),
                         lid_turntable=hits(A['lid'], A['tt']), lid_base=hits(A['lid'], A['base']))
                bad = {k: v for k, v in r.items() if v}
                bad_any |= bool(bad); ok &= not bad
                if bad: line.append(f"tilt {th}: {bad}")
            print(f"slide {dx:+5.1f}  swivel {sw:+5.1f}  tilt 0.3..45 ->", "OK" if not bad_any else "COLLISION " + "; ".join(line))
    print("\n=== 45 deg tilt stop test (stop screws vs frame grooves) ===")
    pose(A, 0, 0)
    first = None
    for th in [x * 0.5 for x in range(-12, 110)]:
        pose(A, th, 0)
        h = hits(A['fr'], A['screw']) + hits(A['fr'], A['screwL'])
        if h and first is None and th > 20: first = th
        if th in (-2.0, 0.0, 45.0, 46.0): print(f"tilt {th:5.1f}: screw/frame overlaps = {h}")
    print(f"first tilt angle (above 20 deg) where a screw hits its groove wall: {first}")

    print("\n=== swivel stop test (carriage post vs turntable groove) at each slide position ===")
    test_post = prism('test_post', circle(0, -C['swivel_stop_r'], C['post_d'] / 2, 32, 3.5), F, F + C['post_h'])
    test_post.parent = A['slide']
    for dx in (-tv, 0, tv):
        res = []
        for sw in (-47, -45, 0, 45, 47):
            pose(A, 0, sw, dx)
            res.append(f"{sw:+d}deg:{hits(test_post, A['tt']):3d}")
        print(f"slide {dx:+5.1f}:  " + "  ".join(res) + "   (expect 0 up to +-45, >0 at +-47)")
    bpy.data.objects.remove(test_post, do_unlink=True)

    print("\n=== foldable cap: flatness at 180 deg and clearance when closed ===")
    pose(A, 0, 0, 0, 180)
    lz = min((A['lid'].matrix_world @ v.co).z for v in A['lid'].data.vertices)
    fz = min((A['fr'].matrix_world @ v.co).z for v in A['fr'].data.vertices)
    ltop = max((A['lid'].matrix_world @ v.co).z for v in A['lid'].data.vertices)
    print(f"cap open 180 deg, frame flat: cap outer face z = {lz:.3f}, frame back z = {fz:.3f}  -> flush: {abs(lz - fz) < 0.05}; "
          f"seam between frame edge and cap edge = {C['lid_edge_gap'] + C['lid_axis_off']:.1f} mm")
    print("\n=== snap latch: closed fit and hold ===")
    pose(A, 0.3, 0, 0, 0)
    print(f"closed: lid/frame overlaps = {hits(A['lid'], A['fr'])}  (0 = hooks sit in the grooves)")
    for phi in (0.3, 0.6, 1.0, 1.5, 2.0, 3.0, 4.0):
        pose(A, 0.3, 0, 0, phi)
        print(f"lid lifted {phi:3.1f} deg at the hinge -> hook nubs hit the plate edge: {hits(A['lid'], A['fr'])} overlaps "
              f"{'(latch holds, needs a pull to flex the hooks)' if hits(A['lid'], A['fr']) else '(free)'}")
    print("\n=== foldable cap sweep: first angle (deg from closed) at which the cap touches something ===")
    others = ('fr', 'pcb', 'glass', 'tt', 'base', 'car')
    cases = (("frame flat on base            ", 0.3, 0, 0), ("frame flat, swivel +45        ", 0.3, 45, 0),
             ("tilted 45                    ", 45, 0, 0), ("tilted 45, swivel -45         ", 45, -45, 0),
             ("tilted 45, slide +15          ", 45, 0, C['slide_travel']), ("tilted 20                    ", 20, 0, 0))
    for label, th, sw, dx in cases:
        first = who = None
        for phi in range(8, 331, 2):                          # start past the snap latch (hooks flex out of the grooves by ~5 deg)
            pose(A, th, sw, dx, phi)
            tl = _bvh(A['lid'])
            for k in others:
                if tl.overlap(_bvh(A[k])):
                    first, who = phi, k; break
            if first: break
        print(f"{label}: " + (f"first contact at {first} deg with '{who}'" if first else "no contact up to 330 deg"))
    pose(A, 0, 0)
    print("\n=== M5 wing nut (left of the hinge): clearance at every wing angle ===")
    worst = {}
    for dx in (-tv, 0, tv):
        for sw, th in ((0, 0.3), (0, 20), (0, C['tilt_max']), (-C['swivel_max'], C['tilt_max']), (C['swivel_max'], 0.3)):
            pose(A, th, sw, dx)
            for ang in range(0, 180, 10):
                A['wn'].rotation_euler = (math.radians(ang), 0, 0)
                bpy.context.view_layer.update()
                for k in ('fr', 'tt', 'base', 'car', 'screw', 'screwL', 'pcb', 'lid'):
                    h = hits(A['wn'], A[k])
                    if h: worst.setdefault(k, []).append((dx, sw, th, ang))
    A['wn'].rotation_euler = (0, 0, 0)
    if not worst: print("wing nut clear of everything at every angle tested")
    for k, v in worst.items():
        print(f"wing nut touches '{k}' in {len(v)} cases, e.g. slide/swivel/tilt/angle = {v[:3]}")
    print("\n=== slide tests (turntable / carriage / bolt vs base; parts lifted 0.3 mm to ignore coplanar floor contact) ===")
    A['tt'].location = (0, 0, F + 0.3); A['car'].location = (0, 0, F - C['car_t'] + 0.3)
    bolt = prism('test_bolt', circle(0, 0, 2.5, 24, 3.5), 0, C['base_t'] + 5); bolt.parent = A['slide']
    for dx in (-tv - 0.5, -tv, 0, tv, tv + 0.5):
        res = []
        for sw in (-45, 0, 45):
            pose(A, 0, sw, dx)
            res.append(f"sw{sw:+d}: turntable/base {hits(A['base'], A['tt']):3d}")
        pose(A, 0, 0, dx)
        print(f"slide {dx:+5.1f}:  " + "   ".join(res) + f"   carriage/base {hits(A['base'], A['car']):3d}   bolt/slot {hits(A['base'], bolt):3d}")
    bpy.data.objects.remove(bolt, do_unlink=True)
    A['tt'].location = (0, 0, F); A['car'].location = (0, 0, F - C['car_t'])
    pose(A, 0, 0)
    for k in ('slide', 'swivel', 'tilt', 'lidj'):
        for c in A[k].constraints: c.enabled = True
    return ok


def nonmanifold(ob):
    bm = bmesh.new(); bm.from_mesh(ob.data)
    n = sum(1 for e in bm.edges if not e.is_manifold)
    bm.free(); return n


def export_stl(ob, path, mat=None):
    bpy.ops.object.select_all(action='DESELECT')
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    saved = ob.matrix_world.copy()
    parent = ob.parent
    # export in local (print) orientation: temporarily unparent & zero
    ob.parent = None; ob.matrix_world = mat if mat is not None else Matrix.Identity(4)
    bpy.context.view_layer.update()
    bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True, ascii_format=False,
                          apply_modifiers=True, global_scale=1.0, use_scene_unit=False)
    ob.parent = parent; ob.matrix_world = saved


def bom(C):
    m5_end_min = C['base_t'] - C['m5_nut_depth'] + 3.0
    m5_end_max = C['base_t'] - 0.5
    print("\n=== hardware notes ===")
    print(f"M{C.get('swivel_m', 5)} swivel bolt: head seats at z={C['m5_cb_depth']:.1f}; tip must end between z={m5_end_min:.1f} and z={m5_end_max:.1f}"
          f"  -> length {m5_end_min - C['m5_cb_depth']:.1f} .. {m5_end_max - C['m5_cb_depth']:.1f} mm  (use M{C.get('swivel_m', 5)}x{int(round(m5_end_max - C['m5_cb_depth']))})")
    lay = hinge_layout(C)
    xl, xr = lay[0][1], lay[-1][2]
    bear = xr - C['m4_nut_depth']                              # underside of the M5 hex head seats here
    need = (bear - xl) + C['washer_t'] + C['wn_m'] + 1.5       # head -> left face -> washer -> wing nut thread (+ 1.5 mm tip)
    L = math.ceil(need / 5) * 5
    print(f"M5 hinge bolt (hex head, in the right ear pocket) + M5 wing nut + washer on the LEFT face: stack {xr - xl:.1f} mm, "
          f"head bearing x={bear:.1f}, left face x={xl:.1f} -> min length {need:.1f}, use M5x{L} (tip pokes ~{bear - xl + L - (bear - xl) - (bear - xl) + (bear - xl) - need + 1.5:.1f} mm past the nut)")
    top_pcb = C['plate_t'] + C['standoff_h'] + C['pcb_t']
    cb = C['screw_head_h'] + C['cb_extra']
    need = top_pcb - cb + C['nut_h']
    print(f"PCB screws: head seats at z={cb:.1f} (recess {C['screw_head_d'] + C['cb_clear']:.1f} dia x {cb:.1f} deep, floor {C['plate_t'] - cb:.1f} mm), "
          f"PCB top at z={top_pcb:.1f} -> screw length >= {need:.1f} mm: use M3x14 (or M3x16) + M3 nut")
    ew, lw, g = C['lid_ear_w'], C['lid_lug_w'], C['lid_gap']
    stack = 2 * ew + lw + 2 * g
    bearing = C['lid_head_depth']; seat = stack - C['lid_nut_depth']
    need = seat + 2.4 - bearing
    print(f"cap hinge: 3x M3 screw (left, middle, right): stack {stack:.1f} mm, head recess {C['lid_head_depth']:.1f} deep, nut pocket {C['lid_nut_depth']:.1f} "
          f"-> min length {need:.1f}, use M3x{math.ceil(need / 2) * 2}  + M3 nut (nyloc), tighten for friction")
    print(f"base {C['base_l']:.0f} wide x {C['base_t']:.0f} tall; turntable sunk {C['tt_t']:.0f} mm (top flush with base top); "
          f"slide travel +-{C['slide_travel']:.0f} mm (wall left/right of recess {(C['base_l'] - 2 * (C['tt_r'] + C['pocket_clear'] / 2 + C['slide_travel'])) / 2:.1f} mm)")
    print(f"hinge friction faces: {2 * C['n_lugs']}  (was 2)   turntable diameter: {2 * C['tt_r']:.0f} mm")


def render_preview(A, C, out):
    sc = bpy.context.scene
    sc.render.engine = 'BLENDER_WORKBENCH'
    sc.render.resolution_x, sc.render.resolution_y = 1400, 900
    sc.display.shading.light = 'STUDIO'
    sc.display.shading.color_type = 'OBJECT'
    sc.display.shading.show_cavity = True
    sc.world = bpy.data.worlds.new("w"); sc.world.color = (0.93, 0.93, 0.95)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam")); sc.collection.objects.link(cam)
    cam.data.type = 'ORTHO'
    tgt = bpy.data.objects.new("tgt", None); sc.collection.objects.link(tgt)
    t = cam.constraints.new('TRACK_TO'); t.target = tgt; t.track_axis = 'TRACK_NEGATIVE_Z'; t.up_axis = 'UP_Y'
    sc.camera = cam
    def shot(name, tilt, sw, dx, lid, loc, target, scale):
        pose(A, tilt, sw, dx, lid)
        cam.data.ortho_scale = scale; cam.location = loc; tgt.location = target
        sc.render.filepath = os.path.join(out, f"preview_{name}.png")
        bpy.ops.render.render(write_still=True)
    shot("carry_closed", 0, 0, 0, 0, (330, -250, 220), (30, 70, 20), 420)
    shot("flat_cap_open180", 0, 0, 0, 180, (360, -140, 200), (30, 120, 10), 500)
    shot("tilt45_cap225", 45, 0, 0, 225, (-330, -260, 230), (30, 100, 60), 450)
    for k in ('tt', 'screw', 'screwL', 'car', 'pcb', 'glass', 'base'): A[k].hide_render = True
    shot("lid_hinge_closeup", 0, 0, 0, 40, (-150, 330, 110), (-42, 135, 38), 70)
    shot("snap_closeup", 0, 0, 0, 0, (-260, -90, 40), (-58, 60, 14), 130)
    for k in ('tt', 'screw', 'screwL', 'car', 'pcb', 'glass', 'base'): A[k].hide_render = False
    for k in ('tt', 'screw', 'screwL', 'car', 'fr', 'pcb', 'glass', 'lid'): A[k].hide_render = True
    shot("base_underside", 0, 0, 0, 0, (60, -40, -300), (0, 33, 0), 330)
    A['car'].hide_render = False
    for k in ('car',): A[k].hide_render = True


def main(C, out):
    os.makedirs(out, exist_ok=True)
    reset_scene()
    A = build_assembly(C)
    parts = (('base', 'base'), ('car', 'carriage'), ('tt', 'turntable'), ('fr', 'frame'), ('lid', 'lid'))
    for key, nm in parts:
        print(f"{nm:10s} non-manifold edges: {nonmanifold(A[key])}")
    b = A['base']
    print(f"base footprint {C['base_l']:.0f} x {b['len']:.1f} mm (y from {b['y_min']:.1f} to {b['y_max']:.1f}; frame top edge at y={A['D']['y_top']:.1f})")
    if C['cavity']:
        print(f"base volume {b['vol_solid']:.0f} cm3 solid -> {b['vol_final']:.0f} cm3 with {b['cells']} underside pockets "
              f"(saves {100 * (1 - b['vol_final'] / b['vol_solid']):.0f} %)")
    zmin = min(v.co.z for v in A['fr'].data.vertices)
    print(f"frame lowest point z = {zmin:.3f} (0 = flat back, no protruding screw heads)")
    ok = run_checks(A, C)
    for dx in (-C['slide_travel'], 0, C['slide_travel']):
        pose(A, 0.3, 0, dx)
        xs_ = [(A['fr'].matrix_world @ v.co).x for v in A['fr'].data.vertices]
        print(f"slide {dx:+5.1f}, swivel 0: frame spans x = {min(xs_):.1f} .. {max(xs_):.1f} mm (base left border x = {-C['base_l'] / 2:.0f}, right border {C['base_l'] / 2:.0f}); "
              f"frame top edge y = {A['D']['y_top']:.1f} (base top edge {A['base']['y_max']:.1f})")
    pose(A, 0, 0)
    bom(C)
    for key, nm in parts:
        mat = (Matrix.Translation((0, 0, A['lid']['flip'])) @ Matrix.Rotation(math.pi, 4, 'Y')) if key == 'lid' else None
        export_stl(A[key], os.path.join(out, f"{C['name']}_{nm}.stl"), mat)
    pose(A, 0, 0)
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out, f"{C['name']}_assembly.blend"))
    render_preview(A, C, out)
    print("\nALL CLEAR" if ok else "\nCHECK COLLISIONS ABOVE")


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    out = argv[argv.index("--out") + 1] if "--out" in argv else os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
    main(CFG_MONITOR1, out)
