"""
Ktopo — 3D-printed drilling rover
Parametric CAD model (CadQuery). Generates every printable part as STL (already oriented
for printing) and STEP (to edit in Fusion 360 / FreeCAD / Onshape), plus the full assembly.

Assembly coordinate system (mm):
  X = forward (the drill tip points to +X)
  Y = left
  Z = up (Z = 0 is the floor)

Usage:  python3 ktopo_cad.py             -> exports everything to ./output
        python3 ktopo_cad.py wheel_rim   -> exports only that part
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))
import cadquery as cq  # noqa: E402
import cq_gears  # noqa: E402

V = cq.Vector

# ----------------------------------------------------------------------------------------
# MAIN PARAMETERS
# ----------------------------------------------------------------------------------------
CH_X0, CH_X1 = -150.0, 150.0        # chassis length (300 mm, printed in 2 halves)
CH_YW = 75.0                         # half of the chassis outer width (150 mm)
WALL = 3.0                           # wall thickness
FLOOR_Z0, FLOOR_Z1 = 20.0, 23.0      # chassis floor (20 mm ground clearance)
CH_TOP = 90.0                        # top edge of the walls = axis of the hull and the drill

WHEEL_R = 50.0                       # wheel Ø100
WHEEL_W = 40.0
WHEEL_GAP = 3.0
AXLE_Z = WHEEL_R
AXLE_X = 90.0                        # wheelbase = 180 mm

# JGB37-520 gear motor (manufacturer dimensions)
MOT_GB_D = 37.0                      # gearbox Ø37
MOT_BODY_D = 32.0
MOT_PCD = 31.0                       # 6 × M3 holes on a Ø31 circle
MOT_OFFSET = 7.0                     # output shaft is 7 mm off-center
MOT_LEN = 62.0                       # gearbox (26.5) + motor (22) + encoder (11.75) + clearance
MOT_SHAFT_L = 15.5

# Drill
DRILL_Z = CH_TOP                     # drill axis = hull axis
BULK_X0, BULK_X1 = 116.0, 120.0      # bulkhead that holds the drill motor
COLLAR_X = 150.0                     # back face of the collar = front face of the chassis
GEAR_M = 2.5                         # bevel gear module
GEAR_Z_SIDE = 20                     # teeth on the side gears (A and B)
GEAR_Z_SPIDER = 12                   # teeth on the spider gears
GEAR_FACE = 10.0
XC = 199.0                           # differential center (where all gear axes cross)

# Hardware
M3_CLR, M3_TAP = 3.4, 2.8
M25_TAP = 2.7
M2_TAP = 1.8
M4_CLR, M4_TAP = 4.3, 3.4
B608_OD, B608_W = 22.15, 7.0         # 608 skateboard bearing, with press-fit allowance
ROD_D = 8.0

# Gears (computed once)
_PAIR = cq_gears.BevelGearPair(module=GEAR_M, gear_teeth=GEAR_Z_SIDE, pinion_teeth=GEAR_Z_SPIDER,
                               face_width=GEAR_FACE, backlash=0.15, clearance=0.25)
CONE_H_SIDE = _PAIR.gear.cone_h      # distance from the side gear's back face to the cone apex
CONE_H_SPIDER = _PAIR.pinion.cone_h
X_A_BACK = XC - CONE_H_SIDE          # back face of gear A
X_B_BACK = XC + CONE_H_SIDE          # "back" face of gear B (it faces forward)
PLATE_T = 4.0
X_PLATE_FRONT = X_B_BACK + PLATE_T   # front face of the rear cutter's end plate
X_RIM = 168.0                        # rear cutter rim (just behind the collar)
X_CONE = X_PLATE_FRONT + 1.6         # front cone base (an M8 washer sits in between)
CONE_LEN = 108.0
R_REAR_FRONT, R_REAR_RIM = 50.0, 66.0
R_CONE_BASE = 47.0


# ----------------------------------------------------------------------------------------
# GEOMETRY HELPERS
# ----------------------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(r, p, d, h):
    return cq.Solid.makeCylinder(r, h, V(*p), V(*d))


def cone(r1, r2, p, d, h):
    return cq.Solid.makeCone(r1, r2, h, V(*p), V(*d))


def tube(ro, ri, p, d, h):
    return cyl(ro, p, d, h).cut(cyl(ri, p, d, h))


def fuse(*shapes):
    s = shapes[0]
    if len(shapes) > 1:
        s = s.fuse(*shapes[1:])
    return s.clean()


def cut(base, *tools):
    return base.cut(*tools).clean() if tools else base


def prism(pts, plane, h):
    """Extrudes a 2D polygon (list of points) on a cq.Plane by a distance h."""
    return cq.Workplane(plane).polyline(pts).close().extrude(h).val()


def hex_prism(af, h, p=(0, 0, 0), d=(0, 0, 1)):
    """Hexagonal prism (af = across flats)."""
    plane = cq.Plane(origin=V(*p), normal=V(*d))
    return cq.Workplane(plane).polygon(6, af / math.cos(math.radians(30))).extrude(h).val()


def d_bore(depth, p=(0, 0, 0), d=(0, 0, 1)):
    """D-shaped bore for an 8 mm rod with a 0.5 mm filed flat."""
    plane = cq.Plane(origin=V(*p), normal=V(*d))
    c = cq.Workplane(plane).circle(ROD_D / 2 + 0.1).extrude(depth).val()
    flat = 3.7  # the rod is filed down 0.5 mm (flat at 3.5 mm from the center)
    keep = cq.Workplane(plane).center((flat - 5) / 2, 0).rect(flat + 5, ROD_D + 2).extrude(depth).val()
    return c.intersect(keep)


def ring_points(n, r, a0=0.0):
    return [(r * math.cos(math.radians(a0 + i * 360 / n)), r * math.sin(math.radians(a0 + i * 360 / n)))
            for i in range(n)]


def loc(t=(0, 0, 0), axis=(0, 0, 1), ang=0.0):
    return cq.Location(V(*t), V(*axis), ang)


# ----------------------------------------------------------------------------------------
# CHASSIS (2 halves joined with 7 M3 screws)
# ----------------------------------------------------------------------------------------
def _motor_holes_side(ax, side):
    """Drive motor holes in the side wall (side = +1 left, -1 right)."""
    ystart = (CH_YW - WALL - 1) if side > 0 else -(CH_YW + 1)
    tools = [cyl(7.5, (ax, ystart, AXLE_Z), (0, 1, 0), WALL + 2)]
    cz = AXLE_Z + MOT_OFFSET
    for (dx, dz) in ring_points(12, MOT_PCD / 2, 0):
        tools.append(cyl(M3_CLR / 2, (ax + dx, ystart, cz + dz), (0, 1, 0), WALL + 2))
    return tools


def chassis_half(front=True):
    sg = 1 if front else -1
    xa, xb = (0.0, CH_X1) if front else (CH_X0, 0.0)
    W, T = CH_YW, WALL
    solids = [
        box(xa, xb, -W, W, FLOOR_Z0, FLOOR_Z1),                    # floor
        box(xa, xb, W - T, W, FLOOR_Z0, CH_TOP),                    # left wall
        box(xa, xb, -W, -W + T, FLOOR_Z0, CH_TOP),                  # right wall
    ]
    if front:
        solids.append(box(CH_X1 - T, CH_X1, -W, W, FLOOR_Z0, CH_TOP))   # front wall
    else:
        solids.append(box(CH_X0, CH_X0 + T, -W, W, FLOOR_Z0, CH_TOP))   # back wall
    # joining flange at X = 0
    jx = (0.0, 4.0) if front else (-4.0, 0.0)
    solids += [box(jx[0], jx[1], -W + T, W - T, FLOOR_Z1, 35),
               box(jx[0], jx[1], W - 15, W - T, FLOOR_Z1, 62),
               box(jx[0], jx[1], -W + T, -W + 15, FLOOR_Z1, 62)]
    # cradles under the body of each drive motor
    ax = AXLE_X * sg
    cz = AXLE_Z + MOT_OFFSET
    for side in (1, -1):
        y0, y1 = (30, 40) if side > 0 else (-40, -30)
        solids.append(box(ax - 12, ax + 12, y0, y1, FLOOR_Z1, cz - 6))
    # rails for the electronics deck (45° underside, prints without supports)
    lx = (6.0, 64.0) if front else (-64.0, -6.0)
    for sy in (1, -1):
        yw = sy * (W - T)
        prof = [(yw, 60), (yw - sy * 4, 64), (yw - sy * 4, 67), (yw, 67)]
        solids.append(prism(prof, cq.Plane(origin=V(lx[0], 0, 0), xDir=V(0, 1, 0), normal=V(1, 0, 0)), lx[1] - lx[0]))
    # standoffs for the Raspberry Pi 5 (front half only, the Pi lies on the floor)
    if front:
        for (px, py) in PI_HOLES:
            solids.append(cyl(3.2, (px, py, FLOOR_Z1 - 0.5), (0, 0, 1), 6.5))
    # drill motor bulkhead (front half only)
    if front:
        bk = box(BULK_X0, BULK_X1, -W + T, W - T, FLOOR_Z1, CH_TOP)
        bk = bk.fuse(box(BULK_X0, BULK_X1, -22, 22, CH_TOP - 1, DRILL_Z + MOT_OFFSET),
                     cyl(22, (BULK_X0, 0, DRILL_Z + MOT_OFFSET), (1, 0, 0), BULK_X1 - BULK_X0))
        # stiffening ribs in front of the bulkhead
        for yy in (-28, 28):
            bk = bk.fuse(box(BULK_X1, CH_X1 - T, yy - 1.5, yy + 1.5, FLOOR_Z1, 60))
        # internal guides for the 2 VL53L0X ToF sensors (they look through windows in the front wall)
        for s_ in (1, -1):
            yc = s_ * TOF_Y
            for e in (-1, 1):
                ya, yb = sorted((yc + e * 12.0, yc + e * 15.0))
                bk = bk.fuse(box(CH_X1 - T - 6, CH_X1 - T, ya, yb, FLOOR_Z1, 40))
        solids.append(bk)
    body = fuse(*solids)

    tools = []
    # cradle: cylindrical cut for the motor + slot for a zip tie
    for side in (1, -1):
        y0 = 28 if side > 0 else -42
        tools.append(cyl(MOT_BODY_D / 2 + 0.4, (ax, y0, cz), (0, 1, 0), 14))
        yc = 35 * side
        tools.append(box(ax - 14, ax + 14, yc - 2.6, yc + 2.6, cz - 21, cz - 18.5))
        tools += _motor_holes_side(ax, side)
    # flange screws
    for (yy, zz) in [(-50, 29), (0, 29), (50, 29), (66, 45), (-66, 45), (66, 56), (-66, 56)]:
        tools.append(cyl(M3_CLR / 2, (-6, yy, zz), (1, 0, 0), 12))
    # hull screws (horizontal, through the walls)
    for hx in ((25, 128) if front else (-25, -125)):
        tools.append(cyl(M3_CLR / 2, (hx, -W - 1, 84), (0, 1, 0), 2 * W + 2))
    # deck rail holes
    for hx in ((12, 58) if front else (-12, -58)):
        for yy in (W - T - 2, -(W - T - 2)):
            tools.append(cyl(M3_TAP / 2, (hx, yy, 60), (0, 0, 1), 8))
    # universal M3 grid every 10 mm on the floor (mount modules on standoffs)
    gx = range(10, 61, 10) if front else range(-60, -9, 10)
    for x in gx:
        for y in range(-60, 61, 10):
            if front and 6 <= x <= 66 and -44 <= y <= 44:
                continue  # Raspberry Pi area (it has its own standoffs)
            tools.append(cyl(1.6, (x, y, FLOOR_Z0 - 1), (0, 0, 1), 5))
    # vents under the Raspberry Pi
    if front:
        for y in range(-30, 31, 10):
            tools.append(box(20, 52, y - 2, y + 2, FLOOR_Z0 - 1, FLOOR_Z1 + 1))
        for (px, py) in PI_HOLES:
            tools.append(cyl(M25_TAP / 2, (px, py, FLOOR_Z0 - 1), (0, 0, 1), 12))
    if front:
        # bulkhead: drill motor shaft + 12 holes + cable windows
        tools.append(cyl(7.5, (BULK_X0 - 1, 0, DRILL_Z), (1, 0, 0), 8))
        for (dy, dz) in ring_points(12, MOT_PCD / 2, 0):
            tools.append(cyl(M3_CLR / 2, (BULK_X0 - 1, dy, DRILL_Z + MOT_OFFSET + dz), (1, 0, 0), 8))
        for yy in (-1, 1):
            tools.append(box(BULK_X0 - 1, BULK_X1 + 1, 30 if yy > 0 else -64, 64 if yy > 0 else -30, 30, 80))
        tools.append(box(BULK_X0 - 1, BULK_X1 + 1, -16, 16, 28, 62))
        # front wall: hole for the coupler/shaft, collar screws and ToF sensor windows
        tools.append(cyl(18, (CH_X1 - T - 1, 0, DRILL_Z), (1, 0, 0), T + 2))
        for yy in (-1, 1):
            a = math.radians(-50 if yy > 0 else -130)
            tools.append(cyl(M3_CLR / 2, (CH_X1 - T - 1, 58 * math.cos(a), DRILL_Z + 58 * math.sin(a)),
                             (1, 0, 0), T + 2))
            yc = yy * TOF_Y
            tools.append(box(CH_X1 - T - 3.7, CH_X1 - T - 1.5, yc - 13, yc + 13, FLOOR_Z1, 41))  # PCB slot
            tools.append(box(CH_X1 - T - 1, CH_X1 + 1, yc - 7, yc + 7, 25.5, 34.5))           # window
    else:
        # back wall: main switch Ø12.5, battery lead exit, vent slots
        tools.append(cyl(6.25, (CH_X0 - 1, -35, 55), (1, 0, 0), T + 2))
        tools.append(cyl(6.25, (CH_X0 - 1, 35, 55), (1, 0, 0), T + 2))
        for y in range(-12, 13, 6):
            tools.append(box(CH_X0 - 1, CH_X0 + T + 1, y - 1.75, y + 1.75, 35, 75))
        # slots for the battery's Velcro strap
        for x in (-50, -22):
            for yy in (40, -40):
                tools.append(box(x - 11, x + 11, yy - 1.6, yy + 1.6, FLOOR_Z0 - 1, FLOOR_Z1 + 1))
    return cut(body, *tools)


TOF_Y = 57.0   # lateral position of the ToF sensors (inside the chassis)

# Raspberry Pi 5 lying on the front floor (long side along Y, USB ports to the left)
PI_X0, PI_Y0 = 8.0, -42.5
PI_HOLES = [(PI_X0 + 3.5, PI_Y0 + 3.5), (PI_X0 + 52.5, PI_Y0 + 3.5),
            (PI_X0 + 3.5, PI_Y0 + 61.5), (PI_X0 + 52.5, PI_Y0 + 61.5)]


def deck():
    """Upper deck: the 3 BTS7960 drivers and the ESP32 screw onto its grid."""
    W = CH_YW - WALL - 0.5
    z0, z1 = 67.0, 70.0
    body = box(-64, 64, -W, W, z0, z1)
    tools = []
    for x in range(-55, 56, 10):
        for y in range(-65, 66, 10):
            tools.append(cyl(1.6, (x, y, z0 - 1), (0, 0, 1), 5))
    # cable pass-throughs
    for (x, y) in [(0, 0), (-30, 0), (30, 0)]:
        tools.append(cq.Workplane("XY").workplane(offset=z0 - 1).center(x, y)
                     .slot2D(34, 10, 90).extrude(5).val())
    # screws into the rails
    for hx in (-58, -12, 12, 58):
        for yy in (W - 1.5, -(W - 1.5)):
            tools.append(cyl(M3_CLR / 2, (hx, yy, z0 - 1), (0, 0, 1), 5))
    return cut(body, *tools)


# ----------------------------------------------------------------------------------------
# COLLAR + SPIDER CARRIER (fixed part that holds the differential)
# Local coordinates: drill axis = +Z, z = 0 on the back face (against the chassis)
# ----------------------------------------------------------------------------------------
def collar_carrier():
    R = 70.0
    b = [cyl(R, (0, 0, 0), (0, 0, 1), 6),                    # flange
         tube(R, 64, (0, 0, 0), (0, 0, 1), 16),             # band with the LED channel
         cyl(20, (0, 0, 0), (0, 0, 1), 22),                 # bearing hub
         tube(40, 33, (0, 0, 0), (0, 0, 1), X_PLATE_FRONT - COLLAR_X - 12.6)]  # cage
    zc = XC - COLLAR_X
    # bosses that support the spider gears
    for k in range(4):
        a = math.radians(k * 90)
        d = (math.cos(a), math.sin(a), 0)
        r0 = CONE_H_SPIDER + 1.0
        b.append(cyl(5, (d[0] * r0, d[1] * r0, zc), d, 34 - r0))
        t = (-math.sin(a), math.cos(a), 0)              # tangential direction
        gus = prism([(-5, 0), (5, 0), (0, -6.5)],
                    cq.Plane(origin=V(d[0] * r0, d[1] * r0, zc), xDir=V(*t), normal=V(*d)), 34 - r0)
        b.append(gus)
    body = fuse(*b)
    tools = []
    # channel for the WS2812 LED strip (10 mm) around the band
    tools.append(tube(R + 1, R - 2.5, (0, 0, 2.5), (0, 0, 1), 11))
    # seats for the 608 bearings (one at the back, one at the front) and shaft clearance
    tools.append(cyl(B608_OD / 2, (0, 0, -1), (0, 0, 1), B608_W + 1))
    tools.append(cyl(6.5, (0, 0, 0), (0, 0, 1), 30))
    tools.append(cyl(B608_OD / 2, (0, 0, 22 - B608_W), (0, 0, 1), B608_W + 1))
    # M4 pins for the spider gears
    for k in range(4):
        a = math.radians(k * 90)
        d = (-math.cos(a), -math.sin(a), 0)
        tools.append(cyl(M4_TAP / 2, (math.cos(a) * 45, math.sin(a) * 45, zc), d, 30))
    # windows in the cage (to insert the spider gears and see the mechanism)
    for k in range(4):
        win = prism([(-17, 10), (17, 10), (17, 36), (0, 53), (-17, 36)],      # 45° pointed top
                    cq.Plane(origin=V(28, 0, 0), xDir=V(0, 1, 0), normal=V(1, 0, 0)), 16)
        tools.append(win.rotate(V(0, 0, 0), V(0, 0, 1), 45 + 90 * k))
    # flange screws (angles measured in the rover's Y-Z plane)
    for ang in (-50, -130, 30, 150, 90):
        a = math.radians(ang)
        yy, zz = 58 * math.cos(a), 58 * math.sin(a)
        tools.append(cyl(M3_TAP / 2, (-zz, yy, -1), (0, 0, 1), 8))
    # decorative lightening holes in the flange
    for k in range(8):
        a = math.radians(22.5 + 45 * k)
        tools.append(cyl(5.5, (47 * math.cos(a), 47 * math.sin(a), -1), (0, 0, 1), 8))
    return cut(body, *tools)


COLLAR_LOC = loc((COLLAR_X, 0, DRILL_Z), (0, 1, 0), 90)          # local +Z -> +X


# ----------------------------------------------------------------------------------------
# COUNTER-ROTATING DIFFERENTIAL
# "dif" frame: axis = +Z, origin at the common apex. A at z<0, B at z>0 (B = mirror of A).
# ----------------------------------------------------------------------------------------
def _gear_side_dif():
    g = _PAIR.gear.build(bore_d=None)
    return g.moved(loc((0, 0, -CONE_H_SIDE)))


def _spider_loc_dif(k):
    """Location of spider gear k (0..3) in the dif frame."""
    l = loc((0, 0, CONE_H_SIDE), (0, 1, 0), 90) * loc((0, 0, -CONE_H_SPIDER))
    if GEAR_Z_SPIDER % 2 == 0:
        l = l * loc((0, 0, 0), (0, 0, 1), 180.0 / GEAR_Z_SPIDER)
    return loc((0, 0, 0), (0, 0, 1), 90 * k) * loc((0, 0, -CONE_H_SIDE)) * l


def gear_A():
    """Side gear A (turns with the shaft and the front cone). Local: back face at z = 0."""
    g = _PAIR.gear.build(bore_d=None)
    return cut(g, d_bore(40, (0, 0, -5)))


def spider():
    p = _PAIR.pinion.build(bore_d=None)
    return cut(p, cyl(M4_CLR / 2, (0, 0, -1), (0, 0, 1), 40))


DIF_LOC = loc((XC, 0, DRILL_Z), (0, 1, 0), 90)                   # dif frame -> assembly
GEAR_A_LOC = DIF_LOC * loc((0, 0, -CONE_H_SIDE))


def spider_locs():
    return [DIF_LOC * _spider_loc_dif(k) for k in range(4)]


# ----------------------------------------------------------------------------------------
# REAR CUTTER (counter-rotating) with gear B built in
# Local: z = 0 on the front face of the end plate (print bed), z grows backwards.
# ----------------------------------------------------------------------------------------
REAR_LOC = loc((X_PLATE_FRONT, 0, DRILL_Z), (0, 1, 0), -90)      # local +Z -> -X


def rear_cutter():
    L = X_PLATE_FRONT - X_RIM
    t = 3.0
    shell = cone(R_REAR_FRONT, R_REAR_RIM, (0, 0, 0), (0, 0, 1), L).cut(
        cone(R_REAR_FRONT - t, R_REAR_RIM - t, (0, 0, PLATE_T), (0, 0, 1), L - PLATE_T + 0.01))
    parts = [shell]
    # gear B = mirror of A, moved into this cutter's local frame
    gB = _gear_side_dif().mirror("XY")
    gB = gB.moved(REAR_LOC.inverse * DIF_LOC)
    parts.append(gB)
    # helical blades (left-hand helix: they push soil backwards while turning the other way)
    ang = math.degrees(math.atan((R_REAR_RIM - R_REAR_FRONT) / L))
    for k in range(6):
        z0 = 3.0
        hgt = (L - 9) / math.cos(math.radians(ang))
        r0 = R_REAR_FRONT + (R_REAR_RIM - R_REAR_FRONT) * z0 / L - 0.8
        h = cq.Wire.makeHelix(pitch=150 / math.cos(math.radians(ang)), height=hgt, radius=r0,
                              center=V(0, 0, z0), angle=ang, lefthand=True)
        prof = cq.Workplane(cq.Plane(origin=V(r0, 0, z0), xDir=V(1, 0, 0), normal=V(0, -1, 0))) \
            .polyline([(0, -6.5), (5.0, -1.5), (5.0, 1.5), (0, 6.5)]).close()
        blade = prof.sweep(cq.Workplane().add(h), isFrenet=True).val()
        parts.append(blade.rotate(V(0, 0, 0), V(0, 0, 1), 60 * k))
    # cutting teeth on the rim (tunnel-boring-machine style)
    for k in range(12):
        a = 15 + 30 * k
        zt = L - 7
        rt = R_REAR_FRONT + (R_REAR_RIM - R_REAR_FRONT) * zt / L
        tooth = prism([(rt - 2, zt - 4), (rt + 3.5, zt + 1.5), (rt + 3.5, zt + 4), (rt - 2, zt + 4)],
                      cq.Plane(origin=V(0, -3, 0), xDir=V(1, 0, 0), normal=V(0, -1, 0)), -6)
        tooth = tooth.rotate(V(0, 0, 0), V(0, 0, 1), a)
        parts.append(tooth)
    body = fuse(*parts)
    tools = [cyl(B608_OD / 2, (0, 0, -1), (0, 0, 1), B608_W + 1),        # 608 bearing
             cyl(7.0, (0, 0, 0), (0, 0, 1), 30),                         # shaft clearance
             box(-100, 100, -100, 100, -20, 0),                          # flat base on the bed
             box(-100, 100, -100, 100, L, L + 20)]
    return cut(body, *tools)


# ----------------------------------------------------------------------------------------
# FRONT CONE (turns with the shaft) — local: base at z = 0 (bed), tip towards +Z
# ----------------------------------------------------------------------------------------
CONE_LOC = loc((X_CONE, 0, DRILL_Z), (0, 1, 0), 90)


def front_cone():
    L, R0, Rt = CONE_LEN, R_CONE_BASE, 4.0
    body = cone(R0, Rt, (0, 0, 0), (0, 0, 1), L)
    parts = [body]
    ang = math.degrees(math.atan((R0 - Rt) / L))
    for k in range(3):
        z0 = 2.0
        r0 = R0 - (R0 - Rt) * z0 / L - 0.8
        hgt = (L - 14) / math.cos(math.radians(ang))
        h = cq.Wire.makeHelix(pitch=75 / math.cos(math.radians(ang)), height=hgt, radius=r0,
                              center=V(0, 0, z0), angle=-ang, lefthand=False)
        prof = cq.Workplane(cq.Plane(origin=V(r0, 0, z0), xDir=V(1, 0, 0), normal=V(0, 1, 0))) \
            .polyline([(0, -5.5), (4.5, -1.0), (4.5, 1.0), (0, 5.5)]).close()
        ridge = prof.sweep(cq.Workplane().add(h), isFrenet=True).val()
        parts.append(ridge.rotate(V(0, 0, 0), V(0, 0, 1), 120 * k))
    # ring of teeth at the base
    for k in range(10):
        tooth = box(R0 - 3, R0 + 2.5, -3, 3, 0, 8).rotate(V(0, 0, 0), V(0, 0, 1), 36 * k + 18)
        parts.append(tooth)
    # carbide-style tip (decorative)
    parts.append(cone(Rt + 1, 0.8, (0, 0, L - 1), (0, 0, 1), 9))
    body = fuse(*parts)
    tools = [d_bore(42, (0, 0, -1)),
             box(-100, 100, -100, 100, -20, 0),                          # flat base on the bed
             # captive M3 nut (inserted from below) + radial M3x16 screw with counterbore
             box(5.0, 7.9, -3.0, 3.0, -1, 11),
             cyl(1.7, (0, 0, 7), (1, 0, 0), 30),
             cyl(3.3, (16, 0, 7), (1, 0, 0), 40)]
    return cut(body, *tools)


def spacer():
    """Spacer between the collar's front bearing and gear A."""
    L = X_A_BACK - (COLLAR_X + 22)
    return tube(6.5, 4.2, (0, 0, 0), (0, 0, 1), L)


SPACER_LOC = loc((COLLAR_X + 22, 0, DRILL_Z), (0, 1, 0), 90)


def led_diffuser():
    return tube(71.6, 70.15, (0, 0, 0), (0, 0, 1), 12.5)


DIFFUSER_LOC = loc((COLLAR_X + 2, 0, DRILL_Z), (0, 1, 0), 90)


# ----------------------------------------------------------------------------------------
# HULL (2 halves) AND CAB
# ----------------------------------------------------------------------------------------
HULL_R = CH_YW


def _half_cyl(r, x0, x1):
    c = cyl(r, (x0, 0, DRILL_Z), (1, 0, 0), x1 - x0)
    return c.intersect(box(x0 - 1, x1 + 1, -r - 1, r + 1, DRILL_Z, DRILL_Z + r + 1))


def _hull_tabs(xs, bed_dir):
    """Screw tabs. bed_dir = +1 if the print bed is towards +X (front hull).
    They have a 45° gusset towards the bed so they print without supports."""
    out, tools = [], []
    for x in xs:
        for s in (1, -1):
            y0 = HULL_R - WALL - 3.5 if s > 0 else -HULL_R + WALL
            g = bed_dir
            prof = [(x - 6 * g, 78), (x + 6 * g, 78), (x + 18 * g, DRILL_Z), (x + 18 * g, DRILL_Z + 8),
                    (x - 6 * g, DRILL_Z + 8)]
            if g < 0:
                prof = prof[::-1]
            tab = prism(prof, cq.Plane(origin=V(0, y0, 0), xDir=V(1, 0, 0), normal=V(0, -1, 0)), -3.5)
            out.append(tab)
            tools.append(cyl(M3_TAP / 2, (x, -HULL_R, 84), (0, 1, 0), 2 * HULL_R))
    return out, tools


def hull_front():
    x0, x1 = 0.0, CH_X1
    R, T = HULL_R, WALL
    shell = _half_cyl(R, x0, x1).cut(_half_cyl(R - T, x0 - 1, x1 + 1))
    plate = _half_cyl(R - 0.01, x1 - T, x1).cut(cyl(40, (x1 - T - 1, 0, DRILL_Z), (1, 0, 0), T + 2))
    lip = _half_cyl(R - T - 0.25, x0 - 5, x0 + 2).cut(_half_cyl(R - T - 2.6, x0 - 6, x0 + 3))
    tabs, ttools = _hull_tabs((25, 128), +1)
    # raised decorative hatches
    deco = []
    for xh in (38, 78):
        for s in (1, -1):
            pad = _half_cyl(R + 1.2, xh, xh + 28).cut(_half_cyl(R - 0.5, xh - 1, xh + 29))
            band = box(xh - 1, xh + 29, -200, 200, DRILL_Z + 22, DRILL_Z + 48)
            pad = pad.intersect(band)
            pad = pad.intersect(box(xh - 1, xh + 29, 0, 200, 0, 300) if s > 0
                                else box(xh - 1, xh + 29, -200, 0, 0, 300))
            deco.append(pad)
    body = fuse(shell, plate, lip, *tabs, *deco)
    tools = list(ttools)
    # panel lines
    for xr in (12, 120):
        tools.append(tube(R + 2, R - 0.8, (xr, 0, DRILL_Z), (1, 0, 0), 1.2))
    # collar screws in the front plate
    for ang in (30, 150, 90):
        a = math.radians(ang)
        tools.append(cyl(M3_CLR / 2, (x1 - T - 1, 58 * math.cos(a), DRILL_Z + 58 * math.sin(a)), (1, 0, 0), T + 2))
    # cab screws and slot for the camera cable
    for (cx, cy) in CAB_SCREWS:
        tools.append(cyl(M3_CLR / 2, (cx, cy, DRILL_Z), (0, 0, 1), R + 10))
    tools.append(box(118, 122, -9, 9, DRILL_Z + 50, DRILL_Z + R + 5))
    return cut(body, *tools)


def hull_rear():
    x0, x1 = CH_X0, 0.0
    R, T = HULL_R, WALL
    shell = _half_cyl(R, x0, x1).cut(_half_cyl(R - T, x0 - 1, x1 + 1))
    plate = _half_cyl(R - 0.01, x0, x0 + T)
    tabs, ttools = _hull_tabs((-25, -125), -1)
    parts = [shell, plate] + tabs
    tools = list(ttools)
    # raised side louvers with slots along X
    for s in (1, -1):
        frame = _half_cyl(R + 2.0, -122, -58).cut(_half_cyl(R - 0.5, -123, -57))
        frame = frame.intersect(box(-123, -57, -200, 200, DRILL_Z + 15, DRILL_Z + 65))
        frame = frame.intersect(box(-123, -57, 0, 200, 0, 300) if s > 0 else box(-123, -57, -200, 0, 0, 300))
        parts.append(frame)
        for k in range(5):
            a = math.radians(22 + 8 * k)
            yy, zz = s * (R) * math.cos(a), DRILL_Z + R * math.sin(a)
            slot = box(-116, -64, -1.6, 1.6, -6, 6)
            slot = slot.rotate(V(0, 0, 0), V(1, 0, 0), -s * (90 - math.degrees(a)))
            tools.append(slot.translate(V(0, yy, zz)))
    # rear vent grille
    for y in range(-40, 41, 8):
        tools.append(box(x0 - 1, x0 + T + 1, y - 2, y + 2, DRILL_Z + 10, DRILL_Z + 45))
    for xr in (-20, -138):
        tools.append(tube(R + 2, R - 0.8, (xr, 0, DRILL_Z), (1, 0, 0), 1.2))
    return cut(fuse(*parts), *tools)


CAB_X0, CAB_X1, CAB_YW = 62.0, 132.0, 32.0
CAB_SCREWS = [(70, 22), (70, -22), (112, 22), (112, -22)]
CAM_TILT = 15.0          # the camera looks 15° down


def _cab_profile(off=0.0):
    """Side profile of the cab. Flat roof (printed upside down) and a visor tilted 15°."""
    tilt = math.tan(math.radians(CAM_TILT))
    xb = CAB_X1 - 34 * tilt
    top = 196 - off
    pts = [(CAB_X0 + off, 140), (CAB_X0 + off, top - 6 + off * 0.4), (CAB_X0 + 6 + off * 0.6, top),
           (CAB_X1 - off, top), (xb - off, 162), (xb - off, 140)]
    return pts


def cab():
    prof = _cab_profile(0)
    # XZ plane with normal -Y: the plane's "v" axis is +Z; it extrudes towards -Y
    outer = prism(prof, cq.Plane(origin=V(0, CAB_YW, 0), xDir=V(1, 0, 0), normal=V(0, -1, 0)), 2 * CAB_YW)
    inner = prism(_cab_profile(2.5),
                  cq.Plane(origin=V(0, CAB_YW - 2.5, 0), xDir=V(1, 0, 0), normal=V(0, -1, 0)), 2 * CAB_YW - 5)
    hullcyl = cyl(HULL_R + 0.2, (CAB_X0 - 5, 0, DRILL_Z), (1, 0, 0), CAB_X1 - CAB_X0 + 10)
    body = outer.cut(inner).cut(hullcyl)
    # screw posts (screwed from inside the hull)
    posts = []
    for (cx, cy) in CAB_SCREWS:
        p = cyl(4, (cx, cy, 150), (0, 0, 1), 40).cut(hullcyl)
        posts.append(p.intersect(outer))
    body = fuse(body, *posts)
    tools = [cyl(M3_TAP / 2, (cx, cy, 150), (0, 0, 1), 30) for (cx, cy) in CAB_SCREWS]
    # Raspberry Pi Camera Module 3 in the visor
    tilt = math.radians(CAM_TILT)
    xb = CAB_X1 - 34 * math.tan(tilt)
    fc = V(xb + (183 - 162) * math.tan(tilt), 0, 183)        # lens center on the visor
    n = V(math.cos(tilt), 0, -math.sin(tilt))                 # outward normal (15° down)
    up = V(math.sin(tilt), 0, math.cos(tilt))
    lens = fc
    pl_out = cq.Plane(origin=lens + up * (-4.0) + n * 2, xDir=V(0, 1, 0), normal=-n)
    tools.append(cq.Workplane(pl_out).rect(14, 19).extrude(8).val())          # lens window
    standoffs = []
    for (dx, dz) in [(-10.5, 0), (10.5, 0), (-10.5, -12.5), (10.5, -12.5)]:
        c = lens + V(0, dx, 0) + up * dz
        pl = cq.Plane(origin=c - n * 2.0, xDir=V(0, 1, 0), normal=-n)
        standoffs.append(cq.Workplane(pl).circle(2.6).extrude(3.0).val())
        tools.append(cq.Workplane(cq.Plane(origin=c - n * 1.0, xDir=V(0, 1, 0), normal=-n))
                     .circle(M2_TAP / 2).extrude(5).val())
    body = fuse(body, *standoffs)
    # side vision slits and hazard lights (decorative)
    for s in (1, -1):
        tools.append(box(80, 112, s * CAB_YW - 3, s * CAB_YW + 3, 182, 186))
        tools.append(box(80, 112, s * CAB_YW - 3, s * CAB_YW + 3, 174, 177))
    return cut(body, *tools)


# ----------------------------------------------------------------------------------------
# WHEELS: rim (PLA/PETG) + tire (TPU 95A)
# Local: axis = +Z, z = 0 on the inner face (the one facing the chassis)
# ----------------------------------------------------------------------------------------
RIM_R = 37.0


def wheel_rim():
    W = WHEEL_W
    shell = tube(RIM_R, RIM_R - 4, (0, 0, 0), (0, 0, 1), W)
    flange_in = tube(RIM_R + 2.5, RIM_R - 1, (0, 0, 0), (0, 0, 1), 1.5)
    flange_out = cone(RIM_R, RIM_R + 2.5, (0, 0, W - 4), (0, 0, 1), 2.5).fuse(
        cyl(RIM_R + 2.5, (0, 0, W - 1.5), (0, 0, 1), 1.5)).cut(cyl(RIM_R - 1, (0, 0, W - 5), (0, 0, 1), 6))
    web = tube(RIM_R - 2, 11, (0, 0, 0), (0, 0, 1), 6)
    hub = cyl(12, (0, 0, 0), (0, 0, 1), W - 4)
    ribs = [box(RIM_R - 1, RIM_R + 1.4, -2, 2, 1.5, W - 4).rotate(V(0, 0, 0), V(0, 0, 1), 45 * k) for k in range(8)]
    spokes = [box(10, RIM_R - 2, -3, 3, 0, 22).rotate(V(0, 0, 0), V(0, 0, 1), 60 * k + 30) for k in range(6)]
    body = fuse(shell, flange_in, flange_out, web, hub, *ribs, *spokes)
    tools = [hex_prism(12.25, 11, (0, 0, -1)),                 # 12 mm hex coupler
             cyl(8.4, (0, 0, -1), (0, 0, 1), 2.5),             # coupler collar
             cyl(M4_CLR / 2, (0, 0, 0), (0, 0, 1), W),
             cyl(4.2, (0, 0, W - 9), (0, 0, 1), 10)]           # M4 screw head
    for k in range(6):
        a = math.radians(60 * k)
        tools.append(cyl(5.5, (23 * math.cos(a), 23 * math.sin(a), -1), (0, 0, 1), 8))
    return cut(body, *tools)


def tire():
    W = WHEEL_W - 5.5          # from z = 1.5 to z = 36 (ends before the outer flange chamfer)
    base = tube(44.0, RIM_R - 0.25, (0, 0, 1.5), (0, 0, 1), W)
    lugs = []
    n = 16
    for row, z0 in ((0, 1.5), (1, 1.5 + W / 2 + 0.5)):
        z1 = z0 + W / 2 - 0.5
        for k in range(n):
            a = 360 / n * k + (360 / n / 2 if row else 0)
            if row == 0:
                lug = box(43, WHEEL_R, -4.2, 4.2, z0, z1)
            else:   # the second row starts with a 45° chamfer so it prints without supports
                lug = prism([(43.5, z0 - 6.5), (WHEEL_R, z0), (WHEEL_R, z1), (43.5, z1)],
                            cq.Plane(origin=V(0, -4.2, 0), xDir=V(1, 0, 0), normal=V(0, -1, 0)), -8.4)
            lugs.append(lug.rotate(V(0, 0, 0), V(0, 0, 1), a))
    body = fuse(base, *lugs)
    tools = [box(RIM_R - 1.5, RIM_R + 1.7, -2.25, 2.25, 0, WHEEL_W).rotate(V(0, 0, 0), V(0, 0, 1), 45 * k)
             for k in range(8)]
    return cut(body, *tools)


def wheel_locs():
    out = []
    for xs in (AXLE_X, -AXLE_X):
        y_in = CH_YW + WHEEL_GAP
        out.append(loc((xs, y_in, AXLE_Z), (1, 0, 0), -90))     # left:  local +Z -> +Y
        out.append(loc((xs, -y_in, AXLE_Z), (1, 0, 0), 90))     # right: local +Z -> -Y
    return out


# ----------------------------------------------------------------------------------------
# BOUGHT PARTS (only for the assembly / 3D viewer, not printed)
# ----------------------------------------------------------------------------------------
def motor_model(shaft_len=MOT_SHAFT_L):
    """JGB37-520 with encoder. Local: front face at z = 0, shaft at the origin pointing to +Z,
    the body is 7 mm off-center towards local +Y and extends towards -Z."""
    c = (0, MOT_OFFSET)
    gb = cyl(MOT_GB_D / 2, (c[0], c[1], -26.5), (0, 0, 1), 26.5)
    body = cyl(MOT_BODY_D / 2, (c[0], c[1], -48.5), (0, 0, 1), 22)
    enc = cyl(MOT_BODY_D / 2, (c[0], c[1], -60.5), (0, 0, 1), 12)
    boss = cyl(6, (0, 0, 0), (0, 0, 1), 2)
    shaft = cyl(3, (0, 0, 0), (0, 0, 1), shaft_len)
    return fuse(gb, body, enc, boss, shaft)


def motor_locs():
    """Drive motors (4) + drill motor."""
    out = []
    for xs in (AXLE_X, -AXLE_X):
        # left: face on the inner wall (y = 72), shaft towards +Y, body pointing up
        out.append(("drive_motor", loc((xs, CH_YW - WALL, AXLE_Z), (1, 0, 0), -90)
                    * loc((0, 0, 0), (0, 0, 1), 180)))
        out.append(("drive_motor", loc((xs, -CH_YW + WALL, AXLE_Z), (1, 0, 0), 90)))
    out.append(("drill_motor", loc((BULK_X0, 0, DRILL_Z), (0, 1, 0), 90) * loc((0, 0, 0), (0, 0, 1), 90)))
    return out


def bought_models():
    """Returns a list of (name, shape in assembly coordinates, color)."""
    out = []
    mm = motor_model()
    for name, l in motor_locs():
        out.append((name, mm.moved(l), (70, 74, 82)))
    # 8 mm rod, flexible coupler, bearings
    x_rod0 = BULK_X1 + 13.5
    out.append(("shaft_8mm", cyl(4, (x_rod0, 0, DRILL_Z), (1, 0, 0), 130), (200, 200, 205)))
    out.append(("coupler_6to8", cyl(9.5, (BULK_X1 + 0.5, 0, DRILL_Z), (1, 0, 0), 25), (190, 190, 200)))
    for xb in (COLLAR_X, COLLAR_X + 22 - B608_W, X_PLATE_FRONT - B608_W):
        out.append(("bearing_608", tube(11, 4, (xb, 0, DRILL_Z), (1, 0, 0), 7), (160, 160, 170)))
    # M4 screws used as spider gear pins
    for k in range(4):
        a = math.radians(k * 90)
        # in the collar's local frame: direction (cos, sin, 0) -> assembly (Z = -x_local, Y = y_local)
        dy, dz = math.sin(a), -math.cos(a)
        out.append(("pin_M4", cyl(2, (XC, dy * 16, DRILL_Z + dz * 16), (0, dy, dz), 26), (120, 120, 130)))
    # Raspberry Pi 5 (on the front floor)
    z = FLOOR_Z1 + 6
    pi = box(PI_X0, PI_X0 + 56, PI_Y0, PI_Y0 + 85, z, z + 1.6)
    pi = fuse(pi, box(PI_X0 + 13, PI_X0 + 43, PI_Y0 + 15, PI_Y0 + 45, z + 1.6, z + 14),  # active cooler
              box(PI_X0 + 2, PI_X0 + 54, PI_Y0 + 66, PI_Y0 + 86, z + 1.6, z + 17))      # USB/Ethernet
    out.append(("raspberry_pi5", pi, (30, 120, 70)))
    # 3S2P 18650 battery (rear, under the deck)
    out.append(("battery_3S2P", box(-64, -8, -35, 35, FLOOR_Z1, FLOOR_Z1 + 38), (40, 90, 170)))
    # DC-DC converters
    out.append(("buck_5V5A_pi", box(-62, -8, 42, 66, FLOOR_Z1 + 5, FLOOR_Z1 + 25), (40, 60, 140)))
    out.append(("buck_5V3A", box(-60, -18, -64, -42, FLOOR_Z1 + 5, FLOOR_Z1 + 20), (40, 60, 140)))
    # BTS7960 drivers and ESP32 on the deck
    dz = 70 + 5
    for (x0, y0, name) in [(-60, 10, "bts7960_left"), (-60, -62, "bts7960_right"), (0, 10, "bts7960_drill")]:
        b = fuse(box(x0, x0 + 50, y0, y0 + 50, dz, dz + 1.6),
                 box(x0 + 4, x0 + 46, y0 + 5, y0 + 45, dz + 1.6, dz + 38))
        out.append((name, b, (180, 40, 40)))
    esp = fuse(box(0, 52, -62, -10, dz, dz + 1.6), box(12, 40, -54, -18, dz + 1.6, dz + 14))
    out.append(("esp32_terminal_board", esp, (25, 25, 30)))
    # camera
    tilt = math.radians(CAM_TILT)
    xb = CAB_X1 - 34 * math.tan(tilt)
    n = V(math.cos(tilt), 0, -math.sin(tilt))
    cam = box(-1.0, 0, -12.5, 12.5, -16.5, 7.5).fuse(cyl(4.3, (0, 0, 0), (1, 0, 0), 4.5))
    cam = cam.rotate(V(0, 0, 0), V(0, 1, 0), CAM_TILT).translate(
        V(xb + (183 - 162) * math.tan(tilt), 0, 183) - n * 5.0)
    out.append(("camera_pi3", cam, (40, 140, 60)))
    # ToF sensors and IMU
    for yy in (TOF_Y, -TOF_Y):
        out.append(("vl53l0x", box(CH_X1 - WALL - 3.6, CH_X1 - WALL - 2.0, yy - 12.5, yy + 12.5, 23.5, 36.5),
                    (110, 40, 160)))
    out.append(("mpu6050", box(-58, -37, -8, 8, 70 + 5, 70 + 6.6), (40, 40, 160)))
    # LED strip (ring)
    out.append(("ws2812_strip", tube(69.6, 67.8, (COLLAR_X + 3, 0, DRILL_Z), (1, 0, 0), 10), (255, 245, 210)))
    return out


# ----------------------------------------------------------------------------------------
# PRINTABLE PARTS REGISTRY
#   name: (function, locations in the assembly, print rotation, color, material, notes)
# ----------------------------------------------------------------------------------------
PRINT_ROT = {
    # rotations (axis, angle) applied to the LOCAL part to put it in printing position
    "chassis_front": [],
    "chassis_rear": [],
    "electronics_deck": [],
    "collar_carrier": [],
    "gear_A": [],
    "spider_gear": [],
    "rear_cutter": [],
    "front_cone": [],
    "shaft_spacer": [],
    "led_diffuser": [],
    "hull_front": [((0, 1, 0), 90)],        # printed standing on its front plate
    "hull_rear": [((0, 1, 0), -90)],        # standing on its back plate
    "cab": [((1, 0, 0), 180)],              # upside down, on its flat roof
    "wheel_rim": [],
    "wheel_tire_TPU": [],
}


def build_parts():
    P = {}
    P["chassis_front"] = dict(fn=lambda: chassis_half(True), locs=[loc()], color=(214, 120, 40),
                              mat="PETG", qty=1, note="3 mm walls, 4 perimeters, 25 % infill")
    P["chassis_rear"] = dict(fn=lambda: chassis_half(False), locs=[loc()], color=(214, 120, 40),
                             mat="PETG", qty=1, note="Same settings as the front half")
    P["electronics_deck"] = dict(fn=deck, locs=[loc()], color=(60, 60, 64), mat="PLA/PETG", qty=1,
                                 note="M3 grid every 10 mm")
    P["collar_carrier"] = dict(fn=collar_carrier, locs=[COLLAR_LOC], color=(225, 170, 40), mat="PETG",
                               qty=1, note="Fixed part: holds 2 bearings and the 4 spider gears")
    P["gear_A"] = dict(fn=gear_A, locs=[GEAR_A_LOC], color=(240, 200, 60), mat="PETG/Nylon", qty=1,
                       note="100 % infill or 6 perimeters. D-shaped bore")
    P["spider_gear"] = dict(fn=spider, locs=spider_locs(), color=(170, 185, 205), mat="PETG/Nylon", qty=4,
                            note="100 % infill")
    P["rear_cutter"] = dict(fn=rear_cutter, locs=[REAR_LOC], color=(120, 170, 230), mat="PETG", qty=1,
                            note="Includes gear B. Turns OPPOSITE to the cone")
    P["front_cone"] = dict(fn=front_cone, locs=[CONE_LOC], color=(235, 95, 60), mat="PETG", qty=1,
                           note="Turns with the shaft. 3 perimeters, 15 % infill")
    P["shaft_spacer"] = dict(fn=spacer, locs=[SPACER_LOC], color=(240, 200, 60), mat="PLA", qty=1,
                             note="Goes between the bearing and gear A")
    P["led_diffuser"] = dict(fn=led_diffuser, locs=[DIFFUSER_LOC], color=(240, 240, 250), mat="Clear PETG",
                             qty=1, note="Vase mode or 2 perimeters")
    P["hull_front"] = dict(fn=hull_front, locs=[loc()], color=(214, 120, 40), mat="PLA/PETG", qty=1,
                           note="Print standing up, no supports")
    P["hull_rear"] = dict(fn=hull_rear, locs=[loc()], color=(214, 120, 40), mat="PLA/PETG", qty=1,
                          note="Print standing up, no supports. Remove it to swap the battery")
    P["cab"] = dict(fn=cab, locs=[loc()], color=(60, 60, 64), mat="PLA/PETG", qty=1,
                    note="Print upside down on its roof. Holds the camera")
    P["wheel_rim"] = dict(fn=wheel_rim, locs=wheel_locs(), color=(60, 60, 64), mat="PETG", qty=4,
                          note="12 mm hex coupler + M4 screw")
    P["wheel_tire_TPU"] = dict(fn=tire, locs=wheel_locs(), color=(35, 35, 38), mat="TPU 95A", qty=4,
                               note="No TPU? Print it in PETG")
    return P


if __name__ == "__main__":
    import export_all
    export_all.main(sys.argv[1:])
