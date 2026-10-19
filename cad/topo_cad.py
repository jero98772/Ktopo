"""
TOPO S14 — Carro perforador impreso en 3D
Modelo CAD paramétrico (CadQuery). Genera todas las piezas imprimibles en STL (orientadas
para imprimir) y STEP (para editar en Fusion 360 / FreeCAD / Onshape), más el ensamblaje.

Sistema de coordenadas del ensamblaje (mm):
  X = hacia adelante (la punta del taladro está en +X)
  Y = hacia la izquierda
  Z = hacia arriba (Z = 0 es el piso)

Uso:  python3 topo_cad.py            -> exporta todo a ./salida
      python3 topo_cad.py rueda_rin  -> exporta solo esa pieza
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))
import cadquery as cq  # noqa: E402
import cq_gears  # noqa: E402

V = cq.Vector

# ----------------------------------------------------------------------------------------
# PARÁMETROS PRINCIPALES
# ----------------------------------------------------------------------------------------
CH_X0, CH_X1 = -150.0, 150.0        # largo del chasis (300 mm, se imprime en 2 mitades)
CH_YW = 75.0                         # medio ancho exterior del chasis (150 mm)
WALL = 3.0                           # pared
FLOOR_Z0, FLOOR_Z1 = 20.0, 23.0      # piso del chasis (20 mm de altura libre)
CH_TOP = 90.0                        # borde superior de la bandeja / eje del casco y taladro

WHEEL_R = 50.0                       # rueda Ø100
WHEEL_W = 40.0
WHEEL_GAP = 3.0
AXLE_Z = WHEEL_R
AXLE_X = 90.0                        # distancia entre ejes = 180 mm

# Motor JGB37-520 (medidas del fabricante)
MOT_GB_D = 37.0                      # caja reductora Ø37
MOT_BODY_D = 32.0
MOT_PCD = 31.0                       # 6 agujeros M3 en círculo Ø31
MOT_OFFSET = 7.0                     # el eje está 7 mm descentrado
MOT_LEN = 62.0                       # caja (26.5) + motor (22) + encoder (11.75) + holgura
MOT_SHAFT_L = 15.5

# Taladro
DRILL_Z = CH_TOP                     # eje del taladro coincide con el eje del casco
BULK_X0, BULK_X1 = 116.0, 120.0      # mamparo donde se atornilla el motor del taladro
COLLAR_X = 150.0                     # cara trasera del collar = cara frontal del chasis
GEAR_M = 2.5                         # módulo de los engranajes cónicos
GEAR_Z_SIDE = 20                     # dientes engranajes laterales (A y B)
GEAR_Z_SPIDER = 12                   # dientes satélites
GEAR_FACE = 10.0
XC = 199.0                           # centro del diferencial (punto donde se cruzan los ejes)

# Tornillería
M3_CLR, M3_TAP = 3.4, 2.8
M25_TAP = 2.7
M2_TAP = 1.8
M4_CLR, M4_TAP = 4.3, 3.4
B608_OD, B608_W = 22.15, 7.0         # rodamiento 608 (patineta) con ajuste
ROD_D = 8.0

# Engranajes (se calculan una vez)
_PAIR = cq_gears.BevelGearPair(module=GEAR_M, gear_teeth=GEAR_Z_SIDE, pinion_teeth=GEAR_Z_SPIDER,
                               face_width=GEAR_FACE, backlash=0.15, clearance=0.25)
CONE_H_SIDE = _PAIR.gear.cone_h      # distancia de la cara trasera del engranaje lateral al ápice
CONE_H_SPIDER = _PAIR.pinion.cone_h
X_A_BACK = XC - CONE_H_SIDE          # cara trasera del engranaje A
X_B_BACK = XC + CONE_H_SIDE          # cara "trasera" (mira hacia adelante) del engranaje B
PLATE_T = 4.0
X_PLATE_FRONT = X_B_BACK + PLATE_T   # cara frontal de la tapa del cortador trasero
X_RIM = 168.0                        # borde del cortador trasero (detrás del collar)
X_CONE = X_PLATE_FRONT + 1.6         # base del cono delantero (arandela M8 en medio)
CONE_LEN = 108.0
R_REAR_FRONT, R_REAR_RIM = 50.0, 66.0
R_CONE_BASE = 47.0


# ----------------------------------------------------------------------------------------
# AYUDANTES GEOMÉTRICOS
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
    """Extruye un polígono 2D (lista de puntos) sobre un cq.Plane una distancia h."""
    return cq.Workplane(plane).polyline(pts).close().extrude(h).val()


def hex_prism(af, h, p=(0, 0, 0), d=(0, 0, 1)):
    """Prisma hexagonal (af = entre caras planas)."""
    plane = cq.Plane(origin=V(*p), normal=V(*d))
    return cq.Workplane(plane).polygon(6, af / math.cos(math.radians(30))).extrude(h).val()


def d_bore(depth, p=(0, 0, 0), d=(0, 0, 1)):
    """Agujero en D para varilla de 8 mm con un plano limado de 0.5 mm."""
    plane = cq.Plane(origin=V(*p), normal=V(*d))
    c = cq.Workplane(plane).circle(ROD_D / 2 + 0.1).extrude(depth).val()
    flat = 3.7  # la varilla se lima 0.5 mm (plano a 3.5 mm del centro)
    keep = cq.Workplane(plane).center((flat - 5) / 2, 0).rect(flat + 5, ROD_D + 2).extrude(depth).val()
    return c.intersect(keep)


def ring_points(n, r, a0=0.0):
    return [(r * math.cos(math.radians(a0 + i * 360 / n)), r * math.sin(math.radians(a0 + i * 360 / n)))
            for i in range(n)]


def loc(t=(0, 0, 0), axis=(0, 0, 1), ang=0.0):
    return cq.Location(V(*t), V(*axis), ang)


# ----------------------------------------------------------------------------------------
# CHASIS (2 mitades unidas con 7 tornillos M3)
# ----------------------------------------------------------------------------------------
def _motor_holes_side(ax, side):
    """Agujeros del motor de tracción en la pared lateral (side=+1 izquierda, -1 derecha)."""
    y0 = side * (CH_YW - WALL) - (0 if side > 0 else 0)
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
        box(xa, xb, -W, W, FLOOR_Z0, FLOOR_Z1),                    # piso
        box(xa, xb, W - T, W, FLOOR_Z0, CH_TOP),                    # pared izquierda
        box(xa, xb, -W, -W + T, FLOOR_Z0, CH_TOP),                  # pared derecha
    ]
    if front:
        solids.append(box(CH_X1 - T, CH_X1, -W, W, FLOOR_Z0, CH_TOP))   # frente
    else:
        solids.append(box(CH_X0, CH_X0 + T, -W, W, FLOOR_Z0, CH_TOP))   # parte trasera
    # brida de unión en X = 0
    jx = (0.0, 4.0) if front else (-4.0, 0.0)
    solids += [box(jx[0], jx[1], -W + T, W - T, FLOOR_Z1, 35),
               box(jx[0], jx[1], W - 15, W - T, FLOOR_Z1, 62),
               box(jx[0], jx[1], -W + T, -W + 15, FLOOR_Z1, 62)]
    # cunas de apoyo bajo el cuerpo de cada motor de tracción
    ax = AXLE_X * sg
    cz = AXLE_Z + MOT_OFFSET
    for side in (1, -1):
        y0, y1 = (30, 40) if side > 0 else (-40, -30)
        solids.append(box(ax - 12, ax + 12, y0, y1, FLOOR_Z1, cz - 6))
    # rieles para la bandeja de electrónica
    lx = (6.0, 64.0) if front else (-64.0, -6.0)
    for sy in (1, -1):
        yw = sy * (W - T)
        prof = [(yw, 60), (yw - sy * 4, 64), (yw - sy * 4, 67), (yw, 67)]
        solids.append(prism(prof, cq.Plane(origin=V(lx[0], 0, 0), xDir=V(0, 1, 0), normal=V(1, 0, 0)), lx[1] - lx[0]))
    # postes para la Raspberry Pi 5 (solo mitad delantera, la Pi va en el piso)
    if front:
        for (px, py) in PI_HOLES:
            solids.append(cyl(3.2, (px, py, FLOOR_Z1 - 0.5), (0, 0, 1), 6.5))
    # mamparo del motor del taladro (solo mitad delantera)
    if front:
        bk = box(BULK_X0, BULK_X1, -W + T, W - T, FLOOR_Z1, CH_TOP)
        bk = bk.fuse(box(BULK_X0, BULK_X1, -22, 22, CH_TOP - 1, DRILL_Z + MOT_OFFSET),
                     cyl(22, (BULK_X0, 0, DRILL_Z + MOT_OFFSET), (1, 0, 0), BULK_X1 - BULK_X0))
        # nervios de refuerzo detrás del mamparo
        for yy in (-28, 28):
            bk = bk.fuse(box(BULK_X1, CH_X1 - T, yy - 1.5, yy + 1.5, FLOOR_Z1, 60))
        # guías internas para los 2 sensores ToF VL53L0X (miran por ventanas en el frente)
        for s_ in (1, -1):
            yc = s_ * TOF_Y
            for e in (-1, 1):
                ya, yb = sorted((yc + e * 12.0, yc + e * 15.0))
                bk = bk.fuse(box(CH_X1 - T - 6, CH_X1 - T, ya, yb, FLOOR_Z1, 40))
        solids.append(bk)
    body = fuse(*solids)

    tools = []
    # cuna: corte cilíndrico para el motor + ranura para amarra plástica
    for side in (1, -1):
        y0 = 28 if side > 0 else -42
        tools.append(cyl(MOT_BODY_D / 2 + 0.4, (ax, y0, cz), (0, 1, 0), 14))
        yc = 35 * side
        tools.append(box(ax - 14, ax + 14, yc - 2.6, yc + 2.6, cz - 21, cz - 18.5))
        tools += _motor_holes_side(ax, side)
    # tornillos de unión de la brida
    for (yy, zz) in [(-50, 29), (0, 29), (50, 29), (66, 45), (-66, 45), (66, 56), (-66, 56)]:
        tools.append(cyl(M3_CLR / 2, (-6, yy, zz), (1, 0, 0), 12))
    # tornillos del casco (horizontales en las paredes)
    for hx in ((25, 128) if front else (-25, -125)):
        tools.append(cyl(M3_CLR / 2, (hx, -W - 1, 84), (0, 1, 0), 2 * W + 2))
    # agujeros de los rieles de la bandeja
    for hx in ((12, 58) if front else (-12, -58)):
        for yy in (W - T - 2, -(W - T - 2)):
            tools.append(cyl(M3_TAP / 2, (hx, yy, 60), (0, 0, 1), 8))
    # rejilla universal M3 cada 10 mm en el piso (para atornillar módulos con separadores)
    gx = range(10, 61, 10) if front else range(-60, -9, 10)
    for x in gx:
        for y in range(-60, 61, 10):
            if front and 6 <= x <= 66 and -44 <= y <= 44:
                continue  # zona de la Raspberry Pi (tiene sus propios postes)
            tools.append(cyl(1.6, (x, y, FLOOR_Z0 - 1), (0, 0, 1), 5))
    # ventilación bajo la Raspberry Pi
    if front:
        for y in range(-30, 31, 10):
            tools.append(box(20, 52, y - 2, y + 2, FLOOR_Z0 - 1, FLOOR_Z1 + 1))
        for (px, py) in PI_HOLES:
            tools.append(cyl(M25_TAP / 2, (px, py, FLOOR_Z0 - 1), (0, 0, 1), 12))
    if front:
        # mamparo: eje del motor del taladro + 12 agujeros + ventanas para cables
        tools.append(cyl(7.5, (BULK_X0 - 1, 0, DRILL_Z), (1, 0, 0), 8))
        for (dy, dz) in ring_points(12, MOT_PCD / 2, 0):
            tools.append(cyl(M3_CLR / 2, (BULK_X0 - 1, dy, DRILL_Z + MOT_OFFSET + dz), (1, 0, 0), 8))
        for yy in (-1, 1):
            tools.append(box(BULK_X0 - 1, BULK_X1 + 1, 30 if yy > 0 else -64, 64 if yy > 0 else -30, 30, 80))
        tools.append(box(BULK_X0 - 1, BULK_X1 + 1, -16, 16, 28, 62))
        # frente: paso del acople/eje, tornillos del collar y de los sensores ToF
        tools.append(cyl(18, (CH_X1 - T - 1, 0, DRILL_Z), (1, 0, 0), T + 2))
        for yy in (-1, 1):
            a = math.radians(-50 if yy > 0 else -130)
            tools.append(cyl(M3_CLR / 2, (CH_X1 - T - 1, 58 * math.cos(a), DRILL_Z + 58 * math.sin(a)),
                             (1, 0, 0), T + 2))
            yc = yy * TOF_Y
            tools.append(box(CH_X1 - T - 3.7, CH_X1 - T - 1.5, yc - 13, yc + 13, FLOOR_Z1, 41))  # ranura PCB
            tools.append(box(CH_X1 - T - 1, CH_X1 + 1, yc - 7, yc + 7, 25.5, 34.5))           # ventana
    else:
        # trasera: interruptor principal Ø12.5, salida del cable de la batería, rejilla
        tools.append(cyl(6.25, (CH_X0 - 1, -35, 55), (1, 0, 0), T + 2))
        tools.append(cyl(6.25, (CH_X0 - 1, 35, 55), (1, 0, 0), T + 2))
        for y in range(-12, 13, 6):
            tools.append(box(CH_X0 - 1, CH_X0 + T + 1, y - 1.75, y + 1.75, 35, 75))
        # ranuras para la correa de velcro de la batería
        for x in (-50, -22):
            for yy in (40, -40):
                tools.append(box(x - 11, x + 11, yy - 1.6, yy + 1.6, FLOOR_Z0 - 1, FLOOR_Z1 + 1))
    return cut(body, *tools)


TOF_Y = 57.0   # posición lateral de los sensores ToF (dentro del chasis)

# Raspberry Pi 5 acostada en el piso delantero (lado largo a lo largo de Y, USB hacia la izquierda)
PI_X0, PI_Y0 = 8.0, -42.5
PI_HOLES = [(PI_X0 + 3.5, PI_Y0 + 3.5), (PI_X0 + 52.5, PI_Y0 + 3.5),
            (PI_X0 + 3.5, PI_Y0 + 61.5), (PI_X0 + 52.5, PI_Y0 + 61.5)]


def deck():
    """Bandeja superior: 3 drivers BTS7960 + ESP32 se atornillan en la rejilla."""
    W = CH_YW - WALL - 0.5
    z0, z1 = 67.0, 70.0
    body = box(-64, 64, -W, W, z0, z1)
    tools = []
    for x in range(-55, 56, 10):
        for y in range(-65, 66, 10):
            tools.append(cyl(1.6, (x, y, z0 - 1), (0, 0, 1), 5))
    # pasacables
    for (x, y) in [(0, 0), (-30, 0), (30, 0)]:
        tools.append(cq.Workplane("XY").workplane(offset=z0 - 1).center(x, y)
                     .slot2D(34, 10, 90).extrude(5).val())
    # tornillos a los rieles
    for hx in (-58, -12, 12, 58):
        for yy in (W - 1.5, -(W - 1.5)):
            tools.append(cyl(M3_CLR / 2, (hx, yy, z0 - 1), (0, 0, 1), 5))
    # muescas para que pase la brida de unión del chasis
    return cut(body, *tools)


# ----------------------------------------------------------------------------------------
# COLLAR + PORTA-SATÉLITES (pieza fija que sostiene el diferencial)
# Coordenadas locales: eje del taladro = +Z, z = 0 en la cara trasera (pegada al chasis)
# ----------------------------------------------------------------------------------------
def collar_carrier():
    R = 70.0
    b = [cyl(R, (0, 0, 0), (0, 0, 1), 6),                    # brida
         tube(R, 64, (0, 0, 0), (0, 0, 1), 16),             # banda con canal para LEDs
         cyl(20, (0, 0, 0), (0, 0, 1), 22),                 # buje de rodamientos
         tube(40, 33, (0, 0, 0), (0, 0, 1), X_PLATE_FRONT - COLLAR_X - 12.6)]  # jaula
    zc = XC - COLLAR_X
    # almohadillas de apoyo de los satélites
    for k in range(4):
        a = math.radians(k * 90)
        d = (math.cos(a), math.sin(a), 0)
        r0 = CONE_H_SPIDER + 1.0
        b.append(cyl(5, (d[0] * r0, d[1] * r0, zc), d, 34 - r0))
        t = (-math.sin(a), math.cos(a), 0)              # dirección tangencial
        gus = prism([(-5, 0), (5, 0), (0, -6.5)],
                    cq.Plane(origin=V(d[0] * r0, d[1] * r0, zc), xDir=V(*t), normal=V(*d)), 34 - r0)
        b.append(gus)
    body = fuse(*b)
    tools = []
    # canal para tira LED WS2812 (10 mm) alrededor
    tools.append(tube(R + 1, R - 2.5, (0, 0, 2.5), (0, 0, 1), 11))
    # asientos de rodamientos 608 (uno por detrás, otro por delante) y paso del eje
    tools.append(cyl(B608_OD / 2, (0, 0, -1), (0, 0, 1), B608_W + 1))
    tools.append(cyl(6.5, (0, 0, 0), (0, 0, 1), 30))
    tools.append(cyl(B608_OD / 2, (0, 0, 22 - B608_W), (0, 0, 1), B608_W + 1))
    # pasadores M4 de los satélites
    for k in range(4):
        a = math.radians(k * 90)
        d = (-math.cos(a), -math.sin(a), 0)
        tools.append(cyl(M4_TAP / 2, (math.cos(a) * 45, math.sin(a) * 45, zc), d, 30))
    # ventanas en la jaula (para meter los satélites y ver el mecanismo)
    for k in range(4):
        win = prism([(-17, 10), (17, 10), (17, 36), (0, 53), (-17, 36)],      # techo a 45°
                    cq.Plane(origin=V(28, 0, 0), xDir=V(0, 1, 0), normal=V(1, 0, 0)), 16)
        tools.append(win.rotate(V(0, 0, 0), V(0, 0, 1), 45 + 90 * k))
    # tornillos de la brida (ángulos medidos en el plano Y-Z del carro)
    for ang in (-50, -130, 30, 150, 90):
        a = math.radians(ang)
        yy, zz = 58 * math.cos(a), 58 * math.sin(a)
        tools.append(cyl(M3_TAP / 2, (-zz, yy, -1), (0, 0, 1), 8))
    # aligeramiento decorativo en la brida
    for k in range(8):
        a = math.radians(22.5 + 45 * k)
        tools.append(cyl(5.5, (47 * math.cos(a), 47 * math.sin(a), -1), (0, 0, 1), 8))
    return cut(body, *tools)


COLLAR_LOC = loc((COLLAR_X, 0, DRILL_Z), (0, 1, 0), 90)          # local +Z -> +X


# ----------------------------------------------------------------------------------------
# DIFERENCIAL CONTRARROTANTE
# Marco "dif": eje = +Z, origen en el ápice común. A en z<0, B en z>0 (B = espejo de A).
# ----------------------------------------------------------------------------------------
def _gear_side_dif():
    g = _PAIR.gear.build(bore_d=None)
    return g.moved(loc((0, 0, -CONE_H_SIDE)))


def _spider_loc_dif(k):
    """Ubicación del satélite k (0..3) en el marco dif."""
    l = loc((0, 0, CONE_H_SIDE), (0, 1, 0), 90) * loc((0, 0, -CONE_H_SPIDER))
    if GEAR_Z_SPIDER % 2 == 0:
        l = l * loc((0, 0, 0), (0, 0, 1), 180.0 / GEAR_Z_SPIDER)
    return loc((0, 0, 0), (0, 0, 1), 90 * k) * loc((0, 0, -CONE_H_SIDE)) * l


def gear_A():
    """Engranaje lateral A (gira con el eje y el cono delantero). Local: cara trasera en z=0."""
    g = _PAIR.gear.build(bore_d=None)
    return cut(g, d_bore(40, (0, 0, -5)))


def spider():
    p = _PAIR.pinion.build(bore_d=None)
    return cut(p, cyl(M4_CLR / 2, (0, 0, -1), (0, 0, 1), 40))


DIF_LOC = loc((XC, 0, DRILL_Z), (0, 1, 0), 90)                   # marco dif -> ensamblaje
GEAR_A_LOC = DIF_LOC * loc((0, 0, -CONE_H_SIDE))


def spider_locs():
    return [DIF_LOC * _spider_loc_dif(k) for k in range(4)]


# ----------------------------------------------------------------------------------------
# CORTADOR TRASERO (gira al revés) con el engranaje B integrado
# Local: z = 0 en la cara frontal de la tapa (cama de impresión), z crece hacia atrás.
# ----------------------------------------------------------------------------------------
REAR_LOC = loc((X_PLATE_FRONT, 0, DRILL_Z), (0, 1, 0), -90)      # local +Z -> -X


def rear_cutter():
    L = X_PLATE_FRONT - X_RIM
    t = 3.0
    shell = cone(R_REAR_FRONT, R_REAR_RIM, (0, 0, 0), (0, 0, 1), L).cut(
        cone(R_REAR_FRONT - t, R_REAR_RIM - t, (0, 0, PLATE_T), (0, 0, 1), L - PLATE_T + 0.01))
    parts = [shell]
    # engranaje B = espejo de A, llevado al marco local de este cortador
    gB = _gear_side_dif().mirror("XY")
    gB = gB.moved(REAR_LOC.inverse * DIF_LOC)
    parts.append(gB)
    # aspas helicoidales (hélice izquierda: empujan la tierra hacia atrás girando al revés)
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
    # dientes de corte en el borde (tipo tuneladora)
    for k in range(12):
        a = 15 + 30 * k
        zt = L - 7
        rt = R_REAR_FRONT + (R_REAR_RIM - R_REAR_FRONT) * zt / L
        tooth = prism([(rt - 2, zt - 4), (rt + 3.5, zt + 1.5), (rt + 3.5, zt + 4), (rt - 2, zt + 4)],
                      cq.Plane(origin=V(0, -3, 0), xDir=V(1, 0, 0), normal=V(0, -1, 0)), -6)
        tooth = tooth.rotate(V(0, 0, 0), V(0, 0, 1), a)
        parts.append(tooth)
    body = fuse(*parts)
    tools = [cyl(B608_OD / 2, (0, 0, -1), (0, 0, 1), B608_W + 1),        # rodamiento 608
             cyl(7.0, (0, 0, 0), (0, 0, 1), 30),                         # paso libre
             box(-100, 100, -100, 100, -20, 0),                          # base plana para la cama
             box(-100, 100, -100, 100, L, L + 20)]
    return cut(body, *tools)


# ----------------------------------------------------------------------------------------
# CONO DELANTERO (gira con el eje)  — local: base en z=0 (cama), punta hacia +Z
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
    # anillo de dientes en la base
    for k in range(10):
        tooth = box(R0 - 3, R0 + 2.5, -3, 3, 0, 8).rotate(V(0, 0, 0), V(0, 0, 1), 36 * k + 18)
        parts.append(tooth)
    # punta de metal duro (decorativa)
    parts.append(cone(Rt + 1, 0.8, (0, 0, L - 1), (0, 0, 1), 9))
    body = fuse(*parts)
    tools = [d_bore(42, (0, 0, -1)),
             box(-100, 100, -100, 100, -20, 0),                          # base plana para la cama
             # tuerca M3 cautiva (se mete por debajo) + tornillo M3x16 radial con caja
             box(5.0, 7.9, -3.0, 3.0, -1, 11),
             cyl(1.7, (0, 0, 7), (1, 0, 0), 30),
             cyl(3.3, (16, 0, 7), (1, 0, 0), 40)]
    return cut(body, *tools)


def spacer():
    """Separador entre el rodamiento delantero del collar y el engranaje A."""
    L = X_A_BACK - (COLLAR_X + 22)
    return tube(6.5, 4.2, (0, 0, 0), (0, 0, 1), L)


SPACER_LOC = loc((COLLAR_X + 22, 0, DRILL_Z), (0, 1, 0), 90)


def led_diffuser():
    return tube(71.6, 70.15, (0, 0, 0), (0, 0, 1), 12.5)


DIFFUSER_LOC = loc((COLLAR_X + 2, 0, DRILL_Z), (0, 1, 0), 90)


# ----------------------------------------------------------------------------------------
# CASCO (2 mitades) Y CABINA
# ----------------------------------------------------------------------------------------
HULL_R = CH_YW


def _half_cyl(r, x0, x1):
    c = cyl(r, (x0, 0, DRILL_Z), (1, 0, 0), x1 - x0)
    return c.intersect(box(x0 - 1, x1 + 1, -r - 1, r + 1, DRILL_Z, DRILL_Z + r + 1))


def _hull_tabs(xs, hacia_cama):
    """Pestañas de tornillo. hacia_cama = +1 si la cama de impresión está hacia +X (casco delantero).
    Llevan un refuerzo a 45° hacia la cama para imprimirse sin soportes."""
    out, tools = [], []
    for x in xs:
        for s in (1, -1):
            y0 = HULL_R - WALL - 3.5 if s > 0 else -HULL_R + WALL
            g = hacia_cama
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
    # escotillas decorativas en relieve
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
    # anillos (líneas de panel)
    for xr in (12, 120):
        tools.append(tube(R + 2, R - 0.8, (xr, 0, DRILL_Z), (1, 0, 0), 1.2))
    # tornillos del collar en la placa frontal
    for ang in (30, 150, 90):
        a = math.radians(ang)
        tools.append(cyl(M3_CLR / 2, (x1 - T - 1, 58 * math.cos(a), DRILL_Z + 58 * math.sin(a)), (1, 0, 0), T + 2))
    # tornillos de la cabina y ranura del cable de la cámara
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
    # rejillas laterales (louvers) en relieve con ranuras a lo largo de X
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
    # rejilla trasera
    for y in range(-40, 41, 8):
        tools.append(box(x0 - 1, x0 + T + 1, y - 2, y + 2, DRILL_Z + 10, DRILL_Z + 45))
    for xr in (-20, -138):
        tools.append(tube(R + 2, R - 0.8, (xr, 0, DRILL_Z), (1, 0, 0), 1.2))
    return cut(fuse(*parts), *tools)


CAB_X0, CAB_X1, CAB_YW = 62.0, 132.0, 32.0
CAB_SCREWS = [(70, 22), (70, -22), (112, 22), (112, -22)]
CAM_TILT = 15.0          # la cámara mira 15° hacia abajo


def _cab_profile(off=0.0):
    """Perfil lateral de la cabina. Techo plano (se imprime boca abajo) y visor inclinado 15°."""
    tilt = math.tan(math.radians(CAM_TILT))
    xb = CAB_X1 - 34 * tilt
    top = 196 - off
    pts = [(CAB_X0 + off, 140), (CAB_X0 + off, top - 6 + off * 0.4), (CAB_X0 + 6 + off * 0.6, top),
           (CAB_X1 - off, top), (xb - off, 162), (xb - off, 140)]
    return pts


def cab():
    prof = _cab_profile(0)
    # plano XZ con normal -Y: el eje "v" del plano queda en +Z; se extruye hacia -Y
    outer = prism(prof, cq.Plane(origin=V(0, CAB_YW, 0), xDir=V(1, 0, 0), normal=V(0, -1, 0)), 2 * CAB_YW)
    inner = prism(_cab_profile(2.5),
                  cq.Plane(origin=V(0, CAB_YW - 2.5, 0), xDir=V(1, 0, 0), normal=V(0, -1, 0)), 2 * CAB_YW - 5)
    hullcyl = cyl(HULL_R + 0.2, (CAB_X0 - 5, 0, DRILL_Z), (1, 0, 0), CAB_X1 - CAB_X0 + 10)
    body = outer.cut(inner).cut(hullcyl)
    # postes para los tornillos (desde dentro del casco)
    posts = []
    for (cx, cy) in CAB_SCREWS:
        p = cyl(4, (cx, cy, 150), (0, 0, 1), 40).cut(hullcyl)
        posts.append(p.intersect(outer))
    body = fuse(body, *posts)
    tools = [cyl(M3_TAP / 2, (cx, cy, 150), (0, 0, 1), 30) for (cx, cy) in CAB_SCREWS]
    # cámara Raspberry Pi Camera Module 3 en el visor
    tilt = math.radians(CAM_TILT)
    xb = CAB_X1 - 34 * math.tan(tilt)
    fc = V(xb + (183 - 162) * math.tan(tilt), 0, 183)        # centro del lente sobre el visor
    n = V(math.cos(tilt), 0, -math.sin(tilt))                 # normal hacia afuera (abajo 15°)
    up = V(math.sin(tilt), 0, math.cos(tilt))
    lens = fc
    pl_out = cq.Plane(origin=lens + up * (-4.0) + n * 2, xDir=V(0, 1, 0), normal=-n)
    tools.append(cq.Workplane(pl_out).rect(14, 19).extrude(8).val())          # ventana del lente
    standoffs = []
    for (dx, dz) in [(-10.5, 0), (10.5, 0), (-10.5, -12.5), (10.5, -12.5)]:
        c = lens + V(0, dx, 0) + up * dz
        pl = cq.Plane(origin=c - n * 2.0, xDir=V(0, 1, 0), normal=-n)
        standoffs.append(cq.Workplane(pl).circle(2.6).extrude(3.0).val())
        tools.append(cq.Workplane(cq.Plane(origin=c - n * 1.0, xDir=V(0, 1, 0), normal=-n))
                     .circle(M2_TAP / 2).extrude(5).val())
    body = fuse(body, *standoffs)
    # rendijas laterales de visión y luces de peligro (decorativas)
    for s in (1, -1):
        tools.append(box(80, 112, s * CAB_YW - 3, s * CAB_YW + 3, 182, 186))
        tools.append(box(80, 112, s * CAB_YW - 3, s * CAB_YW + 3, 174, 177))
    return cut(body, *tools)


# ----------------------------------------------------------------------------------------
# RUEDAS: rin (PLA/PETG) + llanta (TPU 95A)
# Local: eje = +Z, z = 0 en la cara interior (la que mira al chasis)
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
    tools = [hex_prism(12.25, 11, (0, 0, -1)),                 # acople hexagonal 12 mm
             cyl(8.4, (0, 0, -1), (0, 0, 1), 2.5),             # collar del acople
             cyl(M4_CLR / 2, (0, 0, 0), (0, 0, 1), W),
             cyl(4.2, (0, 0, W - 9), (0, 0, 1), 10)]           # cabeza del tornillo M4
    for k in range(6):
        a = math.radians(60 * k)
        tools.append(cyl(5.5, (23 * math.cos(a), 23 * math.sin(a), -1), (0, 0, 1), 8))
    return cut(body, *tools)


def tire():
    W = WHEEL_W - 5.5          # de z=1.5 a z=36 (antes del chaflán de la pestaña exterior)
    base = tube(44.0, RIM_R - 0.25, (0, 0, 1.5), (0, 0, 1), W)
    lugs = []
    n = 16
    for row, z0 in ((0, 1.5), (1, 1.5 + W / 2 + 0.5)):
        z1 = z0 + W / 2 - 0.5
        for k in range(n):
            a = 360 / n * k + (360 / n / 2 if row else 0)
            if row == 0:
                lug = box(43, WHEEL_R, -4.2, 4.2, z0, z1)
            else:   # la segunda fila arranca con un chaflán de 45° para imprimir sin soportes
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
        out.append(loc((xs, y_in, AXLE_Z), (1, 0, 0), -90))     # izquierda: +Z local -> +Y
        out.append(loc((xs, -y_in, AXLE_Z), (1, 0, 0), 90))     # derecha:   +Z local -> -Y
    return out


# ----------------------------------------------------------------------------------------
# PIEZAS COMPRADAS (solo para el ensamblaje / visor 3D, no se imprimen)
# ----------------------------------------------------------------------------------------
def motor_model(shaft_len=MOT_SHAFT_L):
    """JGB37-520 con encoder. Local: cara frontal en z=0, eje en el origen apuntando a +Z,
    el cuerpo está 7 mm descentrado en +Y local y se extiende hacia -Z."""
    c = (0, MOT_OFFSET)
    gb = cyl(MOT_GB_D / 2, (c[0], c[1], -26.5), (0, 0, 1), 26.5)
    body = cyl(MOT_BODY_D / 2, (c[0], c[1], -48.5), (0, 0, 1), 22)
    enc = cyl(MOT_BODY_D / 2, (c[0], c[1], -60.5), (0, 0, 1), 12)
    boss = cyl(6, (0, 0, 0), (0, 0, 1), 2)
    shaft = cyl(3, (0, 0, 0), (0, 0, 1), shaft_len)
    return fuse(gb, body, enc, boss, shaft)


def motor_locs():
    """Motores de tracción (4) + motor del taladro."""
    out = []
    for xs in (AXLE_X, -AXLE_X):
        # izquierda: cara en la pared interior (y = 72), eje hacia +Y, cuerpo hacia arriba
        out.append(("motor_traccion", loc((xs, CH_YW - WALL, AXLE_Z), (1, 0, 0), -90)
                    * loc((0, 0, 0), (0, 0, 1), 180)))
        out.append(("motor_traccion", loc((xs, -CH_YW + WALL, AXLE_Z), (1, 0, 0), 90)))
    out.append(("motor_taladro", loc((BULK_X0, 0, DRILL_Z), (0, 1, 0), 90) * loc((0, 0, 0), (0, 0, 1), 90)))
    return out


def bought_models():
    """Devuelve lista de (nombre, shape en coordenadas del ensamblaje, color)."""
    out = []
    mm = motor_model()
    for name, l in motor_locs():
        out.append((name, mm.moved(l), (70, 74, 82)))
    # varilla de 8 mm, acople flexible, rodamientos
    x_rod0 = BULK_X1 + 13.5
    out.append(("eje_8mm", cyl(4, (x_rod0, 0, DRILL_Z), (1, 0, 0), 130), (200, 200, 205)))
    out.append(("acople_6a8", cyl(9.5, (BULK_X1 + 0.5, 0, DRILL_Z), (1, 0, 0), 25), (190, 190, 200)))
    for xb in (COLLAR_X, COLLAR_X + 22 - B608_W, X_PLATE_FRONT - B608_W):
        out.append(("rodamiento_608", tube(11, 4, (xb, 0, DRILL_Z), (1, 0, 0), 7), (160, 160, 170)))
    # tornillos M4 de los satélites
    for k in range(4):
        a = math.radians(k * 90)
        # en el collar local: dirección (cos,sin,0) -> ensamblaje (Z = -x_local, Y = y_local)
        dy, dz = math.sin(a), -math.cos(a)
        out.append(("pasador_M4", cyl(2, (XC, dy * 16, DRILL_Z + dz * 16), (0, dy, dz), 26), (120, 120, 130)))
    # Raspberry Pi 5 (en el piso delantero)
    z = FLOOR_Z1 + 6
    pi = box(PI_X0, PI_X0 + 56, PI_Y0, PI_Y0 + 85, z, z + 1.6)
    pi = fuse(pi, box(PI_X0 + 13, PI_X0 + 43, PI_Y0 + 15, PI_Y0 + 45, z + 1.6, z + 14),  # ventilador activo
              box(PI_X0 + 2, PI_X0 + 54, PI_Y0 + 66, PI_Y0 + 86, z + 1.6, z + 17))      # USB/Ethernet
    out.append(("raspberry_pi5", pi, (30, 120, 70)))
    # batería 3S2P 18650 (trasera, bajo la bandeja)
    out.append(("bateria_3S2P", box(-64, -8, -35, 35, FLOOR_Z1, FLOOR_Z1 + 38), (40, 90, 170)))
    # convertidores DC-DC
    out.append(("buck_5V5A_pi", box(-62, -8, 42, 66, FLOOR_Z1 + 5, FLOOR_Z1 + 25), (40, 60, 140)))
    out.append(("buck_5V3A", box(-60, -18, -64, -42, FLOOR_Z1 + 5, FLOOR_Z1 + 20), (40, 60, 140)))
    # drivers BTS7960 y ESP32 sobre la bandeja
    dz = 70 + 5
    for (x0, y0, name) in [(-60, 10, "bts7960_izq"), (-60, -62, "bts7960_der"), (0, 10, "bts7960_taladro")]:
        b = fuse(box(x0, x0 + 50, y0, y0 + 50, dz, dz + 1.6),
                 box(x0 + 4, x0 + 46, y0 + 5, y0 + 45, dz + 1.6, dz + 38))
        out.append((name, b, (180, 40, 40)))
    esp = fuse(box(0, 52, -62, -10, dz, dz + 1.6), box(12, 40, -54, -18, dz + 1.6, dz + 14))
    out.append(("esp32_con_bornera", esp, (25, 25, 30)))
    # cámara
    tilt = math.radians(CAM_TILT)
    xb = CAB_X1 - 34 * math.tan(tilt)
    n = V(math.cos(tilt), 0, -math.sin(tilt))
    cam = box(-1.0, 0, -12.5, 12.5, -16.5, 7.5).fuse(cyl(4.3, (0, 0, 0), (1, 0, 0), 4.5))
    cam = cam.rotate(V(0, 0, 0), V(0, 1, 0), CAM_TILT).translate(
        V(xb + (183 - 162) * math.tan(tilt), 0, 183) - n * 5.0)
    out.append(("camara_pi3", cam, (40, 140, 60)))
    # sensores ToF e IMU
    for yy in (TOF_Y, -TOF_Y):
        out.append(("vl53l0x", box(CH_X1 - WALL - 3.6, CH_X1 - WALL - 2.0, yy - 12.5, yy + 12.5, 23.5, 36.5),
                    (110, 40, 160)))
    out.append(("mpu6050", box(-58, -37, -8, 8, 70 + 5, 70 + 6.6), (40, 40, 160)))
    # tira LED (anillo)
    out.append(("tira_ws2812", tube(69.6, 67.8, (COLLAR_X + 3, 0, DRILL_Z), (1, 0, 0), 10), (255, 245, 210)))
    return out


# ----------------------------------------------------------------------------------------
# REGISTRO DE PIEZAS IMPRIMIBLES
#   nombre: (función, ubicaciones en el ensamblaje, rotación para imprimir, color, material, notas)
# ----------------------------------------------------------------------------------------
PRINT_ROT = {
    # rotaciones (eje, ángulo) aplicadas a la pieza LOCAL para dejarla en posición de impresión
    "chasis_delantero": [],
    "chasis_trasero": [],
    "bandeja_electronica": [],
    "collar_portasatelites": [],
    "engranaje_A": [],
    "satelite": [],
    "cortador_trasero": [],
    "cono_delantero": [],
    "espaciador_eje": [],
    "difusor_led": [],
    "casco_delantero": [((0, 1, 0), 90)],        # se imprime parado sobre su placa frontal
    "casco_trasero": [((0, 1, 0), -90)],         # parado sobre su tapa trasera
    "cabina": [((1, 0, 0), 180)],                # boca abajo, sobre el techo plano
    "rueda_rin": [],
    "rueda_llanta_TPU": [],
}


def build_parts():
    P = {}
    P["chasis_delantero"] = dict(fn=lambda: chassis_half(True), locs=[loc()], color=(214, 120, 40),
                                 mat="PETG", qty=1, note="Pared 3 mm, 4 perímetros, 25 % relleno")
    P["chasis_trasero"] = dict(fn=lambda: chassis_half(False), locs=[loc()], color=(214, 120, 40),
                               mat="PETG", qty=1, note="Igual que la delantera")
    P["bandeja_electronica"] = dict(fn=deck, locs=[loc()], color=(60, 60, 64), mat="PLA/PETG", qty=1,
                                    note="Rejilla M3 cada 10 mm")
    P["collar_portasatelites"] = dict(fn=collar_carrier, locs=[COLLAR_LOC], color=(225, 170, 40), mat="PETG",
                                      qty=1, note="Pieza fija: sostiene 2 rodamientos y los 4 satélites")
    P["engranaje_A"] = dict(fn=gear_A, locs=[GEAR_A_LOC], color=(240, 200, 60), mat="PETG/Nylon", qty=1,
                            note="100 % relleno o 6 perímetros. Agujero en D")
    P["satelite"] = dict(fn=spider, locs=spider_locs(), color=(170, 185, 205), mat="PETG/Nylon", qty=4,
                         note="100 % relleno")
    P["cortador_trasero"] = dict(fn=rear_cutter, locs=[REAR_LOC], color=(120, 170, 230), mat="PETG", qty=1,
                                 note="Incluye el engranaje B. Gira AL REVÉS del cono")
    P["cono_delantero"] = dict(fn=front_cone, locs=[CONE_LOC], color=(235, 95, 60), mat="PETG", qty=1,
                               note="Gira con el eje. 3 perímetros, 15 % relleno")
    P["espaciador_eje"] = dict(fn=spacer, locs=[SPACER_LOC], color=(240, 200, 60), mat="PLA", qty=1,
                               note="Va entre el rodamiento y el engranaje A")
    P["difusor_led"] = dict(fn=led_diffuser, locs=[DIFFUSER_LOC], color=(240, 240, 250), mat="PETG transparente",
                            qty=1, note="Modo jarrón o 2 perímetros")
    P["casco_delantero"] = dict(fn=hull_front, locs=[loc()], color=(214, 120, 40), mat="PLA/PETG", qty=1,
                                note="Imprimir parado, sin soportes")
    P["casco_trasero"] = dict(fn=hull_rear, locs=[loc()], color=(214, 120, 40), mat="PLA/PETG", qty=1,
                              note="Imprimir parado, sin soportes. Se quita para cambiar batería")
    P["cabina"] = dict(fn=cab, locs=[loc()], color=(60, 60, 64), mat="PLA/PETG", qty=1,
                       note="Imprimir boca abajo sobre el techo. Lleva la cámara")
    P["rueda_rin"] = dict(fn=wheel_rim, locs=wheel_locs(), color=(60, 60, 64), mat="PETG", qty=4,
                          note="Acople hexagonal 12 mm + tornillo M4")
    P["rueda_llanta_TPU"] = dict(fn=tire, locs=wheel_locs(), color=(35, 35, 38), mat="TPU 95A", qty=4,
                                 note="Si no tienes TPU, imprímela en PETG")
    return P


if __name__ == "__main__":
    import export_all
    export_all.main(sys.argv[1:])
