"""Exporta todas las piezas del TOPO S14: STL (orientadas para imprimir), STEP, ensamblaje GLB/STEP,
renders de revisión y un JSON con la lista de piezas."""
import json
import math
import os
import time

import cadquery as cq
import numpy as np
import trimesh

import topo_cad as T

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "salida")
BED = (220, 220, 250)
PETG_DENS = 1.27   # g/cm3
FILL_FACTOR = 0.45  # fracción aproximada de material real con 3-4 perímetros y 20-25 % de relleno


def to_trimesh(shape, tol=0.08, ang=0.25, color=None):
    verts, tris = shape.tessellate(tol, ang)
    v = np.array([[p.x, p.y, p.z] for p in verts], dtype=np.float64)
    f = np.array(tris, dtype=np.int64)
    m = trimesh.Trimesh(v, f, process=True)
    if color is not None:
        m.visual.face_colors = np.tile(np.array([*color, 255], np.uint8), (len(m.faces), 1))
    return m


def print_orient(name, shape):
    s = shape
    for axis, ang in T.PRINT_ROT.get(name, []):
        s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(*axis), ang)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


def main(only=None):
    os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "step"), exist_ok=True)
    P = T.build_parts()
    names = [n for n in P if not only or n in only]
    meta = []
    asm_meshes = []      # (nombre_nodo, trimesh, grupo)
    assy = cq.Assembly(name="TOPO_S14")
    for name in names:
        d = P[name]
        t = time.time()
        local = d["fn"]()
        pr = print_orient(name, local)
        bb = pr.BoundingBox()
        fits = bb.xlen <= BED[0] and bb.ylen <= BED[1] and bb.zlen <= BED[2]
        cq.exporters.export(pr, os.path.join(OUT, "stl", f"{name}.stl"), tolerance=0.03, angularTolerance=0.15)
        cq.exporters.export(pr, os.path.join(OUT, "step", f"{name}.step"))
        vol = local.Volume() / 1000.0
        meta.append(dict(nombre=name, cantidad=d["qty"], material=d["mat"], nota=d["note"],
                         medida_mm=[round(bb.xlen), round(bb.ylen), round(bb.zlen)], cabe_220=fits,
                         volumen_cm3=round(vol, 1),
                         gramos_aprox=round(vol * PETG_DENS * FILL_FACTOR * d["qty"])))
        for i, l in enumerate(d["locs"]):
            placed = local.moved(l)
            node = name if len(d["locs"]) == 1 else f"{name}_{i + 1}"
            tol = 0.05 if name in ("engranaje_A", "satelite", "cortador_trasero") else 0.12
            asm_meshes.append((node, to_trimesh(placed, tol=tol, color=d["color"]), "impreso"))
            assy.add(placed, name=node, color=cq.Color(*[c / 255 for c in d["color"]]))
        print(f"  {name:24s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:6.1f} mm  {'OK' if fits else 'NO CABE'}"
              f"  {time.time() - t:4.1f}s", flush=True)
    if only:
        return
    # piezas compradas
    count = {}
    for name, shape, color in T.bought_models():
        count[name] = count.get(name, 0) + 1
        node = f"{name}_{count[name]}" if name in ("motor_traccion", "rodamiento_608", "pasador_M4",
                                                    "vl53l0x") else name
        asm_meshes.append((node, to_trimesh(shape, tol=0.2, ang=0.4, color=color), "comprado"))
        assy.add(shape, name=node, color=cq.Color(*[c / 255 for c in color]))
    assy.save(os.path.join(OUT, "step", "TOPO_S14_ensamble.step"))

    # GLB para el visor (Y arriba como pide glTF: rotamos -90° en X)
    scene = trimesh.Scene()
    R = trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0])
    for node, m, grupo in asm_meshes:
        mm = m.copy()
        mm.apply_scale(0.001)  # metros
        mm.apply_transform(R)
        scene.add_geometry(mm, node_name=node, geom_name=node)
    scene.export(os.path.join(OUT, "TOPO_S14.glb"))
    json.dump(dict(piezas=meta, nodos=[(n, g) for n, _, g in asm_meshes]),
              open(os.path.join(OUT, "piezas.json"), "w"), ensure_ascii=False, indent=1)
    total = sum(p["gramos_aprox"] for p in meta)
    print(f"Filamento aproximado: {total} g")


if __name__ == "__main__":
    import sys
    main(sys.argv[1:])
