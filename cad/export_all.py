"""Exports every Ktopo part: STL (oriented for printing), STEP, the assembly as GLB/STEP,
and a JSON file with the parts list."""
import json
import math
import os
import time

import cadquery as cq
import numpy as np
import trimesh

import ktopo_cad as K

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
BED = (220, 220, 250)
PETG_DENS = 1.27    # g/cm3
FILL_FACTOR = 0.45  # rough fraction of solid material with 3-4 perimeters and 20-25 % infill


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
    for axis, ang in K.PRINT_ROT.get(name, []):
        s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(*axis), ang)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


def main(only=None):
    os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "step"), exist_ok=True)
    P = K.build_parts()
    names = [n for n in P if not only or n in only]
    meta = []
    asm_meshes = []      # (node_name, trimesh, group)
    assy = cq.Assembly(name="KTOPO")
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
        meta.append(dict(name=name, qty=d["qty"], material=d["mat"], note=d["note"],
                         size_mm=[round(bb.xlen), round(bb.ylen), round(bb.zlen)], fits_220=fits,
                         volume_cm3=round(vol, 1),
                         grams_approx=round(vol * PETG_DENS * FILL_FACTOR * d["qty"])))
        for i, l in enumerate(d["locs"]):
            placed = local.moved(l)
            node = name if len(d["locs"]) == 1 else f"{name}_{i + 1}"
            tol = 0.05 if name in ("gear_A", "spider_gear", "rear_cutter") else 0.12
            asm_meshes.append((node, to_trimesh(placed, tol=tol, color=d["color"]), "printed"))
            assy.add(placed, name=node, color=cq.Color(*[c / 255 for c in d["color"]]))
        print(f"  {name:22s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:6.1f} mm  {'OK' if fits else 'TOO BIG'}"
              f"  {time.time() - t:4.1f}s", flush=True)
    if only:
        return
    # bought parts
    count = {}
    for name, shape, color in K.bought_models():
        count[name] = count.get(name, 0) + 1
        node = f"{name}_{count[name]}" if name in ("drive_motor", "bearing_608", "pin_M4", "vl53l0x") else name
        asm_meshes.append((node, to_trimesh(shape, tol=0.2, ang=0.4, color=color), "bought"))
        assy.add(shape, name=node, color=cq.Color(*[c / 255 for c in color]))
    assy.save(os.path.join(OUT, "step", "Ktopo_assembly.step"))

    # GLB for the viewer (glTF is Y-up: rotate -90° about X)
    scene = trimesh.Scene()
    R = trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0])
    for node, m, group in asm_meshes:
        mm = m.copy()
        mm.apply_scale(0.001)  # meters
        mm.apply_transform(R)
        scene.add_geometry(mm, node_name=node, geom_name=node)
    scene.export(os.path.join(OUT, "Ktopo.glb"))
    json.dump(dict(parts=meta, nodes=[(n, g) for n, _, g in asm_meshes]),
              open(os.path.join(OUT, "parts.json"), "w"), indent=1)
    total = sum(p["grams_approx"] for p in meta)
    print(f"Approximate filament: {total} g")


if __name__ == "__main__":
    import sys
    main(sys.argv[1:])
