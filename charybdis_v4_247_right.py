from build123d import *
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
import math

def make_loft_solid(pts_a, pts_b):
    wa = Wire.make_polygon([Vector(*p) for p in pts_a], close=True)
    for shift in range(len(pts_b)):
        rotated = pts_b[shift:] + pts_b[:shift]
        for rev in [False, True]:
            pts = list(reversed(rotated)) if rev else rotated
            wb = Wire.make_polygon([Vector(*p) for p in pts], close=True)
            for ruled in [False, True]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(wa.wrapped)
                    gen.AddWire(wb.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  Loft OK: shift={shift} rev={rev} ruled={ruled} vol={s.volume:.1f}")
                        return s
                except Exception:
                    continue
    raise ValueError("Could not produce valid loft")

def make_ruled_solid(pts_a, pts_b):
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.BRepLib import BRepLib

    def tri_face(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def attempt(cap_b_reversed):
        sew = BRepBuilderAPI_Sewing(1e-2)  # slightly looser tolerance

        # Cap A
        sew.Add(tri_face(pts_a[0], pts_a[1], pts_a[2]))
        sew.Add(tri_face(pts_a[0], pts_a[2], pts_a[3]))

        # Cap B
        if cap_b_reversed:
            sew.Add(tri_face(pts_b[0], pts_b[3], pts_b[2]))
            sew.Add(tri_face(pts_b[0], pts_b[2], pts_b[1]))
        else:
            sew.Add(tri_face(pts_b[0], pts_b[1], pts_b[2]))
            sew.Add(tri_face(pts_b[0], pts_b[2], pts_b[3]))

        # Side quads
        n = len(pts_a)
        for i in range(n):
            j = (i + 1) % n
            sew.Add(tri_face(pts_a[i], pts_a[j], pts_b[j]))
            sew.Add(tri_face(pts_a[i], pts_b[j], pts_b[i]))

        sew.Perform()
        sewn = sew.SewedShape()
        print(f"    Sewn shape type: {sewn.ShapeType()}")

        try:
            shell = TopoDS.Shell_s(sewn)
        except Exception as e:
            print(f"    Shell downcast failed: {e}")
            return None

        builder = BRepBuilderAPI_MakeSolid()
        builder.Add(shell)
        builder.Build()
        solid_shape = builder.Shape()

        # Fix orientation
        BRepLib.OrientClosedSolid_s(TopoDS.Solid_s(solid_shape))

        s = Solid(solid_shape)
        print(f"    valid={s.is_valid}, volume={s.volume:.1f}")
        return s if s.is_valid and abs(s.volume) > 0 else None

    for rev in [True, False]:
        print(f"  Trying cap_b_reversed={rev}...")
        result = attempt(rev)
        if result is not None:
            # volume may be negative if normals flipped — wrap in abs check
            if result.volume < 0:
                print("  Volume negative, flipping all windings...")
                pts_a_r = list(reversed(pts_a))
                pts_b_r = list(reversed(pts_b))
                pts_a, pts_b = pts_a_r, pts_b_r
                result = attempt(not rev)
            if result and result.volume > 0:
                print(f"  Ruled solid OK: vol={result.volume:.1f}")
                return result

    raise ValueError("Could not build ruled solid — see debug output above")

# ══════════════════════════════════════════════════════════════
# Shared plane helpers
# ══════════════════════════════════════════════════════════════

# ── Frame 1 planes ────────────────────────────────────────────
dy1, dz1 = 18.902, -35.252
len1     = math.sqrt(dy1**2 + dz1**2)          # 40.0
_orig1   = Vector(1789.205, 1461.711, 186.561)
_xd      = Vector(1.0, 0.0, 0.0)
_n1      = Vector(0.0, dy1/len1, dz1/len1)      # (0,  0.4726, -0.8813)
_n1_slot = Vector(0.0, 0.8813,  0.4726)         # perpendicular (slot normal)
pl1      = Plane(origin=_orig1,            x_dir=_xd, z_dir=_n1)
pl1_tgt  = Plane(origin=_orig1 + _n1*40,  x_dir=_xd, z_dir=_n1)

# ── Frame 2 planes ────────────────────────────────────────────
dy2, dz2 = 9.744, -38.815
len2     = math.sqrt(dy2**2 + dz2**2)           # ≈ 40.0
_orig2   = Vector(1789.205, 1253.461, 125.816)
_n2      = Vector(0.0, dy2/len2, dz2/len2)       # (0,  0.2436, -0.9699)
_n2_slot = Vector(0.0, 0.9699,   0.2436)         # perpendicular (slot normal)
pl2      = Plane(origin=_orig2,            x_dir=_xd, z_dir=_n2)
pl2_tgt  = Plane(origin=_orig2 + _n2*40,  x_dir=_xd, z_dir=_n2)

# ── Frame 3 planes ────────────────────────────────────────────
_orig3   = Vector(1789.205, 1036.686, 117.635)
pl3      = Plane(origin=_orig3, x_dir=_xd, z_dir=(0.0, 0.0, 1.0))
pl3_tgt  = Plane(origin=_orig3 + Vector(0, 0, -40.0), x_dir=_xd, z_dir=(0.0, 0.0, 1.0))

# ── Frame 4 planes ────────────────────────────────────────────
dy4, dz4 = -9.745, -38.795
len4     = math.sqrt(dy4**2 + dz4**2)           # ≈ 40.0
_orig4   = Vector(1789.205, 989.327, 121.095)
_n4      = Vector(0.0, dy4/len4, dz4/len4)      # (0, -0.2436, -0.9699)
_n4_slot = Vector(0.0, -0.9699,  0.2436)        # perpendicular (slot normal)
pl4      = Plane(origin=_orig4,            x_dir=_xd, z_dir=_n4)
pl4_tgt  = Plane(origin=_orig4 + _n4*40,   x_dir=_xd, z_dir=_n4)

# ══════════════════════════════════════════════════════════════
# Frame part 1 & 2
# ══════════════════════════════════════════════════════════════
print("Building frame...")
with BuildPart() as frame:

    # ── Frame 1: outer 185×185, inner 139×139 ─────────────────
    with BuildSketch(pl1):
        with Locations((92.5,  -92.5)): Rectangle(185.0, 185.0)
        with Locations((85.0,  -85.0)): Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=40.0)

    with BuildSketch(pl1_tgt):
        with Locations(( 37.75, -85.0)): Rectangle(44.5, 139.0)
        with Locations((132.25, -85.0)): Rectangle(44.5, 139.0)
    extrude(amount=-40.0, mode=Mode.SUBTRACT)

    with BuildSketch(Plane(origin=Vector(1849.205, 1610.185, 251.421),
                           x_dir=_xd, z_dir=_n1_slot)):
        with Locations((25.0, 13.5)): Rectangle(50.0, 27.0)
    extrude(amount=-7.1, mode=Mode.SUBTRACT)

    with BuildSketch(Plane(origin=Vector(1849.205, 1475.346, 179.12),
                           x_dir=_xd, z_dir=_n1_slot)):
        with Locations((25.0, 13.5)): Rectangle(50.0, 27.0)
    extrude(amount=7.1, mode=Mode.SUBTRACT)

    # ── Frame 2: outer 185×170, inner 139×139 ─────────────────
    with BuildSketch(pl2):
        with Locations((92.5, -85.0)): Rectangle(185.0, 170.0)
        with Locations((85.0, -85.0)): Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=40.0)

    with BuildSketch(pl2_tgt):
        with Locations(( 37.75, -85.0)): Rectangle(44.5, 139.0)
        with Locations((132.25, -85.0)): Rectangle(44.5, 139.0)
    extrude(amount=-40.0, mode=Mode.SUBTRACT)

    with BuildSketch(Plane(origin=Vector(1849.205, 1413.262, 152.552),
                           x_dir=_xd, z_dir=_n2_slot)):
        with Locations((25.0, 13.5)): Rectangle(50.0, 27.1)
    extrude(amount=-7.1, mode=Mode.SUBTRACT)

    with BuildSketch(Plane(origin=Vector(1849.205, 1264.871, 115.279),
                           x_dir=_xd, z_dir=_n2_slot)):
        with Locations((25.0, 13.5)): Rectangle(50.0, 27.1)
    extrude(amount=7.1, mode=Mode.SUBTRACT)

print(f"  Frame valid={frame.part.is_valid}, volume={frame.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame part 3
# ══════════════════════════════════════════════════════════════
print("Building frame 3...")
with BuildPart() as frame3:
    # ── Frame 3: outer 185×170, inner 139×139 ─────────────────
    with BuildSketch(pl3):
        with Locations((92.5, 85.0)):
            Rectangle(185.0, 170.0)
        with Locations((85.0, 85.0)):
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=-40.0)

    with BuildSketch(pl3_tgt):
        with Locations((37.75, 85.0)):
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)):
            Rectangle(44.5, 139.0)
    extrude(amount=40.0, mode=Mode.SUBTRACT)

    # ── Pocket 1 at Y=1198.186 ────────────────────────────────
    _origin_p1 = Vector(1849.205, 1198.186, 77.635)
    plane_p1 = Plane(origin=_origin_p1, x_dir=_xd, z_dir=(0.0, -1.0, 0.0))
    with BuildSketch(plane_p1):
        with Locations((25.0, 13.5)):
            Rectangle(50.0, 27.0)
    extrude(amount=7.1, mode=Mode.SUBTRACT) # Depth of 7.0 but using 7.1 to guarantee clean boolean subtracts

    # ── Pocket 2 at Y=1045.186 ────────────────────────────────
    _origin_p2 = Vector(1849.205, 1045.186, 77.635)
    plane_p2 = Plane(origin=_origin_p2, x_dir=_xd, z_dir=(0.0, -1.0, 0.0))
    with BuildSketch(plane_p2):
        with Locations((25.0, 13.5)):
            Rectangle(50.0, 27.0)
    extrude(amount=-7.1, mode=Mode.SUBTRACT)

print(f"  Frame 3 valid={frame3.part.is_valid}, volume={frame3.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 4
# ══════════════════════════════════════════════════════════════
print("Building frame 4...")
with BuildPart() as frame4:

    # ── Frame 4: outer 185×185, inner 139×139 ─────────────────
    with BuildSketch(pl4):
        with Locations((92.5, 92.5)): 
            Rectangle(185.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=40.0)

    # ── Subtract: two slots through the body ──────────────────
    with BuildSketch(pl4_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-40.0, mode=Mode.SUBTRACT)

    # ── Pocket 1 at Y=971.338 ─────────────────────────────────
    with BuildSketch(Plane(origin=Vector(1849.205, 971.338, 84.371),
                           x_dir=_xd, z_dir=_n4_slot)):
        with Locations((25.0, 13.5)): 
            Rectangle(50.0, 27.0)
    extrude(amount=7.1, mode=Mode.SUBTRACT)

    # ── Pocket 2 at Y=822.948 ─────────────────────────────────
    with BuildSketch(Plane(origin=Vector(1849.205, 822.948, 121.644),
                           x_dir=_xd, z_dir=_n4_slot)):
        with Locations((25.0, 13.5)): 
            Rectangle(50.0, 27.0)
    extrude(amount=-7.1, mode=Mode.SUBTRACT)

print(f"  Frame 4 valid={frame4.part.is_valid}, volume={frame4.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Loft 1: front wall
# ══════════════════════════════════════════════════════════════
print("Building loft1...")
loft1 = make_loft_solid(
    [(1789.205, 1480.613, 151.309),
     (2009.379, 1480.613, 151.309),
     (2008.642, 1428.083, 128.436),
     (1789.205, 1428.083, 128.436)],
    [(1789.205, 1461.711, 186.561),
     (2009.379, 1461.711, 186.561),
     (2008.642, 1418.339, 167.231),
     (1789.205, 1418.339, 167.231)]
)

# ══════════════════════════════════════════════════════════════
# Loft 2: bottom front wall
# ══════════════════════════════════════════════════════════════
print("Building loft2...")
loft2 = make_loft_solid(
    [   # source — trapezoid
        (1789.205, 1263.205,  87.021),   # top-left
        (2006.331, 1263.205,  87.021),   # top-right
        (2005.538, 1206.686,  77.635),   # bottom-right
        (1789.205, 1206.686,  77.635),   # bottom-left
    ],
    [   # target — rectangle, right edge coplanar with source
        (1789.205, 1253.461, 125.816),   # top-left
        (2006.331, 1253.461, 125.816),   # top-right  ← matches source top-right X? No — use source
        (2005.538, 1206.686, 117.635),   # bottom-right
        (1789.205, 1206.686, 117.635),   # bottom-left
    ]
)

# ══════════════════════════════════════════════════════════════
# Loft 3: Third wall segment
# ══════════════════════════════════════════════════════════════
print("Building loft3...")
loft3 = make_loft_solid(
    [   # source — trapezoid
        (1789.205, 1036.686,  77.635),   # top-left
        (2003.155, 1036.686,  77.635),   # top-right
        (2002.354,  979.582,  82.300),   # bottom-right
        (1789.205,  979.582,  82.300),   # bottom-left
    ],
    [   # target — right edge X-coordinates matched to source for coplanarity
        (1789.205, 1036.686, 117.635),   # top-left
        (2003.155, 1036.686, 117.635),   # top-right (overridden from 1974.205)
        (2002.354,  989.327, 121.095),   # bottom-right (overridden from 1974.205)
        (1789.205,  989.327, 121.095),   # bottom-left
    ]
)

# ══════════════════════════════════════════════════════════════
# Loft 4: Left side wall segment
# ══════════════════════════════════════════════════════════════
print("Building loft4...")
loft4 = make_ruled_solid(
    [   # upper profile
        (1747.944, 1036.319, 116.895),
        (1789.205, 1036.686, 117.635),
        (1789.205,  989.327, 121.095),
        (1748.267,  988.927, 119.844),
    ],
    [   # lower profile
        (1744.180, 1036.736,  77.075),
        (1789.205, 1036.686,  77.635),
        (1789.205,  979.582,  82.300),
        (1744.615,  979.587,  81.121),
    ]
)

def make_loft5():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.BRepLib import BRepLib
    from OCP.ShapeFix import ShapeFix_Solid
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Source cap ────────────────────────────────────────────
    A1  = (1744.180, 1036.736,  77.075)
    A2  = (1789.205, 1036.686,  77.635)
    A3  = (1789.205, 1206.686,  77.635)
    A4  = (1744.193, 1206.727,  78.855)
    Am  = (1766.696, 1121.709,  77.800)
    A2m = (1789.205, 1107.314,  77.635)   # split point on A2→A3

    # ── Target cap ────────────────────────────────────────────
    B1  = (1747.944, 1036.319, 116.895)
    B2  = (1789.205, 1036.686, 117.635)
    B3  = (1789.205, 1107.314, 117.635)
    B4  = (1789.205, 1206.686, 117.635)
    B5  = (1747.957, 1206.310, 118.675)
    Bm  = (1768.578, 1121.500, 117.710)

    faces = []

    # Source cap — fan from Am, CCW winding (normal points -Z / outward)
    faces += [
        tri(Am, A2,  A1),
        tri(Am, A2m, A2),
        tri(Am, A3,  A2m),
        tri(Am, A4,  A3),
        tri(Am, A1,  A4),
    ]

    # Target cap — fan from Bm, CW winding (normal points +Z / outward)
    faces += [
        tri(Bm, B1,  B2),
        tri(Bm, B2,  B3),
        tri(Bm, B3,  B4),
        tri(Bm, B4,  B5),
        tri(Bm, B5,  B1),
    ]

    # Side walls
    faces += [
        # Bottom edge: A1–A2 ↔ B1–B2
        tri(A1, B2, B1), tri(A1, A2, B2),
        # Right lower: A2–A2m ↔ B2–B3
        tri(A2, B3, B2), tri(A2, A2m, B3),
        # Right upper: A2m–A3 ↔ B3–B4
        tri(A2m, B4, B3), tri(A2m, A3, B4),
        # Top: A3–A4 ↔ B4–B5
        tri(A3, B5, B4), tri(A3, A4, B5),
        # Left: A4–A1 ↔ B5–B1
        tri(A4, B1, B5), tri(A4, A1, B1),
    ]

    sew = BRepBuilderAPI_Sewing(1e-2)
    for f in faces:
        sew.Add(f)
    sew.Perform()
    sewn = sew.SewedShape()
    print(f"    Sewn type: {sewn.ShapeType()}")

    # Use ShapeFix_Solid — handles open shells and bad orientation
    try:
        shell = TopoDS.Shell_s(sewn)
    except Exception as e:
        raise ValueError(f"Shell cast failed: {e}")

    fix = ShapeFix_Solid()
    fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
    fix.Perform()
    s = Solid(fix.Solid())
    print(f"    After fix: valid={s.is_valid}, volume={s.volume:.1f}")

    # If volume is negative the cap windings are flipped — invert all faces
    if s.volume < 0:
        print("    Volume negative — reversing cap windings...")
        faces_r = []
        # Flip source cap winding
        faces_r += [
            tri(Am, A1,  A2),
            tri(Am, A2,  A2m),
            tri(Am, A2m, A3),
            tri(Am, A3,  A4),
            tri(Am, A4,  A1),
        ]
        # Flip target cap winding
        faces_r += [
            tri(Bm, B2,  B1),
            tri(Bm, B3,  B2),
            tri(Bm, B4,  B3),
            tri(Bm, B5,  B4),
            tri(Bm, B1,  B5),
        ]
        # Keep side walls same
        faces_r += faces[10:]
        sew2 = BRepBuilderAPI_Sewing(1e-2)
        for f in faces_r:
            sew2.Add(f)
        sew2.Perform()
        shell2 = TopoDS.Shell_s(sew2.SewedShape())
        fix2 = ShapeFix_Solid()
        fix2.Init(BRepBuilderAPI_MakeSolid(shell2).Solid())
        fix2.Perform()
        s = Solid(fix2.Solid())
        print(f"    After flip: valid={s.is_valid}, volume={s.volume:.1f}")

    if s.is_valid and s.volume > 0:
        print(f"  loft5 OK: vol={s.volume:.1f}")
        return s
    raise ValueError("make_loft5 failed — see debug above")

loft5 = make_loft5()

# ══════════════════════════════════════════════════════════════
# Loft 6: Left panel lower segment
# ══════════════════════════════════════════════════════════════
print("Building loft6...")

def make_loft6():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Source cap (lower profile) ────────────────────────────
    A1  = (1744.615,  979.587,  81.121)   # top-left
    A2  = (1789.205,  979.582,  82.300)   # top-right
    A3  = (1789.205,  785.608, 131.023)   # bottom-right
    A4  = (1749.186,  785.115, 127.594)   # bottom-left
    Am  = (1768.053,  882.473, 105.510)   # interior midpoint

    # ── Target cap (upper profile) ────────────────────────────
    B1  = (1748.267,  988.927, 119.844)   # top-left
    B2  = (1789.205,  989.327, 121.095)   # top-right
    B3  = (1789.205,  809.901, 166.164)   # bottom-right
    B4  = (1752.495,  809.040, 162.831)   # bottom-left
    Bm  = (1769.793,  899.299, 142.484)   # interior midpoint

    faces = []

    # Source cap — fan from Am, CCW winding (normal outward)
    faces += [
        tri(Am, A2,  A1),
        tri(Am, A3,  A2),
        tri(Am, A4,  A3),
        tri(Am, A1,  A4),
    ]

    # Target cap — fan from Bm, CW winding (normal outward)
    faces += [
        tri(Bm, B1,  B2),
        tri(Bm, B2,  B3),
        tri(Bm, B3,  B4),
        tri(Bm, B4,  B1),
    ]

    # Side walls — 4 edges, each as a quad split into 2 triangles
    faces += [
        # Top edge:    A1–A2 ↔ B1–B2
        tri(A1, B2, B1), tri(A1, A2, B2),
        # Right edge:  A2–A3 ↔ B2–B3
        tri(A2, B3, B2), tri(A2, A3, B3),
        # Bottom edge: A3–A4 ↔ B3–B4
        tri(A3, B4, B3), tri(A3, A4, B4),
        # Left edge:   A4–A1 ↔ B4–B1
        tri(A4, B1, B4), tri(A4, A1, B1),
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        sewn = sew.SewedShape()
        print(f"    Sewn type: {sewn.ShapeType()}")
        shell = TopoDS.Shell_s(sewn)
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)
    print(f"    After fix: valid={s.is_valid}, volume={s.volume:.1f}")

    if s.is_valid and s.volume > 0:
        print(f"  loft6 OK: vol={s.volume:.1f}")
        return s

    # Volume negative — flip all cap windings, keep side walls
    print("    Volume negative — reversing cap windings...")
    faces_r = []
    faces_r += [
        tri(Am, A1,  A2),
        tri(Am, A2,  A3),
        tri(Am, A3,  A4),
        tri(Am, A4,  A1),
    ]
    faces_r += [
        tri(Bm, B2,  B1),
        tri(Bm, B3,  B2),
        tri(Bm, B4,  B3),
        tri(Bm, B1,  B4),
    ]
    faces_r += faces[8:]   # side walls unchanged

    s = build_solid(faces_r)
    print(f"    After flip: valid={s.is_valid}, volume={s.volume:.1f}")

    if s.is_valid and s.volume > 0:
        print(f"  loft6 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft6 failed — see debug above")

loft6 = make_loft6()

# ══════════════════════════════════════════════════════════════
# Loft 7: Left panel upper-mid segment
# ══════════════════════════════════════════════════════════════
print("Building loft7...")

def make_loft7():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Source cap (upper profile) ────────────────────────────
    A1  = (1747.957, 1206.310, 118.675)   # bottom-left
    A2  = (1789.205, 1206.686, 117.635)   # bottom-right
    A3  = (1789.205, 1253.461, 125.816)   # top-right
    A4  = (1748.730, 1252.997, 127.310)   # top-left
    Am  = (1768.774, 1229.863, 122.359)   # interior midpoint

    # ── Target cap (lower profile) ────────────────────────────
    B1  = (1744.193, 1206.727,  78.855)   # bottom-left
    B2  = (1789.205, 1206.686,  77.635)   # bottom-right
    B3  = (1789.205, 1263.205,  87.021)   # top-right
    B4  = (1745.080, 1263.145,  88.791)   # top-left
    Bm  = (1766.921, 1234.941,  83.076)   # interior midpoint

    faces = []

    # Source cap — fan from Am, CCW winding
    faces += [
        tri(Am, A2,  A1),
        tri(Am, A3,  A2),
        tri(Am, A4,  A3),
        tri(Am, A1,  A4),
    ]

    # Target cap — fan from Bm, CW winding
    faces += [
        tri(Bm, B1,  B2),
        tri(Bm, B2,  B3),
        tri(Bm, B3,  B4),
        tri(Bm, B4,  B1),
    ]

    # Side walls
    faces += [
        # Bottom: A1–A2 ↔ B1–B2
        tri(A1, B2, B1), tri(A1, A2, B2),
        # Right:  A2–A3 ↔ B2–B3
        tri(A2, B3, B2), tri(A2, A3, B3),
        # Top:    A3–A4 ↔ B3–B4
        tri(A3, B4, B3), tri(A3, A4, B4),
        # Left:   A4–A1 ↔ B4–B1
        tri(A4, B1, B4), tri(A4, A1, B1),
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        sewn = sew.SewedShape()
        print(f"    Sewn type: {sewn.ShapeType()}")
        shell = TopoDS.Shell_s(sewn)
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)
    print(f"    After fix: valid={s.is_valid}, volume={s.volume:.1f}")

    if s.is_valid and s.volume > 0:
        print(f"  loft7 OK: vol={s.volume:.1f}")
        return s

    # Volume negative — flip cap windings, keep side walls
    print("    Volume negative — reversing cap windings...")
    faces_r = []
    faces_r += [
        tri(Am, A1,  A2),
        tri(Am, A2,  A3),
        tri(Am, A3,  A4),
        tri(Am, A4,  A1),
    ]
    faces_r += [
        tri(Bm, B2,  B1),
        tri(Bm, B3,  B2),
        tri(Bm, B4,  B3),
        tri(Bm, B1,  B4),
    ]
    faces_r += faces[8:]   # side walls unchanged

    s = build_solid(faces_r)
    print(f"    After flip: valid={s.is_valid}, volume={s.volume:.1f}")

    if s.is_valid and s.volume > 0:
        print(f"  loft7 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft7 failed — see debug above")

loft7 = make_loft7()

# ══════════════════════════════════════════════════════════════
# Loft 8: Left panel upper segment
# ══════════════════════════════════════════════════════════════
print("Building loft8...")

def make_loft8():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Source cap (lower profile) ────────────────────────────
    A1  = (1745.080, 1263.145,  88.791)   # bottom-left
    A2  = (1789.205, 1263.205,  87.021)   # bottom-right
    A3  = (1789.205, 1428.083, 128.436)   # top-right
    A4  = (1748.989, 1427.583, 131.746)   # top-left
    Am  = (1768.120, 1345.504, 108.999)   # interior midpoint

    # ── Target cap (upper profile) ────────────────────────────
    B1  = (1748.730, 1252.997, 127.310)   # bottom-left
    B2  = (1789.205, 1253.461, 125.816)   # bottom-right
    B3  = (1789.205, 1418.339, 167.231)   # top-right
    B4  = (1752.640, 1417.434, 170.265)   # top-left
    Bm  = (1769.945, 1335.558, 147.655)   # interior midpoint

    faces = []

    # Source cap — fan from Am, CCW winding
    faces += [
        tri(Am, A2,  A1),
        tri(Am, A3,  A2),
        tri(Am, A4,  A3),
        tri(Am, A1,  A4),
    ]

    # Target cap — fan from Bm, CW winding
    faces += [
        tri(Bm, B1,  B2),
        tri(Bm, B2,  B3),
        tri(Bm, B3,  B4),
        tri(Bm, B4,  B1),
    ]

    # Side walls
    faces += [
        # Bottom: A1–A2 ↔ B1–B2
        tri(A1, B2, B1), tri(A1, A2, B2),
        # Right:  A2–A3 ↔ B2–B3
        tri(A2, B3, B2), tri(A2, A3, B3),
        # Top:    A3–A4 ↔ B3–B4
        tri(A3, B4, B3), tri(A3, A4, B4),
        # Left:   A4–A1 ↔ B4–B1
        tri(A4, B1, B4), tri(A4, A1, B1),
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        sewn = sew.SewedShape()
        print(f"    Sewn type: {sewn.ShapeType()}")
        shell = TopoDS.Shell_s(sewn)
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)
    print(f"    After fix: valid={s.is_valid}, volume={s.volume:.1f}")

    if s.is_valid and s.volume > 0:
        print(f"  loft8 OK: vol={s.volume:.1f}")
        return s

    # Volume negative — flip cap windings, keep side walls
    print("    Volume negative — reversing cap windings...")
    faces_r = []
    faces_r += [
        tri(Am, A1,  A2),
        tri(Am, A2,  A3),
        tri(Am, A3,  A4),
        tri(Am, A4,  A1),
    ]
    faces_r += [
        tri(Bm, B2,  B1),
        tri(Bm, B3,  B2),
        tri(Bm, B4,  B3),
        tri(Bm, B1,  B4),
    ]
    faces_r += faces[8:]   # side walls unchanged

    s = build_solid(faces_r)
    print(f"    After flip: valid={s.is_valid}, volume={s.volume:.1f}")

    if s.is_valid and s.volume > 0:
        print(f"  loft8 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft8 failed — see debug above")

loft8 = make_loft8()

# ══════════════════════════════════════════════════════════════
# Loft 9: Left panel top segment
# ══════════════════════════════════════════════════════════════
print("Building loft9...")

def make_loft9():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Source cap (lower profile, pentagon) ──────────────────
    A1  = (1748.989, 1427.583, 131.746)   # bottom-left
    A2  = (1789.205, 1428.083, 128.436)   # bottom-right
    A3  = (1789.205, 1480.613, 151.309)   # top-right
    A4  = (1751.146, 1479.871, 155.066)   # top-left
    A5  = (1749.789, 1446.981, 140.398)   # mid-left (splits left edge)
    Am  = (1769.636, 1454.038, 141.639)   # interior midpoint

    # ── Target cap (upper profile, pentagon) ──────────────────
    B1  = (1752.640, 1417.434, 170.265)   # bottom-left
    B2  = (1789.205, 1418.339, 167.231)   # bottom-right
    B3  = (1789.205, 1461.711, 186.561)   # top-right
    B4  = (1754.462, 1460.602, 189.962)   # top-left
    B5  = (1752.700, 1418.871, 170.920)   # near-bottom-left (1.58mm step)
    Bm  = (1771.378, 1439.522, 178.505)   # interior midpoint

    faces = []

    # Source cap — fan from Am, CCW winding (5 triangles)
    faces += [
        tri(Am, A2,  A1),
        tri(Am, A3,  A2),
        tri(Am, A4,  A3),
        tri(Am, A5,  A4),
        tri(Am, A1,  A5),
    ]

    # Target cap — fan from Bm, CW winding (5 triangles)
    faces += [
        tri(Bm, B1,  B2),
        tri(Bm, B2,  B3),
        tri(Bm, B3,  B4),
        tri(Bm, B4,  B5),
        tri(Bm, B5,  B1),
    ]

    # Side walls — 5 edges matched between profiles
    faces += [
        # Bottom:     A1–A2  ↔  B1–B2
        tri(A1, B2, B1), tri(A1, A2, B2),
        # Right:      A2–A3  ↔  B2–B3
        tri(A2, B3, B2), tri(A2, A3, B3),
        # Top:        A3–A4  ↔  B3–B4
        tri(A3, B4, B3), tri(A3, A4, B4),
        # Left-upper: A4–A5  ↔  B4–B5
        tri(A4, B5, B4), tri(A4, A5, B5),
        # Left-lower: A5–A1  ↔  B5–B1
        tri(A5, B1, B5), tri(A5, A1, B1),
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        sewn = sew.SewedShape()
        print(f"    Sewn type: {sewn.ShapeType()}")
        shell = TopoDS.Shell_s(sewn)
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)
    print(f"    After fix: valid={s.is_valid}, volume={s.volume:.1f}")

    if s.is_valid and s.volume > 0:
        print(f"  loft9 OK: vol={s.volume:.1f}")
        return s

    # Volume negative — flip cap windings, keep side walls
    print("    Volume negative — reversing cap windings...")
    faces_r = []
    faces_r += [
        tri(Am, A1,  A2),
        tri(Am, A2,  A3),
        tri(Am, A3,  A4),
        tri(Am, A4,  A5),
        tri(Am, A5,  A1),
    ]
    faces_r += [
        tri(Bm, B2,  B1),
        tri(Bm, B3,  B2),
        tri(Bm, B4,  B3),
        tri(Bm, B5,  B4),
        tri(Bm, B1,  B5),
    ]
    faces_r += faces[10:]   # side walls unchanged (10 faces = 5 edges × 2 tris)

    s = build_solid(faces_r)
    print(f"    After flip: valid={s.is_valid}, volume={s.volume:.1f}")

    if s.is_valid and s.volume > 0:
        print(f"  loft9 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft9 failed — see debug above")

loft9 = make_loft9()

# ══════════════════════════════════════════════════════════════
# Loft 10: Left panel tall upper segment
# ══════════════════════════════════════════════════════════════
print("Building loft10...")

def make_loft10():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    from OCP.BRepLib import BRepLib

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    A1  = (1751.146, 1479.871, 155.066)
    A2  = (1789.205, 1480.613, 151.309)
    A3  = (1789.205, 1630.435, 231.642)
    A4  = (1758.717, 1628.846, 236.608)
    Am  = (1772.068, 1554.941, 193.656)
    Am2 = (1773.015, 1592.291, 213.891)
    Am3 = (1771.122, 1517.592, 173.422)

    B1  = (1754.462, 1460.602, 189.962)
    B2  = (1789.205, 1461.711, 186.561)
    B3  = (1789.205, 1624.753, 273.983)
    B4  = (1762.701, 1622.723, 278.699)
    Bm  = (1773.893, 1542.447, 232.301)
    Bm2 = (1774.923, 1583.092, 254.321)
    Bm3 = (1772.863, 1501.802, 210.281)

    def build_from_faces(face_list):
        sew = BRepBuilderAPI_Sewing(1e-3)
        for f in face_list: sew.Add(f)
        sew.Perform()
        sewn = sew.SewedShape()
        print(f"    Sewn: {sewn.ShapeType()}")
        shell = TopoDS.Shell_s(sewn)
        # Try both shell orientations, with and without ShapeFix
        for sh in [shell, TopoDS.Shell_s(shell.Reversed())]:
            ms = BRepBuilderAPI_MakeSolid(); ms.Add(sh); ms.Build()
            raw = Solid(ms.Shape())
            if raw.is_valid and raw.volume > 0:
                return raw
            fixer = ShapeFix_Solid()
            fixer.Init(ms.Solid()); fixer.Perform()
            fixed = Solid(fixer.Solid())
            if fixed.is_valid and fixed.volume > 0:
                return fixed
        # Accept best positive-abs-volume result even if not fully valid
        best = None
        for sh in [shell, TopoDS.Shell_s(shell.Reversed())]:
            ms = BRepBuilderAPI_MakeSolid(); ms.Add(sh); ms.Build()
            s = Solid(ms.Shape())
            if best is None or abs(s.volume) > abs(best.volume):
                best = s
        return best

    # ── Side wall winding: tri(P1, Q2, Q1), tri(P1, P2, Q2)
    # This is the exact same pattern used by the working loft9.
    # Source boundary: A1→Am3→A2→A3→Am2→A4→A1 (6 edges)
    # Target boundary: B1→Bm3→B2→B3→Bm2→B4→B1 (6 edges)

    def make_faces(src_cap_fwd, tgt_cap_fwd):
        f = []
        # Source cap (6 tris fanning from Am)
        if src_cap_fwd:
            f += [tri(Am, Am3, A1), tri(Am, A2, Am3), tri(Am, A3, A2),
                  tri(Am, Am2, A3), tri(Am, A4, Am2), tri(Am, A1, A4)]
        else:
            f += [tri(Am, A1, Am3), tri(Am, Am3, A2), tri(Am, A2, A3),
                  tri(Am, A3, Am2), tri(Am, Am2, A4), tri(Am, A4, A1)]
        # Target cap (6 tris fanning from Bm)
        if tgt_cap_fwd:
            f += [tri(Bm, B1, Bm3), tri(Bm, Bm3, B2), tri(Bm, B2, B3),
                  tri(Bm, B3, Bm2), tri(Bm, Bm2, B4), tri(Bm, B4, B1)]
        else:
            f += [tri(Bm, Bm3, B1), tri(Bm, B2, Bm3), tri(Bm, B3, B2),
                  tri(Bm, Bm2, B3), tri(Bm, B4, Bm2), tri(Bm, B1, B4)]
        # Side walls: tri(P1, Q2, Q1), tri(P1, P2, Q2)  ← loft9's proven pattern
        f += [
            tri(A1,  Bm3, B1),  tri(A1,  Am3, Bm3),   # A1→Am3 | B1→Bm3
            tri(Am3, B2,  Bm3), tri(Am3, A2,  B2),     # Am3→A2 | Bm3→B2
            tri(A2,  B3,  B2),  tri(A2,  A3,  B3),     # A2→A3  | B2→B3
            tri(A3,  Bm2, B3),  tri(A3,  Am2, Bm2),   # A3→Am2 | B3→Bm2
            tri(Am2, B4,  Bm2), tri(Am2, A4,  B4),     # Am2→A4 | Bm2→B4
            tri(A4,  B1,  B4),  tri(A4,  A1,  B1),     # A4→A1  | B4→B1
        ]
        return f

    for src_fwd in [True, False]:
        for tgt_fwd in [True, False]:
            print(f"  Trying src_fwd={src_fwd} tgt_fwd={tgt_fwd}...")
            s = build_from_faces(make_faces(src_fwd, tgt_fwd))
            if s is not None and s.is_valid and s.volume > 0:
                print(f"  loft10 OK: vol={s.volume:.1f}")
                return s

    # Last resort — return best abs volume
    s = build_from_faces(make_faces(True, True))
    if s is not None and abs(s.volume) > 0:
        print(f"  loft10 best effort: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft10 failed")

loft10 = make_loft10()

# ══════════════════════════════════════════════════════════════
# Fuse
# ══════════════════════════════════════════════════════════════
print("Fusing...")
with BuildPart() as part:
    add(frame.part)
    add(frame3.part)
    add(frame4.part) 
    add(loft1)
    add(loft2)
    add(loft3)
    add(loft4)
    add(loft5)
    add(loft6)
    add(loft7)
    add(loft8)
    add(loft9)
    add(loft10)
print(f"  Final valid={part.part.is_valid}, volume={part.part.volume:.1f}")

try:
    from ocp_vscode import show
    show(part)
except ImportError:
    pass

export_stl(part.part, "output_frame_block.stl")
print("Done.")