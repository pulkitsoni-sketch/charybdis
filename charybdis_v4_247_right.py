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

# ── Frame 5 planes ────────────────────────────────────────────
_orig5   = Vector(1585.216, 1460.447, 205.96)
_corner5 = Vector(1754.462, 1460.602, 189.962) # 170mm edge

_xd5     = (_corner5 - _orig5).normalized()
_n5      = Vector(-3.316, 19.269, -34.896).normalized()
    
pl5      = Plane(origin=_orig5, x_dir=_xd5, z_dir=-_n5)
pl5_tgt  = Plane(origin=_orig5 + _n5 * 40.0, x_dir=_xd5, z_dir=-_n5)

# ── Frame 6 planes ────────────────────────────────────────────
_orig6   = Vector(1583.394, 1417.279, 186.263) # Top-Left
_corn_x  = Vector(1752.640, 1417.434, 170.265) # Top-Right
_corn_y  = Vector(1579.485, 1252.842, 143.307) # Bottom-Left
_ext_tgt = Vector(1579.744, 1427.428, 147.744) # Extrude target (Lower profile)

# Explicitly lock X and Y directions to the frame's true edges
_xd6 = (_corn_x - _orig6).normalized()
_yd6 = (_corn_y - _orig6).normalized()
_zd6 = _xd6.cross(_yd6).normalized()

# The local (85, 85) center is now mathematically forced to map into your polygon bounds
pl6 = Plane(origin=_orig6, x_dir=_xd6, z_dir=_zd6)

# Calculate exact extrude depth dynamically from the target vector
_ext_amt6 = (_ext_tgt - _orig6).dot(_zd6) 
pl6_tgt = Plane(origin=_orig6 + _zd6 * _ext_amt6, x_dir=_xd6, z_dir=_zd6)

# ── Frame 7 planes ────────────────────────────────────────────
_orig7   = Vector(1578.711, 1206.155, 134.673) # TL
_corn7_x = Vector(1747.957, 1206.310, 118.675) # TR
_corn7_y = Vector(1578.699, 1036.164, 132.893) # BL
_ext_tg7 = Vector(1574.947, 1206.572, 94.853)  # Lower TL

_xd7 = (_corn7_x - _orig7).normalized()
_yd7 = (_corn7_y - _orig7).normalized()
_zd7 = _xd7.cross(_yd7).normalized()

pl7 = Plane(origin=_orig7, x_dir=_xd7, z_dir=_zd7)
_ext_amt7 = (_ext_tg7 - _orig7).dot(_zd7)
pl7_tgt = Plane(origin=_orig7 + _zd7 * _ext_amt7, x_dir=_xd7, z_dir=_zd7)

# ── Frame 8 planes ────────────────────────────────────────────
_orig8   = Vector(1559.110, 988.753, 137.724) # TL (Upper)
_corn8_x = Vector(1748.267, 988.927, 119.844) # TR (Upper)
_corn8_y = Vector(1563.338, 808.867, 180.711) # BL (Upper)
_ext_tg8 = Vector(1555.458, 979.414, 99.001)  # TL (Lower)

# Map X and Y directions to the frame's physical edges
_xd8 = (_corn8_x - _orig8).normalized()
_yd8 = (_corn8_y - _orig8).normalized()
_zd8 = _xd8.cross(_yd8).normalized()

pl8 = Plane(origin=_orig8, x_dir=_xd8, z_dir=_zd8)
_ext_amt8 = (_ext_tg8 - _orig8).dot(_zd8)
pl8_tgt = Plane(origin=_orig8 + _zd8 * _ext_amt8, x_dir=_xd8, z_dir=_zd8)

# ── Frame 9 planes ────────────────────────────────────────────
_orig9   = Vector(1370.361, 1585.716, 183.018) # TL (Upper)
_corn9_x = Vector(1537.778, 1585.460, 153.500) # TR (approx, for X-axis)
_corn9_y = Vector(1385.788, 1748.740, 269.101) # BL (Upper)
_ext_tg9 = Vector(1364.268, 1604.626, 148.300) # TL (Lower)

# Map X and Y directions to the frame's true edges
_xd9 = (_corn9_x - _orig9).normalized()
_yd9 = (_corn9_y - _orig9).normalized()
_zd9 = _xd9.cross(_yd9).normalized()

pl9 = Plane(origin=_orig9, x_dir=_xd9, z_dir=_zd9)
_ext_amt9 = (_ext_tg9 - _orig9).dot(_zd9)
pl9_tgt = Plane(origin=_orig9 + _zd9 * _ext_amt9, x_dir=_xd9, z_dir=_zd9)

# ── Frame 10 planes ───────────────────────────────────────────
_orig10   = Vector(1366.938, 1542.348, 163.984) # TL (Upper)
_corn10_x = Vector(1534.356, 1542.091, 134.466) # TR (Upper)
_corn10_y = Vector(1359.496, 1377.478, 123.210) # BL (Upper)
_ext_tg10 = Vector(1360.217, 1552.100, 125.778) # TL (Lower)

# Map X and Y directions to the frame's physical edges
_xd10 = (_corn10_x - _orig10).normalized()
_yd10 = (_corn10_y - _orig10).normalized()
_zd10 = _xd10.cross(_yd10).normalized()

pl10 = Plane(origin=_orig10, x_dir=_xd10, z_dir=_zd10)
_ext_amt10 = (_ext_tg10 - _orig10).dot(_zd10)
pl10_tgt = Plane(origin=_orig10 + _zd10 * _ext_amt10, x_dir=_xd10, z_dir=_zd10)

# ── Frame 11 planes ───────────────────────────────────────────
_orig11   = Vector(1358.005, 1330.705, 115.156) # Top-Left (Upper)
_corn11_x = Vector(1525.422, 1330.448, 85.638)  # Top-Right (Upper)
_corn11_y = Vector(1357.746, 1160.705, 115.167) # Bottom-Left (Upper)
_ext_tg11 = Vector(1351.059, 1330.713, 75.763)  # Top-Left (Lower)

# Map X and Y directions to the frame's true edges
_xd11 = (_corn11_x - _orig11).normalized()
_yd11 = (_corn11_y - _orig11).normalized()
_zd11 = _xd11.cross(_yd11).normalized()

pl11 = Plane(origin=_orig11, x_dir=_xd11, z_dir=_zd11)
_ext_amt11 = (_ext_tg11 - _orig11).dot(_zd11)
pl11_tgt = Plane(origin=_orig11 + _zd11 * _ext_amt11, x_dir=_xd11, z_dir=_zd11)

# ── Frame 12 planes ───────────────────────────────────────────
_orig12   = Vector(1358.275, 1113.346, 118.578) # Top-Left (Upper)
_corn12_x = Vector(1525.692, 1113.089, 89.060)  # Top-Right (Upper)
_corn12_y = Vector(1365.828, 933.911, 162.974) # Bottom-Left (Upper)
_ext_tg12 = Vector(1351.524, 1103.609, 80.373)  # Top-Left (Lower)

# Map X and Y directions to the frame's true edges
_xd12 = (_corn12_x - _orig12).normalized()
_yd12 = (_corn12_y - _orig12).normalized()
_zd12 = _xd12.cross(_yd12).normalized()

pl12 = Plane(origin=_orig12, x_dir=_xd12, z_dir=_zd12)
_ext_amt12 = (_ext_tg12 - _orig12).dot(_zd12)
pl12_tgt = Plane(origin=_orig12 + _zd12 * _ext_amt12, x_dir=_xd12, z_dir=_zd12)

# ── Frame 13 planes ───────────────────────────────────────────
_orig13   = Vector(1146.640, 1136.058, 131.779) # Top-Left (Upper)
_corn13_x = Vector(1310.035, 1137.930,  84.888) # Top-Right (Upper)
_corn13_y = Vector(1160.666,  956.365, 173.478) # Bottom-Left (Upper)
_ext_tg13 = Vector(1136.023, 1126.556,  94.402) # Top-Left (Lower)

# Map X and Y directions to the frame's physical edges
_xd13 = (_corn13_x - _orig13).normalized()
_yd13 = (_corn13_y - _orig13).normalized()
_zd13 = _xd13.cross(_yd13).normalized()

pl13 = Plane(origin=_orig13, x_dir=_xd13, z_dir=_zd13)
_ext_amt13 = (_ext_tg13 - _orig13).dot(_zd13)
pl13_tgt = Plane(origin=_orig13 + _zd13 * _ext_amt13, x_dir=_xd13, z_dir=_zd13)

# ── Frame 14 planes ───────────────────────────────────────────
_orig14   = Vector(1143.758, 1353.421, 130.414) # Top-Left (Upper)
_corn14_x = Vector(1307.153, 1355.293,  83.524) # Top-Right (Upper)
_corn14_y = Vector(1145.266, 1183.434, 128.880) # Bottom-Left (Upper)
_ext_tg14 = Vector(1132.722, 1353.670,  91.967) # Top-Left (Lower)

# Map X and Y directions to the frame's physical edges
_xd14 = (_corn14_x - _orig14).normalized()
_yd14 = (_corn14_y - _orig14).normalized()
_zd14 = _xd14.cross(_yd14).normalized()

pl14 = Plane(origin=_orig14, x_dir=_xd14, z_dir=_zd14)
_ext_amt14 = (_ext_tg14 - _orig14).dot(_zd14)
pl14_tgt = Plane(origin=_orig14 + _zd14 * _ext_amt14, x_dir=_xd14, z_dir=_zd14)

# ── Frame 15 planes ───────────────────────────────────────────
_orig15   = Vector(1145.601, 1400.141, 138.699) # Top-Left (Upper)
_corn15_x = Vector(1308.995, 1402.013,  91.809) # Top-Right (Upper)
_corn15_y = Vector(1155.565, 1564.748, 179.993) # Bottom-Left (Upper)
_ext_tg15 = Vector(1134.811, 1410.126, 101.499) # Top-Left (Lower)

# Map X and Y directions to the frame's true edges
_xd15 = (_corn15_x - _orig15).normalized()
_yd15 = (_corn15_y - _orig15).normalized()
_zd15 = _xd15.cross(_yd15).normalized()

pl15 = Plane(origin=_orig15, x_dir=_xd15, z_dir=_zd15)
_ext_amt15 = (_ext_tg15 - _orig15).dot(_zd15)
pl15_tgt = Plane(origin=_orig15 + _zd15 * _ext_amt15, x_dir=_xd15, z_dir=_zd15)

# ── Frame 16 planes ───────────────────────────────────────────
_orig16   = Vector(1160.513, 1607.997, 198.964) # Top-Left (Upper)
_corn16_x = Vector(1323.908, 1609.869, 152.074) # Top-Right (Upper)
_corn16_y = Vector(1171.851, 1689.239, 241.713) # Bottom-Left (Upper)
_ext_tg16 = Vector(1150.620, 1627.117, 165.251) # Top-Left (Lower)

# Map X and Y directions to the frame's true edges
_xd16 = (_corn16_x - _orig16).normalized()
_yd16 = (_corn16_y - _orig16).normalized()
_zd16 = _xd16.cross(_yd16).normalized()

pl16 = Plane(origin=_orig16, x_dir=_xd16, z_dir=_zd16)
_ext_amt16 = (_ext_tg16 - _orig16).dot(_zd16)
pl16_tgt = Plane(origin=_orig16 + _zd16 * _ext_amt16, x_dir=_xd16, z_dir=_zd16)

# ── Frame 17 planes ───────────────────────────────────────────
_orig17   = Vector(955.678, 1105.308, 238.558) # Top-Left (Upper)
_corn17_x = Vector(1115.424, 1104.254, 180.420) # Top-Right (Upper)
_corn17_y = Vector(970.307, 926.077, 282.003) # Bottom-Left (Upper)
_ext_tg17 = Vector(942.366, 1095.399, 202.162) # Top-Left (Lower)

# Map X and Y directions to the frame's physical edges
_xd17 = (_corn17_x - _orig17).normalized()
_yd17 = (_corn17_y - _orig17).normalized()
_zd17 = _xd17.cross(_yd17).normalized()

pl17 = Plane(origin=_orig17, x_dir=_xd17, z_dir=_zd17)
_ext_amt17 = (_ext_tg17 - _orig17).dot(_zd17)
pl17_tgt = Plane(origin=_orig17 + _zd17 * _ext_amt17, x_dir=_xd17, z_dir=_zd17)

# ── Frame 18 planes ───────────────────────────────────────────
_orig18   = Vector(954.701, 1152.651, 235.018) # Top-Left (Upper)
_corn18_x = Vector(1114.448, 1151.597, 176.880) # Top-Right (Upper)
_corn18_y = Vector(955.445, 1322.646, 233.981) # Bottom-Left (Upper)
_ext_tg18 = Vector(941.021, 1152.481, 197.430) # Top-Left (Lower)

# Map X and Y directions to the frame's true edges
_xd18 = (_corn18_x - _orig18).normalized()
_yd18 = (_corn18_y - _orig18).normalized()
_zd18 = _xd18.cross(_yd18).normalized()

pl18 = Plane(origin=_orig18, x_dir=_xd18, z_dir=_zd18)
_ext_amt18 = (_ext_tg18 - _orig18).dot(_zd18)
pl18_tgt = Plane(origin=_orig18 + _zd18 * _ext_amt18, x_dir=_xd18, z_dir=_zd18)

# ── Frame 19 planes ───────────────────────────────────────────
_orig19   = Vector(958.448, 1369.454, 241.383) # Top-Left (Upper)
_corn19_x = Vector(1118.194, 1368.401, 183.245) # Top-Right (Upper)
_corn19_y = Vector(973.333, 1534.503, 279.294) # Bottom-Left (Upper)
_ext_tg19 = Vector(945.222, 1379.034, 204.869) # Top-Left (Lower)

# Map X and Y directions to the frame's true edges
_xd19 = (_corn19_x - _orig19).normalized()
_yd19 = (_corn19_y - _orig19).normalized()
_zd19 = _xd19.cross(_yd19).normalized()

pl19 = Plane(origin=_orig19, x_dir=_xd19, z_dir=_zd19)
_ext_amt19 = (_ext_tg19 - _orig19).dot(_zd19)
pl19_tgt = Plane(origin=_orig19 + _zd19 * _ext_amt19, x_dir=_xd19, z_dir=_zd19)

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
# Frame 5
# ══════════════════════════════════════════════════════════════
print("Building frame 5...")
with BuildPart() as frame5:

    # ── Frame 5: outer 170×185, inner 139×139 ─────────────────
    with BuildSketch(pl5):
        with Locations((85.0, 92.5)): 
            Rectangle(170.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=-40.0)

    # ── Subtract: two slots through the body ──────────────────
    with BuildSketch(pl5_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=40.0, mode=Mode.SUBTRACT)

    # ── Pocket 1 (Explicit Global Coordinates) ────────────────
    pt1_A = Vector(1648.826, 1621.297, 242.882)
    pt1_B = Vector(1698.604, 1621.343, 238.177)
    pt1_C = Vector(1651.065, 1608.291, 266.437)
    tgt1  = Vector(1698.293, 1615.209, 234.819)
    
    pl5_pocket1 = Plane(
        origin=pt1_A, 
        x_dir=(pt1_B - pt1_A).normalized(), 
        z_dir=(pt1_B - pt1_A).cross(pt1_C - pt1_A).normalized()
    )
    
    with BuildSketch(pl5_pocket1):
        # Align.MIN forces the 50x27 rectangle to start exactly at pt1_A
        Rectangle(50.0, 27.0, align=(Align.MIN, Align.MIN))
        
    # Dot product projects the target vector onto the plane normal to get exact signed depth
    amt1 = (tgt1 - pt1_B).dot(pl5_pocket1.z_dir)
    extrude(amount=amt1+1, mode=Mode.SUBTRACT)

    # ── Pocket 2 (Explicit Global Coordinates) ────────────────
    pt2_A = Vector(1642.012, 1487.220, 169.495)
    pt2_B = Vector(1691.791, 1487.265, 164.790)
    pt2_C = Vector(1644.251, 1474.213, 193.049)
    tgt2  = Vector(1642.324, 1493.354, 172.852)

    pl5_pocket2 = Plane(
        origin=pt2_A,
        x_dir=(pt2_B - pt2_A).normalized(),
        z_dir=(pt2_B - pt2_A).cross(pt2_C - pt2_A).normalized()
    )

    with BuildSketch(pl5_pocket2):
        Rectangle(50.0, 27.0, align=(Align.MIN, Align.MIN))
        
    amt2 = (tgt2 - pt2_A).dot(pl5_pocket2.z_dir)
    extrude(amount=amt2-1, mode=Mode.SUBTRACT)

print(f"  Frame 5 valid={frame5.part.is_valid}, volume={frame5.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 6
# ══════════════════════════════════════════════════════════════
print("Building frame 6...")
with BuildPart() as frame6:

    # ── Frame 6: outer 170×170, inner 139×139 ─────────────────
    with BuildSketch(pl6):
        with Locations((85.0, 85.0)): 
            Rectangle(170.0, 170.0)
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt6)

    # ── Subtract: two slots through the body ──────────────────
    with BuildSketch(pl6_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt6, mode=Mode.SUBTRACT)

# ── Pocket 1 (Overshot + Increased Breadth) ─────────────
    pt1_A = Vector(1639.282, 1419.260, 139.950)
    pt1_B = Vector(1689.060, 1419.306, 135.245)
    pt1_C = Vector(1641.746, 1412.410, 165.950)
    tgt1  = Vector(1639.121, 1412.489, 138.181)
    
    pl6_p1_base = Plane(origin=pt1_A, x_dir=(pt1_B-pt1_A).normalized(), 
                        z_dir=(pt1_B-pt1_A).cross(pt1_C-pt1_A).normalized())
    
    pl6_p1_os = Plane(origin=pl6_p1_base.origin - pl6_p1_base.z_dir * 1.0, 
                      x_dir=pl6_p1_base.x_dir, z_dir=pl6_p1_base.z_dir)
    
    with BuildSketch(pl6_p1_os):
        with Locations((25.0, 13.5)): 
            Rectangle(52.0, 29.0)
        
    depth1 = (tgt1 - pt1_A).dot(pl6_p1_base.z_dir)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2 (Overshot + Increased Breadth) ─────────────
    pt2_A = Vector(1635.764, 1271.267, 101.290)
    pt2_B = Vector(1685.542, 1271.313, 96.585)
    pt2_C = Vector(1638.228, 1264.417, 127.291)
    tgt2  = Vector(1635.925, 1278.038, 103.059)

    pl6_p2_base = Plane(origin=pt2_A, x_dir=(pt2_B-pt2_A).normalized(),
                        z_dir=(pt2_B-pt2_A).cross(pt2_C-pt2_A).normalized())

    pl6_p2_os = Plane(origin=pl6_p2_base.origin - pl6_p2_base.z_dir * 1.0,
                      x_dir=pl6_p2_base.x_dir, z_dir=pl6_p2_base.z_dir)

    with BuildSketch(pl6_p2_os):
        with Locations((25.0, 13.5)): 
            Rectangle(52.0, 29.0)
        
    depth2 = (tgt2 - pt2_A).dot(pl6_p2_base.z_dir)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 6 valid={frame6.part.is_valid}, volume={frame6.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 7
# ══════════════════════════════════════════════════════════════
print("Building frame 7...")
with BuildPart() as frame7:

    # ── Main body: outer 170x170, inner 139x139 ───────────────
    with BuildSketch(pl7):
        with Locations((85.0, 85.0)): 
            Rectangle(170.0, 170.0)
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt7)

    # ── Subtract: two slots through the body ──────────────────
    with BuildSketch(pl7_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt7, mode=Mode.SUBTRACT)

   # ── Pocket 1 (Overshot + Increased Breadth) ─────────────
    pt1_A = Vector(1634.680, 1198.127, 89.118)
    pt1_B = Vector(1684.458, 1198.173, 84.412)
    pt1_C = Vector(1637.221, 1197.845, 115.996)
    tgt1  = Vector(1684.458, 1191.173, 84.339)
    
    pl7_p1_base = Plane(origin=pt1_A, x_dir=(pt1_B-pt1_A).normalized(), 
                        z_dir=(pt1_B-pt1_A).cross(pt1_C-pt1_A).normalized())
    
    pl7_p1_os = Plane(origin=pl7_p1_base.origin - pl7_p1_base.z_dir * 1.0, 
                      x_dir=pl7_p1_base.x_dir, z_dir=pl7_p1_base.z_dir)
    
    with BuildSketch(pl7_p1_os):
        with Locations((25.0, 13.5)): 
            Rectangle(52.0, 29.0)
        
    amt1 = (tgt1 - pt1_A).dot(pl7_p1_base.z_dir)
    extrude(amount=amt1 + (2.0 if amt1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2 (Overshot + Increased Breadth) ─────────────
    pt2_A = Vector(1634.669, 1045.135, 87.515)
    pt2_B = Vector(1684.447, 1045.181, 82.810)
    pt2_C = Vector(1637.210, 1044.854, 114.394)
    tgt2  = Vector(1684.447, 1052.181, 82.884)

    pl7_p2_base = Plane(origin=pt2_A, x_dir=(pt2_B-pt2_A).normalized(),
                        z_dir=(pt2_B-pt2_A).cross(pt2_C-pt2_A).normalized())

    pl7_p2_os = Plane(origin=pl7_p2_base.origin - pl7_p2_base.z_dir * 1.0,
                      x_dir=pl7_p2_base.x_dir, z_dir=pl7_p2_base.z_dir)

    with BuildSketch(pl7_p2_os):
        with Locations((25.0, 13.5)): 
            Rectangle(52.0, 29.0)
        
    amt2 = (tgt2 - pt2_A).dot(pl7_p2_base.z_dir)
    extrude(amount=amt2 + (2.0 if amt2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 7 valid={frame7.part.is_valid}, volume={frame7.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 8 (Corrected)
# ══════════════════════════════════════════════════════════════
print("Building frame 8...")
with BuildPart() as frame8:

    # ── Main body: 190x185 outer, 139x139 inner ───────────────
    # Calculated Local Offset for Inner Cutout: (105.2, 84.5)
    with BuildSketch(pl8):
        with Locations((95.0, 92.5)): 
            Rectangle(190.0, 185.0)
        with Locations((105.2, 84.5)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt8)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    # Calculated Local Offsets for Slots: (57.95, 84.5) & (152.03, 84.5)
    with BuildSketch(pl8_tgt):
        with Locations((57.95, 84.5)): 
            Rectangle(44.5, 139.0)
        with Locations((152.03, 84.5)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt8, mode=Mode.SUBTRACT)

    # ── Pocket 1 (Corrected Coordinates & 29mm Breadth) ───────
    # ptA: Start of Edge 1 (27mm breadth edge)
    # ptB: End of Edge 1 / End of Edge 2
    # ptC: Start of Edge 2 (50mm length edge)
    p1_A = Vector(1685.075, 971.268, 88.743)
    p1_B = Vector(1687.540, 977.572, 114.881)
    p1_C = Vector(1637.762, 977.526, 119.586)
    tgt1 = Vector(1685.235, 964.461, 90.369)
    
    # Plane built at the 27mm edge start (ptA)
    pl8_p1_base = Plane(origin=p1_A, x_dir=(p1_B-p1_A).normalized(), 
                        z_dir=(p1_B-p1_A).cross(p1_B-p1_C).normalized())
    
    # 1.0mm depth overshot
    pl8_p1_os = Plane(origin=pl8_p1_base.origin - pl8_p1_base.z_dir * 1.0, 
                      x_dir=pl8_p1_base.x_dir, z_dir=pl8_p1_base.z_dir)
    
    with BuildSketch(pl8_p1_os):
        # 29mm Breadth covers the original 27mm; 52mm Length covers the 50mm
        # Centered at (13.5, 25.0) to lock the original corner ptA
        with Locations((13.5, -25.0)): 
            Rectangle(29.0, 50.0) 
        
    depth1 = (tgt1 - p1_A).dot(pl8_p1_base.z_dir)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2 (Corrected Coordinates & 29mm Breadth) ───────
    p2_A = Vector(1688.572, 822.496, 124.295)
    p2_B = Vector(1691.037, 828.801, 150.432)
    p2_C = Vector(1641.259, 828.755, 155.137)
    tgt2 = Vector(1688.412, 829.303, 122.668)

    pl8_p2_base = Plane(origin=p2_A, x_dir=(p2_B-p2_A).normalized(),
                        z_dir=(p2_B-p2_A).cross(p2_B-p2_C).normalized())

    pl8_p2_os = Plane(origin=pl8_p2_base.origin - pl8_p2_base.z_dir * 1.0,
                      x_dir=pl8_p2_base.x_dir, z_dir=pl8_p2_base.z_dir)

    with BuildSketch(pl8_p2_os):
        with Locations((13.5, -25.0)): 
            Rectangle(29.0, 50.0)
        
    depth2 = (tgt2 - p2_A).dot(pl8_p2_base.z_dir)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 8 valid={frame8.part.is_valid}, volume={frame8.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 9
# ══════════════════════════════════════════════════════════════
print("Building frame 9...")
with BuildPart() as frame9:

    # ── Main body: 185x185 outer, 139x139 inner ───────────────
    with BuildSketch(pl9):
        with Locations((92.5, 92.5)): 
            Rectangle(185.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt9)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl9_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt9, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # Corners of the original 27x50 profile
    p1_c1 = Vector(1486.065, 1746.775, 204.348)
    p1_c2 = Vector(1490.177, 1734.011, 227.783)
    p1_c3 = Vector(1440.937, 1734.086, 236.465)
    
    # Target center from "extrude upto" midpoint
    t1_c1 = Vector(1436.241, 1740.682, 209.773)
    t1_c3 = Vector(1489.594, 1727.843, 224.526)
    
    # Calculate geometric center of the pocket profile
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    # Orient plane using the original edges
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # Place plane at center and shift 1mm back for overshoot[cite: 4]
    pl9_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl9_p1):
        # Center-aligned 29x52 ensures the coordinate center remains static[cite: 4]
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1424.066, 1612.025, 141.837)
    p2_c2 = Vector(1428.178, 1599.261, 165.271)
    p2_c3 = Vector(1477.419, 1599.186, 156.590)
    
    t2_c1 = Vector(1473.890, 1618.118, 136.412)
    t2_c3 = Vector(1428.762, 1605.430, 168.529)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl9_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl9_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 9 valid={frame9.part.is_valid}, volume={frame9.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 10
# ══════════════════════════════════════════════════════════════
print("Building frame 10...")
with BuildPart() as frame10:

    # ── Main body: 170x170 outer, 139x139 inner ───────────────
    with BuildSketch(pl10):
        with Locations((85.0, 85.0)): 
            Rectangle(170.0, 170.0)
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt10)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl10_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt10, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # p1 corners (Upper)[cite: 4]
    p1_c1 = Vector(1468.174, 1543.690, 104.639)
    p1_c2 = Vector(1472.711, 1537.108, 130.429)
    p1_c3 = Vector(1423.470, 1537.183, 139.110)
    
    # t1 corners (Lower)[cite: 4]
    t1_c1 = Vector(1467.867, 1536.902, 102.961)
    t1_c3 = Vector(1423.164, 1530.394, 137.431)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    pl10_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl10_p1):
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1461.476, 1395.308, 67.942)
    p2_c2 = Vector(1466.013, 1388.725, 93.732)
    p2_c3 = Vector(1412.236, 1395.383, 76.624)
    
    t2_c1 = Vector(1461.783, 1402.096, 69.621)
    t2_c3 = Vector(1417.079, 1395.589, 104.092)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl10_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl10_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 10 valid={frame10.part.is_valid}, volume={frame10.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 11
# ══════════════════════════════════════════════════════════════
print("Building frame 11...")
with BuildPart() as frame11:

    # ── Main body: 170x170 outer, 139x139 inner ───────────────
    with BuildSketch(pl11):
        with Locations((85.0, 85.0)): 
            Rectangle(170.0, 170.0)
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt11)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl11_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt11, mode=Mode.SUBTRACT)

    # ── Pocket 1: Center Alignment + Overshot ─────────────────
    # Provided corner coordinates for 27x50 footprint
    p1_c1 = Vector(1459.375, 1322.047, 56.664)
    p1_c2 = Vector(1464.064, 1322.041, 83.254)
    p1_c3 = Vector(1414.823, 1322.117, 91.936)
    
    # Midpoints for centering and target depth
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = Vector(1459.365, 1315.047, 56.665) # Approx center from extrude target
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # 1mm depth overshot (entry)
    pl11_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl11_p1):
        # 29x52 expansion centered on original coordinates
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Center Alignment + Overshot ─────────────────
    p2_c1 = Vector(1459.143, 1169.047, 56.674)
    p2_c2 = Vector(1463.831, 1169.041, 83.264)
    p2_c3 = Vector(1414.590, 1169.117, 91.946)
    
    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = Vector(1459.153, 1176.047, 56.674)

    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl11_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl11_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 11 valid={frame11.part.is_valid}, volume={frame11.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 12
# ══════════════════════════════════════════════════════════════
print("Building frame 12...")
with BuildPart() as frame12:

    # ── Main body: 170x185 outer, 139x139 inner ───────────────
    with BuildSketch(pl12):
        with Locations((85.0, 92.5)): 
            Rectangle(170.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt12)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl12_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt12, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # corners of the original 27x50 profile
    p1_c1 = Vector(1410.960, 1095.274, 71.995)
    p1_c2 = Vector(1415.517, 1101.846, 97.783)
    p1_c3 = Vector(1464.757, 1101.771, 89.101)
    
    # target center midpoint
    t1_c1 = Vector(1411.245, 1088.484, 73.674)
    t1_c3 = Vector(1465.043, 1094.981, 90.781)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # Place plane at center and shift 1mm back for overshoot
    pl12_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl12_p1):
        # 29x52 expansion centered on original coordinates
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1417.206, 946.876, 108.711)
    p2_c2 = Vector(1421.763, 953.449, 134.500)
    p2_c3 = Vector(1471.003, 953.373, 125.818)
    
    t2_c1 = Vector(1416.920, 953.666, 107.032)
    t2_c3 = Vector(1470.717, 960.163, 124.138)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl12_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl12_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 12 valid={frame12.part.is_valid}, volume={frame12.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 13
# ══════════════════════════════════════════════════════════════
print("Building frame 13...")
with BuildPart() as frame13:

    # ── Main body: 170x185 outer, 139x139 inner ───────────────
    with BuildSketch(pl13):
        with Locations((85.0, 92.5)): 
            Rectangle(170.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt13)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl13_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt13, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # p1 corners (Upper Profile)
    p1_c1 = Vector(1194.336, 1118.960,  79.769)
    p1_c2 = Vector(1201.503, 1125.374, 104.998)
    p1_c3 = Vector(1249.560, 1125.925,  91.207)
    
    # Midpoints for centering and target depth
    p1_center = (p1_c1 + p1_c3) * 0.5
    # Target center from extrude-upto profile
    t1_c1 = Vector(1194.867, 1112.161,  81.347)
    t1_c3 = Vector(1250.091, 1119.126,  92.784)
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # 1mm depth overshot for clean subtract
    pl13_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl13_p1):
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1205.936,  970.350, 114.256)
    p2_c2 = Vector(1213.102,  976.764, 139.485)
    p2_c3 = Vector(1261.160,  977.314, 125.693)
    
    t2_c1 = Vector(1205.405,  977.149, 112.678)
    t2_c3 = Vector(1260.629,  984.114, 124.116)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl13_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl13_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 13 valid={frame13.part.is_valid}, volume={frame13.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 14
# ══════════════════════════════════════════════════════════════
print("Building frame 14...")
with BuildPart() as frame14:

    # ── Main body: 170x170 outer, 139x139 inner ───────────────
    with BuildSketch(pl14):
        with Locations((85.0, 85.0)): 
            Rectangle(170.0, 170.0)
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt14)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl14_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt14, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # Corners of the original 27x50 footprint
    p1_c1 = Vector(1190.466, 1345.831,  75.341)
    p1_c2 = Vector(1238.523, 1346.382,  61.550)
    p1_c3 = Vector(1245.973, 1346.214,  87.501)
    
    # Midpoints for centering and target depth
    p1_center = (p1_c1 + p1_c3) * 0.5
    # Calculate target center from provided "extrude upto" data
    t1_c1 = Vector(1190.528, 1338.832,  75.278)
    t1_c3 = Vector(1246.035, 1339.214,  87.438)
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # 1mm depth overshot (entry)
    pl14_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl14_p1):
        # 29x52 expansion centered on original coordinates
        Rectangle(52.0, 29.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1191.823, 1192.844,  73.961)
    p2_c2 = Vector(1239.880, 1193.394,  60.169)
    p2_c3 = Vector(1247.329, 1193.226,  86.121)
    
    t2_c1 = Vector(1191.761, 1199.843,  74.024)
    t2_c3 = Vector(1247.267, 1200.226,  86.184)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl14_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl14_p2):
        Rectangle(52.0, 29.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 14 valid={frame14.part.is_valid}, volume={frame14.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 15
# ══════════════════════════════════════════════════════════════
print("Building frame 15...")
with BuildPart() as frame15:

    # ── Main body: 170x170 outer, 139x139 inner ───────────────
    with BuildSketch(pl15):
        with Locations((85.0, 85.0)): 
            Rectangle(170.0, 170.0)
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt15)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl15_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt15, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # p1 corners (Upper Profile)
    p1_c1 = Vector(1201.945, 1567.164, 124.179)
    p1_c2 = Vector(1209.229, 1560.424, 149.289)
    p1_c3 = Vector(1257.286, 1560.974, 135.498)
    
    # Target center from extrude-upto profile
    t1_c1 = Vector(1201.535, 1560.386, 122.478)
    t1_c3 = Vector(1256.876, 1554.197, 133.797)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # 1mm depth overshot for clean subtract
    pl15_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl15_p1):
        # 29.0mm breadth / 52.0mm length overshot
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1192.978, 1419.017,  87.014)
    p2_c2 = Vector(1200.261, 1412.277, 112.125)
    p2_c3 = Vector(1248.318, 1412.828,  98.333)
    
    t2_c1 = Vector(1193.388, 1425.795,  88.715)
    t2_c3 = Vector(1248.728, 1419.606, 100.034)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl15_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl15_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 15 valid={frame15.part.is_valid}, volume={frame15.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 16
# ══════════════════════════════════════════════════════════════
print("Building frame 16...")
with BuildPart() as frame16:

    # ── Main body: 170x185 outer, 139x139 inner ───────────────
    with BuildSketch(pl16):
        with Locations((85.0, 92.5)): 
            Rectangle(170.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt16)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl16_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt16, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # p1 corners (Upper Profile)
    p1_c1 = Vector(1228.082, 1769.622, 223.339)
    p1_c2 = Vector(1234.761, 1756.716, 246.095)
    p1_c3 = Vector(1282.818, 1757.266, 232.304)
    
    # Midpoints for centering and target depth
    p1_center = (p1_c1 + p1_c3) * 0.5
    # Target center from extrude-upto profile
    t1_c1 = Vector(1227.224, 1763.474, 220.104)
    t1_c3 = Vector(1281.960, 1751.118, 229.069)
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # 1mm depth overshot for clean subtract
    pl16_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl16_p1):
        # 29x52 expansion centered on original coordinates
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1209.330, 1635.243, 152.630)
    p2_c2 = Vector(1216.008, 1622.337, 175.386)
    p2_c3 = Vector(1264.066, 1622.888, 161.595)
    
    t2_c1 = Vector(1210.188, 1641.391, 155.865)
    t2_c3 = Vector(1264.924, 1629.036, 164.830)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl16_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl16_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 16 valid={frame16.part.is_valid}, volume={frame16.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 17
# ══════════════════════════════════════════════════════════════
print("Building frame 17...")
with BuildPart() as frame17:

    # ── Main body: 170x185 outer, 139x139 inner ───────────────
    with BuildSketch(pl17):
        with Locations((85.0, 92.5)): 
            Rectangle(170.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt17)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl17_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt17, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # Corners of the original 27x50 footprint
    p1_c1 = Vector(999.420, 1086.792, 183.639)
    p1_c2 = Vector(1008.405, 1093.481, 208.206)
    p1_c3 = Vector(1055.389, 1093.171, 191.107)
    
    # Target center from extrude-upto profile
    t1_c1 = Vector(999.973, 1080.011, 185.283)
    t1_c3 = Vector(1055.942, 1086.389, 192.751)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # 1mm depth overshot for clean subtract
    pl17_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl17_p1):
        # 29x52 expansion centered on original coordinates
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1011.519, 938.564, 219.569)
    p2_c2 = Vector(1020.504, 945.252, 244.136)
    p2_c3 = Vector(1067.488, 944.943, 227.037)
    
    t2_c1 = Vector(1010.965, 945.346, 217.926)
    t2_c3 = Vector(1066.934, 951.724, 225.393)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl17_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl17_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 17 valid={frame17.part.is_valid}, volume={frame17.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 18: Corrected Pocket Dimensions
# ══════════════════════════════════════════════════════════════
print("Building frame 18 (Corrected)...")
with BuildPart() as frame18:

    # ── Main body: 170x170 outer, 139x139 inner ───────────────
    with BuildSketch(pl18):
        with Locations((85.0, 85.0)): 
            Rectangle(170.0, 170.0)
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt18)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl18_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt18, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # C1 (998.1, 1313.6, 175.9) -> C2 (1007.3, 1313.7, 201.3) = 27mm
    # C2 (1007.3, 1313.7, 201.3) -> C3 (1054.3, 1313.4, 184.2) = 50mm
    p1_c1 = Vector(998.108, 1313.605, 175.926)
    p1_c2 = Vector(1007.343, 1313.719, 201.297)
    p1_c3 = Vector(1054.327, 1313.409, 184.198)
    
    t1_c1 = Vector(998.078, 1306.605, 175.969)
    t1_c3 = Vector(1054.296, 1306.410, 184.241)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized() # Direction of 27mm side
    p1_yd = (p1_c3 - p1_c2).normalized() # Direction of 50mm side
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    pl18_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl18_p1):
        # CORRECTED: Width=29 (X), Length=52 (Y)
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(997.439, 1160.609, 176.859)
    p2_c2 = Vector(1006.673, 1160.723, 202.231)
    p2_c3 = Vector(1053.658, 1160.414, 185.131)
    
    t2_c1 = Vector(997.470, 1167.609, 176.816)
    t2_c3 = Vector(1053.688, 1167.413, 185.089)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized() # Direction of 27mm side
    p2_yd = (p2_c3 - p2_c2).normalized() # Direction of 50mm side
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl18_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl18_p2):
        # CORRECTED: Width=29 (X), Length=52 (Y)
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 18 valid={frame18.part.is_valid}, volume={frame18.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 19
# ══════════════════════════════════════════════════════════════
print("Building frame 19...")
with BuildPart() as frame19:

    # ── Main body: 170x170 outer, 139x139 inner ───────────────
    with BuildSketch(pl19):
        with Locations((85.0, 85.0)): 
            Rectangle(170.0, 170.0)
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt19)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl19_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt19, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # Corners of the original 27x50 footprint
    p1_c1 = Vector(1015.744, 1535.459, 220.365)
    p1_c2 = Vector(1024.672, 1528.992, 245.012)
    p1_c3 = Vector(1071.656, 1528.683, 227.913)
    
    # Target center from extrude-upto profile
    t1_c1 = Vector(1015.131, 1528.663, 218.804)
    t1_c3 = Vector(1071.043, 1521.886, 226.352)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # 1mm depth overshot for clean subtract
    pl19_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl19_p1):
        # 29x52 expansion centered on original coordinates
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1002.347, 1386.915, 186.245)
    p2_c2 = Vector(1011.275, 1380.448, 210.892)
    p2_c3 = Vector(1058.259, 1380.138, 193.793)
    
    t2_c1 = Vector(1002.960, 1393.711, 187.806)
    t2_c3 = Vector(1058.872, 1386.935, 195.354)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl19_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl19_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 19 valid={frame19.part.is_valid}, volume={frame19.part.volume:.1f}")

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
# Loft 10a: upper triangular section (Am2/Bm2 region)
# Source: A4–Am2–A3   Target: B4–Bm2–B3
# ══════════════════════════════════════════════════════════════
print("Building loft10a (upper triangle)...")
loft10a = make_loft_solid(
    [(1758.717, 1628.846, 236.608),   # A4
     (1773.015, 1592.291, 213.891),   # Am2
     (1789.205, 1630.435, 231.642)],  # A3
    [(1762.701, 1622.723, 278.699),   # B4
     (1774.923, 1583.092, 254.321),   # Bm2
     (1789.205, 1624.753, 273.983)]   # B3
)

# ══════════════════════════════════════════════════════════════
# Loft 10b: lower triangular section (Am3/Bm3 region)
# Source: A1–Am3–A2   Target: B1–Bm3–B2
# ══════════════════════════════════════════════════════════════
print("Building loft10b (lower triangle)...")
loft10b = make_loft_solid(
    [(1751.146, 1479.871, 155.066),   # A1
     (1771.122, 1517.592, 173.422),   # Am3
     (1789.205, 1480.613, 151.309)],  # A2
    [(1754.462, 1460.602, 189.962),   # B1
     (1772.863, 1501.802, 210.281),   # Bm3
     (1789.205, 1461.711, 186.561)]   # B2
)

# ══════════════════════════════════════════════════════════════
# Loft 11: 5-Sided Pentagonal Connector
# ══════════════════════════════════════════════════════════════
print("Building loft 11...")

def make_loft11():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Source cap (lower/inner profile, 5 points) ────────────────
    A1  = (1581.900, 1479.716, 171.064)   # top-left
    A2  = (1751.146, 1479.871, 155.066)   # top-right
    A3  = (1749.789, 1446.981, 140.398)   # mid-right
    A4  = (1748.989, 1427.583, 131.746)   # bottom-right
    A5  = (1579.744, 1427.428, 147.744)   # bottom-left
    # Interior midpoint for fanning triangles
    Am  = tuple(sum(x)/5 for x in zip(A1, A2, A3, A4, A5))

    # ── Target cap (upper/outer profile, 5 points) ────────────────
    B1  = (1585.216, 1460.447, 205.960)   # top-left
    B2  = (1754.462, 1460.602, 189.962)   # top-right
    B3  = (1752.700, 1418.871, 170.920)   # mid-right
    B4  = (1752.640, 1417.434, 170.265)   # bottom-right
    B5  = (1583.394, 1417.279, 186.263)   # bottom-left
    Bm  = tuple(sum(x)/5 for x in zip(B1, B2, B3, B4, B5))

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
        tri(A1, B2, B1), tri(A1, A2, B2), # Top
        tri(A2, B3, B2), tri(A2, A3, B3), # Upper-right
        tri(A3, B4, B3), tri(A3, A4, B4), # Lower-right
        tri(A4, B5, B4), tri(A4, A5, B5), # Bottom
        tri(A5, B1, B5), tri(A5, A1, B1), # Left
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft11 OK: vol={s.volume:.1f}")
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
    faces_r += faces[10:]   # side walls unchanged (10 faces)

    s = build_solid(faces_r)

    if s.is_valid and s.volume > 0:
        print(f"  loft11 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft11 failed")

loft11 = make_loft11()

# ══════════════════════════════════════════════════════════════
# Loft 12: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 12 (manual)...")

def make_loft12():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (1578.711, 1206.155, 134.673)
    A2 = (1579.485, 1252.842, 143.307)
    A3 = (1748.730, 1252.997, 127.310)
    A4 = (1747.957, 1206.310, 118.675)
    # Centroid for fanning triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (1574.947, 1206.572, 94.853)
    B2 = (1575.834, 1262.990, 104.789)
    B3 = (1745.080, 1263.145, 88.791)
    B4 = (1744.193, 1206.727, 78.855)
    # Centroid for fanning triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (Quads split into 2 triangles)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Edge 1
        tri(A2, B3, B2), tri(A2, A3, B3), # Edge 2
        tri(A3, B4, B3), tri(A3, A4, B4), # Edge 3
        tri(A4, B1, B4), tri(A4, A1, B1), # Edge 4
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft12 OK: vol={s.volume:.1f}")
        return s

    # Re-build with flipped cap windings if volume is negative
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:]
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft12 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft12 failed")

loft12 = make_loft12()

# ══════════════════════════════════════════════════════════════
# Loft 13: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 13 (manual)...")

def make_loft13():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (1578.699, 1036.164, 132.893)
    A2 = (1747.944, 1036.319, 116.895)
    A3 = (1748.267, 988.927, 119.844)
    A4 = (1579.021, 988.772, 135.842)
    # Centroid for fanning triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (1574.935, 1036.581, 93.073)
    B2 = (1744.180, 1036.736, 77.075)
    B3 = (1744.615, 979.587, 81.121)
    B4 = (1575.369, 979.432, 97.119)
    # Centroid for fanning triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (Quads split into 2 triangles)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Edge 1
        tri(A2, B3, B2), tri(A2, A3, B3), # Edge 2
        tri(A3, B4, B3), tri(A3, A4, B4), # Edge 3
        tri(A4, B1, B4), tri(A4, A1, B1), # Edge 4
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft13 OK: vol={s.volume:.1f}")
        return s

    # Volume negative — flip all cap windings, keep side walls
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:]
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft13 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft13 failed")

loft13 = make_loft13()

# ══════════════════════════════════════════════════════════════
# Loft 14: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 14 (manual)...")

def make_loft14():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (1366.938, 1542.348, 163.984)
    A2 = (1370.361, 1585.716, 183.018)
    A3 = (1537.778, 1585.460, 153.500)
    A4 = (1534.356, 1542.091, 134.466)
    # Centroid for fanning triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (1360.217, 1552.100, 125.778)
    B2 = (1364.268, 1604.626, 148.300)
    B3 = (1531.686, 1604.369, 118.782)
    B4 = (1527.635, 1551.843,  96.260)
    # Centroid for fanning triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (Quads split into 2 triangles)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Edge 1 (CCW side)
        tri(A2, B3, B2), tri(A2, A3, B3), # Edge 2
        tri(A3, B4, B3), tri(A3, A4, B4), # Edge 3
        tri(A4, B1, B4), tri(A4, A1, B1), # Edge 4
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft14 OK: vol={s.volume:.1f}")
        return s

    # Re-build with flipped windings if volume is negative
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] # Keep side walls
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft14 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft14 failed")

loft14 = make_loft14()

# ══════════════════════════════════════════════════════════════
# Loft 15: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 15 (manual)...")

def make_loft15():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (1358.005, 1330.705, 115.156)   # top-left
    A2 = (1359.496, 1377.478, 123.210)   # bottom-left
    A3 = (1526.914, 1377.221,  93.692)   # bottom-right
    A4 = (1525.422, 1330.448,  85.638)   # top-right
    # Centroid for fanning triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (1351.059, 1330.713,  75.763)   # top-left
    B2 = (1352.775, 1387.230,  85.004)   # bottom-left
    B3 = (1520.193, 1386.974,  55.486)   # bottom-right
    B4 = (1518.477, 1330.456,  46.245)   # top-right
    # Centroid for fanning triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW winding)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW winding)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (4 edges, each as a quad split into 2 triangles)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Left wall
        tri(A2, B3, B2), tri(A2, A3, B3), # Bottom wall
        tri(A3, B4, B3), tri(A3, A4, B4), # Right wall
        tri(A4, B1, B4), tri(A4, A1, B1), # Top wall
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft15 OK: vol={s.volume:.1f}")
        return s

    # Volume check / orientation fix logic from charybdis_v4_247_right_3.py
    print("    Volume negative or solid invalid — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:]
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft15 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft15 failed — check vertex alignment")

loft15 = make_loft15()

# ══════════════════════════════════════════════════════════════
# Loft 16: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 16 (manual)...")

def make_loft16():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (1357.746, 1160.705, 115.167)
    A2 = (1525.164, 1160.448,  85.649)
    A3 = (1525.692, 1113.089,  89.060)
    A4 = (1358.275, 1113.346, 118.578)
    # Centroid for fanning triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (1350.801, 1160.713,  75.775)
    B2 = (1518.218, 1160.456,  46.257)
    B3 = (1518.941, 1103.352,  50.855)
    B4 = (1351.524, 1103.609,  80.373)
    # Centroid for fanning triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW winding)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW winding)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (4 edges, each as a quad split into 2 triangles)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Edge 1
        tri(A2, B3, B2), tri(A2, A3, B3), # Edge 2
        tri(A3, B4, B3), tri(A3, A4, B4), # Edge 3
        tri(A4, B1, B4), tri(A4, A1, B1), # Edge 4
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft16 OK: vol={s.volume:.1f}")
        return s

    # Re-build with flipped windings if volume is negative or invalid
    print("    Volume negative or invalid — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] 
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft16 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft16 failed — verify vertex sequential alignment")

loft16 = make_loft16()

# ══════════════════════════════════════════════════════════════
# Loft 17: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 17 (manual)...")

def make_loft17():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (1145.266, 1183.434, 128.880) # Top-Left
    A2 = (1308.660, 1185.307,  81.990) # Top-Right
    A3 = (1310.035, 1137.930,  84.888) # Bottom-Right
    A4 = (1146.640, 1136.058, 131.779) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (1134.230, 1183.683,  90.434) # Top-Left
    B2 = (1297.624, 1185.556,  43.543) # Top-Right
    B3 = (1299.418, 1128.428,  47.512) # Bottom-Right
    B4 = (1136.023, 1126.556,  94.402) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW winding)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW winding)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (4 quads split into 2 triangles each)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Top wall
        tri(A2, B3, B2), tri(A2, A3, B3), # Right wall
        tri(A3, B4, B3), tri(A3, A4, B4), # Bottom wall
        tri(A4, B1, B4), tri(A4, A1, B1), # Left wall
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft17 OK: vol={s.volume:.1f}")
        return s

    # Orientation Check: If volume is negative, reverse cap windings
    print("    Volume negative or invalid — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] # Retain side walls
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft17 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft17 failed — verify coordinate order")

loft17 = make_loft17()

# ══════════════════════════════════════════════════════════════
# Loft 18: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 18 (manual)...")

def make_loft18():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (1143.758, 1353.421, 130.414) # Top-Left
    A2 = (1145.601, 1400.141, 138.699) # Bottom-Left
    A3 = (1308.995, 1402.013,  91.809) # Bottom-Right
    A4 = (1307.153, 1355.293,  83.524) # Top-Right
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (1132.722, 1353.670,  91.967) # Top-Left
    B2 = (1134.811, 1410.126, 101.499) # Bottom-Left
    B3 = (1298.205, 1411.998,  54.609) # Bottom-Right
    B4 = (1296.117, 1355.542,  45.077) # Top-Right
    # Centroid for fanning end-cap triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW winding)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW winding)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (4 quads split into 2 triangles each)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Left wall
        tri(A2, B3, B2), tri(A2, A3, B3), # Bottom wall
        tri(A3, B4, B3), tri(A3, A4, B4), # Right wall
        tri(A4, B1, B4), tri(A4, A1, B1), # Top wall
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft18 OK: vol={s.volume:.1f}")
        return s

    # Orientation Check: If volume is negative or invalid, reverse cap windings
    print("    Volume negative or invalid — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] # Retain side walls
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft18 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft18 failed — check vertex sequential alignment")

loft18 = make_loft18()

# ══════════════════════════════════════════════════════════════
# Loft 19: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 19 (manual)...")

def make_loft19():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (1155.565, 1564.748, 179.993)
    A2 = (1160.513, 1607.997, 198.964)
    A3 = (1323.908, 1609.869, 152.074)
    A4 = (1318.960, 1566.620, 133.103)
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (1144.775, 1574.734, 142.793)
    B2 = (1150.620, 1627.117, 165.251)
    B3 = (1314.014, 1628.989, 118.361)
    B4 = (1308.170, 1576.606,  95.903)
    # Centroid for fanning end-cap triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW winding)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW winding)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (4 quads split into 2 triangles each)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Left wall
        tri(A2, B3, B2), tri(A2, A3, B3), # Bottom wall
        tri(A3, B4, B3), tri(A3, A4, B4), # Right wall
        tri(A4, B1, B4), tri(A4, A1, B1), # Top wall
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft19 OK: vol={s.volume:.1f}")
        return s

    # Reverse cap windings if volume is negative
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:]
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft19 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft19 failed — verify coordinate order")

loft19 = make_loft19()

# ══════════════════════════════════════════════════════════════
# Loft 20: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 20 (manual)...")

def make_loft20():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (954.701, 1152.651, 235.018) # Top-Left
    A2 = (955.678, 1105.308, 238.558) # Bottom-Left
    A3 = (1115.424, 1104.254, 180.420) # Bottom-Right
    A4 = (1114.448, 1151.597, 176.880) # Top-Right
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (941.021, 1152.481, 197.430) # Top-Left
    B2 = (942.366, 1095.399, 202.162) # Bottom-Left
    B3 = (1102.113, 1094.346, 144.025) # Bottom-Right
    B4 = (1100.767, 1151.428, 139.292) # Top-Right
    # Centroid for fanning end-cap triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW winding)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW winding)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (4 quads split into 2 triangles each)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Left wall
        tri(A2, B3, B2), tri(A2, A3, B3), # Bottom wall
        tri(A3, B4, B3), tri(A3, A4, B4), # Right wall
        tri(A4, B1, B4), tri(A4, A1, B1), # Top wall
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft20 OK: vol={s.volume:.1f}")
        return s

    # Volume Orientation Check
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:]
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft20 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft20 failed — check coordinate sequential order")

loft20 = make_loft20()

# ══════════════════════════════════════════════════════════════
# Loft 21: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 21 (manual)...")

def make_loft21():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (955.445, 1322.646, 233.981) # Top-Left
    A2 = (1115.191, 1321.592, 175.843) # Top-Right
    A3 = (1118.194, 1368.401, 183.245) # Bottom-Right
    A4 = (958.448, 1369.454, 241.383) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (941.764, 1322.476, 196.393) # Top-Left
    B2 = (1101.511, 1321.423, 138.256) # Top-Right
    B3 = (1104.968, 1377.981, 146.731) # Bottom-Right
    B4 = (945.222, 1379.034, 204.869) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []

    # Source cap - fan from Am (CCW winding)
    faces += [
        tri(Am, A2, A1), tri(Am, A3, A2), 
        tri(Am, A4, A3), tri(Am, A1, A4)
    ]

    # Target cap - fan from Bm (CW winding)
    faces += [
        tri(Bm, B1, B2), tri(Bm, B2, B3), 
        tri(Bm, B3, B4), tri(Bm, B4, B1)
    ]

    # Side walls (4 quads split into 2 triangles each)
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Top wall
        tri(A2, B3, B2), tri(A2, A3, B3), # Right wall
        tri(A3, B4, B3), tri(A3, A4, B4), # Bottom wall
        tri(A4, B1, B4), tri(A4, A1, B1), # Left wall
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list:
            sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)

    if s.is_valid and s.volume > 0:
        print(f"  loft21 OK: vol={s.volume:.1f}")
        return s

    # Orientation logic for solid validation
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:]
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft21 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft21 failed — check vertex sequential alignment")

loft21 = make_loft21()

# ══════════════════════════════════════════════════════════════
# Fuse
# ══════════════════════════════════════════════════════════════
print("Fusing...")
with BuildPart() as part:
    add(frame.part)
    add(frame3.part)
    add(frame4.part)
    add(frame5.part)
    add(frame6.part)
    add(frame7.part)
    add(frame8.part)
    add(frame9.part)
    add(frame10.part)
    add(frame11.part)
    add(frame12.part)
    add(frame13.part)
    add(frame14.part)
    add(frame15.part)
    add(frame16.part)
    add(frame17.part)
    add(frame18.part)
    add(frame19.part)
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
    add(loft10a)
    add(loft10b)
    add(loft11)
    add(loft12)
    add(loft13)
    add(loft14)
    add(loft15)
    add(loft16)
    add(loft17)
    add(loft18)
    add(loft19)
    add(loft20)
    add(loft21)
print(f"  Final valid={part.part.is_valid}, volume={part.part.volume:.1f}")

try:
    from ocp_vscode import show
    show(part)
except ImportError:
    pass

export_stl(part.part, "output_frame_block.stl")
print("Done.")
