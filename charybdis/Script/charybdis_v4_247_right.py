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

# ── Frame 20 planes ───────────────────────────────────────────
_orig20   = Vector(980.134, 1577.956, 297.194) # Top-Left (Upper)
_corn20_x = Vector(1139.880, 1576.903, 239.056) # Top-Right (Upper)
_corn20_y = Vector(1010.747, 1741.363, 378.348) # Bottom-Left (Upper)
_ext_tg20 = Vector(1127.906, 1595.655, 205.815) # Top-Left (Lower)

# Map X and Y directions to the frame's true edges
_xd20 = (_corn20_x - _orig20).normalized()
_yd20 = (_corn20_y - _orig20).normalized()
_zd20 = _xd20.cross(_yd20).normalized()

pl20 = Plane(origin=_orig20, x_dir=_xd20, z_dir=_zd20)
_ext_amt20 = (_ext_tg20 - _orig20).dot(_zd20)
pl20_tgt = Plane(origin=_orig20 + _zd20 * _ext_amt20, x_dir=_xd20, z_dir=_zd20)

# ── Frame 21 planes (Updated) ─────────────────────────────────
_orig21   = Vector(773.836, 1582.250, 384.030) # Top-Left (Upper)
_corn21_x = Vector(943.905, 1581.423, 311.223) # Top-Right (Upper)
_corn21_y = Vector(808.374, 1746.018, 462.847) # Bottom-Left (Upper)
_ext_tg21 = Vector(722.160, 1601.039, 367.635) # Top-Left (Lower)

# Map X and Y directions to the frame's true edges
_xd21 = (_corn21_x - _orig21).normalized()
_yd21 = (_corn21_y - _orig21).normalized()
_zd21 = _xd21.cross(_yd21).normalized()

pl21 = Plane(origin=_orig21, x_dir=_xd21, z_dir=_zd21)
_ext_amt21 = (_ext_tg21 - _orig21).dot(_zd21)
pl21_tgt = Plane(origin=_orig21 + _zd21 * _ext_amt21, x_dir=_xd21, z_dir=_zd21)

# ── Frame 22 planes ───────────────────────────────────────────
_orig22   = Vector(919.830, 1372.672, 257.358) # Top-Left (Upper)
_corn22_x = Vector(936.262, 1537.890, 293.865) # Top-Right (Upper)
_corn22_y = Vector(749.761, 1373.498, 330.165) # Bottom-Left (Upper)
_ext_tg22 = Vector(716.140, 1383.006, 302.270) # Top-Left (Lower)

# Map X and Y directions to the frame's physical edges[cite: 1]
_xd22 = (_corn22_x - _orig22).normalized()
_yd22 = (_corn22_y - _orig22).normalized()
_zd22 = _xd22.cross(_yd22).normalized()

pl22 = Plane(origin=_orig22, x_dir=_xd22, z_dir=_zd22)
_ext_amt22 = (_ext_tg22 - _orig22).dot(_zd22)
pl22_tgt = Plane(origin=_orig22 + _zd22 * _ext_amt22, x_dir=_xd22, z_dir=_zd22)

## ── Frame 23 planes (Corrected) ───────────────────────────────
_orig23   = Vector(746.367, 1156.665, 324.699) # TL (Upper)[cite: 1]
_corn23_x = Vector(916.436, 1155.838, 251.892) # TR (Upper)[cite: 1]
_corn23_y = Vector(746.504, 1326.657, 323.088) # BL (Upper)[cite: 1]
_ext_tg23 = Vector(716.140, 1156.399, 294.130) # TL (Lower)[cite: 1]

# Map X and Y directions to the frame's true edges[cite: 1]
_xd23 = (_corn23_x - _orig23).normalized()
_yd23 = (_corn23_y - _orig23).normalized()
_zd23 = _xd23.cross(_yd23).normalized()

pl23 = Plane(origin=_orig23, x_dir=_xd23, z_dir=_zd23)
_ext_amt23 = (_ext_tg23 - _orig23).dot(_zd23)
pl23_tgt = Plane(origin=_orig23 + _zd23 * _ext_amt23, x_dir=_xd23, z_dir=_zd23)

# ── Frame 24 planes ───────────────────────────────────────────
_orig24   = Vector(747.691, 1109.337, 328.329) # Top-Left (Upper)
_corn24_x = Vector(917.760, 1108.510, 255.522) # Top-Right (Upper)
_corn24_y = Vector(765.285,  930.297, 371.460) # Bottom-Left (Upper)
_ext_tg24 = Vector(716.140, 1099.346, 299.726) # Top-Left (Lower)

# Map X and Y directions to the frame's true edges
_xd24 = (_corn24_x - _orig24).normalized()
_yd24 = (_corn24_y - _orig24).normalized()
_zd24 = _xd24.cross(_yd24).normalized()

pl24 = Plane(origin=_orig24, x_dir=_xd24, z_dir=_zd24)
_ext_amt24 = (_ext_tg24 - _orig24).dot(_zd24)
pl24_tgt = Plane(origin=_orig24 + _zd24 * _ext_amt24, x_dir=_xd24, z_dir=_zd24)

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
# Frame 20
# ══════════════════════════════════════════════════════════════
print("Building frame 20...")
with BuildPart() as frame20:

    # ── Main body: 170x185 outer, 139x139 inner ───────────────
    with BuildSketch(pl20):
        with Locations((85.0, 92.5)): 
            Rectangle(170.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt20)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl20_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt20, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # p1 corners (Upper Profile)
    p1_c1 = Vector(1051.265, 1738.986, 314.279)
    p1_c2 = Vector(1059.348, 1726.329, 336.717)
    p1_c3 = Vector(1106.332, 1726.019, 319.617)
    
    # Target center from extrude-upto profile
    t1_c1 = Vector(1050.107, 1732.803, 311.208)
    t1_c3 = Vector(1105.174, 1719.836, 316.547)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # 1mm depth overshot for clean subtract
    pl20_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl20_p1):
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(1025.948, 1603.845, 247.162)
    p2_c2 = Vector(1034.030, 1591.187, 269.600)
    p2_c3 = Vector(1081.014, 1590.877, 252.500)
    
    t2_c1 = Vector(1027.106, 1610.027, 250.233)
    t2_c3 = Vector(1082.173, 1597.060, 255.571)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl20_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl20_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 20 valid={frame20.part.is_valid}, volume={frame20.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 21
# ══════════════════════════════════════════════════════════════
print("Building frame 21...")
with BuildPart() as frame21:

    # ── Main body: 185x185 outer, 139x139 inner ───────────────
    with BuildSketch(pl21):
        with Locations((92.5, 92.5)): 
            Rectangle(185.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt21)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl21_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt21, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    p1_c1 = Vector(859.074, 1743.485, 390.734)
    p1_c2 = Vector(868.429, 1730.927, 412.729)
    p1_c3 = Vector(914.394, 1730.703, 393.051)
    
    t1_c1 = Vector(857.767, 1737.288, 387.752)
    t1_c3 = Vector(913.087, 1724.507, 390.069)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    pl21_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl21_p1):
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    # Updated coordinates based on your new profile
    p2_c1 = Vector(830.510, 1608.044, 325.550)
    p2_c2 = Vector(839.865, 1595.486, 347.545)
    p2_c3 = Vector(885.830, 1595.262, 327.867)
    
    t2_c1 = Vector(831.817, 1614.241, 328.532)
    t2_c3 = Vector(887.137, 1601.459, 330.850)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl21_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl21_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    # 10.0mm depth overshot maintained from original Frame 21 script
    extrude(amount=depth2 + (10.0 if depth2 > 0 else -10.0), mode=Mode.SUBTRACT)

print(f"  Frame 21 valid={frame21.part.is_valid}, volume={frame21.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 22
# ══════════════════════════════════════════════════════════════
print("Building frame 22...")
with BuildPart() as frame22:

    # ── Main body: 185x170 outer, 139x139 inner ───────────────[cite: 1]
    with BuildSketch(pl22):
        with Locations((92.5, 85.0)): 
            Rectangle(185.0, 170.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt22)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────[cite: 1]
    with BuildSketch(pl22_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt22, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────[cite: 1]
    # Corners of the original 27x50 footprint
    p1_c1 = Vector(865.022, 1539.316, 279.898)
    p1_c2 = Vector(875.323, 1532.958, 304.033)
    p1_c3 = Vector(829.359, 1533.182, 323.710)
    
    # Target center from extrude-upto profile
    t1_c1 = Vector(864.346, 1532.513, 278.395)
    t1_c3 = Vector(828.682, 1526.379, 322.207)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # Place plane at center and shift 1mm back for overshoot[cite: 1]
    pl22_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl22_p1):
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────[cite: 1]
    p2_c1 = Vector(804.269, 1390.842, 266.719)
    p2_c2 = Vector(814.570, 1384.485, 290.854)
    p2_c3 = Vector(860.535, 1384.262, 271.176)
    
    t2_c1 = Vector(850.910, 1397.422, 248.545)
    t2_c3 = Vector(804.945, 1397.646, 268.222)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl22_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl22_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 22 valid={frame22.part.is_valid}, volume={frame22.part.volume:.1f}")

# ══════════════════════════════════════════════════════════════
# Frame 23 (Corrected Subtractions)
# ══════════════════════════════════════════════════════════════
print("Building frame 23 (Corrected)...")
with BuildPart() as frame23:

    # ── Main body: 185x170 outer, 139x139 inner ───────────────
    # Calculated Local Offset for Inner Cutout: (99.25, 84.55)[cite: 1]
    with BuildSketch(pl23):
        with Locations((92.5, 85.0)): 
            Rectangle(185.0, 170.0)
        with Locations((99.25, 84.55)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt23)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    # Calculated Local Offsets for Slots: (51.75, 84.55) & (146.75, 84.55)[cite: 1]
    with BuildSketch(pl23_tgt):
        with Locations((51.75, 84.55)): 
            Rectangle(44.5, 139.0)
        with Locations((146.75, 84.55)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt23, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    p1_c1 = Vector(845.665, 1317.263, 237.204)
    p1_c2 = Vector(856.292, 1317.489, 262.024)
    p1_c3 = Vector(810.327, 1317.713, 281.701)
    t1_center = (Vector(845.660, 1310.263, 237.270) + Vector(810.322, 1310.713, 281.768)) * 0.5
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    pl23_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    with BuildSketch(pl23_p1):
        Rectangle(29.0, 52.0)
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    p2_c1 = Vector(799.578, 1164.493, 258.332)
    p2_c2 = Vector(810.205, 1164.720, 283.152)
    p2_c3 = Vector(856.169, 1164.496, 263.474)
    t2_center = (Vector(845.548, 1171.269, 238.588) + Vector(799.584, 1171.493, 258.266)) * 0.5

    p2_center = (p2_c1 + p2_c3) * 0.5
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl23_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)
    with BuildSketch(pl23_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 23 valid={frame23.part.is_valid}, volume={frame23.part.volume:.1f}")


# ══════════════════════════════════════════════════════════════
# Frame 24
# ══════════════════════════════════════════════════════════════
print("Building frame 24...")
with BuildPart() as frame24:

    # ── Main body: 185x185 outer, 139x139 inner ───────────────
    with BuildSketch(pl24):
        with Locations((92.5, 92.5)): 
            Rectangle(185.0, 185.0)
        with Locations((85.0, 85.0)): 
            Rectangle(139.0, 139.0, mode=Mode.SUBTRACT)
    extrude(amount=_ext_amt24)

    # ── Subtract: two slots (44.5mm wide) ─────────────────────
    with BuildSketch(pl24_tgt):
        with Locations((37.75, 85.0)): 
            Rectangle(44.5, 139.0)
        with Locations((132.25, 85.0)): 
            Rectangle(44.5, 139.0)
    extrude(amount=-_ext_amt24, mode=Mode.SUBTRACT)

    # ── Pocket 1: Geometric Center Alignment ──────────────────
    # Corners of the 27x50 footprint (Upper)
    p1_c1 = Vector(802.170, 1090.706, 265.224)
    p1_c2 = Vector(812.482, 1097.503, 289.234)
    p1_c3 = Vector(858.446, 1097.280, 269.556)
    
    # Target center from extrude-upto profile (Lower)
    t1_c1 = Vector(802.836, 1083.931, 266.856)
    t1_c3 = Vector(859.112, 1090.505, 271.188)
    
    p1_center = (p1_c1 + p1_c3) * 0.5
    t1_center = (t1_c1 + t1_c3) * 0.5
    
    p1_xd = (p1_c2 - p1_c1).normalized()
    p1_yd = (p1_c3 - p1_c2).normalized()
    p1_zd = p1_xd.cross(p1_yd).normalized()
    
    # 1mm depth overshot for clean boolean subtraction
    pl24_p1 = Plane(origin=p1_center - p1_zd * 1.0, x_dir=p1_xd, z_dir=p1_zd)
    
    with BuildSketch(pl24_p1):
        # 29x52 expansion centered on original coordinates to ensure edge-break
        Rectangle(29.0, 52.0) 
        
    depth1 = (t1_center - p1_center).dot(p1_zd)
    extrude(amount=depth1 + (2.0 if depth1 > 0 else -2.0), mode=Mode.SUBTRACT)

    # ── Pocket 2: Geometric Center Alignment ──────────────────
    # Corners of the 27x50 footprint (Upper)
    p2_c1 = Vector(816.721, 942.635, 300.894)
    p2_c2 = Vector(827.033, 949.432, 324.904)
    p2_c3 = Vector(872.997, 949.209, 305.227)
    
    # Target center from extrude-upto profile (Lower)
    t2_c1 = Vector(816.055, 949.410, 299.262)
    t2_c3 = Vector(872.331, 955.983, 303.595)

    p2_center = (p2_c1 + p2_c3) * 0.5
    t2_center = (t2_c1 + t2_c3) * 0.5
    
    p2_xd = (p2_c2 - p2_c1).normalized()
    p2_yd = (p2_c3 - p2_c2).normalized()
    p2_zd = p2_xd.cross(p2_yd).normalized()

    pl24_p2 = Plane(origin=p2_center - p2_zd * 1.0, x_dir=p2_xd, z_dir=p2_zd)

    with BuildSketch(pl24_p2):
        Rectangle(29.0, 52.0)
        
    depth2 = (t2_center - p2_center).dot(p2_zd)
    extrude(amount=depth2 + (2.0 if depth2 > 0 else -2.0), mode=Mode.SUBTRACT)

print(f"  Frame 24 valid={frame24.part.is_valid}, volume={frame24.part.volume:.1f}")

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
# Loft 22: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 22 (manual)...")

def make_loft22():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (1133.080, 1533.450, 221.156)
    A2 = (1139.880, 1576.903, 239.056)
    A3 = (980.134, 1577.956, 297.194)
    A4 = (973.333, 1534.503, 279.294)
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (1119.854, 1543.030, 184.642)
    B2 = (1127.906, 1595.655, 205.815)
    B3 = (968.160, 1596.708, 263.952)
    B4 = (960.107, 1544.083, 242.780)
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
        tri(A1, B2, B1), tri(A1, A2, B2), # Edge 1 wall
        tri(A2, B3, B2), tri(A2, A3, B3), # Edge 2 wall
        tri(A3, B4, B3), tri(A3, A4, B4), # Edge 3 wall
        tri(A4, B1, B4), tri(A4, A1, B1), # Edge 4 wall
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
        print(f"  loft22 OK: vol={s.volume:.1f}")
        return s

    # Orientation logic
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:]
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft22 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft22 failed — verify coordinate sequential order")

loft22 = make_loft22()

# ══════════════════════════════════════════════════════════════
# Loft 23: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 23 (manual)...")

def make_loft23():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (943.905, 1581.423, 311.223) # Top-Left
    A2 = (980.134, 1577.956, 297.194) # Top-Right
    A3 = (1010.747, 1741.363, 378.348) # Bottom-Right
    A4 = (978.443, 1745.191, 390.040) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (930.046, 1600.028, 278.638) # Top-Left
    B2 = (968.160, 1596.708, 263.952) # Top-Right
    B3 = (996.291, 1746.866, 338.527) # Bottom-Right
    B4 = (961.783, 1750.518, 351.065) # Bottom-Left
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
        print(f"  loft23 OK: vol={s.volume:.1f}")
        return s

    # Orientation logic
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:]
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft23 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft23 failed — check vertex sequential alignment")

loft23 = make_loft23()

# ══════════════════════════════════════════════════════════════
# Loft 24: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 24 (manual)...")

def make_loft24():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (936.262, 1537.890, 293.865) # Top-Left
    A2 = (973.333, 1534.503, 279.294) # Top-Right
    A3 = (980.134, 1577.956, 297.194) # Bottom-Right
    A4 = (943.905, 1581.423, 311.223) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (921.001, 1547.309, 258.110) # Top-Left
    B2 = (960.107, 1544.083, 242.780) # Top-Right
    B3 = (968.160, 1596.708, 263.952) # Bottom-Right
    B4 = (930.046, 1600.028, 278.638) # Bottom-Left
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
        print(f"  loft24 OK: vol={s.volume:.1f}")
        return s

    # Orientation Flip Logic
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] # Keep original side wall winding
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft24 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft24 failed — check vertex sequential order")

loft24 = make_loft24()

# ══════════════════════════════════════════════════════════════
# Loft 25: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 25 (manual)...")

def make_loft25():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (919.830, 1372.672, 257.358) # Top-Left
    A2 = (958.448, 1369.454, 241.383) # Top-Right
    A3 = (973.333, 1534.503, 279.294) # Bottom-Right
    A4 = (936.262, 1537.890, 293.865) # Bottom-Left
    # Centroid for end-cap fanning
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (904.569, 1382.090, 221.603) # Top-Left
    B2 = (945.222, 1379.034, 204.869) # Top-Right
    B3 = (960.107, 1544.083, 242.780) # Bottom-Right
    B4 = (921.001, 1547.309, 258.110) # Bottom-Left
    # Centroid for end-cap fanning
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
        print(f"  loft25 OK: vol={s.volume:.1f}")
        return s

    # Orientation Flip Logic
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] 
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft25 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft25 failed — check coordinate sequential order")

loft25 = make_loft25()

# ══════════════════════════════════════════════════════════════
# Loft 26: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 26 (manual)...")

def make_loft26():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (916.573, 1325.830, 250.280) # Top-Left
    A2 = (955.445, 1322.646, 233.981) # Top-Right
    A3 = (958.448, 1369.454, 241.383) # Bottom-Right
    A4 = (919.830, 1372.672, 257.358) # Bottom-Left
    # Centroid for end-cap fanning
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (900.830, 1325.494, 213.510) # Top-Left
    B2 = (941.764, 1322.476, 196.393) # Top-Right
    B3 = (945.222, 1379.034, 204.869) # Bottom-Right
    B4 = (904.569, 1382.090, 221.603) # Bottom-Left
    # Centroid for end-cap fanning
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
        print(f"  loft26 OK: vol={s.volume:.1f}")
        return s

    # Orientation Flip Logic
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] 
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft26 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft26 failed — check coordinate sequential order")

loft26 = make_loft26()

# ══════════════════════════════════════════════════════════════
# Loft 27: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 27 (manual)...")

def make_loft27():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (916.436, 1155.838, 251.892) # Top-Left
    A2 = (954.701, 1152.651, 235.018) # Top-Right
    A3 = (955.445, 1322.646, 233.981) # Bottom-Right
    A4 = (916.573, 1325.830, 250.280) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (900.693, 1155.502, 215.122) # Top-Left
    B2 = (941.021, 1152.481, 197.430) # Top-Right
    B3 = (941.764, 1322.476, 196.393) # Bottom-Right
    B4 = (900.830, 1325.494, 213.510) # Bottom-Left
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
        print(f"  loft27 OK: vol={s.volume:.1f}")
        return s

    # Orientation Flip Logic
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] 
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft27 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft27 failed — check vertex sequential alignment")

loft27 = make_loft27()

# ══════════════════════════════════════════════════════════════
# Loft 28: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 28 (manual)...")

def make_loft28():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (916.436, 1155.838, 251.892) # Top-Left
    A2 = (954.701, 1152.651, 235.018) # Top-Right
    A3 = (955.678, 1105.308, 238.558) # Bottom-Right
    A4 = (917.760, 1108.510, 255.522) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (900.693, 1155.502, 215.122) # Top-Left
    B2 = (941.021, 1152.481, 197.430) # Top-Right
    B3 = (942.366, 1095.399, 202.162) # Bottom-Right
    B4 = (902.484, 1098.440, 219.952) # Bottom-Left
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
        print(f"  loft28 OK: vol={s.volume:.1f}")
        return s

    # Orientation Flip Logic
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] 
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft28 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft28 failed — verify vertex sequential alignment")

loft28 = make_loft28()

# ══════════════════════════════════════════════════════════════
# Loft 29: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 29 (manual)...")

def make_loft29():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (917.760, 1108.510, 255.522) # Top-Left
    A2 = (955.678, 1105.308, 238.558) # Top-Right
    A3 = (970.307,  926.077, 282.003) # Bottom-Right
    A4 = (935.354,  929.471, 298.652) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (902.484, 1098.440, 219.952) # Top-Left
    B2 = (942.366, 1095.399, 202.162) # Top-Right
    B3 = (958.182,  901.637, 249.130) # Bottom-Right
    B4 = (921.504,  904.884, 266.579) # Bottom-Left
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
        print(f"  loft29 OK: vol={s.volume:.1f}")
        return s

    # Orientation Flip Logic
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] 
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft29 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft29 failed — verify vertex sequential alignment")

loft29 = make_loft29()

# ══════════════════════════════════════════════════════════════
# Loft 30: Manual Quadrilateral Stitching (Fallback for non-planar twist)
# ══════════════════════════════════════════════════════════════
print("Building loft 30 (manual)...")

def make_loft30():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (773.836, 1582.250, 384.030) # Top-Left
    A2 = (943.905, 1581.423, 311.223) # Top-Right
    A3 = (936.262, 1537.890, 293.865) # Bottom-Right
    A4 = (766.193, 1538.717, 366.672) # Bottom-Left
    # Centroid for fanning end-cap triangles
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (722.160, 1601.039, 367.635) # Top-Left
    B2 = (930.046, 1600.028, 278.638) # Top-Right
    B3 = (921.001, 1547.309, 258.110) # Bottom-Right
    B4 = (718.504, 1548.293, 344.800) # Bottom-Left
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
        tri(A4, B1, B4), tri(A4, A1, B1), # Left wall (Edge 4)
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
        print(f"  loft30 OK: vol={s.volume:.1f}")
        return s

    # Orientation Flip Logic
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] 
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft30 OK: vol={s.volume:.1f}")
        return s

    raise ValueError("make_loft30 failed — verify vertex sequential alignment")

loft30 = make_loft30()

# ══════════════════════════════════════════════════════════════
# Loft 31: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 31 (manual)...")

def make_loft31():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper Profile) ─────────────────────────────
    # These match the "Source" coordinates you provided
    A1 = (749.761, 1373.498, 330.165) # Top-Left
    A2 = (919.830, 1372.672, 257.358) # Top-Right
    A3 = (916.573, 1325.830, 250.280) # Bottom-Right
    A4 = (746.504, 1326.657, 323.088) # Bottom-Left
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower Profile) ─────────────────────────────
    # These are mapped to the "extrude upto" target to create 3D volume
    B1 = (716.140, 1383.006, 302.270) 
    B2 = (904.569, 1382.090, 221.603)
    B3 = (900.830, 1325.494, 213.510)
    B4 = (712.401, 1326.410, 294.177)
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
    # This follows the exact P1->Q2->Q1 and P1->P2->Q2 pattern in your script
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
        print(f"  loft31 OK: vol={s.volume:.1f}")
        return s

    # Orientation Flip Logic (Silent fallback from your original script)
    print("    Volume negative — reversing cap windings...")
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:] 
    
    s = build_solid(faces_r)
    if s.is_valid and s.volume > 0:
        print(f"  loft31 OK: vol={s.volume:.1f}")
        return s

    return s

loft31 = make_loft31()

# ══════════════════════════════════════════════════════════════
# Loft 32: Manual Quadrilateral Stitching
# ══════════════════════════════════════════════════════════════
print("Building loft 32 (manual)...")

def make_loft32():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # ── Profile A (Upper) ─────────────────────────────────────
    A1 = (746.367, 1156.665, 324.699) # TL
    A2 = (916.436, 1155.838, 251.892) # TR
    A3 = (917.760, 1108.510, 255.522) # BR
    A4 = (747.691, 1109.337, 328.329) # BL
    Am = tuple(sum(x)/4 for x in zip(A1, A2, A3, A4))

    # ── Profile B (Lower) ─────────────────────────────────────
    B1 = (716.140, 1156.399, 294.130) # TL
    B2 = (900.693, 1155.502, 215.122) # TR
    B3 = (902.484, 1098.440, 219.952) # BR
    B4 = (716.140, 1099.346, 299.726) # BL
    Bm = tuple(sum(x)/4 for x in zip(B1, B2, B3, B4))

    faces = []
    # Source cap (CCW)
    faces += [tri(Am, A2, A1), tri(Am, A3, A2), tri(Am, A4, A3), tri(Am, A1, A4)]
    # Target cap (CW)
    faces += [tri(Bm, B1, B2), tri(Bm, B2, B3), tri(Bm, B3, B4), tri(Bm, B4, B1)]
    # Side walls
    faces += [
        tri(A1, B2, B1), tri(A1, A2, B2), # Top
        tri(A2, B3, B2), tri(A2, A3, B3), # Right
        tri(A3, B4, B3), tri(A3, A4, B4), # Bottom
        tri(A4, B1, B4), tri(A4, A1, B1), # Left
    ]

    def build_solid(face_list):
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in face_list: sew.Add(f)
        sew.Perform()
        shell = TopoDS.Shell_s(sew.SewedShape())
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        return Solid(fix.Solid())

    s = build_solid(faces)
    if s.is_valid and s.volume > 0:
        print(f"  loft32 OK: vol={s.volume:.1f}")
        return s

    # Orientation Flip fallback
    faces_r = [tri(Am, A1, A2), tri(Am, A2, A3), tri(Am, A3, A4), tri(Am, A4, A1)]
    faces_r += [tri(Bm, B2, B1), tri(Bm, B3, B2), tri(Bm, B4, B3), tri(Bm, B1, B4)]
    faces_r += faces[8:]
    s = build_solid(faces_r)
    return s

loft32 = make_loft32()

# ══════════════════════════════════════════════════════════════
# Loft 33: Frame-5 outer face (quad) → heptagonal profile
# ══════════════════════════════════════════════════════════════
#
# Profile A — Quadrilateral (inner side, frame-5 face)
#   Vertex correspondence:  A1=D  A2=A  A3=B  A4=C
#   Edge lengths:  D-A=185mm, A-B=140.8mm, B-C=185mm, C-D=140.4mm
#
# Profile B — Heptagon (outer side, loft target)
#   Vertex correspondence:  B1=E  B2=H  B3=I  B4=J  B5=K  B6=G  B7=F
#   Edge lengths (going around): E-H=170mm, H-I=43mm, I-J=72mm,
#                                J-K=34mm, K-G=69mm, G-F=170mm, F-E=141mm
#
# Side-wall mapping (how the 4-vertex A is stretched to meet 7-vertex B):
#   A1(D)  ↔ B1(E)   — bottom-left corners, spatially closest pair
#   A2(A)  ↔ B2(H)   — upper-left, A:Y=1748 / H:Y=1754
#   A2→A3 (25%) ↔ B3(I)   — I extends above A in Y (1792 vs 1748)
#   A2→A3 (50%) ↔ B4(J)
#   A2→A3 (75%) ↔ B5(K)
#   A3(B)  ↔ B6(G)   — lower-right, B:Y=1622 / G:Y=1628
#   A4(C)  ↔ B7(F)   — bottom-right, C==_orig5 / F≈_orig5
#
# Construction strategy:
#   1. Try BRepOffsetAPI_ThruSections (handles different edge counts natively,
#      tries both wire orderings and ruled/smooth variants).
#   2. Fallback: Manual triangle sewing — expand Profile A from 4→7 vertices
#      via linear interpolation along the A2–A3 edge, then stitch 7 aligned
#      quad strips plus centroid-fan caps on both ends.

print("Building loft 33 (quad→heptagon, Frame-5 outer bridge)...")

def make_loft33():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    # ── Profile A: Quadrilateral ──────────────────────────────
    A1 = (1552.55,  1585.437, 150.895)   # D  bottom-left
    A2 = (1567.978, 1748.461, 236.978)   # A  top-left
    A3 = (1593.455, 1622.567, 294.696)   # B  top-right
    A4 = (1585.216, 1460.447, 205.96)    # C  bottom-right  (== _orig5)

    # ── Profile B: Heptagon ───────────────────────────────────
    B1 = (1546.458, 1604.346, 116.177)   # E  bottom-left
    B2 = (1560.635, 1754.152, 195.281)   # H  upper-left
    B3 = (1564.227, 1792.115, 215.327)   # I  topmost
    B4 = (1577.633, 1727.326, 243.38)    # J  upper-right
    B5 = (1584.527, 1697.468, 258.591)   # K  right
    B6 = (1589.471, 1628.691, 252.606)   # G  lower-right
    B7 = (1581.9,   1479.716, 171.064)   # F  bottom-right

    pts_a = [A1, A2, A3, A4]
    pts_b = [B1, B2, B3, B4, B5, B6, B7]

    wa = Wire.make_polygon([Vector(*p) for p in pts_a], close=True)
    wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)

    # ── Attempt 1: BRepOffsetAPI_ThruSections ─────────────────
    # OCC's ThruSections handles wires with different numbers of edges.
    # We try both wire orderings and ruled/smooth to maximize success rate.
    for ruled in [False, True]:
        for w1, w2, label in [(wa, wb, "A→B"), (wb, wa, "B→A")]:
            try:
                gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                gen.AddWire(w1.wrapped)
                gen.AddWire(w2.wrapped)
                gen.Build()
                s = Solid(gen.Shape())
                if s.is_valid and s.volume > 0:
                    print(f"  loft33 ThruSections OK: {label} ruled={ruled} vol={s.volume:.1f}")
                    return s
                print(f"  loft33 ThruSections {label} ruled={ruled}: valid={s.is_valid} vol={s.volume:.1f}")
            except Exception as e:
                print(f"  loft33 ThruSections {label} ruled={ruled} raised: {e}")

    # ── Attempt 2: Manual triangle sewing ─────────────────────
    # Expand Profile A (4 verts) → 7 verts by inserting three lerp points
    # on the A2–A3 edge.  The resulting PA[i] ↔ PB[i] alignment is:
    #   PA[0]=A1(D),  PA[1]=A2(A),  PA[2]=A2–A3(25%),
    #   PA[3]=A2–A3(50%), PA[4]=A2–A3(75%), PA[5]=A3(B), PA[6]=A4(C)
    def lerp(p, q, t):
        return tuple(p[i] + t * (q[i] - p[i]) for i in range(3))

    PA = [
        A1,
        A2,
        lerp(A2, A3, 0.25),
        lerp(A2, A3, 0.50),
        lerp(A2, A3, 0.75),
        A3,
        A4,
    ]
    PB = [B1, B2, B3, B4, B5, B6, B7]

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    Am = tuple(sum(v[i] for v in pts_a) / 4 for i in range(3))
    Bm = tuple(sum(v[i] for v in pts_b) / 7 for i in range(3))

    def build_solid(cap_a_reversed=False):
        faces = []
        cap_a = list(reversed(pts_a)) if cap_a_reversed else pts_a
        na = len(cap_a)

        # Cap A: centroid-fan, winding chosen by cap_a_reversed
        if cap_a_reversed:
            # CW cap (normal points away from B)
            for i in range(na):
                faces.append(tri(Am, cap_a[i], cap_a[(i + 1) % na]))
        else:
            # CCW cap (normal points away from B)
            for i in range(na):
                faces.append(tri(Am, cap_a[(i + 1) % na], cap_a[i]))

        # Cap B: centroid-fan, opposite winding to Cap A
        if cap_a_reversed:
            for i in range(7):
                faces.append(tri(Bm, PB[(i + 1) % 7], PB[i]))
        else:
            for i in range(7):
                faces.append(tri(Bm, PB[i], PB[(i + 1) % 7]))

        # Side walls: 7 quad strips (PA[i]–PA[i+1] ↔ PB[i]–PB[i+1])
        for i in range(7):
            j = (i + 1) % 7
            faces.append(tri(PA[i], PB[j], PB[i]))
            faces.append(tri(PA[i], PA[j], PB[j]))

        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces:
            sew.Add(f)
        sew.Perform()

        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except Exception as e:
            print(f"    Shell cast failed (cap_a_rev={cap_a_reversed}): {e}")
            return None

        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    cap_a_rev={cap_a_reversed}: valid={s.is_valid}, vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for rev in [False, True]:
        result = build_solid(cap_a_reversed=rev)
        if result is not None:
            print(f"  loft33 manual OK: vol={result.volume:.1f}")
            return result

    raise ValueError("loft33: all construction strategies failed — check vertex ordering")

loft33 = make_loft33()

# ══════════════════════════════════════════════════════════════
# Loft 34: Sliver face (8-gon) → outer hexagon
# ══════════════════════════════════════════════════════════════
#
# Context / shared vertices with loft33
# ──────────────────────────────────────
#   Profile A vertex PA  = loft33 Profile A vertex A2 (top-left corner)
#   Profile A vertex PI  = loft33 Profile A vertex A3 (bottom-right corner)
#   Profile B vertex Q1  = loft33 Profile B vertex B3 / I
#   Profile B vertex Q5  = loft33 Profile B vertex B6 / G
#   Profile B vertex Q6  = loft33 Profile B vertex B4 / J
#
# Profile A is a sliver whose closing edge PI→PA (140.8 mm) is the SAME
# edge as loft33 Profile A's A3–A2 side wall — loft34 is the adjacent face.
#
# Profile A — 8-gon (PG and PH were only 1.1 mm apart in the source data
#   and have been merged into their midpoint PGH to avoid near-degenerate
#   triangles.  The omitted original vertices were:
#     PG = (1595.205, 1612.400, 313.109)
#     PH = (1595.113, 1612.933, 312.144)
#   merged → PGH ≈ (1595.159, 1612.667, 312.627))
#
#   Going around: PA→PB→PC→PD→PE→PF→PGH→PI→(PA)
#   Perimeter: ~378.7 mm
#   Both the long arc (PA→…→PI, ~238 mm) and the short closing edge
#   (PI→PA, 140.8 mm) decrease/increase Y monotonically, matching
#   Profile B's topology.
#
# Profile B — hexagon
#   Going around: Q1→Q2→Q3→Q4→Q5→Q6→(Q1)
#   Perimeter: ~475.8 mm
#
# Alignment
# ─────────
#   Best correspondence (minimises avg cross-profile arc-length distance):
#     PA  ↔ Q1   (avg 59 mm vs 85 mm for PA↔Q6 alignment)
#     PI  ↔ Q5   (both are the "lower right" corner)
#   Long arc  A: PA→PB→PC→PD→PE→PF→PGH→PI   (Y decreasing 1748→1613)
#   Long side B: Q1→Q2→Q3→Q4→Q5             (Y decreasing 1792→1629)
#   Short side A: PI→PA                      (Y increasing 1623→1748)
#   Short side B: Q5→Q6→Q1                  (Y increasing 1629→1792)
#
# Construction strategy
# ─────────────────────
#   1. BRepOffsetAPI_ThruSections — OCC's native lofter handles unequal
#      edge counts natively; tries ruled/smooth × A→B/B→A orderings.
#   2. Arc-length parameterised zipper sewing (fallback) — parameterises
#      both polygons by cumulative arc-length (t ∈ [0,1]), merges the
#      t-values from both vertex lists into one shared sample grid, then
#      interpolates positions on each polygon at every shared t.  The
#      resulting matched rings are stitched as quad side strips.  All four
#      winding-direction combinations are tried.

print("Building loft 34 (8-gon → hexagon, sliver bridge)...")

def make_loft34():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    # ── Profile A: 8-gon (PG+PH merged) ─────────────────────
    _PG  = (1595.205, 1612.400, 313.109)
    _PH  = (1595.113, 1612.933, 312.144)
    PGH  = tuple((_PG[i] + _PH[i]) / 2.0 for i in range(3))

    A = [
        (1567.978, 1748.461, 236.978),   # 0  PA  — shared w/ loft33 A2, also long-arc start
        (1571.024, 1739.006, 254.337),   # 1  PB
        (1588.369, 1685.171, 353.182),   # 2  PC  — Z-peak of Profile A
        (1588.556, 1666.976, 342.905),   # 3  PD
        (1589.767, 1648.785, 332.801),   # 4  PE
        (1591.988, 1630.595, 322.869),   # 5  PF
        PGH,                              # 6  PGH — merged near-degenerate pair
        (1593.455, 1622.567, 294.696),   # 7  PI  — shared w/ loft33 A3, closing-edge end
    ]

    # ── Profile B: hexagon ────────────────────────────────────
    B = [
        (1564.227, 1792.115, 215.327),   # 0  Q1  — shared w/ loft33 B3/I; long-side start
        (1581.975, 1787.845, 276.144),   # 1  Q2
        (1597.362, 1783.965, 337.594),   # 2  Q3  — Z-peak of Profile B
        (1593.417, 1706.328, 295.100),   # 3  Q4
        (1589.471, 1628.691, 252.606),   # 4  Q5  — shared w/ loft33 B6/G; long-side end
        (1577.633, 1727.326, 243.380),   # 5  Q6  — shared w/ loft33 B4/J; short-side mid
    ]

    wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
    wb = Wire.make_polygon([Vector(*p) for p in B], close=True)

    # ── Attempt 1: BRepOffsetAPI_ThruSections ────────────────
    for ruled in [False, True]:
        for w1, w2, tag in [(wa, wb, 'A→B'), (wb, wa, 'B→A')]:
            try:
                gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                gen.AddWire(w1.wrapped)
                gen.AddWire(w2.wrapped)
                gen.Build()
                s = Solid(gen.Shape())
                if s.is_valid and s.volume > 0:
                    print(f"  loft34 ThruSections OK: {tag} ruled={ruled} vol={s.volume:.1f}")
                    return s
                print(f"  loft34 ThruSections {tag} ruled={ruled}: valid={s.is_valid} vol={s.volume:.1f}")
            except Exception as e:
                print(f"  loft34 ThruSections {tag} ruled={ruled}: {e}")

    # ── Attempt 2: Arc-length parameterised zipper sewing ────
    #
    # Both profiles are parameterised by cumulative arc-length t ∈ [0,1].
    # The merged t-grid contains every vertex t-value from BOTH polygons,
    # ensuring that each original vertex appears exactly in the side-wall
    # mesh and that no quad spans across a vertex kink.
    #
    # Alignment: A[0](PA) ↔ B[0](Q1)  — confirmed best by cross-profile
    # distance analysis (avg 59 mm vs 85 mm for the PA↔Q6 alternative).

    def arc_params(pts):
        """Return (t_values, total_length) for a closed polygon."""
        n = len(pts)
        segs = [
            math.sqrt(sum((pts[(i + 1) % n][j] - pts[i][j]) ** 2 for j in range(3)))
            for i in range(n)
        ]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum / total)
            cum += s
        return t, total

    def point_at_t(pts, t_vals, t):
        """Interpolate position on closed polygon at normalised arc-length t."""
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t0 = t_vals[i]
            t1 = t_vals[(i + 1) % n] if i < n - 1 else 1.0
            if t0 <= t <= t1 + 1e-10:
                dt = t1 - t0
                s  = (t - t0) / dt if dt > 1e-12 else 0.0
                p, q = pts[i], pts[(i + 1) % n]
                return tuple(p[j] + s * (q[j] - p[j]) for j in range(3))
        return pts[0]   # t==1.0 wraps to t==0.0

    tA, _ = arc_params(A)
    tB, _ = arc_params(B)

    # Merged t-grid: a sample exists at every vertex of both polygons
    all_t = sorted(set(tA + tB))

    ring_A = [point_at_t(A, tA, t) for t in all_t]   # len == len(all_t)
    ring_B = [point_at_t(B, tB, t) for t in all_t]

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    Am = tuple(sum(v[i] for v in A) / len(A) for i in range(3))
    Bm = tuple(sum(v[i] for v in B) / len(B) for i in range(3))

    def build_solid(rev_a=False, rev_b=False):
        cap_a = list(reversed(A)) if rev_a else A
        cap_b = list(reversed(B)) if rev_b else B
        rA    = list(reversed(ring_A)) if rev_a else ring_A
        rB    = list(reversed(ring_B)) if rev_b else ring_B

        faces = []
        na = len(cap_a)
        nb = len(cap_b)

        # Cap A: centroid fan
        # CCW winding (normal points outward from A's side)
        for i in range(na):
            faces.append(tri(Am, cap_a[(i + 1) % na], cap_a[i]))

        # Cap B: centroid fan, opposite winding to Cap A
        for i in range(nb):
            faces.append(tri(Bm, cap_b[i], cap_b[(i + 1) % nb]))

        # Side walls: quad strips over the merged-t rings
        # Each quad: rA[i]–rA[i+1] (Profile A edge) ↔ rB[i]–rB[i+1] (Profile B edge)
        n = len(rA)
        for i in range(n):
            j = (i + 1) % n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))

        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces:
            sew.Add(f)
        sew.Perform()

        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except Exception as e:
            print(f"    Shell cast failed (rev_a={rev_a}, rev_b={rev_b}): {e}")
            return None

        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a}, rev_b={rev_b}: valid={s.is_valid}, vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for ra, rb in [(False, False), (True, False), (False, True), (True, True)]:
        result = build_solid(rev_a=ra, rev_b=rb)
        if result is not None:
            print(f"  loft34 arc-length zipper OK: vol={result.volume:.1f}")
            return result

    raise ValueError(
        "loft34: all construction strategies failed — verify vertex order / profile orientation"
    )

loft34 = make_loft34()

print("Building loft 35...")
loft35 = make_loft_solid(
    [(1531.686, 1604.369, 118.782),
     (1581.9,   1479.716, 171.064),
     (1546.458, 1604.346, 116.177)],
    [(1537.778, 1585.46,  153.5),
     (1585.216, 1460.447, 205.96),
     (1552.55,  1585.437, 150.895)]
)

print("Building loft 36...")

def make_loft36():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    A = [(1537.778, 1585.46,  153.5),
         (1585.216, 1460.447, 205.96),
         (1583.394, 1417.279, 186.263),
         (1534.356, 1542.091, 134.466)]

    B = [(1531.686, 1604.369, 118.782),
         (1528.307, 1560.566, 100.0),
         (1527.635, 1551.843,  96.26),
         (1531.42,  1542.806, 100.0),
         (1579.744, 1427.428, 147.744),
         (1581.9,   1479.716, 171.064)]

    wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
    wb = Wire.make_polygon([Vector(*p) for p in B], close=True)

    for ruled in [False, True]:
        for w1, w2 in [(wa, wb), (wb, wa)]:
            try:
                gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                gen.AddWire(w1.wrapped)
                gen.AddWire(w2.wrapped)
                gen.Build()
                s = Solid(gen.Shape())
                if s.is_valid and s.volume > 0:
                    print(f"  loft36 OK ruled={ruled} vol={s.volume:.1f}")
                    return s
            except Exception as e:
                print(f"  loft36 ThruSections failed: {e}")

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def lerp(p, q, t):
        return tuple(p[i] + t*(q[i]-p[i]) for i in range(3))

    # Expand A (4-gon) to 6 by splitting A1→A4 edge into 3 segments
    # A1↔B1, lerp(1/3)↔B2, lerp(2/3)↔B3, A4↔B4, A3↔B5, A2↔B6
    PA = [A[0], lerp(A[0], A[3], 1/3), lerp(A[0], A[3], 2/3), A[3], A[2], A[1]]
    PB = list(B)

    Am = tuple(sum(v[i] for v in A)/4 for i in range(3))
    Bm = tuple(sum(v[i] for v in B)/6 for i in range(3))

    def build(rev_a=False, rev_b=False):
        ca = list(reversed(A)) if rev_a else A
        cb = list(reversed(B)) if rev_b else B
        rA = list(reversed(PA)) if rev_a else PA
        rB = list(reversed(PB)) if rev_b else PB
        faces = []
        for i in range(4):
            faces.append(tri(Am, ca[(i+1)%4], ca[i]))
        for i in range(6):
            faces.append(tri(Bm, cb[i], cb[(i+1)%6]))
        for i in range(6):
            j = (i+1)%6
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
        r = build(ra, rb)
        if r: return r

    raise ValueError("loft36 failed")

loft36 = make_loft36()

print("Building loft 37...")

def make_loft37():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1534.356, 1542.091, 134.466),
        (1583.394, 1417.279, 186.263),
        (1579.485, 1252.842, 143.307),
        (1533.598, 1361.408, 100.0),
        (1526.914, 1377.221,  93.692),
        (1528.065, 1402.728, 100.0),
    ]
    # B reversed so A[0](top)↔B[0](top), A[1](right)↔B[1](right) etc.
    B = [
        (1531.42,  1542.806, 100.0),
        (1579.744, 1427.428, 147.744),
        (1575.834, 1262.99,  104.789),
        (1570.43,  1275.033, 100.0),
        (1520.193, 1386.974,  55.486),
        (1527.635, 1551.843,  96.26),
    ]
    B_orig = [
        (1531.42,  1542.806, 100.0),
        (1527.635, 1551.843,  96.26),
        (1520.193, 1386.974,  55.486),
        (1570.43,  1275.033, 100.0),
        (1575.834, 1262.99,  104.789),
        (1579.744, 1427.428, 147.744),
    ]

    # Attempt 1: ThruSections
    for pts_b in [B, B_orig]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft37 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except Exception as e:
                    print(f"  loft37 ThruSections: {e}")

    # Attempt 2: Manual 6-6 stitching
    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    Am = tuple(sum(v[i] for v in A)/6 for i in range(3))

    def build(PA, PB, rev_a=False, rev_b=False):
        Bm = tuple(sum(v[i] for v in PB)/6 for i in range(3))
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        rA = list(reversed(PA)) if rev_a else PA
        rB = list(reversed(PB)) if rev_b else PB
        faces = []
        for i in range(6):
            faces.append(tri(Am, ca[(i+1)%6], ca[i]))
        for i in range(6):
            faces.append(tri(Bm, cb[i], cb[(i+1)%6]))
        for i in range(6):
            j = (i+1)%6
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_orig]:
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft37 manual OK vol={r.volume:.1f}")
                return r

    # Attempt 3: Arc-length zipper (handles mismatched shapes)
    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total)
            cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t0 = ts[i]
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if t0 <= t <= t1+1e-10:
                s = (t-t0)/(t1-t0) if (t1-t0) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    for pts_b in [B, B_orig]:
        tA = arc_params(A)
        tB = arc_params(pts_b)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(A, tA, t) for t in all_t]
        rB = [pt_at(pts_b, tB, t) for t in all_t]
        Bm = tuple(sum(v[i] for v in pts_b)/6 for i in range(3))

        def build_zip(rev_a=False, rev_b=False):
            ca = list(reversed(A)) if rev_a else A
            cb = list(reversed(pts_b)) if rev_b else pts_b
            rzA = list(reversed(rA)) if rev_a else rA
            rzB = list(reversed(rB)) if rev_b else rB
            faces = []
            for i in range(6):
                faces.append(tri(Am, ca[(i+1)%6], ca[i]))
            for i in range(6):
                faces.append(tri(Bm, cb[i], cb[(i+1)%6]))
            n = len(rzA)
            for i in range(n):
                j = (i+1)%n
                faces.append(tri(rzA[i], rzB[j], rzB[i]))
                faces.append(tri(rzA[i], rzA[j], rzB[j]))
            sew = BRepBuilderAPI_Sewing(1e-2)
            for f in faces: sew.Add(f)
            sew.Perform()
            try:
                shell = TopoDS.Shell_s(sew.SewedShape())
            except:
                return None
            fix = ShapeFix_Solid()
            fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
            fix.Perform()
            s = Solid(fix.Solid())
            print(f"    zip rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
            return s if s.is_valid and s.volume > 0 else None

        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build_zip(ra, rb)
            if r:
                print(f"  loft37 zipper OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft37 failed")

loft37 = make_loft37()

print("Building loft 38...")

def make_loft38():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1526.914, 1377.221,  93.692),
        (1533.598, 1361.408, 100.0),
        (1579.485, 1252.842, 143.307),
        (1578.711, 1206.155, 134.673),
        (1541.03,  1294.043, 100.0),
        (1525.422, 1330.448,  85.638),
    ]
    B_orig = [
        (1520.193, 1386.974,  55.486),
        (1570.43,  1275.033, 100.0),
        (1575.834, 1262.99,  104.789),
        (1575.407, 1235.798, 100.0),
        (1574.947, 1206.572,  94.853),
        (1518.477, 1330.456,  46.245),
    ]
    B_rev = list(reversed(B_orig))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total)
            cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a=False, rev_b=False):
        Am = tuple(sum(v[i] for v in PA)/len(PA) for i in range(3))
        Bm = tuple(sum(v[i] for v in PB)/len(PB) for i in range(3))
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        na, nb = len(ca), len(cb)
        for i in range(na):
            faces.append(tri(Am, ca[(i+1)%na], ca[i]))
        for i in range(nb):
            faces.append(tri(Bm, cb[i], cb[(i+1)%nb]))
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B_orig, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft38 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft38 OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft38 failed")

loft38 = make_loft38()

print("Building loft 39...")

def make_loft39():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1525.422, 1330.448,  85.638),
        (1541.03,  1294.043, 100.0),
        (1578.711, 1206.155, 134.673),
        (1578.699, 1036.164, 132.893),
        (1541.426, 1122.695, 100.0),
        (1525.164, 1160.448,  85.649),
    ]
    B = [
        (1518.477, 1330.456,  46.245),
        (1574.947, 1206.572,  94.853),
        (1574.935, 1036.581,  93.073),
        (1518.218, 1160.456,  46.257),
    ]

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total)
            cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a=False, rev_b=False):
        Am = tuple(sum(v[i] for v in PA)/len(PA) for i in range(3))
        Bm = tuple(sum(v[i] for v in PB)/len(PB) for i in range(3))
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        na, nb = len(ca), len(cb)
        for i in range(na):
            faces.append(tri(Am, ca[(i+1)%na], ca[i]))
        for i in range(nb):
            faces.append(tri(Bm, cb[i], cb[(i+1)%nb]))
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
    wb = Wire.make_polygon([Vector(*p) for p in B], close=True)
    for ruled in [False, True]:
        for w1, w2 in [(wa, wb), (wb, wa)]:
            try:
                gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                gen.AddWire(w1.wrapped)
                gen.AddWire(w2.wrapped)
                gen.Build()
                s = Solid(gen.Shape())
                if s.is_valid and s.volume > 0:
                    print(f"  loft39 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                    return s
            except: pass

    for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
        r = build(A, B, ra, rb)
        if r:
            print(f"  loft39 OK vol={r.volume:.1f}")
            return r

    raise ValueError("loft39 failed")

loft39 = make_loft39()

print("Building loft 40...")

def make_loft40():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1525.164, 1160.448,  85.649),
        (1541.426, 1122.695, 100.0),
        (1578.699, 1036.164, 132.893),
        (1579.021,  988.772, 135.842),
        (1538.164, 1084.017, 100.0),
        (1525.692, 1113.089,  89.06),
    ]
    B = [
        (1518.218, 1160.456,  46.257),
        (1574.935, 1036.581,  93.073),
        (1575.369,  979.432,  97.119),
        (1518.941, 1103.352,  50.855),
    ]
    B_rev = list(reversed(B))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total)
            cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a=False, rev_b=False):
        Am = tuple(sum(v[i] for v in PA)/len(PA) for i in range(3))
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        na = len(ca)
        for i in range(na):
            faces.append(tri(Am, ca[(i+1)%na], ca[i]))
        nb = len(cb)
        if nb == 4:
            if not rev_b:
                faces += [tri(cb[0], cb[1], cb[2]), tri(cb[0], cb[2], cb[3])]
            else:
                faces += [tri(cb[0], cb[2], cb[1]), tri(cb[0], cb[3], cb[2])]
        else:
            Bm = tuple(sum(v[i] for v in cb)/nb for i in range(3))
            for i in range(nb):
                faces.append(tri(Bm, cb[i], cb[(i+1)%nb]))
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft40 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft40 OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft40 failed")

loft40 = make_loft40()

print("Building loft 41...")

def make_loft41():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1518.941, 1103.352,  50.855),
        (1575.369,  979.432,  97.119),
        (1555.458,  979.414,  99.001),
    ]
    B = [
        (1525.692, 1113.089,  89.06),
        (1533.205, 1085.137, 100.0),
        (1559.11,   988.753, 137.724),
        (1579.021,  988.772, 135.842),
        (1538.164, 1084.017, 100.0),
    ]
    B_rev = list(reversed(B))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total)
            cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a=False, rev_b=False):
        Am = tuple(sum(v[i] for v in PA)/len(PA) for i in range(3))
        Bm = tuple(sum(v[i] for v in PB)/len(PB) for i in range(3))
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        faces.append(tri(ca[0], ca[1], ca[2]))
        nb = len(cb)
        for i in range(nb):
            faces.append(tri(Bm, cb[i], cb[(i+1)%nb]))
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft41 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft41 OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft41 failed")

loft41 = make_loft41()

print("Building loft 42...")

def make_loft42():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1525.692, 1113.089,  89.06),
        (1533.205, 1085.137, 100.0),
        (1559.11,   988.753, 137.724),
        (1561.224,  898.81,  159.217),
        (1563.338,  808.867, 180.711),
        (1548.291,  871.26,  157.084),
        (1533.245,  933.654, 133.456),
        (1529.469, 1023.371, 111.258),
        (1527.554, 1068.873, 100.0),
    ]
    # B1/B2 are 4.3mm apart, B6/B7 are 4.8mm apart — merged
    B = [
        (1518.941, 1103.352,  50.855),
        tuple(((1555.458+1555.556)/2, (979.414+975.234)/2, (99.001+100.0)/2)),
        (1558.596,  845.893, 130.909),
        (1543.269,  848.286, 121.739),
        (1529.588,  850.421, 113.436),
        tuple(((1527.302+1527.106)/2, (904.723+909.368)/2, (100.0+98.851)/2)),
        (1523.024, 1006.36,   74.853),
    ]
    B_rev = list(reversed(B))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total)
            cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a=False, rev_b=False):
        Am = tuple(sum(v[i] for v in PA)/len(PA) for i in range(3))
        Bm = tuple(sum(v[i] for v in PB)/len(PB) for i in range(3))
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        na, nb = len(ca), len(cb)
        for i in range(na):
            faces.append(tri(Am, ca[(i+1)%na], ca[i]))
        for i in range(nb):
            faces.append(tri(Bm, cb[i], cb[(i+1)%nb]))
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft42 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft42 OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft42 failed")

loft42 = make_loft42()

print("Building loft 43...")

def make_loft43():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1346.582, 1772.353, 237.571),
        (1385.788, 1748.74,  269.101),
        (1370.361, 1585.716, 183.018),
        (1323.908, 1609.869, 152.074),
    ]
    B = [
        (1334.85,  1778.299, 196.927),
        (1378.445, 1754.431, 227.403),
        (1364.268, 1604.626, 148.3),
        (1314.014, 1628.989, 118.361),
    ]
    B_rev = list(reversed(B))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total)
            cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a=False, rev_b=False):
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        if not rev_a:
            faces += [tri(ca[0], ca[1], ca[2]), tri(ca[0], ca[2], ca[3])]
        else:
            faces += [tri(ca[0], ca[2], ca[1]), tri(ca[0], ca[3], ca[2])]
        if not rev_b:
            faces += [tri(cb[0], cb[2], cb[1]), tri(cb[0], cb[3], cb[2])]
        else:
            faces += [tri(cb[0], cb[1], cb[2]), tri(cb[0], cb[2], cb[3])]
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft43 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft43 OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft43 failed")

loft43 = make_loft43()

print("Building loft 44...")

def make_loft44():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    A = [
        (1323.908, 1609.869, 152.074),
        (1370.361, 1585.716, 183.018),
        (1366.938, 1542.348, 163.984),
        (1318.96,  1566.62,  133.103),
    ]
    # Reordered to match A winding: upper-left→upper-right→lower-right→lower-left
    B = [
        (1314.014, 1628.989, 118.361),
        (1364.268, 1604.626, 148.3),
        (1360.217, 1552.1,   125.778),
        (1309.236, 1586.163, 100.0),
    ]

    # Attempt 1: make_loft_solid
    for pts_b in [B, list(reversed(B))]:
        try:
            s = make_loft_solid(A, pts_b)
            if s.is_valid and s.volume > 0:
                print(f"  loft44 make_loft_solid OK vol={s.volume:.1f}")
                return s
        except: pass

    # Attempt 2: make_ruled_solid
    for pts_b in [B, list(reversed(B))]:
        try:
            s = make_ruled_solid(A, pts_b)
            if s.is_valid and s.volume > 0:
                print(f"  loft44 make_ruled_solid OK vol={s.volume:.1f}")
                return s
        except: pass

    # Attempt 3: manual diagonal-cap stitching
    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def build(PA, PB, rev_a=False, rev_b=False):
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        faces = []
        if not rev_a:
            faces += [tri(ca[0], ca[1], ca[2]), tri(ca[0], ca[2], ca[3])]
        else:
            faces += [tri(ca[0], ca[2], ca[1]), tri(ca[0], ca[3], ca[2])]
        if not rev_b:
            faces += [tri(cb[0], cb[2], cb[1]), tri(cb[0], cb[3], cb[2])]
        else:
            faces += [tri(cb[0], cb[1], cb[2]), tri(cb[0], cb[2], cb[3])]
        for i in range(4):
            j = (i+1)%4
            faces += [tri(ca[i], cb[j], cb[i]), tri(ca[i], ca[j], cb[j])]
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, list(reversed(B))]:
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft44 manual OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft44 failed")

loft44 = make_loft44()

print("Building loft 45...")

def make_loft45():
    # Simplify both profiles to 4 dominant corners.
    # A: drop near-degenerate A5(16.78mm+33.72mm edges) and A6
    # B: merge near-degenerate B2-B3 (8.89mm), drop B5
    A = [
        (1359.496, 1377.478, 123.21),   # bottom-right
        (1366.938, 1542.348, 163.984),  # top-right
        (1318.96,  1566.62,  133.103),  # top-left
        (1308.995, 1402.013,  91.809),  # bottom-left
    ]
    _b2 = (1308.17,  1576.606,  95.903)
    _b3 = (1315.308, 1573.245, 100.0)
    _b23 = tuple((_b2[i]+_b3[i])/2 for i in range(3))
    B = [
        (1352.775, 1387.23,   85.004),  # bottom-right
        (1360.217, 1552.1,   125.778),  # top-right
        _b23,                            # top-left (merged)
        (1298.205, 1411.998,  54.609),  # bottom-left
    ]

    for pts_b in [B, list(reversed(B))]:
        try:
            s = make_loft_solid(A, pts_b)
            if s.is_valid and s.volume > 0:
                print(f"  loft45 make_loft_solid OK vol={s.volume:.1f}")
                return s
        except: pass

    for pts_b in [B, list(reversed(B))]:
        try:
            s = make_ruled_solid(A, pts_b)
            if s.is_valid and s.volume > 0:
                print(f"  loft45 make_ruled_solid OK vol={s.volume:.1f}")
                return s
        except: pass

    raise ValueError("loft45 failed")

loft45 = make_loft45()

# ══════════════════════════════════════════════════════════════
# Loft 46: 6-gon to 4-gon (Explicit Exact Triangulation)
# ══════════════════════════════════════════════════════════════
print("Building loft 46...")

def make_loft46():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    # ── Profile A (6 Edges) ──────────────────────────────
    A = [
        (1322.168, 1395.613, 100.0),    # A[0]
        (1359.496, 1377.478, 123.21),   # A[1]
        (1358.005, 1330.705, 115.156),  # A[2]
        (1333.64,  1342.486, 100.0),    # A[3]
        (1307.153, 1355.293,  83.524),  # A[4]
        (1308.995, 1402.013,  91.809),  # A[5]
    ]
    
    # ── Profile B (4 Edges) ──────────────────────────────
    B = [
        (1352.775, 1387.23,   85.004),  # B[0]
        (1351.059, 1330.713,  75.763),  # B[1]
        (1296.117, 1355.542,  45.077),  # B[2]
        (1298.205, 1411.998,  54.609),  # B[3]
    ]

    def tri(p1, p2, p3):
        # Generates a flat, robust face
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # Centroids for end-cap fanning
    Am = tuple(sum(v[i] for v in A)/6 for i in range(3))
    Bm = tuple(sum(v[i] for v in B)/4 for i in range(3))

    def build_solid(flipped=False):
        faces = []
        
        if not flipped:
            # Source Cap (CCW)
            for i in range(6): 
                faces.append(tri(Am, A[(i+1)%6], A[i]))
            
            # Target Cap (CW)
            for i in range(4): 
                faces.append(tri(Bm, B[i], B[(i+1)%4]))
            
            # Explicit Side Walls (Guarantees no missing triangles)
            faces.append(tri(A[5], A[0], B[3]))
            faces.append(tri(A[0], B[0], B[3]))
            faces.append(tri(A[0], A[1], B[0]))
            faces.append(tri(A[1], B[1], B[0]))
            faces.append(tri(A[1], A[2], B[1]))
            faces.append(tri(A[2], A[3], B[1]))
            faces.append(tri(A[3], B[2], B[1]))
            faces.append(tri(A[3], A[4], B[2]))
            faces.append(tri(A[4], B[3], B[2]))
            faces.append(tri(A[4], A[5], B[3]))
        
        else:
            # Source Cap Reversed
            for i in range(6): 
                faces.append(tri(Am, A[i], A[(i+1)%6]))
            
            # Target Cap Reversed
            for i in range(4): 
                faces.append(tri(Bm, B[(i+1)%4], B[i]))
            
            # Side Walls Reversed
            faces.append(tri(A[5], B[3], A[0]))
            faces.append(tri(A[0], B[3], B[0]))
            faces.append(tri(A[0], B[0], A[1]))
            faces.append(tri(A[1], B[0], B[1]))
            faces.append(tri(A[1], B[1], A[2]))
            faces.append(tri(A[2], B[1], A[3]))
            faces.append(tri(A[3], B[1], B[2]))
            faces.append(tri(A[3], B[2], A[4]))
            faces.append(tri(A[4], B[2], B[3]))
            faces.append(tri(A[4], B[3], A[5]))

        # Filter out any accidental degenerates
        faces = [f for f in faces if f is not None]

        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except Exception:
            return None
            
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        return s if s.is_valid and s.volume > 0 else None

    # Attempt 1: Standard orientation
    s = build_solid(flipped=False)
    if s: 
        print(f"  Loft 46 (Exact Manual) OK: vol={s.volume:.1f}")
        return s
        
    # Attempt 2: Auto-flip to resolve normals if the volume was inverted
    s = build_solid(flipped=True)
    if s: 
        print(f"  Loft 46 (Exact Manual, Flipped) OK: vol={s.volume:.1f}")
        return s
        
    raise ValueError("Loft 46 failed to resolve closed volume geometry.")

loft46 = make_loft46()

# ══════════════════════════════════════════════════════════════
# Loft 47: 6-gon to 4-gon (Explicit Exact Triangulation)
# ══════════════════════════════════════════════════════════════
print("Building loft 47...")

def make_loft47():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    # ── Profile A (6 Edges - Traced continuously) ───────────────
    A = [
        (1358.005, 1330.705, 115.156),  # A[0] (Right Top)
        (1333.640, 1342.486, 100.000),  # A[1] (Mid Top)
        (1307.153, 1355.293,  83.524),  # A[2] (Left Top)
        (1308.660, 1185.307,  81.990),  # A[3] (Left Bottom)
        (1335.306, 1171.952, 100.000),  # A[4] (Mid Bottom)
        (1357.746, 1160.705, 115.167),  # A[5] (Right Bottom)
    ]
    
    # ── Profile B (4 Edges - Traced continuously) ───────────────
    B = [
        (1351.059, 1330.713, 75.763),   # B[0] (Right Top)
        (1350.801, 1160.713, 75.775),   # B[1] (Right Bottom)
        (1297.624, 1185.556, 43.543),   # B[2] (Left Bottom)
        (1296.117, 1355.542, 45.077),   # B[3] (Left Top)
    ]

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # Centroids for end-cap fanning
    Am = tuple(sum(v[i] for v in A)/6 for i in range(3))
    Bm = tuple(sum(v[i] for v in B)/4 for i in range(3))

    def build_solid(flipped=False):
        faces = []
        
        if not flipped:
            # Source Cap (CCW)
            for i in range(6): 
                faces.append(tri(Am, A[(i+1)%6], A[i]))
            
            # Target Cap (CW)
            for i in range(4): 
                faces.append(tri(Bm, B[i], B[(i+1)%4]))
            
            # Explicit Side Walls (Guarantees no missing triangles)
            # Top Wall (A0-A1-A2 mapped to B0-B3)
            faces.append(tri(A[0], A[1], B[0]))
            faces.append(tri(A[1], B[3], B[0]))
            faces.append(tri(A[1], A[2], B[3]))
            # Left Wall (A2-A3 mapped to B3-B2)
            faces.append(tri(A[2], A[3], B[3]))
            faces.append(tri(A[3], B[2], B[3]))
            # Bottom Wall (A3-A4-A5 mapped to B2-B1)
            faces.append(tri(A[3], A[4], B[2]))
            faces.append(tri(A[4], B[1], B[2]))
            faces.append(tri(A[4], A[5], B[1]))
            # Right Wall (A5-A0 mapped to B1-B0)
            faces.append(tri(A[5], A[0], B[1]))
            faces.append(tri(A[0], B[0], B[1]))
        
        else:
            # Source Cap Reversed
            for i in range(6): 
                faces.append(tri(Am, A[i], A[(i+1)%6]))
            
            # Target Cap Reversed
            for i in range(4): 
                faces.append(tri(Bm, B[(i+1)%4], B[i]))
            
            # Side Walls Reversed
            faces.append(tri(A[0], B[0], A[1]))
            faces.append(tri(A[1], B[0], B[3]))
            faces.append(tri(A[1], B[3], A[2]))
            faces.append(tri(A[2], B[3], A[3]))
            faces.append(tri(A[3], B[3], B[2]))
            faces.append(tri(A[3], B[2], A[4]))
            faces.append(tri(A[4], B[2], B[1]))
            faces.append(tri(A[4], B[1], A[5]))
            faces.append(tri(A[5], B[1], A[0]))
            faces.append(tri(A[0], B[1], B[0]))

        # Filter out any accidental degenerates
        faces = [f for f in faces if f is not None]

        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except Exception:
            return None
            
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        return s if s.is_valid and s.volume > 0 else None

    # Attempt 1: Standard orientation
    s = build_solid(flipped=False)
    if s: 
        print(f"  Loft 47 (Exact Manual) OK: vol={s.volume:.1f}")
        return s
        
    # Attempt 2: Auto-flip to resolve normals if the volume was inverted
    s = build_solid(flipped=True)
    if s: 
        print(f"  Loft 47 (Exact Manual, Flipped) OK: vol={s.volume:.1f}")
        return s
        
    raise ValueError("Loft 47 failed to resolve closed volume geometry.")

loft47 = make_loft47()

# ══════════════════════════════════════════════════════════════
# Loft 48: 6-gon to 4-gon (Explicit Exact Triangulation)
# ══════════════════════════════════════════════════════════════
print("Building loft 48...")

def make_loft48():
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    # ── Profile A (6 Edges - Traced continuously) ───────────────
    A = [
        (1357.746, 1160.705, 115.167),  # A[0] (Top Right)
        (1358.275, 1113.346, 118.578),  # A[1] (Bottom Right)
        (1331.673, 1126.903, 100.000),  # A[2] (Bottom Mid)
        (1310.035, 1137.930,  84.888),  # A[3] (Bottom Left)
        (1308.660, 1185.307,  81.990),  # A[4] (Top Left)
        (1335.306, 1171.952, 100.000),  # A[5] (Top Mid)
    ]
    
    # ── Profile B (4 Edges - Traced continuously) ───────────────
    B = [
        (1351.524, 1103.609,  80.373),  # B[0] (Bottom Right)
        (1299.418, 1128.428,  47.512),  # B[1] (Bottom Left)
        (1297.624, 1185.556,  43.543),  # B[2] (Top Left)
        (1350.801, 1160.713,  75.775),  # B[3] (Top Right)
    ]

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    # Centroids for end-cap fanning
    Am = tuple(sum(v[i] for v in A)/6 for i in range(3))
    Bm = tuple(sum(v[i] for v in B)/4 for i in range(3))

    def build_solid(flipped=False):
        faces = []
        
        if not flipped:
            # Source Cap (CCW)
            for i in range(6): 
                faces.append(tri(Am, A[(i+1)%6], A[i]))
            
            # Target Cap (CW)
            for i in range(4): 
                faces.append(tri(Bm, B[i], B[(i+1)%4]))
            
            # Explicit Side Walls (Guarantees no missing triangles)
            # Right Wall (A0-A1 mapped to B3-B0)
            faces.append(tri(A[0], A[1], B[0]))
            faces.append(tri(A[0], B[0], B[3]))
            # Bottom Wall (A1-A2-A3 mapped to B0-B1)
            faces.append(tri(A[1], A[2], B[0]))
            faces.append(tri(A[2], B[1], B[0]))
            faces.append(tri(A[2], A[3], B[1]))
            # Left Wall (A3-A4 mapped to B1-B2)
            faces.append(tri(A[3], A[4], B[2]))
            faces.append(tri(A[3], B[2], B[1]))
            # Top Wall (A4-A5-A0 mapped to B2-B3)
            faces.append(tri(A[4], A[5], B[2]))
            faces.append(tri(A[5], B[3], B[2]))
            faces.append(tri(A[5], A[0], B[3]))
        
        else:
            # Source Cap Reversed
            for i in range(6): 
                faces.append(tri(Am, A[i], A[(i+1)%6]))
            
            # Target Cap Reversed
            for i in range(4): 
                faces.append(tri(Bm, B[(i+1)%4], B[i]))
            
            # Side Walls Reversed
            faces.append(tri(A[0], B[0], A[1]))
            faces.append(tri(A[0], B[3], B[0]))
            faces.append(tri(A[1], B[0], A[2]))
            faces.append(tri(A[2], B[0], B[1]))
            faces.append(tri(A[2], B[1], A[3]))
            faces.append(tri(A[3], B[2], A[4]))
            faces.append(tri(A[3], B[1], B[2]))
            faces.append(tri(A[4], B[2], A[5]))
            faces.append(tri(A[5], B[2], B[3]))
            faces.append(tri(A[5], B[3], A[0]))

        # Filter out any accidental degenerates
        faces = [f for f in faces if f is not None]

        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except Exception:
            return None
            
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        return s if s.is_valid and s.volume > 0 else None

    # Attempt 1: Standard orientation
    s = build_solid(flipped=False)
    if s: 
        print(f"  Loft 48 (Exact Manual) OK: vol={s.volume:.1f}")
        return s
        
    # Attempt 2: Auto-flip to resolve normals if the volume was inverted
    s = build_solid(flipped=True)
    if s: 
        print(f"  Loft 48 (Exact Manual, Flipped) OK: vol={s.volume:.1f}")
        return s
        
    raise ValueError("Loft 48 failed to resolve closed volume geometry.")

loft48 = make_loft48()

print("Building loft 49...")

def make_loft49():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1331.673, 1126.903, 100.0),
        (1358.275, 1113.346, 118.578),
        (1362.051, 1023.628, 140.776),
        (1365.828,  933.911, 162.974),
        (1324.061,  958.238, 126.588),
        (1317.048, 1048.084, 105.738),
        (1315.118, 1072.811, 100.0),
        (1310.035, 1137.93,   84.888),
    ]
    B = [
        (1351.524, 1103.609,  80.373),
        (1354.863, 1024.282, 100.0),
        (1355.607, 1006.617, 104.371),
        (1359.689,  909.625, 128.369),
        (1323.92,   929.085, 100.0),
        (1314.581,  934.166,  92.593),
        (1306.999, 1031.297,  70.053),
        (1299.418, 1128.428,  47.512),
    ]
    B_rev = list(reversed(B))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total); cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a=False, rev_b=False):
        Am = tuple(sum(v[i] for v in PA)/len(PA) for i in range(3))
        Bm = tuple(sum(v[i] for v in PB)/len(PB) for i in range(3))
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        na, nb = len(ca), len(cb)
        for i in range(na):
            faces.append(tri(Am, ca[(i+1)%na], ca[i]))
        for i in range(nb):
            faces.append(tri(Bm, cb[i], cb[(i+1)%nb]))
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft49 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft49 OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft49 failed")

loft49 = make_loft49()

print("Building loft 50...")

def make_loft50():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1139.88,  1576.903, 239.056),  # 0
        (1155.659, 1661.125, 280.884),  # 1
        (1171.437, 1745.346, 322.712),  # 2
        (1177.769, 1760.659, 304.565),  # 3
        (1183.188, 1770.481, 284.462),  # 4
        (1171.851, 1689.239, 241.713),  # 5
        (1160.513, 1607.997, 198.964),  # 6
    ]
    B = [
        (1127.906, 1595.655, 205.815),
        (1141.972, 1670.734, 243.102),
        (1156.037, 1745.813, 280.389),
        (1171.455, 1776.426, 243.817),
        (1150.62,  1627.117, 165.251),
    ]
    B_rev = list(reversed(B))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def cap_tris_A(pts, flip):
        # A is convex at index 4 — fan from there
        pivot = 4
        n = len(pts)
        result = []
        for i in range(1, n-1):
            a = pivot
            b = (pivot + i) % n
            c = (pivot + i + 1) % n
            if flip:
                result.append(tri(pts[a], pts[c], pts[b]))
            else:
                result.append(tri(pts[a], pts[b], pts[c]))
        return result

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total); cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, flip_a=False, rev_b=False):
        Bm = tuple(sum(v[i] for v in PB)/len(PB) for i in range(3))
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_b: rB = list(reversed(rB))
        faces = []
        faces += cap_tris_A(PA, flip_a)
        nb = len(cb)
        for i in range(nb):
            faces.append(tri(Bm, cb[i], cb[(i+1)%nb]))
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    flip_a={flip_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft50 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass

    for fa, rb in [(False,False),(True,False),(False,True),(True,True)]:
        r = build(A, B, fa, rb)
        if r:
            print(f"  loft50 OK vol={r.volume:.1f}")
            return r

    raise ValueError("loft50 failed")

loft50 = make_loft50()

print("Building loft 51...")

def make_loft51():
    A = [
        (1139.88,  1576.903, 239.056),
        (1160.513, 1607.997, 198.964),
        (1155.565, 1564.748, 179.993),
        (1133.08,  1533.45,  221.156),
    ]
    B = [
        (1127.906, 1595.655, 205.815),
        (1150.62,  1627.117, 165.251),
        (1144.775, 1574.734, 142.793),
        (1119.854, 1543.03,  184.642),
    ]

    for pts_b in [B, list(reversed(B))]:
        try:
            s = make_loft_solid(A, pts_b)
            if s.is_valid and s.volume > 0:
                print(f"  loft51 make_loft_solid OK vol={s.volume:.1f}")
                return s
        except: pass

    for pts_b in [B, list(reversed(B))]:
        try:
            s = make_ruled_solid(A, pts_b)
            if s.is_valid and s.volume > 0:
                print(f"  loft51 make_ruled_solid OK vol={s.volume:.1f}")
                return s
        except: pass

    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def build(PA, PB, ra, rb):
        ca = list(reversed(PA)) if ra else PA
        cb = list(reversed(PB)) if rb else PB
        faces = []
        if not ra:
            faces += [tri(ca[0], ca[1], ca[2]), tri(ca[0], ca[2], ca[3])]
        else:
            faces += [tri(ca[0], ca[2], ca[1]), tri(ca[0], ca[3], ca[2])]
        if not rb:
            faces += [tri(cb[0], cb[2], cb[1]), tri(cb[0], cb[3], cb[2])]
        else:
            faces += [tri(cb[0], cb[1], cb[2]), tri(cb[0], cb[2], cb[3])]
        for i in range(4):
            j = (i+1)%4
            faces += [tri(ca[i], cb[j], cb[i]), tri(ca[i], ca[j], cb[j])]
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    ra={ra} rb={rb}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, list(reversed(B))]:
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft51 manual OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft51 failed")

loft51 = make_loft51()

print("Building loft 52...")

def make_loft52():
    A = [
        (1133.08,  1533.45,  221.156),
        (1155.565, 1564.748, 179.993),
        (1145.601, 1400.141, 138.699),
        (1118.194, 1368.401, 183.245),
    ]
    B = [
        (1119.854, 1543.03,  184.642),
        (1144.775, 1574.734, 142.793),
        (1134.811, 1410.126, 101.499),
        (1104.968, 1377.981, 146.731),
    ]

    for pts_b in [B, list(reversed(B))]:
        try:
            s = make_loft_solid(A, pts_b)
            if s.is_valid and s.volume > 0:
                print(f"  loft52 make_loft_solid OK vol={s.volume:.1f}")
                return s
        except: pass

    for pts_b in [B, list(reversed(B))]:
        try:
            s = make_ruled_solid(A, pts_b)
            if s.is_valid and s.volume > 0:
                print(f"  loft52 make_ruled_solid OK vol={s.volume:.1f}")
                return s
        except: pass

    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def build(PA, PB, ra, rb):
        ca = list(reversed(PA)) if ra else PA
        cb = list(reversed(PB)) if rb else PB
        faces = []
        if not ra:
            faces += [tri(ca[0], ca[1], ca[2]), tri(ca[0], ca[2], ca[3])]
        else:
            faces += [tri(ca[0], ca[2], ca[1]), tri(ca[0], ca[3], ca[2])]
        if not rb:
            faces += [tri(cb[0], cb[2], cb[1]), tri(cb[0], cb[3], cb[2])]
        else:
            faces += [tri(cb[0], cb[1], cb[2]), tri(cb[0], cb[2], cb[3])]
        for i in range(4):
            j = (i+1)%4
            faces += [tri(ca[i], cb[j], cb[i]), tri(ca[i], ca[j], cb[j])]
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    ra={ra} rb={rb}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, list(reversed(B))]:
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft52 manual OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft52 failed")

loft52 = make_loft52()

print("Building loft 53...")

def make_loft53():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1118.194, 1368.401, 183.245),
        (1145.601, 1400.141, 138.699),
        (1143.758, 1353.421, 130.414),
        (1115.191, 1321.592, 175.843),
    ]
    B = [
        (1104.968, 1377.981, 146.731),
        (1134.811, 1410.126, 101.499),
        (1134.482, 1401.247, 100.0),
        (1132.722, 1353.67,   91.967),
        (1127.306, 1348.074, 100.0),
        (1101.511, 1321.423, 138.256),
    ]
    B_rev = list(reversed(B))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total); cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a=False, rev_b=False):
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        if not rev_a:
            faces += [tri(ca[0], ca[1], ca[2]), tri(ca[0], ca[2], ca[3])]
        else:
            faces += [tri(ca[0], ca[2], ca[1]), tri(ca[0], ca[3], ca[2])]
        Bm = tuple(sum(v[i] for v in cb)/len(cb) for i in range(3))
        nb = len(cb)
        for i in range(nb):
            faces.append(tri(Bm, cb[i], cb[(i+1)%nb]))
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft53 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft53 OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft53 failed")

loft53 = make_loft53()

print("Building loft 54...")

def make_loft54():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1143.758, 1353.421, 130.414),
        (1145.266, 1183.434, 128.88),
        (1114.448, 1151.597, 176.88),
        (1115.191, 1321.592, 175.843),
    ]
    B = [
        (1132.722, 1353.67,   91.967),
        (1134.23,  1183.683,  90.434),
        (1127.678, 1177.368, 100.0),
        (1100.767, 1151.428, 139.292),
        (1101.511, 1321.423, 138.256),
        (1127.306, 1348.074, 100.0),
    ]
    B_rev = list(reversed(B))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total); cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a=False, rev_b=False):
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        if not rev_a:
            faces += [tri(ca[0], ca[1], ca[2]), tri(ca[0], ca[2], ca[3])]
        else:
            faces += [tri(ca[0], ca[2], ca[1]), tri(ca[0], ca[3], ca[2])]
        Bm = tuple(sum(v[i] for v in cb)/len(cb) for i in range(3))
        nb = len(cb)
        for i in range(nb):
            faces.append(tri(Bm, cb[i], cb[(i+1)%nb]))
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    rev_a={rev_a} rev_b={rev_b}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft54 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            r = build(A, pts_b, ra, rb)
            if r:
                print(f"  loft54 OK vol={r.volume:.1f}")
                return r

    raise ValueError("loft54 failed")

loft54 = make_loft54()

print("Building loft 55...")

def make_loft55():
    # A[0]↔B2, A[1]↔B0, A[2]↔B4, A[3]↔B3  (nearest spatial pairs)
    A = [
        (1114.448, 1151.597, 176.88),   # 0 top-left
        (1145.266, 1183.434, 128.88),   # 1 top-right
        (1146.64,  1136.058, 131.779),  # 2 bottom-right
        (1115.424, 1104.254, 180.42),   # 3 bottom-left
    ]
    B = [
        (1100.767, 1151.428, 139.292),  # →A[0] top-left
        (1134.23,  1183.683,  90.434),  # →A[1] top-right
        (1132.198, 1122.922, 100.0),    # →A[2] bottom-right
        (1102.113, 1094.346, 144.025),  # →A[3] bottom-left
    ]

    try:
        s = make_loft_solid(A, B)
        if s.is_valid and s.volume > 0:
            print(f"  loft55 make_loft_solid OK vol={s.volume:.1f}")
            return s
    except: pass

    try:
        s = make_ruled_solid(A, B)
        if s.is_valid and s.volume > 0:
            print(f"  loft55 make_ruled_solid OK vol={s.volume:.1f}")
            return s
    except: pass

    try:
        s = make_loft_solid(A, list(reversed(B)))
        if s.is_valid and s.volume > 0:
            print(f"  loft55 make_loft_solid rev OK vol={s.volume:.1f}")
            return s
    except: pass

    try:
        s = make_ruled_solid(A, list(reversed(B)))
        if s.is_valid and s.volume > 0:
            print(f"  loft55 make_ruled_solid rev OK vol={s.volume:.1f}")
            return s
    except: pass

    raise ValueError("loft55 failed")

loft55 = make_loft55()

print("Building loft 56...")

def make_loft56():
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_Sewing,
                                     BRepBuilderAPI_MakeSolid,
                                     BRepBuilderAPI_MakeFace)
    from OCP.TopoDS import TopoDS
    from OCP.ShapeFix import ShapeFix_Solid
    import math

    A = [
        (1146.64,  1136.058, 131.779),
        (1160.666,  956.365, 173.478),
        (1130.053,  925.024, 223.865),
        (1115.424, 1104.254, 180.42),
    ]
    # Drop B[2]=(1136.023,1126.556,94.402) — 7.7mm near-degenerate edge
    B = [
        (1102.113, 1094.346, 144.025),  # 0
        (1132.198, 1122.922, 100.0),    # 1
        (1137.906, 1102.435, 100.0),    # 2
        (1151.186,  932.293, 139.483),  # 3
        (1117.928,  900.583, 190.992),  # 4
    ]
    B_rev = list(reversed(B))

    def tri(p1, p2, p3):
        w = Wire.make_polygon([Vector(*p1), Vector(*p2), Vector(*p3)], close=True)
        return BRepBuilderAPI_MakeFace(w.wrapped).Face()

    def fan(pts, pivot, flip):
        n = len(pts)
        result = []
        for i in range(1, n-1):
            a = pivot
            b = (pivot + i) % n
            c = (pivot + i + 1) % n
            if flip:
                result.append(tri(pts[a], pts[c], pts[b]))
            else:
                result.append(tri(pts[a], pts[b], pts[c]))
        return result

    def arc_params(pts):
        n = len(pts)
        segs = [math.sqrt(sum((pts[(i+1)%n][j]-pts[i][j])**2 for j in range(3))) for i in range(n)]
        total = sum(segs)
        t, cum = [], 0.0
        for s in segs:
            t.append(cum/total); cum += s
        return t

    def pt_at(pts, ts, t):
        n = len(pts)
        t = t % 1.0
        for i in range(n):
            t1 = ts[(i+1)%n] if i < n-1 else 1.0
            if ts[i] <= t <= t1+1e-10:
                s = (t-ts[i])/(t1-ts[i]) if (t1-ts[i]) > 1e-12 else 0.0
                p, q = pts[i], pts[(i+1)%n]
                return tuple(p[j]+s*(q[j]-p[j]) for j in range(3))
        return pts[0]

    def build(PA, PB, rev_a, rev_b, a_piv, b_piv, fa, fb):
        ca = list(reversed(PA)) if rev_a else PA
        cb = list(reversed(PB)) if rev_b else PB
        tA = arc_params(PA)
        tB = arc_params(PB)
        all_t = sorted(set(tA + tB))
        rA = [pt_at(PA, tA, t) for t in all_t]
        rB = [pt_at(PB, tB, t) for t in all_t]
        if rev_a: rA = list(reversed(rA))
        if rev_b: rB = list(reversed(rB))
        faces = []
        faces += fan(ca, a_piv, fa)
        faces += fan(cb, b_piv, fb)
        n = len(rA)
        for i in range(n):
            j = (i+1)%n
            faces.append(tri(rA[i], rB[j], rB[i]))
            faces.append(tri(rA[i], rA[j], rB[j]))
        sew = BRepBuilderAPI_Sewing(1e-2)
        for f in faces: sew.Add(f)
        sew.Perform()
        try:
            shell = TopoDS.Shell_s(sew.SewedShape())
        except:
            return None
        fix = ShapeFix_Solid()
        fix.Init(BRepBuilderAPI_MakeSolid(shell).Solid())
        fix.Perform()
        s = Solid(fix.Solid())
        print(f"    ra={rev_a} rb={rev_b} ap={a_piv} bp={b_piv} fa={fa} fb={fb}: valid={s.is_valid} vol={s.volume:.1f}")
        return s if s.is_valid and s.volume > 0 else None

    for pts_b in [B, B_rev]:
        wa = Wire.make_polygon([Vector(*p) for p in A], close=True)
        wb = Wire.make_polygon([Vector(*p) for p in pts_b], close=True)
        for ruled in [False, True]:
            for w1, w2 in [(wa, wb), (wb, wa)]:
                try:
                    gen = BRepOffsetAPI_ThruSections(isSolid=True, ruled=ruled)
                    gen.AddWire(w1.wrapped)
                    gen.AddWire(w2.wrapped)
                    gen.Build()
                    s = Solid(gen.Shape())
                    if s.is_valid and s.volume > 0:
                        print(f"  loft56 ThruSections OK ruled={ruled} vol={s.volume:.1f}")
                        return s
                except: pass

    # A: fan from index 1 (convex top-right)
    # B: fan from index 3 or 4 (convex bottom)
    for pts_b in [B, B_rev]:
        for ra, rb in [(False,False),(True,False),(False,True),(True,True)]:
            for ap in [1, 0]:
                for bp in [3, 4, 0]:
                    for fa, fb in [(False,False),(True,False),(False,True),(True,True)]:
                        r = build(A, pts_b, ra, rb, ap, bp, fa, fb)
                        if r:
                            print(f"  loft56 OK vol={r.volume:.1f}")
                            return r

    raise ValueError("loft56 failed")

loft56 = make_loft56()

# ══════════════════════════════════════════════════════════════
# Extruded Profile Body
# Outer profile (148 edges) minus inner subtract profile (267 edges)
# Extruded to Z = 50 mm
# ══════════════════════════════════════════════════════════════
print("Building extruded profile body...")
 
 
def make_body():
    # ──────────────────────────────────────────────────────────
    # Outer profile edges  (start_xy, end_xy)
    # ──────────────────────────────────────────────────────────
    outer_edges = [
        ((471.572, 570.72),    (743.398, 266.936)),   # 1
        ((743.398, 266.936),   (750.049, 260.741)),   # 2
        ((750.049, 260.741),   (757.715, 255.852)),   # 3
        ((757.715, 255.852),   (766.132, 252.433)),   # 4
        ((766.132, 252.433),   (775.038, 250.593)),   # 5
        ((775.038, 250.593),   (784.111, 250.396)),   # 6
        ((784.111, 250.396),   (793.083, 251.845)),   # 7
        ((793.083, 251.845),   (801.635, 254.89)),    # 8
        ((801.635, 254.89),    (809.501, 259.434)),   # 9
        ((809.501, 259.434),   (828.916, 272.705)),   # 10
        ((828.916, 272.705),   (847.854, 284.799)),   # 11
        ((847.854, 284.799),   (866.327, 295.778)),   # 12
        ((866.327, 295.778),   (884.345, 305.703)),   # 13
        ((884.345, 305.703),   (901.917, 314.636)),   # 14
        ((901.917, 314.636),   (919.055, 322.639)),   # 15
        ((919.055, 322.639),   (935.769, 329.774)),   # 16
        ((935.769, 329.774),   (952.068, 336.102)),   # 17
        ((952.068, 336.102),   (983.467, 346.587)),   # 18
        ((983.467, 346.587),   (1013.593, 354.669)),  # 19
        ((1013.593, 354.669),  (1043.394, 360.943)),  # 20
        ((1043.394, 360.943),  (1073.336, 365.679)),  # 21
        ((1073.336, 365.679),  (1103.851, 369.131)),  # 22
        ((1103.851, 369.131),  (1135.371, 371.555)),  # 23
        ((1135.371, 371.555),  (1168.328, 373.205)),  # 24
        ((1168.328, 373.205),  (1203.156, 374.337)),  # 25
        ((1203.156, 374.337),  (1240.284, 375.204)),  # 26
        ((1240.284, 375.204),  (1280.147, 376.063)),  # 27
        ((1280.147, 376.063),  (1301.239, 376.568)),  # 28
        ((1301.239, 376.568),  (1323.176, 377.167)),  # 29
        ((1323.176, 377.167),  (1346.012, 377.891)),  # 30
        ((1346.012, 377.891),  (1369.803, 378.772)),  # 31
        ((1369.803, 378.772),  (1398.837, 382.366)),  # 32
        ((1398.837, 382.366),  (1428.796, 386.876)),  # 33
        ((1428.796, 386.876),  (1449.309, 390.922)),  # 34
        ((1449.309, 390.922),  (1469.445, 396.203)),  # 35
        ((1469.445, 396.203),  (1489.105, 402.692)),  # 36
        ((1489.105, 402.692),  (1508.191, 410.357)),  # 37
        ((1508.191, 410.357),  (1526.611, 419.161)),  # 38
        ((1526.611, 419.161),  (1544.274, 429.061)),  # 39
        ((1544.274, 429.061),  (1561.092, 440.007)),  # 40
        ((1561.092, 440.007),  (1576.984, 451.947)),  # 41
        ((1576.984, 451.947),  (1584.347, 458.367)),  # 42
        ((1584.347, 458.367),  (1586.129, 459.64)),   # 43
        ((1586.129, 459.64),   (1605.188, 476.474)),  # 44
        ((1605.188, 476.474),  (1623.002, 492.404)),  # 45
        ((1623.002, 492.404),  (1639.659, 507.482)),  # 46
        ((1639.659, 507.482),  (1655.251, 521.756)),  # 47
        ((1655.251, 521.756),  (1669.866, 535.277)),  # 48
        ((1669.866, 535.277),  (1683.594, 548.094)),  # 49
        ((1683.594, 548.094),  (1696.526, 560.258)),  # 50
        ((1696.526, 560.258),  (1708.75,  571.818)),  # 51
        ((1708.75,  571.818),  (1731.436, 593.324)),  # 52
        ((1731.436, 593.324),  (1752.37,  613.013)),  # 53
        ((1752.37,  613.013),  (1772.27,  631.283)),  # 54
        ((1772.27,  631.283),  (1791.82,  648.501)),  # 55
        ((1791.82,  648.501),  (1811.306, 664.672)),  # 56
        ((1811.306, 664.672),  (1830.976, 679.739)),  # 57
        ((1830.976, 679.739),  (1851.116, 693.683)),  # 58
        ((1851.116, 693.683),  (1872.014, 706.485)),  # 59
        ((1872.014, 706.485),  (1893.96,  718.124)),  # 60
        ((1893.96,  718.124),  (1917.393, 728.641)),  # 61
        ((1917.393, 728.641),  (1942.698, 738.006)),  # 62
        ((1942.698, 738.006),  (1970.174, 746.142)),  # 63
        ((1970.174, 746.142),  (1982.547, 750.497)),  # 64
        ((1982.547, 750.497),  (1994.042, 756.828)),  # 65
        ((1994.042, 756.828),  (2004.341, 764.949)),  # 66
        ((2004.341, 764.949),  (2013.181, 774.633)),  # 67
        ((2013.181, 774.633),  (2020.326, 785.596)),  # 68
        ((2020.326, 785.596),  (2025.628, 797.596)),  # 69
        ((2025.628, 797.596),  (2028.931, 810.264)),  # 70
        ((2028.931, 810.264),  (2030.166, 823.314)),  # 71
        ((2030.166, 823.314),  (2041.103, 1603.392)), # 72  ← long run up right side
        ((2040.387, 1615.257), (2041.103, 1603.392)), # 73
        ((2037.919, 1626.888), (2040.387, 1615.257)), # 74
        ((2033.755, 1638.022), (2037.919, 1626.888)), # 75
        ((2028.006, 1648.39),  (2033.755, 1638.022)), # 76
        ((2020.79,  1657.791), (2028.006, 1648.39)),  # 77
        ((2012.243, 1666.041), (2020.79,  1657.791)), # 78
        ((2002.597, 1672.917), (2012.243, 1666.041)), # 79
        ((1992.022, 1678.301), (2002.597, 1672.917)), # 80
        ((1924.513, 1705.859), (1992.022, 1678.301)), # 81
        ((1889.598, 1719.521), (1924.513, 1705.859)), # 82
        ((1853.861, 1733.071), (1889.598, 1719.521)), # 83
        ((1835.298, 1739.948), (1853.861, 1733.071)), # 84
        ((1815.593, 1747.155), (1835.298, 1739.948)), # 85
        ((1773.912, 1761.973), (1815.593, 1747.155)), # 86
        ((1731.233, 1776.329), (1773.912, 1761.973)), # 87
        ((1710.279, 1782.957), (1731.233, 1776.329)), # 88
        ((1689.989, 1789.016), (1710.279, 1782.957)), # 89
        ((1646.231, 1801.856), (1689.989, 1789.016)), # 90
        ((1601.351, 1813.915), (1646.231, 1801.856)), # 91
        ((1556.052, 1824.76),  (1601.351, 1813.915)), # 92
        ((1511.087, 1833.95),  (1556.052, 1824.76)),  # 93
        ((1468.068, 1841.006), (1511.087, 1833.95)),  # 94
        ((1427.092, 1845.907), (1468.068, 1841.006)), # 95
        ((1387.612, 1848.766), (1427.092, 1845.907)), # 96
        ((1349.081, 1849.694), (1387.612, 1848.766)), # 97
        ((786.222,  1849.694), (1349.081, 1849.694)), # 98  ← top flat
        ((771.515,  1848.331), (786.222,  1849.694)), # 99
        ((757.304,  1844.284), (771.515,  1848.331)), # 100
        ((744.088,  1837.699), (757.304,  1844.284)), # 101
        ((732.297,  1828.788), (744.088,  1837.699)), # 102
        ((722.363,  1817.881), (732.297,  1828.788)), # 103
        ((714.593,  1805.32),  (722.363,  1817.881)), # 104
        ((709.268,  1791.557), (714.593,  1805.32)),  # 105
        ((706.561,  1777.043), (709.268,  1791.557)), # 106
        ((701.402,  1719.069), (706.561,  1777.043)), # 107
        ((696.722,  1662.235), (701.402,  1719.069)), # 108
        ((688.548,  1549.727), (696.722,  1662.235)), # 109
        ((684.931,  1492.923), (688.548,  1549.727)), # 110
        ((681.544,  1434.998), (684.931,  1492.923)), # 111
        ((678.326,  1375.387), (681.544,  1434.998)), # 112
        ((675.257,  1314.753), (678.326,  1375.387)), # 113
        ((673.803,  1285.514), (675.257,  1314.753)), # 114
        ((672.378,  1257.105), (673.803,  1285.514)), # 115
        ((669.524,  1202.731), (672.378,  1257.105)), # 116
        ((666.515,  1151.543), (669.524,  1202.731)), # 117
        ((663.172,  1103.455), (666.515,  1151.543)), # 118
        ((659.314,  1058.381), (663.172,  1103.455)), # 119
        ((654.761,  1016.232), (659.314,  1058.381)), # 120
        ((649.333,   976.924), (654.761,  1016.232)), # 121
        ((642.851,   940.368), (649.333,   976.924)), # 122
        ((638.579,   919.817), (642.851,   940.368)), # 123
        ((634.056,   900.42),  (638.579,   919.817)), # 124
        ((629.308,   882.136), (634.056,   900.42)),  # 125
        ((624.361,   864.922), (629.308,   882.136)), # 126
        ((619.241,   848.736), (624.361,   864.922)), # 127
        ((613.973,   833.536), (619.241,   848.736)), # 128
        ((608.585,   819.28),  (613.973,   833.536)), # 129
        ((603.102,   805.925), (608.585,   819.28)),  # 130
        ((597.376,   793.056), (603.102,   805.925)), # 131
        ((591.604,   781.055), (597.376,   793.056)), # 132
        ((580.039,   759.469), (591.604,   781.055)), # 133
        ((568.522,   740.631), (580.039,   759.469)), # 134
        ((556.294,   722.923), (568.522,   740.631)), # 135
        ((543.143,   705.997), (556.294,   722.923)), # 136
        ((529.041,   689.827), (543.143,   705.997)), # 137
        ((513.96,    674.386), (529.041,   689.827)), # 138
        ((497.979,   659.741), (513.96,    674.386)), # 139
        ((478.316,   643.667), (497.979,   659.741)), # 140
        ((470.866,   636.596), (478.316,   643.667)), # 141
        ((465.018,   628.15),  (470.866,   636.596)), # 142
        ((461.023,   618.696), (465.018,   628.15)),  # 143
        ((459.043,   608.645), (461.023,   618.696)), # 144
        ((459.043,   608.645), (459.156,   598.386)), # 145  ← bottom turning point
        ((459.156,   598.386), (461.356,   588.377)), # 146
        ((461.356,   588.377), (465.555,   579.02)),  # 147
        ((465.555,   579.02),  (471.572,   570.72)),  # 148  → closes to edge 1 start
    ]
 
    # ──────────────────────────────────────────────────────────
    # Subtract (inner) profile edges  (start_xy, end_xy)
    # ──────────────────────────────────────────────────────────
    inner_edges = [
        ((622.712,  576.732),  (754.26,   405.075)),  # 1
        ((754.26,   405.075),  (755.793,  395.123)),  # 2
        ((755.793,  395.123),  (761.886,  395.123)),  # 3
        ((761.886,  395.123),  (815.726,  324.868)),  # 4
        ((815.726,  324.868),  (820.225,  320.234)),  # 5
        ((820.225,  320.234),  (825.69,   316.774)),  # 6
        ((825.69,   316.774),  (831.797,  314.678)),  # 7
        ((831.797,  314.678),  (838.242,  314.004)),  # 8
        ((838.242,  314.004),  (844.681,  314.727)),  # 9
        ((844.681,  314.727),  (850.858,  316.751)),  # 10
        ((850.858,  316.751),  (856.532,  319.936)),  # 11
        ((856.532,  319.936),  (861.515,  324.122)),  # 12
        ((861.515,  324.122),  (885.277,  347.489)),  # 13
        ((885.277,  347.489),  (910.961,  371.336)),  # 14
        ((910.961,  371.336),  (938.069,  395.123)),  # 15
        ((938.069,  395.123),  (948.95,   395.123)),  # 16
        ((948.95,   395.123),  (951.909,  395.355)),  # 17
        ((951.909,  395.355),  (954.455,  396.039)),  # 18
        ((954.455,  396.039),  (956.608,  397.151)),  # 19
        ((956.608,  397.151),  (958.399,  398.651)),  # 20
        ((958.399,  398.651),  (959.865,  400.495)),  # 21
        ((959.865,  400.495),  (961.044,  402.631)),  # 22
        ((961.044,  402.631),  (962.698,  407.578)),  # 23
        ((962.698,  407.578),  (964.147,  418.793)),  # 24
        ((964.147,  418.793),  (964.411,  429.7)),    # 25
        ((964.411,  429.7),    (965.795,  442.126)),  # 26
        ((965.795,  442.126),  (969.975,  453.909)),  # 27
        ((969.975,  453.909),  (976.729,  464.43)),   # 28
        ((976.729,  464.43),   (985.704,  473.134)),  # 29
        ((985.704,  473.134),  (996.426,  479.565)),  # 30
        ((996.426,  479.565),  (1008.332, 483.382)),  # 31
        ((1008.332, 483.382),  (1020.794, 484.387)),  # 32
        ((1020.794, 484.387),  (1033.157, 482.524)),  # 33
        ((1033.157, 482.524),  (1044.77,  477.893)),  # 34
        ((1044.77,  477.893),  (1055.023, 470.738)),  # 35
        ((1055.023, 470.738),  (1063.375, 461.434)),  # 36
        ((1063.375, 461.434),  (1069.387, 450.472)),  # 37
        ((1069.387, 450.472),  (1072.305, 441.872)),  # 38
        ((1072.305, 441.872),  (1074.429, 432.846)),  # 39
        ((1074.429, 432.846),  (1078.063, 414.92)),   # 40
        ((1078.063, 414.92),   (1080.825, 407.048)),  # 41
        ((1080.825, 407.048),  (1082.758, 403.652)),  # 42
        ((1082.758, 403.652),  (1085.186, 400.724)),  # 43
        ((1085.186, 400.724),  (1088.197, 398.342)),  # 44
        ((1088.197, 398.342),  (1091.873, 396.577)),  # 45
        ((1091.873, 396.577),  (1096.283, 395.491)),  # 46
        ((1096.283, 395.491),  (1101.474, 395.123)),  # 47
        ((1101.474, 395.123),  (1234.748, 395.123)),  # 48  ← bottom flat run
        ((1219.594, 448.619),  (1234.748, 395.123)),  # 49  ← notch turning vertex
        ((1219.594, 448.619),  (1239.492, 438.096)),  # 50
        ((1239.492, 438.096),  (1260.368, 429.109)),  # 51
        ((1260.368, 429.109),  (1282.067, 421.723)),  # 52
        ((1282.067, 421.723),  (1304.423, 415.996)),  # 53
        ((1304.423, 415.996),  (1327.269, 411.971)),  # 54
        ((1327.269, 411.971),  (1350.431, 409.677)),  # 55
        ((1350.431, 409.677),  (1373.737, 409.131)),  # 56
        ((1373.737, 409.131),  (1397.009, 410.339)),  # 57
        ((1397.009, 410.339),  (1420.072, 413.291)),  # 58
        ((1420.072, 413.291),  (1442.753, 417.965)),  # 59
        ((1442.753, 417.965),  (1464.879, 424.325)),  # 60
        ((1464.879, 424.325),  (1486.286, 432.324)),  # 61
        ((1486.286, 432.324),  (1506.81,  441.901)),  # 62
        ((1506.81,  441.901),  (1526.297, 452.984)),  # 63
        ((1526.297, 452.984),  (1544.601, 465.489)),  # 64
        ((1544.601, 465.489),  (1561.583, 479.322)),  # 65
        ((1561.583, 479.322),  (1577.115, 494.38)),   # 66
        ((1577.115, 494.38),   (1591.08,  510.547)),  # 67
        ((1591.08,  510.547),  (1603.373, 527.703)),  # 68
        ((1603.373, 527.703),  (1613.9,   545.717)),  # 69
        ((1613.9,   545.717),  (1622.584, 564.455)),  # 70
        ((1622.584, 564.455),  (1629.357, 583.774)),  # 71
        ((1629.357, 583.774),  (1634.17,  603.53)),   # 72
        ((1634.17,  603.53),   (1636.985, 623.572)),  # 73
        ((1636.985, 623.572),  (1637.782, 643.751)),  # 74
        ((1636.554, 663.913),  (1637.782, 643.751)),  # 75  ← turning vertex
        ((1633.311, 683.907),  (1636.554, 663.913)),  # 76
        ((1628.077, 703.582),  (1633.311, 683.907)),  # 77
        ((1620.892, 722.79),   (1628.077, 703.582)),  # 78
        ((1611.81,  741.385),  (1620.892, 722.79)),   # 79
        ((1600.899, 759.228),  (1611.81,  741.385)),  # 80
        ((1588.242, 776.184),  (1600.899, 759.228)),  # 81
        ((1588.242, 776.184),  (1594.611, 776.513)),  # 82  ← turning vertex
        ((1594.611, 776.513),  (1824.354, 776.513)),  # 83
        ((1824.354, 776.513),  (1840.807, 776.566)),  # 84
        ((1840.807, 776.566),  (1844.873, 776.876)),  # 85
        ((1844.873, 776.876),  (1848.492, 777.768)),  # 86
        ((1848.492, 777.768),  (1851.687, 779.209)),  # 87
        ((1851.687, 779.209),  (1854.491, 781.15)),   # 88
        ((1854.491, 781.15),   (1856.941, 783.535)),  # 89
        ((1856.941, 783.535),  (1859.078, 786.303)),  # 90
        ((1859.078, 786.303),  (1862.57,  792.732)),  # 91
        ((1862.57,  792.732),  (1867.409, 807.502)),  # 92
        ((1867.409, 807.502),  (1870.645, 822.36)),   # 93
        ((1870.645, 822.36),   (1874.131, 833.767)),  # 94
        ((1874.131, 833.767),  (1880.014, 844.143)),  # 95
        ((1880.014, 844.143),  (1888.015, 852.991)),  # 96
        ((1888.015, 852.991),  (1897.748, 859.886)),  # 97
        ((1897.748, 859.886),  (1908.748, 864.498)),  # 98
        ((1908.748, 864.498),  (1920.488, 866.607)),  # 99
        ((1920.488, 866.607),  (1932.406, 866.111)),  # 100
        ((1932.406, 866.111),  (1943.931, 863.035)),  # 101
        ((1943.931, 863.035),  (1950.392, 860.969)),  # 102
        ((1950.392, 860.969),  (1956.458, 859.93)),   # 103
        ((1956.458, 859.93),   (1962.137, 859.866)),  # 104
        ((1962.137, 859.866),  (1967.443, 860.709)),  # 105
        ((1967.443, 860.709),  (1972.388, 862.375)),  # 106
        ((1972.388, 862.375),  (1976.98,  864.779)),  # 107
        ((1976.98,  864.779),  (1985.115, 871.465)),  # 108
        ((1985.115, 871.465),  (1991.804, 880.155)),  # 109
        ((1991.804, 880.155),  (1996.901, 890.381)),  # 110
        ((1996.901, 890.381),  (2000.187, 901.836)),  # 111
        ((2000.187, 901.836),  (2001.44,  914.366)),  # 112
        ((2001.44,  914.366),  (2009.803, 1510.879)), # 113 ← long run right side
        ((2004.543, 1525.487), (2009.803, 1510.879)), # 114
        ((1998.778, 1539.05),  (2004.543, 1525.487)), # 115
        ((1992.421, 1551.384), (1998.778, 1539.05)),  # 116
        ((1985.306, 1562.144), (1992.421, 1551.384)), # 117
        ((1977.181, 1570.808), (1985.306, 1562.144)), # 118
        ((1972.642, 1574.151), (1977.181, 1570.808)), # 119
        ((1967.726, 1576.712), (1972.642, 1574.151)), # 120
        ((1962.388, 1578.397), (1967.726, 1576.712)), # 121
        ((1956.586, 1579.12),  (1962.388, 1578.397)), # 122
        ((1950.284, 1578.804), (1956.586, 1579.12)),  # 123
        ((1943.454, 1577.395), (1950.284, 1578.804)), # 124
        ((1930.962, 1575.458), (1943.454, 1577.395)), # 125
        ((1918.36,  1576.45),  (1930.962, 1575.458)), # 126
        ((1906.325, 1580.32),  (1918.36,  1576.45)),  # 127
        ((1895.505, 1586.857), (1906.325, 1580.32)),  # 128
        ((1886.483, 1595.712), (1895.505, 1586.857)), # 129
        ((1879.743, 1606.407), (1886.483, 1595.712)), # 130
        ((1875.649, 1618.368), (1879.743, 1606.407)), # 131
        ((1874.42,  1630.95),  (1875.649, 1618.368)), # 132
        ((1874.42,  1630.95),  (1876.124, 1643.476)), # 133 ← turning vertex
        ((1876.124, 1643.476), (1877.401, 1652.794)), # 134
        ((1876.419, 1661.458), (1877.401, 1652.794)), # 135
        ((1873.526, 1669.569), (1876.419, 1661.458)), # 136
        ((1869.125, 1677.243), (1873.526, 1669.569)), # 137
        ((1863.578, 1684.585), (1869.125, 1677.243)), # 138
        ((1857.171, 1691.679), (1863.578, 1684.585)), # 139
        ((1842.441, 1705.311), (1857.171, 1691.679)), # 140
        ((1840.188, 1706.168), (1842.441, 1705.311)), # 141
        ((1823.765, 1712.261), (1840.188, 1706.168)), # 142
        ((1805.491, 1718.942), (1823.765, 1712.261)), # 143
        ((1785.807, 1726.011), (1805.491, 1718.942)), # 144
        ((1765.154, 1733.269), (1785.807, 1726.011)), # 145
        ((1722.706, 1747.551), (1765.154, 1733.269)), # 146
        ((1701.794, 1754.176), (1722.706, 1747.551)), # 147
        ((1681.678, 1760.19),  (1701.794, 1754.176)), # 148
        ((1681.248, 1760.318), (1681.678, 1760.19)),  # 149
        ((1641.225, 1772.093), (1681.248, 1760.318)), # 150
        ((1599.91,  1783.304), (1641.225, 1772.093)), # 151
        ((1557.947, 1793.565), (1599.91,  1783.304)), # 152
        ((1515.984, 1802.486), (1557.947, 1793.565)), # 153
        ((1482.385, 1808.485), (1515.984, 1802.486)), # 154
        ((1448.907, 1813.311), (1482.385, 1808.485)), # 155
        ((1442.458, 1809.252), (1448.907, 1813.311)), # 156
        ((1436.351, 1804.436), (1442.458, 1809.252)), # 157
        ((1430.601, 1798.828), (1436.351, 1804.436)), # 158
        ((1425.223, 1792.397), (1430.601, 1798.828)), # 159
        ((1420.229, 1785.117), (1425.223, 1792.397)), # 160
        ((1415.629, 1776.963), (1420.229, 1785.117)), # 161
        ((1411.432, 1767.918), (1415.629, 1776.963)), # 162
        ((1407.643, 1757.969), (1411.432, 1767.918)), # 163
        ((1402.25,  1746.708), (1407.643, 1757.969)), # 164
        ((1394.436, 1736.969), (1402.25,  1746.708)), # 165
        ((1384.61,  1729.265), (1394.436, 1736.969)), # 166
        ((1373.289, 1723.999), (1384.61,  1729.265)), # 167
        ((1361.067, 1721.448), (1373.289, 1723.999)), # 168
        ((1348.585, 1721.745), (1361.067, 1721.448)), # 169
        ((1336.498, 1724.875), (1348.585, 1721.745)), # 170
        ((1325.44,  1730.674), (1336.498, 1724.875)), # 171
        ((1315.993, 1738.838), (1325.44,  1730.674)), # 172
        ((1308.651, 1748.937), (1315.993, 1738.838)), # 173
        ((1303.801, 1760.442), (1308.651, 1748.937)), # 174
        ((1301.696, 1772.749), (1303.801, 1760.442)), # 175
        ((1301.277, 1785.885), (1301.696, 1772.749)), # 176
        ((1301.277, 1785.885), (1301.503, 1798.082)), # 177 ← turning vertex
        ((1301.503, 1798.082), (1302.371, 1809.348)), # 178
        ((1302.371, 1809.348), (1303.871, 1819.694)), # 179
        ((813.61,   1819.694), (1303.871, 1819.694)), # 180 ← top flat run
        ((798.88,   1818.326), (813.61,   1819.694)), # 181
        ((784.659,  1814.272), (798.88,   1818.326)), # 182
        ((771.417,  1807.663), (784.659,  1814.272)), # 183
        ((759.633,  1798.741), (771.417,  1807.663)), # 184
        ((749.674,  1787.78),  (759.633,  1798.741)), # 185
        ((741.913,  1775.185), (749.674,  1787.78)),  # 186
        ((736.601,  1761.366), (741.913,  1775.185)), # 187
        ((733.927,  1746.81),  (736.601,  1761.366)), # 188
        ((724.239,  1629.208), (733.927,  1746.81)),  # 189
        ((716.14,   1511.698), (724.239,  1629.208)), # 190
        ((716.14,   1465.52),  (716.14,   1511.698)), # 191
        ((716.14,   1465.52),  (717.387,  1453.452)), # 192 ← turning vertex
        ((717.387,  1453.452), (720.984,  1442.723)), # 193
        ((720.984,  1442.723), (726.608,  1433.273)), # 194
        ((726.608,  1433.273), (733.905,  1425.093)), # 195
        ((733.905,  1425.093), (742.577,  1418.261)), # 196
        ((742.577,  1418.261), (752.422,  1412.95)),  # 197
        ((752.422,  1412.95),  (763.339,  1409.397)), # 198
        ((763.339,  1409.397), (775.306,  1407.837)), # 199
        ((775.306,  1407.837), (787.944,  1405.817)), # 200
        ((787.944,  1405.817), (799.763,  1400.906)), # 201
        ((799.763,  1400.906), (810.111,  1393.373)), # 202
        ((810.111,  1393.373), (818.417,  1383.635)), # 203
        ((818.417,  1383.635), (824.222,  1372.228)), # 204
        ((824.222,  1372.228), (827.207,  1359.782)), # 205
        ((827.207,  1346.983), (827.207,  1359.782)), # 206 ← turning vertex
        ((824.222,  1334.537), (827.207,  1346.983)), # 207
        ((818.417,  1323.13),  (824.222,  1334.537)), # 208
        ((810.111,  1313.392), (818.417,  1323.13)),  # 209
        ((799.763,  1305.859), (810.111,  1313.392)), # 210
        ((787.944,  1300.948), (799.763,  1305.859)), # 211
        ((775.306,  1298.928), (787.944,  1300.948)), # 212
        ((762.629,  1297.069), (775.306,  1298.928)), # 213
        ((751.387,  1292.743), (762.629,  1297.069)), # 214
        ((741.523,  1286.299), (751.387,  1292.743)), # 215
        ((733.036,  1278.129), (741.523,  1286.299)), # 216
        ((726.025,  1268.569), (733.036,  1278.129)), # 217
        ((720.691,  1257.858), (726.025,  1268.569)), # 218
        ((717.308,  1246.131), (720.691,  1257.858)), # 219
        ((716.14,   1233.434), (717.308,  1246.131)), # 220
        ((716.14,    982.59),  (716.14,   1233.434)), # 221
        ((716.14,    982.59),  (717.593,   968.542)), # 222 ← turning vertex
        ((717.593,   968.542), (721.787,   955.661)), # 223
        ((721.787,   955.661), (728.35,    944.013)), # 224
        ((728.35,    944.013), (736.878,   933.77)),  # 225
        ((736.878,   933.77),  (747.035,   925.235)), # 226
        ((747.035,   925.235), (758.598,   918.832)), # 227
        ((758.598,   918.832), (771.463,   915.047)), # 228
        ((771.463,   915.047), (778.377,   914.281)), # 229
        ((778.377,   914.281), (785.615,   914.319)), # 230
        ((785.615,   914.319), (798.229,   913.617)), # 231
        ((798.229,   913.617), (810.341,   910.029)), # 232
        ((810.341,   910.029), (821.303,   903.748)), # 233
        ((821.303,   903.748), (830.524,   895.113)), # 234
        ((830.524,   895.113), (837.509,   884.587)), # 235
        ((837.509,   884.587), (841.882,   872.735)), # 236
        ((841.882,   872.735), (843.41,    860.195)), # 237
        ((842.009,   847.639), (843.41,    860.195)), # 238
        ((837.756,   835.744), (842.009,   847.639)), # 239
        ((830.877,   825.148), (837.756,   835.744)), # 240
        ((821.744,   816.42),  (830.877,   825.148)), # 241
        ((810.847,   810.029), (821.744,   816.42)),  # 242
        ((798.771,   806.319), (810.847,   810.029)), # 243
        ((786.165,   805.488), (798.771,   806.319)), # 244
        ((773.707,   807.583), (786.165,   805.488)), # 245
        ((759.244,   810.898), (773.707,   807.583)), # 246
        ((744.717,   812.572), (759.244,   810.898)), # 247
        ((730.136,   812.858), (744.717,   812.572)), # 248
        ((715.513,   812.046), (730.136,   812.858)), # 249
        ((686.176,   808.135), (715.513,   812.046)), # 250
        ((656.761,   802.206), (686.176,   808.135)), # 251
        ((645.097,   791.625), (656.761,   802.206)), # 252
        ((634.421,   780.067), (645.097,   791.625)), # 253
        ((624.8,     767.618), (634.421,   780.067)), # 254
        ((616.3,     754.366), (624.8,     767.618)), # 255
        ((608.992,   740.408), (616.3,     754.366)), # 256
        ((602.96,    725.881), (608.992,   740.408)), # 257
        ((598.257,   710.889), (602.96,    725.881)), # 258
        ((594.939,   695.532), (598.257,   710.889)), # 259
        ((593.057,   679.913), (594.939,   695.532)), # 260
        ((592.667,   664.21),  (593.057,   679.913)), # 261
        ((592.667,   664.21),  (593.802,   648.572)), # 262 ← turning vertex
        ((593.802,   648.572), (596.492,   633.121)), # 263
        ((596.492,   633.121), (600.757,   618.007)), # 264
        ((600.757,   618.007), (606.565,   603.478)), # 265
        ((606.565,   603.478), (613.891,   589.677)), # 266
        ((613.891,   589.677), (622.712,   576.732)), # 267 → closes to edge 1 start
    ]
 
    # ── Build closed wires from edge lists ─────────────────────
    def build_wire(edge_pairs):
        segs = []
        for (sx, sy), (ex, ey) in edge_pairs:
            # Guard against degenerate edges
            if abs(sx - ex) < 1e-9 and abs(sy - ey) < 1e-9:
                continue
            segs.append(
                Edge.make_line(Vector(sx, sy, 0.0), Vector(ex, ey, 0.0))
            )
        return Wire(segs)
 
    print("  Building outer wire...")
    outer_wire = build_wire(outer_edges)
    print(f"    outer wire valid={outer_wire.is_valid}")
 
    print("  Building inner wire...")
    inner_wire = build_wire(inner_edges)
    print(f"    inner wire valid={inner_wire.is_valid}")
 
    # ── 6 additional subtract hole profiles (24 edges each) ────
    hole_edge_groups = [
        # hole 1 — center ~(1924, 812)
        [
            ((1909.446, 786.685),  (1916.561, 783.738)),
            ((1916.561, 783.738),  (1924.196, 782.733)),
            ((1924.196, 782.733),  (1931.831, 783.738)),
            ((1931.831, 783.738),  (1938.946, 786.685)),
            ((1938.946, 786.685),  (1945.055, 791.374)),
            ((1945.055, 791.374),  (1949.743, 797.483)),
            ((1949.743, 797.483),  (1952.691, 804.598)),
            ((1952.691, 804.598),  (1953.696, 812.233)),
            ((1952.691, 819.868),  (1953.696, 812.233)),
            ((1949.743, 826.983),  (1952.691, 819.868)),
            ((1945.055, 833.093),  (1949.743, 826.983)),
            ((1938.946, 837.781),  (1945.055, 833.093)),
            ((1931.831, 840.728),  (1938.946, 837.781)),
            ((1924.196, 841.733),  (1931.831, 840.728)),
            ((1916.561, 840.728),  (1924.196, 841.733)),
            ((1909.446, 837.781),  (1916.561, 840.728)),
            ((1903.336, 833.093),  (1909.446, 837.781)),
            ((1898.648, 826.983),  (1903.336, 833.093)),
            ((1895.701, 819.868),  (1898.648, 826.983)),
            ((1894.696, 812.233),  (1895.701, 819.868)),
            ((1894.696, 812.233),  (1895.701, 804.598)),
            ((1895.701, 804.598),  (1898.648, 797.483)),
            ((1898.648, 797.483),  (1903.336, 791.374)),
            ((1903.336, 791.374),  (1909.446, 786.685)),
        ],
        # hole 2 — center ~(1929, 1630)
        [
            ((1949.77,  1609.059), (1954.458, 1615.169)),
            ((1954.458, 1615.169), (1957.406, 1622.284)),
            ((1957.406, 1622.284), (1958.411, 1629.919)),
            ((1957.406, 1637.554), (1958.411, 1629.919)),
            ((1954.458, 1644.669), (1957.406, 1637.554)),
            ((1949.77,  1650.779), (1954.458, 1644.669)),
            ((1943.661, 1655.467), (1949.77,  1650.779)),
            ((1936.546, 1658.414), (1943.661, 1655.467)),
            ((1928.911, 1659.419), (1936.546, 1658.414)),
            ((1921.276, 1658.414), (1928.911, 1659.419)),
            ((1914.161, 1655.467), (1921.276, 1658.414)),
            ((1908.051, 1650.779), (1914.161, 1655.467)),
            ((1903.363, 1644.669), (1908.051, 1650.779)),
            ((1900.416, 1637.554), (1903.363, 1644.669)),
            ((1899.411, 1629.919), (1900.416, 1637.554)),
            ((1899.411, 1629.919), (1900.416, 1622.284)),
            ((1900.416, 1622.284), (1903.363, 1615.169)),
            ((1903.363, 1615.169), (1908.051, 1609.059)),
            ((1908.051, 1609.059), (1914.161, 1604.371)),
            ((1914.161, 1604.371), (1921.276, 1601.424)),
            ((1921.276, 1601.424), (1928.911, 1600.419)),
            ((1928.911, 1600.419), (1936.546, 1601.424)),
            ((1936.546, 1601.424), (1943.661, 1604.371)),
            ((1943.661, 1604.371), (1949.77,  1609.059)),
        ],
        # hole 3 — center ~(1356, 1776)
        [
            ((1376.975, 1754.863), (1381.663, 1760.972)),
            ((1381.663, 1760.972), (1384.61,  1768.087)),
            ((1384.61,  1768.087), (1385.615, 1775.722)),
            ((1384.61,  1783.357), (1385.615, 1775.722)),
            ((1381.663, 1790.472), (1384.61,  1783.357)),
            ((1376.975, 1796.582), (1381.663, 1790.472)),
            ((1370.865, 1801.27),  (1376.975, 1796.582)),
            ((1363.75,  1804.217), (1370.865, 1801.27)),
            ((1356.115, 1805.222), (1363.75,  1804.217)),
            ((1348.48,  1804.217), (1356.115, 1805.222)),
            ((1341.365, 1801.27),  (1348.48,  1804.217)),
            ((1335.255, 1796.582), (1341.365, 1801.27)),
            ((1330.567, 1790.472), (1335.255, 1796.582)),
            ((1327.62,  1783.357), (1330.567, 1790.472)),
            ((1326.615, 1775.722), (1327.62,  1783.357)),
            ((1326.615, 1775.722), (1327.62,  1768.087)),
            ((1327.62,  1768.087), (1330.567, 1760.972)),
            ((1330.567, 1760.972), (1335.255, 1754.863)),
            ((1335.255, 1754.863), (1341.365, 1750.175)),
            ((1341.365, 1750.175), (1348.48,  1747.227)),
            ((1348.48,  1747.227), (1356.115, 1746.222)),
            ((1356.115, 1746.222), (1363.75,  1747.227)),
            ((1363.75,  1747.227), (1370.865, 1750.175)),
            ((1370.865, 1750.175), (1376.975, 1754.863)),
        ],
        # hole 4 — center ~(773, 1353)
        [
            ((780.719, 1324.888), (787.834, 1327.835)),
            ((787.834, 1327.835), (793.944, 1332.523)),
            ((793.944, 1332.523), (798.632, 1338.633)),
            ((798.632, 1338.633), (801.579, 1345.747)),
            ((801.579, 1345.747), (802.584, 1353.383)),
            ((801.579, 1361.018), (802.584, 1353.383)),
            ((798.632, 1368.133), (801.579, 1361.018)),
            ((793.944, 1374.242), (798.632, 1368.133)),
            ((787.834, 1378.93),  (793.944, 1374.242)),
            ((780.719, 1381.877), (787.834, 1378.93)),
            ((773.084, 1382.883), (780.719, 1381.877)),
            ((765.449, 1381.877), (773.084, 1382.883)),
            ((758.334, 1378.93),  (765.449, 1381.877)),
            ((752.224, 1374.242), (758.334, 1378.93)),
            ((747.536, 1368.133), (752.224, 1374.242)),
            ((744.589, 1361.018), (747.536, 1368.133)),
            ((743.584, 1353.383), (744.589, 1361.018)),
            ((743.584, 1353.383), (744.589, 1345.747)),
            ((744.589, 1345.747), (747.536, 1338.633)),
            ((747.536, 1338.633), (752.224, 1332.523)),
            ((752.224, 1332.523), (758.334, 1327.835)),
            ((758.334, 1327.835), (765.449, 1324.888)),
            ((765.449, 1324.888), (773.084, 1323.882)),
            ((773.084, 1323.882), (780.719, 1324.888)),
        ],
        # hole 5 — center ~(789, 860)
        [
            ((803.661, 834.371),  (809.77,  839.059)),
            ((809.77,  839.059),  (814.458, 845.169)),
            ((814.458, 845.169),  (817.406, 852.284)),
            ((817.406, 852.284),  (818.411, 859.919)),
            ((817.406, 867.554),  (818.411, 859.919)),
            ((814.458, 874.669),  (817.406, 867.554)),
            ((809.77,  880.779),  (814.458, 874.669)),
            ((803.661, 885.467),  (809.77,  880.779)),
            ((796.546, 888.414),  (803.661, 885.467)),
            ((788.911, 889.419),  (796.546, 888.414)),
            ((781.276, 888.414),  (788.911, 889.419)),
            ((774.161, 885.467),  (781.276, 888.414)),
            ((768.051, 880.779),  (774.161, 885.467)),
            ((763.363, 874.669),  (768.051, 880.779)),
            ((760.416, 867.554),  (763.363, 874.669)),
            ((759.411, 859.919),  (760.416, 867.554)),
            ((759.411, 859.919),  (760.416, 852.284)),
            ((760.416, 852.284),  (763.363, 845.169)),
            ((763.363, 845.169),  (768.051, 839.059)),
            ((768.051, 839.059),  (774.161, 834.371)),
            ((774.161, 834.371),  (781.276, 831.424)),
            ((781.276, 831.424),  (788.911, 830.419)),
            ((788.911, 830.419),  (796.546, 831.424)),
            ((796.546, 831.424),  (803.661, 834.371)),
        ],
        # hole 6 — center ~(1019, 430)
        [
            ((998.051,  409.059),  (1004.161, 404.371)),
            ((1004.161, 404.371),  (1011.276, 401.424)),
            ((1011.276, 401.424),  (1018.911, 400.419)),
            ((1018.911, 400.419),  (1026.546, 401.424)),
            ((1026.546, 401.424),  (1033.661, 404.371)),
            ((1033.661, 404.371),  (1039.77,  409.059)),
            ((1039.77,  409.059),  (1044.458, 415.169)),
            ((1044.458, 415.169),  (1047.406, 422.284)),
            ((1047.406, 422.284),  (1048.411, 429.919)),
            ((1047.406, 437.554),  (1048.411, 429.919)),
            ((1044.458, 444.669),  (1047.406, 437.554)),
            ((1039.77,  450.779),  (1044.458, 444.669)),
            ((1033.661, 455.467),  (1039.77,  450.779)),
            ((1026.546, 458.414),  (1033.661, 455.467)),
            ((1018.911, 459.419),  (1026.546, 458.414)),
            ((1011.276, 458.414),  (1018.911, 459.419)),
            ((1004.161, 455.467),  (1011.276, 458.414)),
            ((998.051,  450.779),  (1004.161, 455.467)),
            ((993.363,  444.669),  (998.051,  450.779)),
            ((990.416,  437.554),  (993.363,  444.669)),
            ((989.411,  429.919),  (990.416,  437.554)),
            ((989.411,  429.919),  (990.416,  422.284)),
            ((990.416,  422.284),  (993.363,  415.169)),
            ((993.363,  415.169),  (998.051,  409.059)),
        ],
    ]
 
    hole_wires = []
    for i, hg in enumerate(hole_edge_groups):
        w = build_wire(hg)
        print(f"    hole {i+1} wire valid={w.is_valid}")
        hole_wires.append(w)
 
    # ── Face with holes, then extrude ──────────────────────────
    print("  Building face with inner subtract...")
    with BuildPart() as part:
        with BuildSketch(Plane.XY):
            add(Face(outer_wire))
            add(Face(inner_wire), mode=Mode.SUBTRACT)
            for hw in hole_wires:
                add(Face(hw), mode=Mode.SUBTRACT)
        extrude(amount=50.0)
 
    s = part.part
    print(f"  Body valid={s.is_valid}, volume={s.volume:.1f} mm³")
 
    if s.volume < 0:
        print("  Volume negative — reversing extrude direction...")
        with BuildPart() as part:
            with BuildSketch(Plane.XY):
                add(Face(outer_wire))
                add(Face(inner_wire), mode=Mode.SUBTRACT)
                for hw in hole_wires:
                    add(Face(hw), mode=Mode.SUBTRACT)
            extrude(amount=-50.0)
        s = part.part
        print(f"  Body (flipped) valid={s.is_valid}, volume={s.volume:.1f} mm³")
 
    return s
 
 
body = make_body()

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
    add(frame20.part)
    add(frame21.part)
    add(frame22.part)
    add(frame23.part)
    add(frame24.part)
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
    add(loft22)
    add(loft23)
    add(loft24)
    add(loft25)
    add(loft26)
    add(loft27)
    add(loft28)
    add(loft29)
    add(loft30)
    add(loft31)
    add(loft32)
    add(loft32.translate((0, -0.01, 0)))
    add(loft33)
    add(loft34)
    add(loft35)
    add(loft36)
    add(loft37)
    add(loft38)
    add(loft39)
    add(loft40)
    add(loft41)
    add(loft42)
    add(loft43)
    add(loft44)
    add(loft45)
    add(loft46)
    add(loft47)
    add(loft48)
    add(loft49)
    add(loft50)
    add(loft51)
    add(loft52)
    add(loft53)
    add(loft54)
    add(loft55)
    add(loft56)
    add(body)
print(f"  Final valid={part.part.is_valid}, volume={part.part.volume:.1f}")

try:
    from ocp_vscode import show
    show(part)
except ImportError:
    pass

export_stl(part.part, "output_frame_block.stl")
print("Done.")