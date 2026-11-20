import cadquery as cq
import numpy as np, trimesh, os
from params import *

OUT = os.path.join(os.path.dirname(__file__), "..", "stl")
os.makedirs(OUT, exist_ok=True)


def funnel_mesh(nt=480, nv=120):
    """空心锥壳：外表面 + 内表面 + 顶环 + 底环。"""
    th = np.linspace(0, 2*np.pi, nt, endpoint=False)
    vs = np.linspace(0, 1, nv+1)
    TH, V = np.meshgrid(th, vs, indexing="ij")   # (nt, nv+1)
    Rout = r_cone(V)
    Z = H*(1-V)
    xo, yo = Rout*np.cos(TH), Rout*np.sin(TH)
    ri = Rout - WALL
    xi, yi = ri*np.cos(TH), ri*np.sin(TH)

    def grid(x, y, z):
        return np.stack([x, y, z], -1)

    top_o, top_i = grid(xo, yo, Z), grid(xi, yi, Z)
    bot_o, bot_i = grid(xo, yo, Z), grid(xi, yi, Z)
    # 顶点顺序：外表面、内表面
    n = nt*(nv+1)
    Vv = np.concatenate([grid(xo, yo, Z).reshape(-1, 3),
                         grid(xi, yi, Z).reshape(-1, 3)], 0)
    F = []

    def o(i, j): return i*(nv+1) + j
    def inn(i, j): return n + i*(nv+1) + j

    for i in range(nt):
        i2 = (i+1) % nt
        for j in range(nv):
            # 外表面（法线朝外）
            F.append([o(i, j), o(i2, j), o(i2, j+1)])
            F.append([o(i, j), o(i2, j+1), o(i, j+1)])
            # 内表面（法线朝内）
            F.append([inn(i, j), inn(i2, j+1), inn(i2, j)])
            F.append([inn(i, j), inn(i, j+1), inn(i2, j+1)])
        # 顶环（z=H）
        F.append([o(i, 0), inn(i, 0), inn(i2, 0)])
        F.append([o(i, 0), inn(i2, 0), o(i2, 0)])
        # 底环（z=0）
        F.append([o(i, nv), o(i2, nv), inn(i2, nv)])
        F.append([o(i, nv), inn(i2, nv), inn(i, nv)])
    m = trimesh.Trimesh(Vv, np.array(F), process=True)
    trimesh.repair.fix_normals(m)
    return m


def rail_mesh(offset, nt=720, nseg=6):
    """沿内壁螺旋扫掠的矩形墙（护栏）。

    offset = 0    → 外护栏：从内壁向内凸出 RAIL_T
    offset = GAP  → 内护栏：再向内 RAIL_GAP
    """
    vs = np.linspace(0, 1, nt)
    th = theta_s(vs)
    Rc = r_cone(vs)
    Rrail = Rc - WALL - RAIL_T/2 - offset
    Z = H*(1-vs)
    # 路径点
    P = np.stack([Rrail*np.cos(th), Rrail*np.sin(th), Z], 1)
    # 径向与竖直方向
    Rd = np.stack([np.cos(th), np.sin(th), np.zeros(nt)], 1)
    Zd = np.tile([0.0, 0.0, 1.0], (nt, 1))
    # 截面矩形：径向 ±RAIL_T/2，z 从 0 到 RAIL_H（贴着锥面，向下嵌入 0.6）
    corners = [(-RAIL_T/2, -0.6), (RAIL_T/2, -0.6), (RAIL_T/2, RAIL_H), (-RAIL_T/2, RAIL_H)]
    rings = [P + a*Rd + b*Zd for a, b in corners]
    V = np.concatenate(rings, 0)
    m = 4
    F = []
    def idx(k, i): return k*nt + (i % nt)
    for i in range(nt):
        j = i + 1
        for k in range(m):
            b = (k+1) % m
            F.append([idx(k, i), idx(k, j), idx(b, j)])
            F.append([idx(k, i), idx(b, j), idx(b, i)])
    mesh = trimesh.Trimesh(V, np.array(F), process=True)
    trimesh.repair.fix_normals(mesh)
    return mesh


def plate():
    import cadquery as cq
    p = cq.Workplane("XY").circle(PLATE_R).extrude(PLATE_H)
    p = p.faces(">Z").workplane().circle(PLATE_R - 12).cutBlind(-3.0)
    return p


def build_body():
    f = funnel_mesh()
    r1 = rail_mesh(0.0)
    r2 = rail_mesh(RAIL_GAP)
    return trimesh.boolean.union([f, r1, r2], engine="manifold")


if __name__ == "__main__":
    f = funnel_mesh()
    print("funnel", len(f.faces), f.is_watertight, round(f.volume/1000, 1), "cm3")
    r1 = rail_mesh(0.0)
    r2 = rail_mesh(RAIL_GAP)
    print("rails", len(r1.faces), r1.is_watertight, len(r2.faces), r2.is_watertight)
    body = build_body()
    print("body", len(body.faces), body.is_watertight, round(body.volume/1000, 1), "cm3",
          "bounds", np.round(body.bounds, 1).tolist())
    body.export(os.path.join(OUT, "vortex_funnel.stl"))
    b = trimesh.creation.icosphere(subdivisions=3, radius=BEAD_R)
    b.export(os.path.join(OUT, "vortex_bead.stl"))
    p = plate()
    cq.exporters.export(p, os.path.join(OUT, "vortex_plate.stl"), tolerance=0.02, angularTolerance=0.3)
    print("plate ok")
