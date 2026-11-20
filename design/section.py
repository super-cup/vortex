import numpy as np, trimesh, os
from shapely.geometry import Polygon
from params import *

OUT = os.path.join(os.path.dirname(__file__), "..", "stl")
body = trimesh.load(os.path.join(OUT, "vortex_funnel.stl"), force="mesh")
# 90° 楔形切除，露出内部
th = np.linspace(0, np.pi/2, 36)
pts = [(0, 0)] + [(70*np.cos(t), 70*np.sin(t)) for t in th]
wedge = trimesh.creation.extrude_polygon(Polygon(pts), height=90)
cut = trimesh.boolean.difference([body, wedge], engine="manifold")
cut.export(os.path.join(OUT, "vortex_funnel_section.stl"))
print("section", len(cut.faces), "bounds", np.round(cut.bounds,1).tolist())

def bead_at(v, dth=np.pi):
    th = 2*np.pi*TURNS*v + dth
    r = r_cone(v) - RAIL_GAP/2
    z = H*(1-v) + 3.0
    b = trimesh.creation.icosphere(subdivisions=3, radius=BEAD_R)
    b.apply_translation([r*np.cos(th), r*np.sin(th), z])
    return b

for i, v in enumerate([0.22, 0.48, 0.74]):
    b = bead_at(v)
    b.export(os.path.join(OUT, f"vortex_bead_{i+1}.stl"))
    print("bead", i+1, "v", v, "z", round(b.bounds[1][2],1))
