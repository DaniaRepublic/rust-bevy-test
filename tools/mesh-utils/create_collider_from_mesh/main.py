import os
import shutil

import coacd
import trimesh

model_path = "/Users/markinivan/Programming/probe/game-examples-bevy/assets/models/WoodenModularHuts/Mesh_0.glb"

save_path = "./assets/models/destructible/" + model_path.split("/")[-1].split(".")[0]
try:
    shutil.rmtree(save_path)
except:
    pass
os.mkdir(save_path)

mesh = trimesh.load_mesh(model_path)
mesh = coacd.Mesh(vertices=mesh.vertices, indices=mesh.faces)

parts = coacd.run_coacd(
    mesh,
    # 1. Force a maximum number of hulls (Set to your target: 13)
    max_convex_hull=13,
    # 2. Increase threshold to allow "uglier" merges (Default is 0.05).
    #    Try 0.1 or 0.15 if 13 hulls look too distorted.
    threshold=0.1,
    # 3. (Optional) Lower resolution to ignore small details like bolts/bevels
    #    that might force a split. Default is 2000.
    resolution=500
)

print(f"coacd produced {len(parts)} parts.")

for i, part in enumerate(parts):
    vertices = part[0]
    faces = part[1]
    shard = trimesh.Trimesh(vertices=vertices, faces=faces)
    shard.export(f"{save_path}/shard.{i+1}.glb")
