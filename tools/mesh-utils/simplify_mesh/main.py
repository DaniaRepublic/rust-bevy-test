import open3d as o3d

model_path = "/Users/markinivan/Programming/probe/game-examples-bevy/assets/models/WoodenModularHuts/floor1.004.glb"

mesh = o3d.io.read_triangle_mesh(model_path)
print("read mesh")
mesh_smp = mesh.simplify_quadric_decimation(target_number_of_triangles=140)
print("simplified mesh")
o3d.io.write_triangle_mesh("output.glb", mesh_smp)
