import bpy
import sys

argv = sys.argv
argv = argv[argv.index("--") + 1:] # get all args after "--"

blender_in = argv[0]
obj_out = argv[1]

bpy.ops.wm.open_mainfile(filepath=blender_in)

filename = obj_out + ".obj"

bpy.ops.export_scene.obj(filepath=filename, check_existing=True, filter_glob="*.obj;*.mtl", use_selection=False, use_animation=False, use_mesh_modifiers=False, use_edges=False, use_smooth_groups=False, use_smooth_groups_bitflags=False, use_normals=True, use_uvs=True, use_materials=True, use_triangles=False, use_nurbs=False, use_vertex_groups=False, use_blen_objects=True, group_by_object=False, group_by_material=False, keep_vertex_order=False, global_scale=1.0, path_mode='AUTO', axis_forward='-Z', axis_up='Y')
