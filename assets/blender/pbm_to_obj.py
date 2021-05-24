import os
import bpy
import bmesh


def curve_to_mesh(context, curve):
    deg = context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(
        curve.evaluated_get(deg), depsgraph=deg)

    new_obj = bpy.data.objects.new(curve.name + "_mesh", me)
    context.collection.objects.link(new_obj)

    for o in context.selected_objects:
        o.select_set(False)

    new_obj.matrix_world = curve.matrix_world
    new_obj.select_set(True)
    context.view_layer.objects.active = new_obj


bpy.ops.object.select_all(action='SELECT')

bpy.ops.object.delete()

path_to_obj_dir = os.path.dirname(os.path.abspath(__file__))

file_list = sorted(os.listdir())
pbm_list = [item for item in file_list if item.endswith('.pbm')]

print("file ", file_list)
for name in pbm_list:
    os.system(f'potrace {name} --svg  ')

file_list = sorted(os.listdir())
svg_list = [item for item in file_list if item.endswith('.svg')]
print("sg list", svg_list)


for item in svg_list:
    path_to_file = os.path.join(path_to_obj_dir, item)
    bpy.ops.import_curve.svg(filepath=path_to_file)

    objects = bpy.context.scene.objects

    for obj in objects:
        obj.select_set(state=True)

    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.join()

    context = bpy.context

    if obj and obj.type == 'CURVE':
        curve_to_mesh(context, obj)

    bpy.ops.object.editmode_toggle()
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.extrude_region_move(MESH_OT_extrude_region={"use_normal_flip": False, "use_dissolve_ortho_edges": False, "mirror": False}, TRANSFORM_OT_translate={"value": (0, 0, 0), "orient_type": 'GLOBAL', "orient_matrix": ((0, 0, 0), (0, 0, 0), (0, 0, 0)), "orient_matrix_type": 'GLOBAL', "constraint_axis": (False, False, False), "mirror": False, "use_proportional_edit": False, "proportional_edit_falloff": 'SMOOTH',
                                                                                                                                                                    "proportional_size": 1, "use_proportional_connected": False, "use_proportional_projected": False, "snap": False, "snap_target": 'CLOSEST', "snap_point": (0, 0, 0), "snap_align": False, "snap_normal": (0, 0, 0), "gpencil_strokes": False, "cursor_transform": False, "texture_space": False, "remove_on_cancel": False, "release_confirm": False, "use_accurate": False, "use_automerge_and_split": False})
    bpy.ops.mesh.extrude_region_move(MESH_OT_extrude_region={"use_normal_flip": False, "use_dissolve_ortho_edges": False, "mirror": False}, TRANSFORM_OT_translate={"value": (0, 0, 0.0030792), "orient_type": 'NORMAL', "orient_matrix": ((0.984234, -0.176872, 0), (0.176872, 0.984234, -0), (0, 0, 1)), "orient_matrix_type": 'NORMAL', "constraint_axis": (False, False, True), "mirror": False, "use_proportional_edit": False,
                                                                                                                                                                    "proportional_edit_falloff": 'SMOOTH', "proportional_size": 1, "use_proportional_connected": False, "use_proportional_projected": False, "snap": False, "snap_target": 'CLOSEST', "snap_point": (0, 0, 0), "snap_align": False, "snap_normal": (0, 0, 0), "gpencil_strokes": False, "cursor_transform": False, "texture_space": False, "remove_on_cancel": False, "release_confirm": False, "use_accurate": False, "use_automerge_and_split": False})

    bpy.ops.mesh.select_all(action='DESELECT')

    #dims = bpy.context.object.dimensions
    #multi = (5)/dims[0]
    bpy.context.object.dimensions = 1, 1, 0.2

    print(path_to_file)
    dest_path = bpy.path.ensure_ext(os.path.splitext(path_to_file)[0], '.obj')

    bpy.ops.export_scene.obj(filepath=dest_path, axis_forward='Y', axis_up='Z')
    bpy.ops.object.select_all(action='SELECT')

    # delete all objects
    bpy.ops.object.delete()
