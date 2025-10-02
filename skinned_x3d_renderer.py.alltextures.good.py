# --- START OF FILE skinned_x3d_renderer.py ---

import x3d
import pynari as anari
import numpy as np
from PIL import Image
import math
import time
import os
import base64
import io

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def unroll_indexed_face_set(positions, texcoords, pos_indices_str, tex_indices_str, shape_name, log_info, all_images_for_logging={}):
    if not pos_indices_str: return None, None, None, None
    use_tex_indices = tex_indices_str is not None and len(tex_indices_str) > 0
    final_positions, final_texcoords, original_indices_map = [], [], []
    pos_polygons, current_pos_poly = [], []
    for idx in pos_indices_str:
        if idx == -1:
            if len(current_pos_poly) >= 3: pos_polygons.append(current_pos_poly)
            current_pos_poly = []
        else: current_pos_poly.append(idx)
    if len(current_pos_poly) >= 3: pos_polygons.append(current_pos_poly)
    tex_polygons = []
    indices_to_use_for_tex = tex_indices_str if use_tex_indices else pos_indices_str
    if indices_to_use_for_tex:
        current_tex_poly = []
        for idx in indices_to_use_for_tex:
            if idx == -1:
                if len(current_tex_poly) >= 3: tex_polygons.append(current_tex_poly)
                current_tex_poly = []
            else: current_tex_poly.append(idx)
        if len(current_tex_poly) >= 3: tex_polygons.append(current_tex_poly)
    if len(pos_polygons) != len(tex_polygons): tex_polygons = pos_polygons
    vertex_count, total_triangles = 0, 0
    for poly_idx, (pos_poly, tex_poly) in enumerate(zip(pos_polygons, tex_polygons)):
        for i in range(1, len(pos_poly) - 1):
            total_triangles += 1
            triangle_pos_indices = [pos_poly[0], pos_poly[i], pos_poly[i+1]]
            triangle_tex_indices = [tex_poly[0], tex_poly[i], tex_poly[i+1]]
            if log_info.get(shape_name, 0) < 3:
                log_info[shape_name] = log_info.get(shape_name, 0) + 1
                if log_info[shape_name] == 1:
                    print(f"  - Detailed trace for first 3 triangles of '{shape_name}':")
                print(f"  - Triangle {log_info[shape_name]}:")
                fallback_msg = "(fallback to coordIndex)" if not use_tex_indices else ""
                for v_num in range(3):
                    c_idx, t_idx = triangle_pos_indices[v_num], triangle_tex_indices[v_num]
                    pos_val = positions[c_idx]
                    uv_val_str = "N/A"
                    if texcoords is not None and t_idx < len(texcoords):
                        uv_val = texcoords[t_idx]
                        uv_val_str = f"[{uv_val[0]:.4f} {uv_val[1]:.4f}]"
                        sampled_colors_str = ""
                        for map_type, img in all_images_for_logging.items():
                            w, h = img.size
                            px, py = int((uv_val[0] % 1.0) * (w - 1)), int((1.0 - (uv_val[1] % 1.0)) * (h - 1))
                            pixel_color = img.getpixel((px, py))
                            sampled_colors_str += f" | sampler_color({map_type}): {pixel_color}"
                    print(f"    - V{v_num+1}: coordIdx[{c_idx}]->pos{pos_val} | texCoordIdx[{t_idx}]{fallback_msg}->uv{uv_val_str}{sampled_colors_str}")
            for v_idx, t_idx in zip(triangle_pos_indices, triangle_tex_indices):
                final_positions.append(positions[v_idx])
                original_indices_map.append(v_idx)
                if texcoords is not None and t_idx < len(texcoords):
                    final_texcoords.append(texcoords[t_idx])
            vertex_count += 3
    print(f"  - Unrolled {len(pos_polygons)} polygons into {total_triangles} triangles for this shape.")
    final_indices = np.arange(vertex_count, dtype=np.uint32)
    return np.array(final_positions, dtype=np.float32), \
           np.array(final_texcoords, dtype=np.float32) if final_texcoords else None, \
           final_indices.reshape(-1, 3), \
           np.array(original_indices_map, dtype=int)

def quat_from_axis_angle(axis, angle):
    axis_norm = np.linalg.norm(axis);
    if axis_norm < 1e-6: return np.array([0, 0, 0, 1], dtype=np.float32)
    axis = axis / axis_norm; half_angle = angle / 2.0; s = np.sin(half_angle)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, np.cos(half_angle)], dtype=np.float32)

def get_matrix_from_quat(q):
    x, y, z, w = q; m = np.identity(4, dtype=np.float32)
    m[0, 0] = 1 - 2*y*y - 2*z*z; m[0, 1] = 2*x*y - 2*w*z; m[0, 2] = 2*x*z + 2*w*y
    m[1, 0] = 2*x*y + 2*w*z; m[1, 1] = 1 - 2*x*x - 2*z*z; m[1, 2] = 2*y*z - 2*w*x
    m[2, 0] = 2*x*z - 2*w*y; m[2, 1] = 2*y*z + 2*w*x; m[2, 2] = 1 - 2*x*x - 2*y*y
    return m

class CameraState:
    def __init__(self, look_from, look_at):
        self.look_at = np.array(look_at, dtype=np.float32)
        vec = np.array(look_from, dtype=np.float32) - self.look_at
        self.radius = np.linalg.norm(vec) if np.linalg.norm(vec) > 0 else 1.0
        self.azimuth = np.arctan2(vec[2], vec[0])
        self.elevation = np.arcsin(vec[1] / self.radius) if self.radius > 0 else 0
        self.is_dragging = False; self.last_mouse_pos = (0, 0)

class HAnimData:
    def __init__(self):
        self.root_joint = None; self.joints = {}; self.joint_name_to_index = {}
        self.skin_indices = []; self.skin_weights = []
        self.joint_matrices = []; self.base_vertices = None
        self.anari_geometries = []; self.anari_vertex_arrays = []
        self.unrolled_to_original_indices = []

class InteractiveAnariRenderer:
    def __init__(self, x3d_scene, width, height):
        self.width = width; self.height = height; self.x3d_scene = x3d_scene
        self.start_time = time.time();
        self.device = anari.newDevice('default')
        if not self.device: raise RuntimeError("Could not create ANARI device.")
        self.log_info = {}
        self._initialize_scene(); self._setup_display()

    def _initialize_scene(self):
        self.world = self.device.newWorld()
        self.surfaces, self.lights, self.def_map = [], [], {}
        self.hanim_humanoid, self.viewpoint_node, self.scene_node = None, None, None
        self._traverse(self.x3d_scene)
        if not self.scene_node: self.scene_node = self.x3d_scene
        self.time_sensors, self.interpolators, self.routes = {}, {}, []
        self._find_scene_elements(self.scene_node)
        if not self.viewpoint_node: self.viewpoint_node = next((n for n in self.def_map.values() if isinstance(n, x3d.Viewpoint)), None)
        if self.viewpoint_node and hasattr(self.viewpoint_node, 'position'):
            pos, center = self.viewpoint_node.position, getattr(self.viewpoint_node, 'centerOfRotation', (0, 1.0, 0))
            self.initial_camera_pos, self.initial_camera_at = (pos[0], pos[1], -pos[2]), center
        else:
            self.initial_camera_pos, self.initial_camera_at = (0, 1.2, 4.0), (0, 1.0, 0)
        for interp in self.interpolators.values():
            if hasattr(interp, 'keyValue') and isinstance(interp, x3d.OrientationInterpolator):
                interp.quaternionValue = [quat_from_axis_angle(aa[:3], aa[3]) for aa in np.array(interp.keyValue).reshape(-1, 4)]
        humanoid_node = next((n for n in self.def_map.values() if isinstance(n, x3d.HAnimHumanoid)), None)
        if humanoid_node:
            self.hanim_humanoid = HAnimData()
            if hasattr(humanoid_node, 'skinCoord') and hasattr(humanoid_node, 'skin'):
                self._setup_hanim_skinned(humanoid_node)
            if self.surfaces: self.world.setParameterArray1D('surface', anari.SURFACE, self.surfaces)
        key_light = self.device.newLight("directional")
        key_light.setParameter('direction', anari.FLOAT32_VEC3, (0.5, -0.8, -0.6))
        key_light.setParameter('irradiance', anari.FLOAT32, 4.0)
        key_light.commitParameters(); self.lights.append(key_light)
        fill_light = self.device.newLight("directional")
        fill_light.setParameter('direction', anari.FLOAT32_VEC3, (-0.3, -0.2, -0.4))
        fill_light.setParameter('irradiance', anari.FLOAT32, 1.5)
        fill_light.commitParameters(); self.lights.append(fill_light)
        if self.lights: self.world.setParameterArray1D('light', anari.LIGHT, self.lights)
        self.world.commitParameters()

    def _traverse(self, node):
        if not hasattr(node, '__dict__'): return
        if isinstance(node, x3d.Scene): self.scene_node = node
        node_def = getattr(node, 'DEF', None)
        if node_def: self.def_map[node_def] = node
        for attr_value in vars(node).values():
            if hasattr(attr_value, '__dict__'): self._traverse(attr_value)
            elif isinstance(attr_value, list):
                for item in attr_value:
                    if hasattr(item, '__dict__'): self._traverse(item)

    def _find_scene_elements(self, node):
        if not hasattr(node, '__dict__'): return
        if isinstance(node, x3d.TimeSensor) and node.DEF: self.time_sensors[node.DEF] = node
        elif isinstance(node, (x3d.OrientationInterpolator, x3d.PositionInterpolator, x3d.ScalarInterpolator)) and node.DEF:
             self.interpolators[node.DEF] = node
        elif isinstance(node, x3d.ROUTE): self.routes.append(node)
        elif isinstance(node, x3d.Viewpoint):
            if not self.viewpoint_node: self.viewpoint_node = node
        for attr_value in vars(node).values():
            if hasattr(attr_value, '__dict__'): self._find_scene_elements(attr_value)
            elif isinstance(attr_value, list):
                for item in attr_value:
                    if hasattr(item, '__dict__'): self._find_scene_elements(item)

    def _multiply_quaternions(self, q1, q2):
        x1, y1, z1, w1 = q1; x2, y2, z2, w2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2; x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2; z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        result = np.array([x, y, z, w], dtype=np.float32); norm = np.linalg.norm(result)
        return result / norm if norm > 1e-6 else result

    def _build_skeleton_map(self, humanoid_node):
        root_joint_ref = humanoid_node.skeleton[0]
        self.hanim_humanoid.root_joint = self.def_map.get(getattr(root_joint_ref, 'USE', None), root_joint_ref)
        if not self.hanim_humanoid.root_joint: raise RuntimeError("Could not resolve the root HAnimJoint.")
        for joint_ref in getattr(humanoid_node, 'joints', []):
            joint_node = self.def_map.get(getattr(joint_ref, 'USE', None), joint_ref)
            if not joint_node or not hasattr(joint_node, 'DEF'): continue
            name = joint_node.DEF
            if name not in self.hanim_humanoid.joints:
                rot = getattr(joint_node, 'rotation', (0, 1, 0, 0))
                self.hanim_humanoid.joints[name] = {'node': joint_node, 'parent': None, 'children': [], 'initial_rotation': quat_from_axis_angle(rot[:3], rot[3]), 'current_rotation': quat_from_axis_angle(rot[:3], rot[3]), 'bind_pose_matrix': np.identity(4, dtype=np.float32), 'inv_bind_pose_matrix': np.identity(4, dtype=np.float32)}
        for name, joint_info in self.hanim_humanoid.joints.items():
            for child_ref in joint_info['node'].children:
                if not child_ref: continue
                child_node = self.def_map.get(getattr(child_ref, 'USE', None), child_ref)
                if child_node and hasattr(child_node, 'DEF') and child_node.DEF in self.hanim_humanoid.joints:
                    child_name = child_node.DEF
                    joint_info['children'].append(child_name)
                    self.hanim_humanoid.joints[child_name]['parent'] = name
        self.hanim_humanoid.joint_name_to_index = {name: i for i, name in enumerate(self.hanim_humanoid.joints.keys())}

    def _calculate_bind_poses(self, joint_name, parent_bind_matrix):
        joint_info = self.hanim_humanoid.joints[joint_name]; parent_name = joint_info['parent']
        parent_center = np.array(self.hanim_humanoid.joints[parent_name]['node'].center) if parent_name else np.zeros(3)
        local_translation_mat=np.identity(4,dtype=np.float32); local_translation_mat[:3,3]=np.array(joint_info['node'].center) - parent_center
        bind_matrix = parent_bind_matrix @ local_translation_mat @ get_matrix_from_quat(joint_info['initial_rotation'])
        joint_info['bind_pose_matrix'], joint_info['inv_bind_pose_matrix'] = bind_matrix, np.linalg.inv(bind_matrix)
        for child_name in joint_info['children']: self._calculate_bind_poses(child_name, bind_matrix)

    def _create_anari_surface(self, positions, texcoords, indices, samplers, material_props):
        geom = self.device.newGeometry("triangle")
        v_array = self.device.newArray1D(anari.FLOAT32_VEC3, positions)
        geom.setParameter("vertex.position", anari.ARRAY, v_array)
        if texcoords is not None and len(texcoords) > 0:
            t_array = self.device.newArray1D(anari.FLOAT32_VEC2, texcoords)
            geom.setParameter("vertex.attribute0", anari.ARRAY, t_array)
        i_array = self.device.newArray1D(anari.UINT32_VEC3, indices)
        geom.setParameter("primitive.index", anari.ARRAY, i_array)
        geom.commitParameters()
        material = self.device.newMaterial("physicallyBased")
        if 'color' in samplers:
            material.setParameter("map.baseColor", anari.SAMPLER, samplers['color'])
            material.setParameter("baseColor", anari.STRING, "attribute0")
        else:
            material.setParameter('baseColor', anari.FLOAT32_VEC3, material_props['diffuse'])
        material.setParameter('metallic', anari.FLOAT32, material_props['metallic'])
        material.setParameter('roughness', anari.FLOAT32, material_props['roughness'])
        if 'normal' in samplers:
            material.setParameter("map.normal", anari.SAMPLER, samplers['normal'])
        if 'occlusion' in samplers:
            material.setParameter("map.occlusion", anari.SAMPLER, samplers['occlusion'])
        if 'metallicRoughness' in samplers:
            material.setParameter("map.metallicRoughness", anari.SAMPLER, samplers['metallicRoughness'])
        material.commitParameters()
        surface = self.device.newSurface()
        surface.setParameter('geometry', anari.GEOMETRY, geom)
        surface.setParameter('material', anari.MATERIAL, material)
        surface.commitParameters()
        self.surfaces.append(surface)
        return geom, v_array

    def _create_anari_sampler_from_texture_node(self, tex_node_ref):
        if not tex_node_ref: return None, None
        tex_node = self.def_map.get(getattr(tex_node_ref, 'USE', None), tex_node_ref)
        img = None
        tex_def_name = getattr(tex_node, 'DEF', 'Unnamed ImageTexture')
        if getattr(tex_node, 'url', None) and tex_node.url:
            texture_url = tex_node.url[0]
            print(f"      - Attempting to load ImageTexture '{tex_def_name}' from url: {texture_url[:60]}...")
            try:
                if texture_url.startswith('data:image'):
                    header, encoded_data = texture_url.split(',', 1)
                    img = Image.open(io.BytesIO(base64.b64decode(encoded_data))).convert('RGB')
                elif os.path.exists(texture_url):
                    img = Image.open(texture_url).convert('RGB')
                if img:
                    print(f"        ... SUCCESS: Loaded image of size {img.size}")
                else:
                    print(f"        ... FAILED: URL is not a data URI or local file.")
            except Exception as e:
                print(f"        ... FAILED: Could not decode or open image. Error: {e}")
                return None, None
        else:
            print(f"      - SKIPPING ImageTexture '{tex_def_name}': No URL provided.")
            return None, None
        if img:
            img_data = (np.array(img) / 255.0).astype(np.float32)
            img_array = self.device.newArray2D(anari.FLOAT32_VEC3, img_data)
            sampler = self.device.newSampler("image2D")
            sampler.setParameter("image", anari.ARRAY, img_array)
            sampler.setParameter("inAttribute", anari.STRING, "attribute0")
            sampler.setParameter("wrapMode", anari.STRING, "repeat")
            sampler.commitParameters()
            return sampler, img
        return None, None

    def _setup_hanim_skinned(self, humanoid_node):
        self._build_skeleton_map(humanoid_node)
        coord_node_container = getattr(humanoid_node, 'skinCoord', None)
        coord_node = self.def_map.get(getattr(coord_node_container, 'USE', None), coord_node_container)
        if not coord_node or not hasattr(coord_node, 'point'): return
        all_vertices_master = np.array(coord_node.point, dtype=np.float32).reshape(-1, 3)
        self.hanim_humanoid.base_vertices = all_vertices_master
        skinned_shapes = [n for n in self.def_map.values() if isinstance(n, x3d.Shape) and hasattr(n.geometry, 'coord') and hasattr(n.geometry.coord, 'USE') and n.geometry.coord.USE == coord_node.DEF]
        print("\n--- Processing Shapes and Materials ---")
        for node in skinned_shapes:
            shape_def = getattr(node, 'DEF', 'Unnamed Shape')
            print(f"\nProcessing Shape: {shape_def}")
            appearance_node = self.def_map.get(getattr(node.appearance, 'USE', None), node.appearance)
            material_props = {'diffuse': (0.8, 0.8, 0.8), 'metallic': 0.1, 'roughness': 0.8}
            target_uv_node, samplers, images_for_logging = None, {}, {}
            if appearance_node:
                mat_node = None
                if hasattr(appearance_node, 'material') and appearance_node.material:
                    mat_node = self.def_map.get(getattr(appearance_node.material, 'USE', None), appearance_node.material)
                if mat_node:
                    mat_def = getattr(mat_node, 'DEF', 'Unnamed Material')
                    material_props['diffuse'] = getattr(mat_node, 'diffuseColor', (0.8, 0.8, 0.8))
                    shininess = getattr(mat_node, 'shininess', 0.2)
                    material_props['roughness'] = np.clip(1.0 - shininess, 0.05, 1.0)
                    print(f"  - Found Material: {mat_def} | baseColor: {material_props['diffuse']} | shininess: {shininess} -> roughness: {material_props['roughness']:.3f}")
                    texture_map = {
                        'color': getattr(mat_node, 'diffuseTexture', None) or getattr(mat_node, 'baseTexture', None),
                        'normal': getattr(mat_node, 'normalTexture', None),
                        'occlusion': getattr(mat_node, 'occlusionTexture', None),
                        'metallicRoughness': getattr(mat_node, 'specularTexture', None)
                    }
                    for map_type, tex_node in texture_map.items():
                        if tex_node:
                            sampler, img = self._create_anari_sampler_from_texture_node(tex_node)
                            if sampler:
                                samplers[map_type] = sampler
                                images_for_logging[map_type] = img
                geo_tex_coord = getattr(node.geometry, 'texCoord', None)
                if geo_tex_coord:
                    uv_def = getattr(geo_tex_coord, 'DEF', '...')
                    if isinstance(geo_tex_coord, x3d.MultiTextureCoordinate) and geo_tex_coord.texCoord:
                        target_uv_node = geo_tex_coord.texCoord[0]
                        uv_def = getattr(target_uv_node, 'DEF', '... (from MultiTexture)')
                    else:
                        target_uv_node = geo_tex_coord
                    print(f"  - Using TextureCoordinate Node: {uv_def}")
            part_texcoords_master = None
            if target_uv_node:
                data_node = self.def_map.get(getattr(target_uv_node, 'USE', None), target_uv_node)
                if data_node and hasattr(data_node, 'point') and data_node.point:
                    part_texcoords_master = np.array(data_node.point, dtype=np.float32).reshape(-1, 2)
            final_pos, final_uvs, final_indices, original_indices_map = unroll_indexed_face_set(
                all_vertices_master, part_texcoords_master, node.geometry.coordIndex,
                getattr(node.geometry, 'texCoordIndex', None), shape_def, self.log_info, images_for_logging
            )
            if final_pos is None: continue
            self.hanim_humanoid.unrolled_to_original_indices.append(original_indices_map)
            samplers = {k: v for k, v in samplers.items() if v is not None}
            if not samplers: print("  - Final decision: Untextured surface.")
            else: print(f"  - Final decision: Textured surface with maps: {list(samplers.keys())}")
            geom, v_array = self._create_anari_surface(final_pos, final_uvs, final_indices, samplers, material_props)
            self.hanim_humanoid.anari_geometries.append(geom)
            self.hanim_humanoid.anari_vertex_arrays.append(v_array)
        self._calculate_bind_poses(self.hanim_humanoid.root_joint.DEF, np.identity(4, dtype=np.float32))
        self.hanim_humanoid.skin_indices = [[] for _ in range(len(all_vertices_master))]; self.hanim_humanoid.skin_weights = [[] for _ in range(len(all_vertices_master))]
        self.hanim_humanoid.joint_matrices = [np.identity(4, dtype=np.float32) for _ in self.hanim_humanoid.joints]
        for joint_name, joint_info in self.hanim_humanoid.joints.items():
            joint_node, joint_idx = joint_info['node'], self.hanim_humanoid.joint_name_to_index[joint_name]
            if hasattr(joint_node, 'skinCoordIndex') and hasattr(joint_node, 'skinCoordWeight'):
                for v_idx, weight in zip(joint_node.skinCoordIndex, joint_node.skinCoordWeight):
                    if v_idx < len(all_vertices_master):
                        self.hanim_humanoid.skin_indices[v_idx].append(joint_idx); self.hanim_humanoid.skin_weights[v_idx].append(weight)

    def _animate_scene(self, t):
        if not self.hanim_humanoid or self.hanim_humanoid.base_vertices is None: return
        self._animate_scene_skinned(t)

    def _update_joint_rotations(self, t):
        timer = next(iter(self.time_sensors.values()), None)
        if not timer or not hasattr(timer, 'cycleInterval'): return
        fraction = (t % timer.cycleInterval) / timer.cycleInterval
        for route in self.routes:
            if "value_changed" in route.fromField and "set_rotation" in route.toField:
                joint_def, interp_def = route.toNode, route.fromNode
                interpolator = self.interpolators.get(interp_def)
                if interpolator and joint_def in self.hanim_humanoid.joints:
                    joint_info = self.hanim_humanoid.joints[joint_def]
                    animation_rotation = self._get_interpolated_rotation(interpolator, fraction)
                    final_rotation = self._multiply_quaternions(joint_info['initial_rotation'], animation_rotation)
                    joint_info['current_rotation'] = final_rotation

    def _animate_scene_skinned(self, t):
        self._update_joint_rotations(t)
        def update_skinning_matrices(joint_name, parent_world_matrix):
            joint_info = self.hanim_humanoid.joints[joint_name]
            parent_name = joint_info['parent']
            parent_center = np.zeros(3, dtype=np.float32)
            if parent_name: parent_center = np.array(self.hanim_humanoid.joints[parent_name]['node'].center)
            local_translation_mat = np.identity(4, dtype=np.float32); local_translation_mat[:3,3] = np.array(joint_info['node'].center) - parent_center
            world_matrix = parent_world_matrix @ local_translation_mat @ get_matrix_from_quat(joint_info['current_rotation'])
            joint_idx = self.hanim_humanoid.joint_name_to_index[joint_name]
            self.hanim_humanoid.joint_matrices[joint_idx] = world_matrix @ joint_info['inv_bind_pose_matrix']
            for child_name in joint_info['children']: update_skinning_matrices(child_name, world_matrix)
        angle = t * 0.5; c, s = np.cos(angle), np.sin(angle)
        rotation_matrix = np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], dtype=np.float32)
        update_skinning_matrices(self.hanim_humanoid.root_joint.DEF, rotation_matrix)
        base_verts_h = np.hstack((self.hanim_humanoid.base_vertices, np.ones((len(self.hanim_humanoid.base_vertices), 1), dtype=np.float32)))
        deformed_master_vertices = np.zeros_like(self.hanim_humanoid.base_vertices)
        for i in range(len(deformed_master_vertices)):
            final_pos, total_weight = np.zeros(4, dtype=np.float32), sum(self.hanim_humanoid.skin_weights[i])
            if total_weight > 1e-5:
                for j_idx, weight in zip(self.hanim_humanoid.skin_indices[i], self.hanim_humanoid.skin_weights[i]):
                    final_pos += (weight / total_weight) * (self.hanim_humanoid.joint_matrices[j_idx] @ base_verts_h[i])
            else:
                final_pos = base_verts_h[i] if not self.hanim_humanoid.skin_indices[i] else self.hanim_humanoid.joint_matrices[self.hanim_humanoid.skin_indices[i][0]] @ base_verts_h[i]
            deformed_master_vertices[i] = final_pos[:3] / final_pos[3] if final_pos[3] != 0 else final_pos[:3]
        for i, geom in enumerate(self.hanim_humanoid.anari_geometries):
            new_unrolled_verts = deformed_master_vertices[self.hanim_humanoid.unrolled_to_original_indices[i]]
            new_v_array = self.device.newArray1D(anari.FLOAT32_VEC3, new_unrolled_verts)
            geom.setParameter("vertex.position", anari.ARRAY, new_v_array); geom.commitParameters()
            self.hanim_humanoid.anari_vertex_arrays[i] = new_v_array

    def _get_interpolated_rotation(self, interpolator, fraction):
        keys, values = interpolator.key, interpolator.quaternionValue
        if fraction <= keys[0]: return values[0]
        if fraction >= keys[-1]: return values[-1]
        for i in range(len(keys) - 1):
            if keys[i] <= fraction < keys[i+1]:
                t = (fraction - keys[i]) / (keys[i+1] - keys[i]); q1, q2 = values[i], values[i+1]
                dot = np.dot(q1, q2)
                if dot < 0: q2, dot = -q2, -dot
                if dot > 0.9995: result = q1 + t * (q2 - q1)
                else:
                    theta_0 = np.arccos(dot); sin_theta_0 = np.sin(theta_0)
                    if sin_theta_0 < 1e-6: return q1
                    s0 = np.sin((1 - t) * theta_0) / sin_theta_0; s1 = np.sin(t * theta_0) / sin_theta_0
                    result = (s0 * q1) + (s1 * q2)
                return result / np.linalg.norm(result)
        return values[-1]

    def _setup_display(self):
        self.cam_state=CameraState(self.initial_camera_pos, self.initial_camera_at)
        self.camera=self.device.newCamera("perspective"); self.camera.setParameter('aspect',anari.FLOAT32,self.width/self.height); self.camera.setParameter('fovy',anari.FLOAT32,math.radians(60.0))
        renderer=self.device.newRenderer("default"); renderer.setParameter('backgroundColor', anari.FLOAT32_VEC4, (0.0, 0.0, 0.0, 1.0)); renderer.commitParameters()
        self.frame=self.device.newFrame(); self.frame.setParameter('size',anari.UINT32_VEC2,(self.width,self.height)); self.frame.setParameter('channel.color',anari.DATA_TYPE,anari.UFIXED8_RGBA_SRGB)
        self.frame.setParameter('world',anari.WORLD,self.world); self.frame.setParameter('camera',anari.CAMERA,self.camera); self.frame.setParameter('renderer',anari.RENDERER,renderer); self.frame.commitParameters()
        fig_bg_color = (0.0, 0.0, 0.0)
        self.fig, self.ax = plt.subplots(figsize=(self.width/100, self.height/100), facecolor=fig_bg_color)
        self.ax.set_facecolor(fig_bg_color)
        self.image_display=self.ax.imshow(np.zeros((self.height,self.width,4),dtype=np.uint8))
        self.ax.set_title("Drag to orbit, Scroll to zoom"); plt.gca().invert_yaxis(); plt.axis('off'); self.fig.tight_layout(pad=0)

    def run(self):
        self._setup_interaction_handlers()
        ani=FuncAnimation(self.fig, self._update_frame, cache_frame_data=False, interval=16, blit=False)
        plt.show()

    def _update_frame(self, frame_num):
        elapsed_time=time.time()-self.start_time
        self._animate_scene(elapsed_time)
        self._update_camera()
        self.frame.render()
        color_data = self.frame.get('channel.color')
        self.image_display.set_data(color_data)
        return [self.image_display]

    def _update_camera(self):
        s=self.cam_state; x=s.radius*np.cos(s.elevation)*np.cos(s.azimuth); y=s.radius*np.sin(s.elevation); z=s.radius*np.cos(s.elevation)*np.sin(s.azimuth)
        pos=s.look_at+np.array([x,y,z]); self.camera.setParameter('position',anari.FLOAT32_VEC3,pos); self.camera.setParameter('at',anari.FLOAT32_VEC3,s.look_at); self.camera.setParameter('up',anari.FLOAT32_VEC3,(0,1,0)); self.camera.commitParameters()

    def _setup_interaction_handlers(self):
        self.fig.canvas.mpl_connect('button_press_event',self._on_press); self.fig.canvas.mpl_connect('button_release_event',self._on_release); self.fig.canvas.mpl_connect('motion_notify_event',self._on_motion); self.fig.canvas.mpl_connect('scroll_event',self._on_scroll)
    def _on_press(self, event):
        if event.inaxes!=self.ax: return
        self.last_mouse_pos=(event.x,event.y); self.cam_state.is_dragging=True
    def _on_release(self, event): self.cam_state.is_dragging=False
    def _on_motion(self, event):
        if not self.cam_state.is_dragging or event.inaxes!=self.ax: return
        dx,dy=event.x-self.last_mouse_pos[0],event.y-self.last_mouse_pos[1]; self.last_mouse_pos=(event.x,event.y); self.cam_state.azimuth-=dx*0.01; self.cam_state.elevation=np.clip(self.cam_state.elevation+dy*0.01,-np.pi/2+0.01,np.pi/2-0.01)
    def _on_scroll(self, event):
        if event.inaxes!=self.ax: return
        self.cam_state.radius=max(0.2,self.cam_state.radius*(0.9 if event.button=='up' else 1.1))

if __name__ == "__main__":
    try:
        from conan_23_Aug2025 import X3D0 as scene_to_render
        print("Rendering scene from 'conan_23_Aug2025.py'...")
    except ImportError:
        print("Could not find 'conan_23_Aug2025.py', falling back to 'walking_man_cc_test.py'")
        try:
            from walking_man_cc_test import X3D0 as scene_to_render
            print("Rendering scene from 'walking_man_cc_test.py'...")
        except ImportError:
            print("Error: No valid X3D scene file found.")
            exit()
    renderer = InteractiveAnariRenderer(scene_to_render, width=1024, height=768)
    renderer.run()
