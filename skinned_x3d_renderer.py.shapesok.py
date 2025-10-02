# --- START OF FILE skinned_x3d_renderer.py (HANDLES DATA URI TEXTURES) ---

import x3d
import pynari as anari
import numpy as np
from PIL import Image
import math
import time
import os
import base64 # <-- IMPORT for decoding
import io     # <-- IMPORT for in-memory files

# --- Matplotlib for real-time display ---
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Helper Functions ---

def parse_x3d_coord_index(coord_index):
    if not coord_index: return []
    flat_indices, polygons, current_poly = list(coord_index), [], []
    for idx in flat_indices:
        if idx == -1:
            if len(current_poly) >= 3: polygons.append(current_poly)
            current_poly = []
        else: current_poly.append(idx)
    if len(current_poly) >= 3: polygons.append(current_poly)
    triangles = []
    for poly in polygons:
        for i in range(1, len(poly) - 1): triangles.extend([poly[0], poly[i], poly[i + 1]])
    return triangles

def quat_from_axis_angle(axis, angle):
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-6: return np.array([0, 0, 0, 1], dtype=np.float32)
    axis = axis / axis_norm
    half_angle = angle / 2.0
    s = np.sin(half_angle)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, np.cos(half_angle)], dtype=np.float32)

def get_matrix_from_quat(q):
    x, y, z, w = q
    m = np.identity(4, dtype=np.float32)
    m[0, 0] = 1 - 2*y*y - 2*z*z; m[0, 1] = 2*x*y - 2*w*z; m[0, 2] = 2*x*z + 2*w*y
    m[1, 0] = 2*x*y + 2*w*z; m[1, 1] = 1 - 2*x*x - 2*z*z; m[1, 2] = 2*y*z - 2*w*x
    m[2, 0] = 2*x*z - 2*w*y; m[2, 1] = 2*y*z + 2*w*x; m[2, 2] = 1 - 2*x*x - 2*y*y
    return m

# --- Camera State Helper ---
class CameraState:
    def __init__(self, look_from, look_at):
        self.look_at = np.array(look_at, dtype=np.float32)
        vec = np.array(look_from, dtype=np.float32) - self.look_at
        self.radius = np.linalg.norm(vec) if np.linalg.norm(vec) > 0 else 1.0
        self.azimuth = np.arctan2(vec[2], vec[0])
        self.elevation = np.arcsin(vec[1] / self.radius) if self.radius > 0 else 0
        self.is_dragging = False
        self.last_mouse_pos = (0, 0)

# --- H-Anim Data Helper ---
class HAnimData:
    def __init__(self):
        self.root_joint = None
        self.joints = {}
        self.joint_name_to_index = {}
        self.is_skinned = False
        self.skin_indices = []
        self.skin_weights = []
        self.joint_matrices = []
        self.base_vertices = None
        self.indices = None
        self.anari_geometries = [] # <-- MODIFICATION: Store multiple geometries
        self.anari_vertex_array = None

# --- Main Renderer Class ---
class InteractiveAnariRenderer:
    def __init__(self, x3d_scene, width, height):
        self.width = width
        self.height = height
        self.x3d_scene = x3d_scene
        self.start_time = time.time()
        self.device = anari.newDevice('helide')
        # self.device = anari.newDevice('default')
        if not self.device: raise RuntimeError("Could not create ANARI device.")
        self._initialize_scene()
        self._setup_display()

    def _initialize_scene(self):
        self.world = self.device.newWorld()
        self.surfaces, self.lights, self.def_map, self.time_sensors, self.interpolators, self.routes = [], [], {}, {}, {}, []
        self.hanim_humanoid, self.viewpoint_node = None, None
        scene_node = self.x3d_scene.Scene
        self._traverse(scene_node)

        if not self.viewpoint_node: self.viewpoint_node = next((n for n in self.def_map.values() if isinstance(n, x3d.Viewpoint)), None)
        if self.viewpoint_node and hasattr(self.viewpoint_node, 'position'):
            pos, center = self.viewpoint_node.position, getattr(self.viewpoint_node, 'centerOfRotation', (0, 1.0, 0))
            print(f"Using Viewpoint '{getattr(self.viewpoint_node, 'DEF', 'N/A')}' at position {pos}")
            # X3D Z-axis is typically opposite ANARI/OpenGL Z-axis
            self.initial_camera_pos, self.initial_camera_at = tuple((pos[0], pos[1], -pos[2])), center
        else:
            self.initial_camera_pos, self.initial_camera_at = (0, 1.2, -4.0), (0, 1.0, 0)

        for interp in self.interpolators.values():
            if hasattr(interp, 'keyValue') and isinstance(interp, x3d.OrientationInterpolator):
                # Convert axis-angle keyValues to quaternions once
                interp.quaternionValue = [quat_from_axis_angle(aa[:3], aa[3]) for aa in np.array(interp.keyValue).reshape(-1, 4)]

        humanoid_node = next((n for n in self.def_map.values() if isinstance(n, x3d.HAnimHumanoid)), None)
        if humanoid_node:
            self.hanim_humanoid = HAnimData()
            if hasattr(humanoid_node, 'skinCoord') and hasattr(humanoid_node, 'skin'):
                print("Detected skinned geometry. Using Linear Blend Skinning path.")
                self._setup_hanim_skinned(humanoid_node)
            # Only add surfaces if they were actually created
            if self.surfaces: self.world.setParameterArray1D('surface', anari.SURFACE, self.surfaces)

        # Process X3D lights
        scene_lights = [n for n in self.def_map.values() if isinstance(n, (x3d.DirectionalLight, x3d.SpotLight, x3d.PointLight))]
        for light_node in scene_lights:
            # ANARI light types need to be chosen based on X3D light type
            # For simplicity, let's treat all as directional for now unless specific parameters are needed
            light = self.device.newLight("directional")
            light.setParameter('direction', anari.FLOAT32_VEC3, getattr(light_node, 'direction', (0,-1,-1)))
            light_color = getattr(light_node, 'color', (1, 1, 1))
            light.setParameter('color', anari.FLOAT32_VEC3, light_color)
            light.setParameter('irradiance', anari.FLOAT32, getattr(light_node, 'intensity', 1.0) * 2.5) # Scale intensity for visibility
            light.commitParameters(); self.lights.append(light)

        # Add a default light if no lights were found in the scene
        if not self.lights:
            light = self.device.newLight("directional")
            light.setParameter('direction', anari.FLOAT32_VEC3, (0.2, -1.0, -1.0)); light.setParameter('irradiance', anari.FLOAT32, 2.0)
            light.commitParameters(); self.lights.append(light)

        if self.lights: self.world.setParameterArray1D('light', anari.LIGHT, self.lights)
        self.world.commitParameters()

    def _traverse(self, node):
        # Base case: if node doesn't have a dictionary (e.g., a primitive type)
        if not hasattr(node, '__dict__'):
            return

        # Store nodes with DEF attributes
        node_def = getattr(node, 'DEF', None)
        if node_def: self.def_map[node_def] = node

        # Identify specific nodes for animation and camera setup
        if isinstance(node, x3d.TimeSensor):
            if node_def: self.time_sensors[node_def] = node
        elif isinstance(node, (x3d.OrientationInterpolator, x3d.PositionInterpolator, x3d.ScalarInterpolator)):
            if node_def: self.interpolators[node_def] = node
        elif isinstance(node, x3d.ROUTE):
            self.routes.append(node)
        elif isinstance(node, x3d.Viewpoint):
            # Only store the first encountered Viewpoint if one isn't already set
            if not self.viewpoint_node: self.viewpoint_node = node

        # Recursively traverse child nodes/attributes
        for attr_value in vars(node).values():
            if hasattr(attr_value, '__dict__'): # Check if it's an X3D node
                self._traverse(attr_value)
            elif isinstance(attr_value, list): # Check if it's a list of X3D nodes
                for item in attr_value:
                    if hasattr(item, '__dict__'):
                        self._traverse(item)

    def _multiply_quaternions(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        result = np.array([x, y, z, w], dtype=np.float32)
        norm = np.linalg.norm(result)

        # Normalize the result to prevent drift
        return result / norm if norm > 1e-6 else result

    def _build_skeleton_map(self, humanoid_node):
        self.hanim_humanoid.root_joint = humanoid_node.skeleton[0]

        def build_map_recursive(joint_node, parent_node=None):
            name = joint_node.DEF
            rot = getattr(joint_node, 'rotation', (0,1,0,0)) # Default rotation
            self.hanim_humanoid.joints[name] = {
                'node': joint_node,
                'parent': parent_node.DEF if parent_node else None,
                'children': [c.DEF for c in joint_node.children if isinstance(c, x3d.HAnimJoint)],
                'initial_rotation': quat_from_axis_angle(rot[:3], rot[3]),
                'current_rotation': quat_from_axis_angle(rot[:3], rot[3]), # Starts at initial
                'bind_pose_matrix': np.identity(4, dtype=np.float32),
                'inv_bind_pose_matrix': np.identity(4, dtype=np.float32)
            }
            for child in joint_node.children:
                if isinstance(child, x3d.HAnimJoint):
                    build_map_recursive(child, parent_node=joint_node)

        build_map_recursive(self.hanim_humanoid.root_joint)
        # Create a name-to-index mapping for faster lookup
        self.hanim_humanoid.joint_name_to_index = {name: i for i, name in enumerate(self.hanim_humanoid.joints.keys())}


    def _calculate_bind_poses(self, joint_name, parent_bind_matrix):
        joint_info = self.hanim_humanoid.joints[joint_name]
        parent_name = joint_info['parent']

        # Calculate local translation based on center (if parent exists)
        parent_center = np.zeros(3)
        if parent_name:
            parent_center = np.array(self.hanim_humanoid.joints[parent_name]['node'].center)

        joint_center = np.array(joint_info['node'].center)

        local_translation_mat = np.identity(4, dtype=np.float32)
        local_translation_mat[:3,3] = joint_center - parent_center

        # Combine local translation and initial rotation to get local bind matrix
        # Note: X3D HAnimJoint's 'rotation' is relative to its own coordinate system
        # and not necessarily parent. This interpretation assumes local rotation.
        bind_matrix = parent_bind_matrix @ local_translation_mat @ get_matrix_from_quat(joint_info['initial_rotation'])

        joint_info['bind_pose_matrix'] = bind_matrix
        joint_info['inv_bind_pose_matrix'] = np.linalg.inv(bind_matrix)

        for child_name in joint_info['children']:
            self._calculate_bind_poses(child_name, bind_matrix)

        # REPLACEMENT FOR _create_anari_surface
    def _create_anari_surface(self, shared_vertex_array, shared_texcoord_array, indices, sampler, diffuse, emissive):
        geom = self.device.newGeometry("triangle")
        geom.setParameter("vertex.position", anari.ARRAY, shared_vertex_array)
        if shared_texcoord_array:
            geom.setParameter("vertex.attribute0", anari.ARRAY, shared_texcoord_array)

        i_array = self.device.newArray1D(anari.UINT32_VEC3, np.array(indices, dtype=np.uint32).reshape(-1, 3))
        geom.setParameter("primitive.index", anari.ARRAY, i_array)
        geom.commitParameters()

        material = self.device.newMaterial("physicallyBased")
        if sampler and shared_texcoord_array:
            material.setParameter('baseColor', anari.FLOAT32_VEC3, (1.0, 1.0, 1.0))
            material.setParameter("map.baseColor", anari.SAMPLER, sampler)
            material.setParameter("map.baseColor.attribute", anari.STRING, "attribute0")
        else:
            material.setParameter('baseColor', anari.FLOAT32_VEC3, diffuse if diffuse is not None else (0.8, 0.8, 0.8))

        if emissive is not None and np.any(np.array(emissive) > 0):
             material.setParameter('emissive', anari.FLOAT32_VEC3, emissive)

        material.setParameter('metallic', anari.FLOAT32, 0.0)
        material.setParameter('roughness', anari.FLOAT32, 0.6)
        material.commitParameters()

        surface = self.device.newSurface()
        surface.setParameter('geometry', anari.GEOMETRY, geom)
        surface.setParameter('material', anari.MATERIAL, material)
        surface.commitParameters()
        self.surfaces.append(surface)

        return geom # <-- MODIFICATION: Return the created geometry handle

        # REPLACEMENT FOR _setup_hanim_skinned
    def _setup_hanim_skinned(self, humanoid_node):
        self._build_skeleton_map(humanoid_node)
        coord_node_container = getattr(humanoid_node, 'skinCoord', None)
        coord_node = None
        if coord_node_container and hasattr(coord_node_container, 'USE') and coord_node_container.USE:
            coord_node = self.def_map.get(coord_node_container.USE)
        else:
            coord_node = coord_node_container
        if not coord_node or not hasattr(coord_node, 'point') or not coord_node.point:
            print("[FATAL] Could not obtain a valid Coordinate node with 'point' data.")
            return

        all_vertices = np.array(coord_node.point, dtype=np.float32).reshape(-1, 3)
        self.hanim_humanoid.base_vertices = all_vertices
        shared_vertex_array = self.device.newArray1D(anari.FLOAT32_VEC3, all_vertices)
        self.hanim_humanoid.anari_vertex_array = shared_vertex_array

        shared_texcoord_array = None
        first_shape_node = next((n for n in humanoid_node.skin if isinstance(n, x3d.Shape)), None)
        if first_shape_node and hasattr(first_shape_node.geometry, 'texCoord') and first_shape_node.geometry.texCoord:
             tex_coord_container = first_shape_node.geometry.texCoord
             actual_tex_coord_node = tex_coord_container.texCoord[0] if isinstance(tex_coord_container, x3d.MultiTextureCoordinate) else tex_coord_container
             if actual_tex_coord_node and hasattr(actual_tex_coord_node, 'point') and actual_tex_coord_node.point:
                 all_texcoords = np.array(actual_tex_coord_node.point, dtype=np.float32).reshape(-1, 2)
                 shared_texcoord_array = self.device.newArray1D(anari.FLOAT32_VEC2, all_texcoords)

        print("\n--- Processing Model Parts ---")
        nodes_to_process = list(humanoid_node.skin)
        part_count = 0
        while nodes_to_process:
            node = nodes_to_process.pop(0)
            if isinstance(node, x3d.Group) and hasattr(node, 'children'):
                nodes_to_process.extend(node.children)
                continue
            if not isinstance(node, x3d.Shape): continue
            part_count += 1
            print(f"\n[PART {part_count}] Processing Shape node...")
            shape_indices = parse_x3d_coord_index(node.geometry.coordIndex)
            if not shape_indices:
                print(f"[PART {part_count}] -> Skipping: No indices found.")
                continue

            mat_node = getattr(node.appearance, 'material', None)
            tex_node, sampler, diffuse_color, emissive_color = None, None, (0.8,0.8,0.8), (0,0,0)
            if mat_node:
                diffuse_color = getattr(mat_node, 'diffuseColor', diffuse_color)
                emissive_color = getattr(mat_node, 'emissiveColor', emissive_color)
                tex_node = getattr(mat_node, 'baseTexture', None) or getattr(mat_node, 'diffuseTexture', None)
            if tex_node and isinstance(tex_node, x3d.ImageTexture):
                texture_url = tex_node.url[0] if tex_node.url else None
                if texture_url:
                    try:
                        if texture_url.startswith('data:image'):
                            header, encoded_data = texture_url.split(',', 1)
                            img = Image.open(io.BytesIO(base64.b64decode(encoded_data))).convert('RGB')
                            print(f"[PART {part_count}] -> Found texture: data URI ({img.width}x{img.height})")
                            img_data = (np.array(img) / 255.0).astype(np.float32)
                            img_array = self.device.newArray2D(anari.FLOAT32_VEC3, img_data)
                            sampler = self.device.newSampler("image2D")
                            sampler.setParameter("image", anari.ARRAY, img_array)
                            sampler.setParameter("wrapMode", anari.STRING, "clampToEdge")
                            sampler.commitParameters()
                    except Exception as e: print(f"Error loading texture: {e}")

            # <-- MODIFICATION: Store the returned geometry handle
            geom_handle = self._create_anari_surface(shared_vertex_array, shared_texcoord_array, shape_indices, sampler, diffuse_color, emissive_color)
            self.hanim_humanoid.anari_geometries.append(geom_handle)
            print(f"[PART {part_count}] -> Surface created successfully.")

        self._calculate_bind_poses(self.hanim_humanoid.root_joint.DEF, np.identity(4, dtype=np.float32))
        self.hanim_humanoid.skin_indices = [[] for _ in range(len(all_vertices))]; self.hanim_humanoid.skin_weights = [[] for _ in range(len(all_vertices))]
        self.hanim_humanoid.joint_matrices = [np.identity(4, dtype=np.float32) for _ in self.hanim_humanoid.joints]
        for joint_name, joint_info in self.hanim_humanoid.joints.items():
            joint_node, joint_idx = joint_info['node'], self.hanim_humanoid.joint_name_to_index[joint_name]
            if hasattr(joint_node, 'skinCoordIndex') and hasattr(joint_node, 'skinCoordWeight'):
                for v_idx, weight in zip(joint_node.skinCoordIndex, joint_node.skinCoordWeight):
                    if v_idx < len(all_vertices):
                        self.hanim_humanoid.skin_indices[v_idx].append(joint_idx); self.hanim_humanoid.skin_weights[v_idx].append(weight)

        print("\n--- Model Processing Complete ---")

    def _animate_scene(self, t):
        # Only animate if H-Anim data exists and base vertices are loaded
        if not self.hanim_humanoid or self.hanim_humanoid.base_vertices is None:
            return

        self._animate_scene_skinned(t)


    def _update_joint_rotations(self, t):
        # Find the first TimeSensor in the scene to drive animation
        timer = next(iter(self.time_sensors.values()), None)
        if not timer or not hasattr(timer, 'cycleInterval'):
            return

        # Calculate animation fraction
        fraction = (t % timer.cycleInterval) / timer.cycleInterval

        for route in self.routes:
            # Check for routes that connect interpolators to joint rotations
            if "value_changed" in route.fromField and "set_rotation" in route.toField:
                joint_def, interp_def = route.toNode, route.fromNode
                interpolator = self.interpolators.get(interp_def)

                if interpolator and joint_def in self.hanim_humanoid.joints:
                    joint_info = self.hanim_humanoid.joints[joint_def]

                    # Get interpolated rotation from the X3D interpolator
                    animation_rotation = self._get_interpolated_rotation(interpolator, fraction)

                    # Combine initial rotation with animated rotation (order matters!)
                    final_rotation = self._multiply_quaternions(joint_info['initial_rotation'], animation_rotation)
                    joint_info['current_rotation'] = final_rotation


    def _animate_scene_skinned(self, t):
        self._update_joint_rotations(t) # Update joint rotations based on time

        # Recursive function to update skinning matrices for each joint
        def update_skinning_matrices(joint_name, parent_world_matrix):
            joint_info = self.hanim_humanoid.joints[joint_name]
            parent_name = joint_info['parent']

            # Calculate local translation relative to parent's center
            parent_center = np.zeros(3, dtype=np.float32)
            if parent_name:
                parent_center = np.array(self.hanim_humanoid.joints[parent_name]['node'].center)

            joint_center = np.array(joint_info['node'].center)

            local_translation_mat = np.identity(4, dtype=np.float32)
            local_translation_mat[:3,3] = joint_center - parent_center

            # Get rotation matrix from current (animated) quaternion
            rotation_mat = get_matrix_from_quat(joint_info['current_rotation'])

            # Calculate local joint matrix
            local_matrix = local_translation_mat @ rotation_mat

            # Calculate world matrix for this joint
            world_matrix = parent_world_matrix @ local_matrix

            # Combine world matrix with inverse bind pose to get skinning matrix
            joint_idx = self.hanim_humanoid.joint_name_to_index[joint_name]
            self.hanim_humanoid.joint_matrices[joint_idx] = world_matrix @ joint_info['inv_bind_pose_matrix']

            # Recurse for children
            for child_name in joint_info['children']:
                update_skinning_matrices(child_name, world_matrix)

        # Start recursion from the root joint
        root_transform = np.identity(4, dtype=np.float32) # Global transform for the root
        update_skinning_matrices(self.hanim_humanoid.root_joint.DEF, root_transform)

        # Prepare base vertices for transformation (add homogeneous coordinate)
        base_verts_h = np.hstack((self.hanim_humanoid.base_vertices, np.ones((len(self.hanim_humanoid.base_vertices), 1), dtype=np.float32)))
        deformed_vertices = np.zeros_like(self.hanim_humanoid.base_vertices)

        # Apply skinning for each vertex
        for i in range(len(deformed_vertices)):
            final_pos, total_weight = np.zeros(4, dtype=np.float32), sum(self.hanim_humanoid.skin_weights[i])

            if total_weight > 1e-5: # Avoid division by zero
                for j_idx, weight in zip(self.hanim_humanoid.skin_indices[i], self.hanim_humanoid.skin_weights[i]):
                    skinning_matrix = self.hanim_humanoid.joint_matrices[j_idx]
                    final_pos += (weight / total_weight) * (skinning_matrix @ base_verts_h[i])
            else:
                # Fallback for vertices with no weights (e.g., if parsing was incomplete)
                # Assign to the first joint or keep static
                if self.hanim_humanoid.skin_indices[i]:
                    final_pos = (self.hanim_humanoid.joint_matrices[self.hanim_humanoid.skin_indices[i][0]] @ base_verts_h[i])
                else:
                    final_pos = base_verts_h[i] # If no joints, stay in place

            # Convert back from homogeneous coordinates
            deformed_vertices[i] = final_pos[:3] / final_pos[3] if final_pos[3] != 0 else final_pos[:3]

        # Update ANARI geometry with new vertex positions
        self._update_anari_vertices(deformed_vertices)


    # REPLACEMENT FOR _update_anari_vertices
    def _update_anari_vertices(self, deformed_vertices):
        new_vertex_array = self.device.newArray1D(anari.FLOAT32_VEC3, deformed_vertices)

        # <-- MODIFICATION: Loop through all geometries and update them
        for geom in self.hanim_humanoid.anari_geometries:
            geom.setParameter("vertex.position", anari.ARRAY, new_vertex_array)
            geom.commitParameters()

        self.hanim_humanoid.anari_vertex_array = new_vertex_array

    def _get_interpolated_rotation(self, interpolator, fraction):
        keys, values = interpolator.key, interpolator.quaternionValue

        # Handle edge cases
        if fraction <= keys[0]: return values[0]
        if fraction >= keys[-1]: return values[-1]

        # Find the correct segment for interpolation
        for i in range(len(keys) - 1):
            if keys[i] <= fraction < keys[i+1]:
                # Calculate local fraction within the segment
                t = (fraction - keys[i]) / (keys[i+1] - keys[i])
                q1, q2 = values[i], values[i+1]

                # Spherical Linear Interpolation (slerp)
                dot = np.dot(q1, q2)

                # Ensure shortest path for rotation (if dot product is negative, invert one quaternion)
                if dot < 0: q2, dot = -q2, -dot

                # Handle very close quaternions (linear interpolation is sufficient)
                if dot > 0.9995:
                    result = q1 + t * (q2 - q1)
                else:
                    theta_0 = np.arccos(dot)
                    sin_theta_0 = np.sin(theta_0)

                    # Avoid division by zero if angles are extremely close
                    if sin_theta_0 < 1e-6: return q1

                    s0 = np.sin((1 - t) * theta_0) / sin_theta_0
                    s1 = np.sin(t * theta_0) / sin_theta_0
                    result = (s0 * q1) + (s1 * q2)

                # Normalize the interpolated quaternion
                return result / np.linalg.norm(result)

        return values[-1] # Fallback to last keyframe value


    def _setup_display(self):
        self.cam_state = CameraState(self.initial_camera_pos, self.initial_camera_at)

        self.camera = self.device.newCamera("perspective")
        self.camera.setParameter('aspect', anari.FLOAT32, self.width/self.height)
        self.camera.setParameter('fovy', anari.FLOAT32, math.radians(60.0))
        self.camera.commitParameters() # Commit after setting parameters

        # Choose a renderer (e.g., 'pathtracer' for realism, 'default' for speed/simple)
        renderer = self.device.newRenderer("pathtracer")
        # renderer = self.device.newRenderer("default") # Option for faster rendering
        renderer.setParameter('backgroundColor', anari.FLOAT32_VEC4, (0.0, 0.0, 0.0, 1.0))
        renderer.commitParameters()

        self.frame = self.device.newFrame()
        self.frame.setParameter('size', anari.UINT32_VEC2, (self.width, self.height))
        self.frame.setParameter('channel.color', anari.DATA_TYPE, anari.UFIXED8_RGBA_SRGB) # Request sRGB output
        self.frame.setParameter('world', anari.WORLD, self.world)
        self.frame.setParameter('camera', anari.CAMERA, self.camera)
        self.frame.setParameter('renderer', anari.RENDERER, renderer)
        self.frame.commitParameters()

        # Matplotlib setup for displaying the rendered image
        self.fig, self.ax = plt.subplots(figsize=(self.width/100, self.height/100))
        self.image_display = self.ax.imshow(np.zeros((self.height, self.width, 4), dtype=np.uint8))
        self.ax.set_title("Drag to orbit, Scroll to zoom")
        plt.gca().invert_yaxis() # Invert y-axis to match ANARI's top-left origin for image data
        plt.axis('off') # Hide axes
        self.fig.tight_layout(pad=0) # Remove padding


    def run(self):
        self._setup_interaction_handlers()
        # Use FuncAnimation for continuous updates
        ani = FuncAnimation(self.fig, self._update_frame, cache_frame_data=False, interval=16, blit=True)
        plt.show()

    def _update_frame(self, frame_num):
        elapsed_time = time.time() - self.start_time

        self._animate_scene(elapsed_time) # Update animations
        self._update_camera()             # Update camera based on interaction

        # Render the frame
        self.frame.render()

        # Get the rendered image and update the matplotlib display
        self.image_display.set_data(self.frame.get('channel.color'))
        return [self.image_display] # Return list of artists for blitting optimization

    def _update_camera(self):
        s = self.cam_state

        # Convert spherical coordinates (azimuth, elevation, radius) to Cartesian
        x = s.radius * np.cos(s.elevation) * np.cos(s.azimuth)
        y = s.radius * np.sin(s.elevation)
        z = s.radius * np.cos(s.elevation) * np.sin(s.azimuth)

        pos = s.look_at + np.array([x,y,z]) # Camera position

        self.camera.setParameter('position', anari.FLOAT32_VEC3, pos)
        self.camera.setParameter('at', anari.FLOAT32_VEC3, s.look_at)
        self.camera.setParameter('up', anari.FLOAT32_VEC3, (0,1,0)) # Y-up convention
        self.camera.commitParameters()

    def _setup_interaction_handlers(self):
        self.fig.canvas.mpl_connect('button_press_event', self._on_press)
        self.fig.canvas.mpl_connect('button_release_event', self._on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self.fig.canvas.mpl_connect('scroll_event', self._on_scroll)

    def _on_press(self, event):
        if event.inaxes != self.ax: return
        self.last_mouse_pos = (event.x, event.y)
        self.cam_state.is_dragging = True

    def _on_release(self, event):
        self.cam_state.is_dragging = False

    def _on_motion(self, event):
        if not self.cam_state.is_dragging or event.inaxes != self.ax: return

        dx, dy = event.x - self.last_mouse_pos[0], event.y - self.last_mouse_pos[1]
        self.last_mouse_pos = (event.x, event.y)

        # Adjust azimuth and elevation based on mouse movement
        self.cam_state.azimuth -= dx * 0.01
        self.cam_state.elevation = np.clip(self.cam_state.elevation + dy * 0.01, -np.pi/2 + 0.01, np.pi/2 - 0.01)

    def _on_scroll(self, event):
        if event.inaxes != self.ax: return

        # Adjust camera radius based on scroll direction
        if event.button == 'up':
            self.cam_state.radius = max(0.2, self.cam_state.radius * 0.9)
        else:
            self.cam_state.radius = self.cam_state.radius * 1.1

# --- Main execution block ---
if __name__ == "__main__":
    # Generic placeholder logic to load an X3D scene.
    # You should have 'walking_man_cc_test.py' or 'JoeSkinTexcoordDisplacerKickUpdate2.py'
    # (or similar X3D scene representations generated by x3d_to_python_parser.py)
    # in the same directory.
    try:
        from walking_man_cc_test import X3D0 as scene_to_render
        print("Rendering scene from 'walking_man_cc_test.py'...")
    except ImportError:
        try:
            from JoeSkinTexcoordDisplacerKickUpdate2 import X3D0 as scene_to_render
            print("Rendering scene from 'JoeSkinTexcoordDisplacerKickUpdate2.py'...")
        except ImportError:
            print("Error: Could not find 'walking_man_cc_test.py' or 'JoeSkinTexcoordDisplacerKickUpdate2.py'.")
            print("Please ensure an X3D scene parsed into a Python module is available.")
            exit()

    renderer = InteractiveAnariRenderer(scene_to_render, width=1024, height=768)
    renderer.run()
