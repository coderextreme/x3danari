import x3d
import pynari as anari
import numpy as np
from PIL import Image
import math
import time
import os

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
    if axis_norm < 1e-6:
        return np.array([0, 0, 0, 1], dtype=np.float32) # [x, y, z, w]
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
        self.is_skinned = False
        # For skinned geo
        self.skin_indices = []
        self.skin_weights = []
        self.joint_matrices = []
        # Common
        self.base_vertices = None
        self.indices = None
        self.anari_geometry = None
        self.anari_vertex_array = None

# --- Main Renderer Class ---
class InteractiveAnariRenderer:
    def __init__(self, x3d_scene, width, height):
        self.width = width
        self.height = height
        self.x3d_scene = x3d_scene
        self.start_time = time.time()
        self.device = anari.newDevice('default')
        if not self.device: raise RuntimeError("Could not create ANARI device.")
        self._initialize_scene()
        self._setup_display()

    def _initialize_scene(self):
        self.world = self.device.newWorld()
        self.surfaces = []
        self.lights = []
        self.def_map = {}
        self.time_sensors = {}
        self.interpolators = {}
        self.routes = []
        self.hanim_humanoid = None
        self.viewpoint_node = None

        scene_node = self.x3d_scene.Scene
        self._traverse(scene_node)

        self.viewpoint_node = self.def_map.get("ThreeQuarterView", self.viewpoint_node)

        if self.viewpoint_node and hasattr(self.viewpoint_node, 'position'):
            pos = self.viewpoint_node.position
            print(f"Using Viewpoint '{self.viewpoint_node.DEF}' at position {pos}")
            self.initial_camera_pos = tuple((pos[0], pos[1], -pos[2]))
            self.initial_camera_at = (0, 1.0, 0) # Look at the character's torso
        else:
            print("No suitable Viewpoint found, using default camera position.")
            self.initial_camera_pos = (0, 1.2, -4.0)
            self.initial_camera_at = (0, 1.0, 0)

        for interp in self.interpolators.values():
            if hasattr(interp, 'keyValue'):
                key_values_aa = np.array(interp.keyValue).reshape(-1, 4)
                interp.quaternionValue = [quat_from_axis_angle(aa[:3], aa[3]) for aa in key_values_aa]

        humanoid_node = next((n for n in self.def_map.values() if isinstance(n, x3d.HAnimHumanoid)), None)

        if humanoid_node:
            self.hanim_humanoid = HAnimData()
            # --- DUAL PATH LOGIC ---
            if hasattr(humanoid_node, 'skinCoord') and hasattr(humanoid_node, 'skin'):
                print("Detected skinned geometry. Using Linear Blend Skinning path.")
                self.hanim_humanoid.is_skinned = True
                self._setup_hanim_skinned(humanoid_node)
            else:
                print("Detected rigid geometry. Using rigid transform path.")
                self.hanim_humanoid.is_skinned = False
                self._setup_hanim_rigid(humanoid_node)

            if self.surfaces:
                self.world.setParameterArray('surface', anari.SURFACE, self.surfaces)

        scene_lights = [node for node in self.def_map.values() if isinstance(node, x3d.DirectionalLight)]
        for light_node in scene_lights:
            light = self.device.newLight("directional")
            light.setParameter('direction', anari.FLOAT32_VEC3, light_node.direction)
            if hasattr(light_node, 'color'):
                light.setParameter('color', anari.FLOAT32_VEC3, light_node.color)
            irradiance = light_node.intensity * 2.5
            light.setParameter('irradiance', anari.FLOAT32, irradiance)
            light.commitParameters()
            self.lights.append(light)

        if not self.lights:
            light = self.device.newLight("directional")
            light.setParameter('direction', anari.FLOAT32_VEC3, (0.2, -1.0, -1.0))
            light.setParameter('irradiance', anari.FLOAT32, 2.0)
            light.commitParameters()
            self.lights.append(light)

        if self.lights:
            self.world.setParameterArray('light', anari.LIGHT, self.lights)
        self.world.commitParameters()

    def _traverse(self, node):
        node_def = getattr(node, 'DEF', None)
        if node_def: self.def_map[node_def] = node

        if isinstance(node, x3d.TimeSensor):
            if node_def: self.time_sensors[node_def] = node
        elif isinstance(node, x3d.OrientationInterpolator):
            if node_def: self.interpolators[node_def] = node
        elif isinstance(node, x3d.ROUTE):
            self.routes.append(node)
        elif isinstance(node, x3d.Viewpoint):
            if not self.viewpoint_node:
                self.viewpoint_node = node

        nodes_to_visit = []
        if hasattr(node, 'children'): nodes_to_visit.extend(node.children)
        if hasattr(node, 'skeleton'): nodes_to_visit.extend(node.skeleton)
        if hasattr(node, 'skin'): nodes_to_visit.extend(node.skin)

        for child in nodes_to_visit:
            self._traverse(child)

    def _get_transform_matrix_from_site(self, node):
        if not isinstance(node, x3d.HAnimSite): return np.identity(4, dtype=np.float32)
        trans = getattr(node, 'translation', (0,0,0)); scale = getattr(node, 'scale', (1,1,1)); rot = getattr(node, 'rotation', (0,1,0,0))
        T = np.identity(4, dtype=np.float32); T[:3, 3] = trans
        R = get_matrix_from_quat(quat_from_axis_angle(rot[:3], rot[3]))
        S = np.diag(list(scale) + [1.0]).astype(np.float32)
        return T @ R @ S

    def _multiply_quaternions(self, q1, q2):
        x1, y1, z1, w1 = q1; x2, y2, z2, w2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2; x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2; z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        result = np.array([x, y, z, w], dtype=np.float32)
        norm = np.linalg.norm(result)
        if norm > 1e-6: result /= norm
        return result

    def _build_skeleton_map(self, humanoid_node):
        self.hanim_humanoid.root_joint = humanoid_node.skeleton[0]
        def build_map_recursive(joint_node, parent_node=None):
            name = joint_node.DEF
            initial_rotation = quat_from_axis_angle(getattr(joint_node, 'rotation', (0,1,0,0))[:3], getattr(joint_node, 'rotation', (0,1,0,0))[3])
            self.hanim_humanoid.joints[name] = {
                'node': joint_node, 'parent': parent_node.DEF if parent_node else None,
                'children': [child.DEF for child in joint_node.children if isinstance(child, x3d.HAnimJoint)],
                'initial_rotation': initial_rotation, 'current_rotation': initial_rotation, 'vertex_range': (0,0),
                'bind_pose_matrix': np.identity(4, dtype=np.float32), 'inv_bind_pose_matrix': np.identity(4, dtype=np.float32)
            }
            for child in joint_node.children:
                if isinstance(child, x3d.HAnimJoint):
                    build_map_recursive(child, parent_node=joint_node)
        build_map_recursive(self.hanim_humanoid.root_joint)

    def _calculate_bind_poses(self, joint_name, parent_bind_matrix):
        joint_info = self.hanim_humanoid.joints[joint_name]
        parent_name = joint_info['parent']
        parent_center = np.array([0,0,0],dtype=np.float32)
        if parent_name: parent_center = np.array(self.hanim_humanoid.joints[parent_name]['node'].center)
        joint_center = np.array(joint_info['node'].center)
        local_translation_mat=np.identity(4,dtype=np.float32); local_translation_mat[:3,3]=joint_center-parent_center
        rotation_mat = get_matrix_from_quat(joint_info['initial_rotation'])
        local_matrix = local_translation_mat @ rotation_mat
        bind_matrix = parent_bind_matrix @ local_matrix
        joint_info['bind_pose_matrix'] = bind_matrix
        joint_info['inv_bind_pose_matrix'] = np.linalg.inv(bind_matrix)
        for child_name in joint_info['children']:
            self._calculate_bind_poses(child_name, bind_matrix)

    def _setup_common_geometry(self, vertices, indices):
        self.hanim_humanoid.base_vertices = np.array(vertices, dtype=np.float32)
        self.hanim_humanoid.indices = np.array(indices, dtype=np.uint32).reshape(-1, 3)

        geom=self.device.newGeometry("triangle")
        self.hanim_humanoid.anari_vertex_array = self.device.newArray(anari.FLOAT32_VEC3, self.hanim_humanoid.base_vertices)
        i_array=self.device.newArray(anari.UINT32_VEC3, self.hanim_humanoid.indices)
        geom.setParameter("vertex.position", anari.ARRAY, self.hanim_humanoid.anari_vertex_array)
        geom.setParameter("primitive.index", anari.ARRAY, i_array)
        geom.commitParameters()
        self.hanim_humanoid.anari_geometry = geom

        mat=self.device.newMaterial("matte"); mat.setParameter('color',anari.FLOAT32_VEC3,(0.8,0.8,0.8)); mat.commitParameters()
        surface=self.device.newSurface(); surface.setParameter('geometry',anari.GEOMETRY,geom); surface.setParameter('material',anari.MATERIAL,mat); surface.commitParameters()
        self.surfaces.append(surface)

    # --- RIGID GEOMETRY PATH ---
    def _setup_hanim_rigid(self, humanoid_node):
        self._build_skeleton_map(humanoid_node)
        all_vertices, all_indices = [], []

        for joint_name, joint_info in self.hanim_humanoid.joints.items():
            joint_node = joint_info['node']
            joint_vertex_start_index = len(all_vertices)

            for segment_node in (c for c in joint_node.children if isinstance(c, x3d.HAnimSegment)):
                site_node = next((c for c in segment_node.children if isinstance(c, x3d.HAnimSite)), None)
                if not site_node: continue
                final_transform = self._get_transform_matrix_from_site(site_node)

                for shape_node in (s for s in site_node.children if isinstance(s, x3d.Shape)):
                    geom = shape_node.geometry; verts, inds = [], []
                    if isinstance(geom, x3d.Cylinder):
                        h2, r, sides = geom.height/2.0, geom.radius, 24
                        for i in range(sides): angle=2*math.pi*i/sides; x,z=r*math.cos(angle),r*math.sin(angle); verts.extend([[x,-h2,z],[x,h2,z]])
                        for i in range(sides): p1,p2=i*2,((i+1)%sides)*2; inds.extend([p1,p2,p2+1,p1,p2+1,p1+1])
                    elif isinstance(geom, x3d.Sphere):
                        r, lats, longs = geom.radius, 16, 24
                        for i in range(lats + 1):
                            lat_angle=math.pi*(-0.5+i/lats); xy,y_val=math.cos(lat_angle),math.sin(lat_angle)
                            for j in range(longs + 1):
                                long_angle=2*math.pi*j/longs; x_val,z_val=math.cos(long_angle),math.sin(long_angle); verts.append([xy*x_val*r,y_val*r,xy*z_val*r])
                        for i in range(lats):
                            for j in range(longs): p1,p2,p3,p4=i*(longs+1)+j,(i+1)*(longs+1)+j,i*(longs+1)+j+1,(i+1)*(longs+1)+j+1; inds.extend([p1,p2,p4,p1,p4,p3])

                    if verts:
                        offset = len(all_vertices)
                        verts_h = np.hstack((np.array(verts, dtype=np.float32), np.ones((len(verts),1))))
                        transformed_verts = (final_transform @ verts_h.T).T[:,:3]
                        all_vertices.extend(transformed_verts.tolist())
                        all_indices.extend((np.array(inds) + offset).tolist())

            joint_info['vertex_range'] = (joint_vertex_start_index, len(all_vertices))

        self._calculate_bind_poses(self.hanim_humanoid.root_joint.DEF, np.identity(4, dtype=np.float32))
        if not all_vertices: print("Warning: No procedural geometry found attached to joints."); return
        self._setup_common_geometry(all_vertices, all_indices)

    # --- SKINNED GEOMETRY PATH ---
    def _setup_hanim_skinned(self, humanoid_node):
        self._build_skeleton_map(humanoid_node)

        coord_node = humanoid_node.skinCoord
        if isinstance(coord_node, x3d.Coordinate) and not coord_node.point and coord_node.USE:
            coord_node = self.def_map.get(coord_node.USE, coord_node)
        if not coord_node or not coord_node.point:
            print("Error: Skinned model has no Coordinate data."); return

        all_vertices = np.array(coord_node.point, dtype=np.float32).reshape(-1, 3)
        all_indices = []
        nodes_to_process = list(humanoid_node.skin)
        while nodes_to_process:
            node = nodes_to_process.pop(0)
            if isinstance(node, x3d.Shape):
                geom = node.geometry
                if isinstance(geom, x3d.IndexedFaceSet) and geom.coordIndex:
                    all_indices.extend(parse_x3d_coord_index(geom.coordIndex))
            elif isinstance(node, x3d.Group) and hasattr(node, 'children'):
                nodes_to_process.extend(node.children)

        self._calculate_bind_poses(self.hanim_humanoid.root_joint.DEF, np.identity(4, dtype=np.float32))

        num_verts = len(all_vertices)
        self.hanim_humanoid.skin_indices = [[] for _ in range(num_verts)]
        self.hanim_humanoid.skin_weights = [[] for _ in range(num_verts)]

        joint_name_to_index = {name: i for i, name in enumerate(self.hanim_humanoid.joints.keys())}
        num_joints = len(self.hanim_humanoid.joints)
        self.hanim_humanoid.joint_matrices = [np.identity(4, dtype=np.float32) for _ in range(num_joints)]

        for joint_name, joint_info in self.hanim_humanoid.joints.items():
            joint_node = joint_info['node']
            joint_idx = joint_name_to_index[joint_name]
            if hasattr(joint_node, 'skinCoordIndex') and hasattr(joint_node, 'skinCoordWeight'):
                for v_idx, weight in zip(joint_node.skinCoordIndex, joint_node.skinCoordWeight):
                    if v_idx < num_verts:
                        self.hanim_humanoid.skin_indices[v_idx].append(joint_idx)
                        self.hanim_humanoid.skin_weights[v_idx].append(weight)

        self._setup_common_geometry(all_vertices, all_indices)

    # --- ANIMATION LOGIC ---
    def _animate_scene(self, t):
        if not self.hanim_humanoid or self.hanim_humanoid.base_vertices is None: return

        if self.hanim_humanoid.is_skinned:
            self._animate_scene_skinned(t)
        else:
            self._animate_scene_rigid(t)

    def _update_joint_rotations(self, t):
        timer = next(iter(self.time_sensors.values()), None)
        if not timer: return
        fraction = (t % timer.cycleInterval) / timer.cycleInterval

        for route in self.routes:
            if "value_changed" in route.fromField:
                joint_def, interp_def = route.toNode, route.fromNode
                interpolator = self.interpolators.get(interp_def)
                if interpolator and joint_def in self.hanim_humanoid.joints:
                    joint_info = self.hanim_humanoid.joints[joint_def]
                    animation_rotation = self._get_interpolated_value(interpolator, fraction)
                    final_rotation = self._multiply_quaternions(animation_rotation, joint_info['initial_rotation'])
                    joint_info['current_rotation'] = final_rotation

    def _animate_scene_rigid(self, t):
        self._update_joint_rotations(t)
        deformed_vertices = np.copy(self.hanim_humanoid.base_vertices)

        def update_transforms(joint_name, parent_world_matrix):
            joint_info = self.hanim_humanoid.joints[joint_name]
            parent_name = joint_info['parent']
            parent_center = np.array([0,0,0],dtype=np.float32)
            if parent_name: parent_center = np.array(self.hanim_humanoid.joints[parent_name]['node'].center)
            joint_center = np.array(joint_info['node'].center)
            local_translation_mat=np.identity(4,dtype=np.float32); local_translation_mat[:3,3]=joint_center-parent_center
            rotation_mat = get_matrix_from_quat(joint_info['current_rotation'])
            local_matrix = local_translation_mat @ rotation_mat
            world_matrix = parent_world_matrix @ local_matrix
            transform_for_verts = world_matrix @ joint_info['inv_bind_pose_matrix']
            start, end = joint_info['vertex_range']
            if end > start:
                verts_h = np.hstack((self.hanim_humanoid.base_vertices[start:end], np.ones((end - start, 1))))
                transformed_verts = (transform_for_verts @ verts_h.T).T[:,:3]
                deformed_vertices[start:end] = transformed_verts
            for child_name in joint_info['children']: update_transforms(child_name, world_matrix)

        update_transforms(self.hanim_humanoid.root_joint.DEF, np.identity(4, dtype=np.float32))
        self._update_anari_vertices(deformed_vertices)

    def _animate_scene_skinned(self, t):
        self._update_joint_rotations(t)

        def update_skinning_matrices(joint_name, parent_world_matrix):
            joint_info = self.hanim_humanoid.joints[joint_name]
            parent_name = joint_info['parent']
            parent_center = np.zeros(3, dtype=np.float32)
            if parent_name: parent_center = np.array(self.hanim_humanoid.joints[parent_name]['node'].center)

            joint_center = np.array(joint_info['node'].center)
            local_translation_mat = np.identity(4, dtype=np.float32); local_translation_mat[:3,3] = joint_center - parent_center
            rotation_mat = get_matrix_from_quat(joint_info['current_rotation'])
            local_matrix = local_translation_mat @ rotation_mat
            world_matrix = parent_world_matrix @ local_matrix

            joint_idx = [i for i, name in enumerate(self.hanim_humanoid.joints.keys()) if name == joint_name][0]
            self.hanim_humanoid.joint_matrices[joint_idx] = world_matrix @ joint_info['inv_bind_pose_matrix']

            for child_name in joint_info['children']:
                update_skinning_matrices(child_name, world_matrix)

        # --- START OF FIX: Apply a 180-degree rotation to the root ---
        y_rot_180_quat = quat_from_axis_angle([0, 1, 0], math.pi)
        root_transform = get_matrix_from_quat(y_rot_180_quat)
        update_skinning_matrices(self.hanim_humanoid.root_joint.DEF, root_transform)
        # --- END OF FIX ---

        base_verts_h = np.hstack((self.hanim_humanoid.base_vertices, np.ones((len(self.hanim_humanoid.base_vertices), 1), dtype=np.float32)))
        deformed_vertices = np.zeros_like(self.hanim_humanoid.base_vertices)

        for i in range(len(deformed_vertices)):
            final_pos = np.zeros(4, dtype=np.float32)
            total_weight = 0.0
            for j_idx, weight in zip(self.hanim_humanoid.skin_indices[i], self.hanim_humanoid.skin_weights[i]):
                skinning_matrix = self.hanim_humanoid.joint_matrices[j_idx]
                final_pos += weight * (skinning_matrix @ base_verts_h[i])
                total_weight += weight

            if total_weight > 1e-5:
                final_pos /= total_weight
            else:
                if self.hanim_humanoid.skin_indices[i]:
                    j_idx = self.hanim_humanoid.skin_indices[i][0]
                    skinning_matrix = self.hanim_humanoid.joint_matrices[j_idx]
                    final_pos = skinning_matrix @ base_verts_h[i]
                else:
                    final_pos = base_verts_h[i]

            deformed_vertices[i] = final_pos[:3]

        self._update_anari_vertices(deformed_vertices)

    def _update_anari_vertices(self, deformed_vertices):
        new_vertex_array = self.device.newArray(anari.FLOAT32_VEC3, deformed_vertices)
        self.hanim_humanoid.anari_geometry.setParameter("vertex.position", anari.ARRAY, new_vertex_array)
        self.hanim_humanoid.anari_geometry.commitParameters()
        self.hanim_humanoid.anari_vertex_array = new_vertex_array

    def _get_interpolated_value(self, interpolator, fraction):
        keys = interpolator.key
        values = interpolator.quaternionValue

        if fraction <= keys[0]: return values[0]
        if fraction >= keys[-1]: return values[-1]

        for i in range(len(keys) - 1):
            if keys[i] <= fraction < keys[i+1]:
                t = (fraction - keys[i]) / (keys[i+1] - keys[i])
                q1, q2 = values[i], values[i+1]
                dot = np.dot(q1, q2)
                if dot < 0: q2, dot = -q2, -dot
                if dot > 0.9995: result = q1 + t * (q2 - q1)
                else:
                    theta_0 = np.arccos(dot)
                    sin_theta_0 = np.sin(theta_0)
                    if sin_theta_0 > 1e-6:
                        s0 = np.sin((1 - t) * theta_0) / sin_theta_0
                        s1 = np.sin(t * theta_0) / sin_theta_0
                        result = (s0 * q1) + (s1 * q2)
                    else: result = q1 + t * (q2 - q1)
                return result / np.linalg.norm(result)
        return values[-1]

    def _setup_display(self):
        self.cam_state=CameraState(self.initial_camera_pos,self.initial_camera_at)
        self.camera=self.device.newCamera("perspective"); self.camera.setParameter('aspect',anari.FLOAT32,self.width/self.height); self.camera.setParameter('fovy',anari.FLOAT32,math.radians(60.0))
        renderer=self.device.newRenderer("default")
        bg_values=np.array(((.9,.9,.9,1.),(.15,.25,.8,1.)),dtype=np.float32).reshape((2,1,4)); bg_gradient=self.device.newArray(anari.FLOAT32_VEC4,bg_values); renderer.setParameter('background',anari.ARRAY,bg_gradient); renderer.commitParameters()
        self.frame=self.device.newFrame(); self.frame.setParameter('size',anari.UINT32_VEC2,(self.width,self.height)); self.frame.setParameter('channel.color',anari.DATA_TYPE,anari.UFIXED8_RGBA_SRGB)
        self.frame.setParameter('world',anari.WORLD,self.world); self.frame.setParameter('camera',anari.CAMERA,self.camera); self.frame.setParameter('renderer',anari.RENDERER,renderer); self.frame.commitParameters()
        self.fig, self.ax=plt.subplots(figsize=(self.width/100,self.height/100)); self.image_display=self.ax.imshow(np.zeros((self.height,self.width,4),dtype=np.uint8))
        self.ax.set_title("Drag to orbit, Scroll to zoom"); plt.gca().invert_yaxis(); plt.axis('off'); self.fig.tight_layout(pad=0)

    def run(self):
        self._setup_interaction_handlers()
        ani=FuncAnimation(self.fig, self._update_frame, cache_frame_data=False, interval=16, blit=True)
        plt.show()

    def _update_frame(self, frame_num):
        elapsed_time=time.time()-self.start_time; self._animate_scene(elapsed_time); self._update_camera(); self.frame.render(); pixel_data=self.frame.get('channel.color'); self.image_display.set_data(pixel_data); return [self.image_display]

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

# --- Main execution block ---
if __name__ == "__main__":
    try:
        from eric5 import X3D0 as scene_to_render
        print("Rendering scene from 'eric5.py'...")
    except ImportError as e:
        print(f"Could not import scene file ({e}). Ensure it is in the same directory.")
        scene_to_render = x3d.X3D(Scene=x3d.Scene(children=[x3d.Viewpoint()]))
    renderer = InteractiveAnariRenderer(scene_to_render, width=1024, height=768)
    renderer.run()
