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

        if self.viewpoint_node:
            pos = self.viewpoint_node.position
            axis, angle = self.viewpoint_node.orientation[:3], self.viewpoint_node.orientation[3]
            rot_quat = quat_from_axis_angle(axis, angle)
            rot_mat = get_matrix_from_quat(rot_quat)
            view_vec_h = rot_mat @ np.array([0, 0, -1, 0], dtype=np.float32)
            at = np.array(pos) + view_vec_h[:3]
            self.initial_camera_pos = tuple(pos)
            self.initial_camera_at = tuple(at)
        else:
            self.initial_camera_pos = (0, 1, -5)
            self.initial_camera_at = (0, 0, 0)

        for interp in self.interpolators.values():
            key_values_aa = np.array(interp.keyValue).reshape(-1, 4)
            interp.quaternionValue = [quat_from_axis_angle(aa[:3], aa[3]) for aa in key_values_aa]

        humanoid_node = next((n for n in self.def_map.values() if isinstance(n, x3d.HAnimHumanoid)), None)

        if humanoid_node:
            self.hanim_humanoid = HAnimData()
            self._setup_hanim_rigid(humanoid_node)
            self.world.setParameterArray('surface', anari.SURFACE, self.surfaces)

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

    def _setup_hanim_rigid(self, humanoid_node):
        self.hanim_humanoid.root_joint = humanoid_node.skeleton[0]
        all_vertices, all_indices = [], []

        def build_skeleton_map(joint_node, parent_node=None):
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
                    build_skeleton_map(child, parent_node=joint_node)
        build_skeleton_map(self.hanim_humanoid.root_joint)

        # --- START OF FIX: Loop over all segments per joint ---
        for joint_name, joint_info in self.hanim_humanoid.joints.items():
            joint_node = joint_info['node']

            # This is the starting index for all geometry vertices controlled by this joint.
            joint_vertex_start_index = len(all_vertices)

            # Iterate through ALL HAnimSegment children of this joint.
            for segment_node in (c for c in joint_node.children if isinstance(c, x3d.HAnimSegment)):
                site_node = next((c for c in segment_node.children if isinstance(c, x3d.HAnimSite)), None)
                if not site_node: continue

                final_transform = self._get_transform_matrix_from_site(site_node)
                geom = site_node.children[0].geometry
                verts, inds = [], []

                if isinstance(geom, x3d.Cylinder):
                    h2, r, sides = geom.height/2.0, geom.radius, 16
                    for i in range(sides):
                        angle=2*math.pi*i/sides; x,z=r*math.cos(angle),r*math.sin(angle); verts.extend([[x,-h2,z],[x,h2,z]])
                    for i in range(sides):
                        p1,p2=i*2,((i+1)%sides)*2; inds.extend([p1,p2,p2+1,p1,p2+1,p1+1])
                elif isinstance(geom, x3d.Sphere):
                    r, lats, longs = geom.radius, 10, 10
                    for i in range(lats + 1):
                        lat_angle=math.pi*(-0.5+i/lats); xy,y_val=math.cos(lat_angle),math.sin(lat_angle)
                        for j in range(longs + 1):
                            long_angle=2*math.pi*j/longs; x_val,z_val=math.cos(long_angle),math.sin(long_angle)
                            verts.append([xy*x_val*r,y_val*r,xy*z_val*r])
                    for i in range(lats):
                        for j in range(longs):
                            p1,p2,p3,p4=i*(longs+1)+j,(i+1)*(longs+1)+j,i*(longs+1)+j+1,(i+1)*(longs+1)+j+1
                            inds.extend([p1,p2,p4,p1,p4,p3])
                elif isinstance(geom, x3d.Cone):
                    h2, r, sides = geom.height/2.0, geom.bottomRadius, 16
                    verts.append([0,h2,0]);
                    for i in range(sides):
                        angle=2*math.pi*i/sides; x,z=r*math.cos(angle),r*math.sin(angle); verts.append([x,-h2,z])
                    for i in range(1,sides+1): inds.extend([0,(i%sides)+1,i])

                if verts:
                    # The offset for indices is based on the *current* total number of vertices
                    offset = len(all_vertices)
                    verts_h = np.hstack((np.array(verts, dtype=np.float32), np.ones((len(verts),1))))
                    transformed_verts = (final_transform @ verts_h.T).T[:,:3]
                    all_vertices.extend(transformed_verts.tolist())
                    all_indices.extend((np.array(inds) + offset).tolist())

            # After processing all segments, set the final vertex range for this joint.
            joint_info['vertex_range'] = (joint_vertex_start_index, len(all_vertices))
        # --- END OF FIX ---

        def calculate_bind_poses(joint_name, parent_bind_matrix):
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
            for child_name in joint_info['children']: calculate_bind_poses(child_name, bind_matrix)

        calculate_bind_poses(self.hanim_humanoid.root_joint.DEF, np.identity(4, dtype=np.float32))

        self.hanim_humanoid.base_vertices = np.array(all_vertices, dtype=np.float32)
        self.hanim_humanoid.indices = np.array(all_indices, dtype=np.uint32).reshape(-1, 3)
        geom=self.device.newGeometry("triangle")
        self.hanim_humanoid.anari_vertex_array = self.device.newArray(anari.FLOAT32_VEC3, self.hanim_humanoid.base_vertices)
        i_array=self.device.newArray(anari.UINT32_VEC3, self.hanim_humanoid.indices)
        geom.setParameter("vertex.position", anari.ARRAY, self.hanim_humanoid.anari_vertex_array)
        geom.setParameter("primitive.index", anari.ARRAY, i_array)
        geom.commitParameters()
        self.hanim_humanoid.anari_geometry = geom
        mat=self.device.newMaterial("matte"); mat.setParameter('color',anari.FLOAT32_VEC3,(0.6,0.6,0.6)); mat.commitParameters()
        surface=self.device.newSurface(); surface.setParameter('geometry',anari.GEOMETRY,geom); surface.setParameter('material',anari.MATERIAL,mat); surface.commitParameters()
        self.surfaces.append(surface)

    def _animate_scene(self, t):
        if not self.hanim_humanoid: return
        self._animate_scene_rigid(t)

    def _animate_scene_rigid(self, t):
        timer = next(iter(self.time_sensors.values()), None)
        if not timer: return

        fraction = (t % timer.cycleInterval) / timer.cycleInterval

        for route in self.routes:
            if "value_changed" in route.fromField:
                joint_def = route.toNode
                interp_def = route.fromNode

                interpolator = self.interpolators.get(interp_def)
                if interpolator and joint_def in self.hanim_humanoid.joints:

                    initial_rotation = self.hanim_humanoid.joints[joint_def]['initial_rotation']
                    animation_rotation = self._get_interpolated_value(interpolator, fraction)

                    final_rotation = self._multiply_quaternions(animation_rotation, initial_rotation)

                    self.hanim_humanoid.joints[joint_def]['current_rotation'] = final_rotation

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

        y_rot_180_quat = quat_from_axis_angle([0, 1, 0], math.pi)
        root_transform = get_matrix_from_quat(y_rot_180_quat)
        update_transforms(self.hanim_humanoid.root_joint.DEF, root_transform)

        self.hanim_humanoid.anari_vertex_array = self.device.newArray(anari.FLOAT32_VEC3, deformed_vertices)
        self.hanim_humanoid.anari_geometry.setParameter("vertex.position", anari.ARRAY, self.hanim_humanoid.anari_vertex_array)
        self.hanim_humanoid.anari_geometry.commitParameters()

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
                if dot < 0:
                    q2, dot = -q2, -dot

                if dot > 0.9995:
                    result = q1 + t * (q2 - q1)
                else:
                    theta_0 = np.arccos(dot)
                    sin_theta_0 = np.sin(theta_0)
                    if sin_theta_0 > 1e-6:
                        s0 = np.sin((1 - t) * theta_0) / sin_theta_0
                        s1 = np.sin(t * theta_0) / sin_theta_0
                        result = (s0 * q1) + (s1 * q2)
                    else:
                        result = q1 + t * (q2 - q1)

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
        from walking_man_gemini3 import X3D0 as scene_to_render
        print("Rendering scene from 'walking_man_gemini3.py'...")
    except ImportError as e:
        print(f"Could not import scene file ({e}). Ensure it is in the same directory.")
        scene_to_render = x3d.X3D(Scene=x3d.Scene(children=[x3d.Viewpoint()]))
    renderer = InteractiveAnariRenderer(scene_to_render, width=1024, height=768)
    renderer.run()
