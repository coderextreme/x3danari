# --- START OF FILE simple_pbr_scene.py ---
import x3d

X3D0 = x3d.X3D()
X3D0.profile = "Immersive"
X3D0.version = "4.0"

Scene = x3d.Scene()
X3D0.Scene = Scene

# Define a viewpoint to look at the scene
Viewpoint = x3d.Viewpoint()
Viewpoint.DEF = "SimpleView"
Viewpoint.description = "Looking at a PBR sphere and box"
Viewpoint.position = [0, 1, 5] # Camera position
Viewpoint.centerOfRotation = [0, 0.5, 0] # What the camera orbits around
Scene.children.append(Viewpoint)

# Add a light source
DirectionalLight = x3d.DirectionalLight()
DirectionalLight.direction = [-0.5, -1, -0.8]
DirectionalLight.intensity = 1.0
Scene.children.append(DirectionalLight)

# Add a red, metallic sphere
Shape_Sphere = x3d.Shape()
Sphere = x3d.Sphere(radius=0.5)
Appearance_Sphere = x3d.Appearance()
PhysicalMaterial_Sphere = x3d.PhysicalMaterial(
    baseColor=[1.0, 0.1, 0.1], # Red
    metallic=0.8,              # Mostly metallic
    roughness=0.2              # Quite smooth
)
Appearance_Sphere.material = PhysicalMaterial_Sphere
Shape_Sphere.geometry = Sphere
Shape_Sphere.appearance = Appearance_Sphere
Scene.children.append(Shape_Sphere)

# Add a floor box
Shape_Box = x3d.Shape()
# A transform to move the box down to act as a floor
Transform_Box = x3d.Transform(translation=[0, -0.5, 0])
Box = x3d.Box(size=[4, 0.1, 4])
Appearance_Box = x3d.Appearance()
PhysicalMaterial_Box = x3d.PhysicalMaterial(
    baseColor=[0.7, 0.7, 0.7], # Grey
    metallic=0.1,
    roughness=0.8
)
Appearance_Box.material = PhysicalMaterial_Box
Shape_Box.geometry = Box
Shape_Box.appearance = Appearance_Box
Transform_Box.children.append(Shape_Box)
Scene.children.append(Transform_Box)
