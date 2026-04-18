Create a sophisticated, interactive 3D renderer in Python that can load, animate, and display skinned H-Anim characters defined in X3D scenes. The final output must be a single, complete, and polished Python script (skinned_x3d_renderer.py).
Core Requirements & Libraries:
Rendering Engine: Use the pynari library as the backend for rendering. The renderer should be built on the ANARI (Analytics Rendering Interface) concepts: Device, World, Surface, Geometry, Material, Camera, and Frame.
Display & Interaction: Use matplotlib to create the display window and handle user interaction. The animation should be driven by matplotlib.animation.FuncAnimation.
Scene Selection GUI: Use tkinter to create a simple, centered startup window that allows the user to select from a predefined list of X3D scene modules to render. The provided scene names are: "JoeDemo5JoeSkin5a", "JoeSkinTexcoordDisplacerKickUpdate2", "conan_23_Aug2025", "eric5", "walking_man_cc_test".
Scene Loading: Use importlib to dynamically load the selected scene module. The script should assume each scene module contains an X3D object named X3D0. Include robust error handling for ImportError and AttributeError.
Numerical Operations: Use numpy for all vector, matrix, and quaternion math.
Image Handling: Use Pillow (PIL) for loading texture images, including support for base64-encoded data URIs.
Detailed Feature Implementation:
X3D Scene Traversal:
Implement a traversal function to parse the input X3D object graph.
Build a map of all nodes with a DEF attribute for easy lookup.
Identify and store all TimeSensor, OrientationInterpolator, and ROUTE nodes to drive the animation.
H-Anim (Humanoid Animation) Pipeline:
Skeleton: Parse the HAnimHumanoid node to identify the skeleton hierarchy, starting from the root HAnimJoint. Build a data structure that maps joint names to their properties (parent, children, initial rotation, etc.).
Bind Pose: Recursively calculate the bind pose matrix and its inverse for every joint in the skeleton.
Skinning Data: Read the skinning weights and indices from the HAnimJoint nodes (skinCoordWeight, skinCoordIndex) and associate them with the master vertex list from the skinCoord node.
Geometry Processing:
Handle IndexedFaceSet geometry. Create a helper function unroll_indexed_face_set to convert polygon face sets into a flat list of triangles suitable for rendering, correctly handling both vertex and texture coordinate indices.
Calculate tangent vectors for normal mapping using a robust calculate_tangents helper function that can safely handle degenerate UV triangles.
Animation System:
Interpolation: In the main animation loop, use the TimeSensor's cycleInterval to calculate a normalized animation fraction.
Rotation: Implement quaternion-based animation. Convert the axis-angle keyValue from OrientationInterpolator nodes into quaternions. Use Spherical Linear Interpolation (SLERP) to smoothly calculate the rotation for the current animation fraction.
Skin Deformation:
Each frame, update each joint's current world transformation matrix based on the interpolated animation data and its parent's transform.
Calculate the final skinning matrix for each joint (world_transform @ inverse_bind_pose_matrix).
Deform all skinned vertices on the CPU by applying the weighted sum of the relevant joint skinning matrices.
Update the anari.ARRAY vertex buffer with the new deformed positions and commit it to the ANARI geometry.
ANARI Rendering Setup:
Materials: Use a physicallyBased ANARI material.
Texturing:
Create anari.SAMPLER ("image2D") objects for textures. Support diffuse/base color, normal, and occlusion maps.
Color Space: Implement a helper function srgb_to_linear and apply it to all color texture data before passing it to ANARI to ensure correct physically-based rendering.
Lighting: Do not use lights from the X3D scene. Instead, create an artistic three-point lighting setup for high-quality visuals:
Key Light: A bright, warm-colored directional light.
Fill Light: A dimmer, cool-colored directional light to soften shadows.
Rim Light: A directional light from the back to separate the character from the background.
Render Quality: Set an ambient radiance and enable multisampling (e.g., 4 pixel samples) in the renderer for anti-aliasing.
User Experience & Interactivity:
Camera Control: Implement an orbit camera.
Mouse Drag: Orbit the camera around a central point (azimuth and elevation).
Mouse Scroll: Zoom the camera in and out (adjust radius).
Intro Animation: Create a special introductory animation sequence. For the first 4 seconds, the character should perform a slow 360-degree rotation. After this intro, the main X3D animation should begin looping. The window title should change from a "loading" message to "Drag to orbit, Scroll to zoom" after the intro completes.
