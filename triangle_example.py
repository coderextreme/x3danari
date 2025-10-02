# --- START OF FILE triangle_example_fixed.py ---

import pynari as anari
import numpy as np
from PIL import Image
import os

def main():
    """
    A final, working "Hello, Triangle" example for pynari v1.4.0.
    This version includes the essential fixes for geometry indices and camera fovy.
    """
    print("Setting up ANARI device...")
    try:
        device = anari.newDevice('default')
        print("Device created.")
    except Exception as e:
        print(f"Error creating device: {e}")
        return

    handle_keeper = []
    img_width, img_height = 512, 512

    # --- Create, Configure, and COMMIT each Scene Component ---
    print("1. Creating and committing scene objects individually...")

    # -- Geometry --
    print("   - Creating geometry...")
    mesh = device.newGeometry("triangle")
    handle_keeper.append(mesh)

    vertex_positions = np.array([-0.8, -0.8, 0.0, 0.8, -0.8, 0.0, 0.0, 0.8, 0.0], dtype=np.float32)
    pos_array = device.newArray1D(anari.FLOAT32_VEC3, vertex_positions)
    handle_keeper.append(pos_array)
    mesh.setParameter("vertex.position", anari.ARRAY, pos_array)

    # FIX 1: A "triangle" geometry needs an index array to define primitives.
    # This tells ANARI to connect vertices 0, 1, and 2 to form a triangle.
    vertex_indices = np.array([0, 1, 2], dtype=np.uint32)
    idx_array = device.newArray1D(anari.UINT32_VEC3, vertex_indices)
    handle_keeper.append(idx_array)
    mesh.setParameter("primitive.index", anari.ARRAY, idx_array)

    mesh.commitParameters()

    # -- Material (Emissive) --
    print("   - Creating emissive material...")
    material = device.newMaterial("matte")
    handle_keeper.append(material)
    material.setParameter("emissive", anari.FLOAT32_VEC3, (1.0, 0.0, 0.0))
    material.commitParameters()

    # -- Surface --
    print("   - Creating surface...")
    surface = device.newSurface()
    handle_keeper.append(surface)
    surface.setParameter("geometry", anari.GEOMETRY, mesh)
    surface.setParameter("material", anari.MATERIAL, material)
    surface.commitParameters()

    # -- World --
    print("   - Creating world...")
    world = device.newWorld()
    handle_keeper.append(world)
    world.setParameterArray1D("surface", anari.SURFACE, [surface])
    world.commitParameters()

    # -- Camera --
    print("   - Creating camera...")
    camera = device.newCamera("perspective")
    handle_keeper.append(camera)
    camera.setParameter("aspect", anari.FLOAT32, float(img_width) / img_height)
    camera.setParameter("position", anari.FLOAT32_VEC3, (0., 0., 3.))
    # FIX 2: A perspective camera needs a field of view (fovy) to be defined.
    camera.setParameter("fovy", anari.FLOAT32, np.deg2rad(60.0))
    camera.commitParameters()

    # -- Renderer --
    print("   - Creating renderer...")
    renderer = device.newRenderer("default")
    handle_keeper.append(renderer)
    renderer.setParameter("backgroundColor", anari.FLOAT32_VEC4, (0.1, 0.1, 0.1, 1.0))
    renderer.setParameter("pixelSamples", anari.INT32, 4)
    renderer.commitParameters()

    # --- Create, Configure, and COMMIT the Frame ---
    print("2. Creating and committing the frame...")
    frame = device.newFrame()
    handle_keeper.append(frame)
    frame.setParameter("size", anari.UINT32_VEC2, (img_width, img_height))
    frame.setParameter("color", anari.DATA_TYPE, anari.UFIXED8_VEC4)
    frame.setParameter("world", anari.WORLD, world)
    frame.setParameter("camera", anari.CAMERA, camera)
    frame.setParameter("renderer", anari.RENDERER, renderer)
    frame.commitParameters()

    # --- Render the validated frame ---
    print("3. Rendering the frame...")
    frame.render()
    print("   Render call finished (it is a blocking call).")

    # --- Map results and save ---
    print("4. Mapping pixel data...")
    rendered_image_data = frame.map("color")

    if not isinstance(rendered_image_data, int):
        print("   Render SUCCEEDED. Saving image...")
        rendered_image = Image.fromarray(rendered_image_data, "RGBA")
        output_filename = "anari_hello_triangle.png"
        rendered_image.save(output_filename)
        print(f"   Image saved to '{os.path.abspath(output_filename)}'")
    else:
        print(f"   Render FAILED. frame.map() returned an integer: {rendered_image_data}")

    # --- Final Cleanup ---
    print("5. Releasing ANARI resources...")
    for handle in reversed(handle_keeper):
        handle.release()
    print("Done. Device is released when script ends.")

if __name__ == "__main__":
    main()
