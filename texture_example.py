import pynari as anari
import numpy as np
from PIL import Image
import os

def main():
    """
    A full, working example for pynari v1.4.0.
    Final correct API pattern:
    - Parameters are set with object.setParameter(name, type, value).
    - Each object is committed with object.commitParameters().
    - ALL handles (objects and arrays) are kept in a list to prevent GC bugs.
    - ALL handles are manually released with object.release() after rendering.
    """
    print("Setting up ANARI device...")
    try:
        device = anari.newDevice('default')
        print("Device created.")
    except Exception as e:
        print(f"Error creating device: {e}")
        return

    # This list prevents the garbage collector from prematurely destroying any handles.
    handle_keeper = []

    img_width, img_height = 512, 512
    texture_filename = "texture.png"

    # --- 1. Create, Configure, and Commit Texture Resources ---
    print("1. Creating texture resources...")

    try:
        img = Image.open(texture_filename).convert("RGBA")
        img_data = np.array(img).astype(np.uint8)
        img_width, img_height = img.size
        print(f"   - Successfully loaded '{texture_filename}'.")
    except (IOError, FileNotFoundError):
        print(f"   - Warning: '{texture_filename}' not found. Generating a procedural checkerboard texture.")
        c1 = np.array([255, 0, 255, 255], dtype=np.uint8)
        c2 = np.array([0, 255, 0, 255], dtype=np.uint8)
        img_data = np.zeros((img_height, img_width, 4), dtype=np.uint8)
        for y in range(img_height):
            for x in range(img_width):
                if ((x // 64) + (y // 64)) % 2 == 0:
                    img_data[y, x] = c1
                else:
                    img_data[y, x] = c2

    texture_format = anari.UINT8_VEC4
    image_array = device.newArray2D(texture_format, img_data)
    handle_keeper.append(image_array)

    image_field = device.newSpatialField("image2D")
    handle_keeper.append(image_field)
    image_field.setParameter("size", anari.UINT32_VEC2, (img_width, img_height))
    image_field.setParameter("type", anari.DATA_TYPE, texture_format)
    image_field.setParameter("data", anari.ARRAY, image_array)
    image_field.commitParameters()
    print("   - Image field committed.")

    sampler = device.newSampler("image2D")
    handle_keeper.append(sampler)
    sampler.setParameter("image", anari.SPATIAL_FIELD, image_field)
    sampler.commitParameters()
    print("   - Sampler committed.")

    # --- 2. Create, Configure, and Commit Scene Geometry and Appearance ---
    print("2. Creating scene geometry and appearance...")

    mesh = device.newGeometry("triangle")
    handle_keeper.append(mesh)
    vertex_positions = np.array([-1.0, -1.0, 0.0, 1.0, -1.0, 0.0, 1.0, 1.0, 0.0, -1.0, 1.0, 0.0], dtype=np.float32)
    vertex_texcoords = np.array([0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0], dtype=np.float32)
    primitive_indices = np.array([0, 1, 2, 2, 3, 0], dtype=np.uint32)

    pos_array = device.newArray1D(anari.FLOAT32_VEC3, vertex_positions)
    handle_keeper.append(pos_array)
    mesh.setParameter("vertex.position", anari.ARRAY, pos_array)

    tex_array = device.newArray1D(anari.FLOAT32_VEC2, vertex_texcoords)
    handle_keeper.append(tex_array)
    mesh.setParameter("vertex.texcoord", anari.ARRAY, tex_array)

    idx_array = device.newArray1D(anari.UINT32_VEC3, primitive_indices)
    handle_keeper.append(idx_array)
    mesh.setParameter("primitive.index", anari.ARRAY, idx_array)
    mesh.commitParameters()
    print("   - Geometry committed.")

    material = device.newMaterial("physicallyBased")
    handle_keeper.append(material)
    material.setParameter("baseColor", anari.FLOAT32_VEC3, (1.0, 1.0, 1.0))
    material.setParameter("baseColorTexture", anari.SAMPLER, sampler)
    material.commitParameters()
    print("   - Material committed.")

    # --- 3. Create, Configure, and Commit Scene Hierarchy and Lighting ---
    print("3. Assembling scene...")

    surface = device.newSurface()
    handle_keeper.append(surface)
    surface.setParameter("geometry", anari.GEOMETRY, mesh)
    surface.setParameter("material", anari.MATERIAL, material)
    surface.commitParameters()
    print("   - Surface committed.")

    light = device.newLight("directional")
    handle_keeper.append(light)
    light.setParameter("direction", anari.FLOAT32_VEC3, (0.0, -1.0, -1.0))
    light.setParameter("color", anari.FLOAT32_VEC3, (1.0, 1.0, 1.0))
    light.setParameter("irradiance", anari.FLOAT32, 3.0)
    light.commitParameters()
    print("   - Light committed.")

    world = device.newWorld()
    handle_keeper.append(world)
    world.setParameterArray1D("surface", anari.SURFACE, [surface])
    world.setParameterArray1D("light", anari.LIGHT, [light])
    world.commitParameters()
    print("   - World committed.")

    # --- 4. Create, Configure, and Commit Camera and Renderer ---
    print("4. Setting up camera and renderer...")

    camera = device.newCamera("perspective")
    handle_keeper.append(camera)
    camera.setParameter("aspect", anari.FLOAT32, float(img_width) / img_height)
    camera.setParameter("position", anari.FLOAT32_VEC3, (0., 0., 2.5))
    camera.commitParameters()
    print("   - Camera committed.")

    renderer = device.newRenderer("default")
    handle_keeper.append(renderer)
    renderer.setParameter("backgroundColor", anari.FLOAT32_VEC4, (0.1, 0.1, 0.1, 1.0))
    renderer.commitParameters()
    print("   - Renderer committed.")

    # --- 5. Create, Commit, and Render the Frame ---
    print("5. Rendering...")
    frame = device.newFrame()
    handle_keeper.append(frame)
    frame.setParameter("size", anari.UINT32_VEC2, (img_width, img_height))
    frame.setParameter("color", anari.DATA_TYPE, anari.UFIXED8_VEC4)
    frame.setParameter("world", anari.WORLD, world)
    frame.setParameter("camera", anari.CAMERA, camera)
    frame.setParameter("renderer", anari.RENDERER, renderer)
    frame.commitParameters()
    print("   - Frame committed.")

    frame.render()
    print("   - Render call finished.")

    # --- 6. Map results and save ---
    print("6. Saving output...")
    rendered_image_data = frame.map("color")

    if not isinstance(rendered_image_data, int):
        print("   - Render SUCCEEDED.")
        rendered_image = Image.fromarray(rendered_image_data, "RGBA")
        output_filename = "anari_textured_quad.png"
        rendered_image.save(output_filename)
        print(f"   - Image saved to '{os.path.abspath(output_filename)}'")
    else:
        print(f"   - Render FAILED. frame.map() returned an integer: {rendered_image_data}")

    # --- 7. Final Cleanup ---
    # Manually release all handles in reverse order of creation
    # to ensure no object is destroyed while still in use.
    print("7. Releasing ANARI resources...")
    for handle in reversed(handle_keeper):
        handle.release()
    device.release()
    print("Done.")

if __name__ == "__main__":
    main()
