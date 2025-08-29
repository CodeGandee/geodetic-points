# Earth Mesh Files

This directory should contain the Earth mesh file for 3D visualization.

## Required Files

- `earth.dae` - Collada format Earth sphere mesh with UV mapping

## Creating the Earth Mesh

Use Blender to create the mesh:

1. Add UV Sphere (default settings are fine)
2. Apply Shade Smooth
3. UV unwrap (should already be proper for default sphere)
4. Assign material with image texture `earth.jpg` (equirectangular)
5. Export as Collada (.dae format)
6. Place the exported file as `earth.dae` in this directory

## Alternative Formats

The package also supports other mesh formats like `.glb` if preferred.
Update the `mesh_resource` parameter in the launch file accordingly.