From:
http://g3d.cs.williams.edu/g3d/data10/index.html

Mesh changes:
* Repositioned head mesh
* Combined all meshes into a single mesh
* Removed duplicate vertices
* Renamed to "head_revised.obj"
* Assigned texture to mesh

Textures changes:
* Renamed "lambertian.jpg" to "HeadTexture_BaseColor.png"
  * Changed resolution to 2048
  * Changed the saturation/hue in the image
* Normal map built from a downscaled height map (named "HeadTexture_Normal.png")
* Roughness map built from diffuse map (named "HeadTexture_Roughness.png")
* Subsurface scattering map built from diffuse map (named "HeadTexture_SSS.png")
  * Increased subsurface scattering strength
* AO map built from mesh with Blender's bake functionality (named "HeadTexture_AO.png")
* Created compressions versions of all textures (.dds)
  * BaseColor, Roughness, SSS, and AO use BC1
  * Normal is uncompressed