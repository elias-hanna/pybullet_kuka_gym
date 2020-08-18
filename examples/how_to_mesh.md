# Step 1
## Create a volume in gmsh for the mesh you want to make
# Step 2
## Create a physical group for this volume (and ONLY for the volume)
# Step 3
## Mesh the volume (MMG3D generates only tetrahedra maybe others work too)
# Step 4
## Save as a mesh file (.msh)
# Step 5
## Open the mesh file in gmsh
# Step 6
## Save as a vtk file
# Miscellaneous
## Some good parameters
### 30 smoothing steps, 0.4 element size factor (for a small object)
