# InjectionTrajectoryPlanner
## Written by: Henry Phalen

Slicer Version 4.10.2
Slicer can be installed as normal - no need to build from source.

Created originally for planning spine injection trajectories, this module allows for the use to set target and entry points for a trajectory and visualize on slices and a 3D rendering of anatomy. 
These trajectories are visualized with a line that is projected into the slice image and an stl model rendering and slice intersections of this model are displayed as well. Currently, this is a needle, but it could be easily changed in the code.

Additionally, once a trajectory is designated, the slices can be toggled between a typical anatomical view (Coronal, Transverse, Saggital planes) and a 'down-axis' trajectory. This switches the axial view to look straight down the line of injection. This view is dynamic, any changes in the two points will cause the image to reslice. Further, care was given to provide a point on 'down trajectory' slice that allows the trajectory to be moved as if at that point (not just at the edges).

The sample images / volumes have been removed for storage. As a result, the import test data button will obviously not work. These can easily be replaced with arbitrary images, or you can just load images into Slicer manually.
