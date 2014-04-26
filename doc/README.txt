------------------------
------------------------
Source Code Installation
------------------------
------------------------

Make sure Qt is installed in D:\Qt. If you insist on installing it somewhere else, you'll be required to change all the following:
1. 2d_near_isometric_deformation_withqt -> Properties
lookup Qt references/dependencies in the C/C++ and Linker tabs and change to the appropriate Qt installation path.
don't forget to do it for both release and debug.
2. deformationscene.h -> Properties
under Custom Build Tool, change to the appropriate Qt installation path in the Command Line and Additional Dependencies.
