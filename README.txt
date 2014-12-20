
This is implementation of paper
"Planar Shape Interpolation with Bounded Distortion"
http://www.cs.technion.ac.il/~renjie/papers/bdmorph.pdf

The code is based on earlier project, implementing the following paper:
 "As-Killing-As-Possible Vector Fields for Planar Deformation"
http://www.stanford.edu/~justso1/assets/kvf_deformation.pdf

http://www.cs.technion.ac.il/~cggc/Upload/Projects/KVFDeformation/index.html


Supported platforms: Linux,Mac,Windows 
(+theoretically whatever CMake and Qt supports)

To compile you need Qt4,Cmake, ffmpeg libaries (for video output) and 
suitesparse libraries (for matrix solving)

On linux you rougly need these packages:

cmake libqt4-dev libqt4-dev-bin
libsuitesparse-metis-dev
libavcodec-dev libavformat-dev libavutil-dev libswscale-dev

(Exact names will vary from distribution to distribution)

On windows and/or OSX you need somehow to obtain these libraries, install Qt4,Cmake,
and edit the CMakefiles.txt to reflect their locations
Compilation on Windows tested under Visual Studio 2008

Personally I advice you that if you are using Windows, to get a real OS, like
Linux or at least OSX.


