Dependencies

* download boost prebuilt win64 vc12 library intsalled to default c:\local\boost_1_58_0
* download Eigen header put in linemod/Eigen
* download flann header put in linemod/flann

Build

Open a VC++ prompt from start menu;
chdir to linemod

mkdir build
cd build
\IHMCPerception\csrc\linemod\build>cmake .. -DPC_EIGEN_INCLUDEDIR=.. -DBOOST_
INCLUDEDIR=c:\local\boost_1_58_0 -DBOOST_LIBRARYDIR=c:\local\boost_1_58_0\lib64-
msvc-12.0

open the solution file with visual studio and build the install target.

this should compile and put the binary in the resource folder