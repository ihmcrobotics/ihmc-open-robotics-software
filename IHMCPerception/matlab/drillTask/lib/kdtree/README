Package: kdtree1.2
Author: Andrea Tagliasacchi ata2@cs.sfu.ca
Date: December 8 2008

%------------------  DESCRIPTION -----------------%
kdtree provides a minimalistic implementation of kd-tree.
The implementation can be used either inside MATLAB by means
of MEX calls, or as a standalone tool, directly from a C/C++
program. The image on the website has been creaed with "fulltest.m"

%------------------  FUNCTIONALITIES -----------------%
This implementation offers the following functionalities:  
- kdtree_build: 		        k-d tree construction O( n log^2(n) )
- kdtree_delete:		        frees memory allocated by kdtree
- kdtree_nearest_neighbor:      nearest neighbor query (for one or more points) 
- kdtree_k_nearest_neighbors:   kNN for a single query point  
- kdtree_range_query:           rectangular range query
- kdtree_ball_query:            queries samples withing distance delta from a point  

%------------------  FILE STRUCTURE -----------------%
Everyone of the scripts/functions is complete of the following:
*.cpp:      the mex implementation of the sources
*.mexmaci:  the compiled version of the mex (intel mac)
*.m:        the comments that you can browse with the "help" command
*_demo.m:   demo file to illustrate the behavior

%------------------  HOW COMPILE  -----------------%
IMPORTANT NOTE: I assume you have a correctly configured MEX environment.
Compiling can be done in two ways. The first is directly inside MATLAB.
You can compile manually each of the files by calling the command mex 
within the kdtree folder from the MATLAB command line. For example:

>> mex kdtree_build.cpp 
 
Alternatively, if you are in a unix environment, you might also be able 
to use the provided makefile. In order to do this you need to change some
of the environment variables in order to make them point to your local 
MATLAB installation.
 
%------------------  DEVELOPMENT -----------------%
As mentioned the *.cpp files contain a MEX interface for MATLAB. 
At the same time, a rich set of examples which run as standalone, 
independently from having a MATLAB installation, is provided.
In order to compile them independetly from the MEX environment, 
a preprocessor condition -D CPPONLY need to be used. 
The makefile uses this flags and  compiles sources in both 
environments: C++ and MEX.
 
---
Feedback is greatly appreciated.