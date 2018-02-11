go-icpz
------------------

Purpose
-------

Open source implementations of the Globally Optimal Iterative Closest Point algorithm for point cloud based surface matching.


Features
--------

The main features provided are:

TBD


Usage
-----

The main way to use this project is:

TBD


A Note on Packaging
-------------------

TBD


Supported Use-Cases
-------------------

TBD


Tested On
---------

 * Windows - Windows 8, VS2013, CMake 3.6.3, Qt 5.4.2
 * Linux - Centos 7, g++ 4.8.5, CMake 3.5.1, Qt 5.6.2
 * Mac - OSX 10.10.5, clang 6.0, CMake 3.9.4, Qt 5.6.2

Minimum CMake version is 3.5. Minimum Qt is version 5. Qt4 is not supported and not planned to be supported.


Build Instructions
------------------

This project can be configured to build against Eigen, Boost, OpenCV, glog, gflags, VTK and PCL.
These were chosen as an example of how to use CMake, and some common
C++ projects. These dependencies are optional, and this project
will compile without them.

Furthermore, these dependencies can be downloaded and built,
or the user can specify directories of previously compiled
libraries.

To download and build dependencies, use CMake to set:

  * BUILD_SUPERBUILD:BOOL=ON

where ```ON``` is the default. Then to build any of Eigen, Boost or OpenCV etc., use CMake to set:

  * BUILD_Eigen:BOOL=ON|OFF
  * BUILD_Boost:BOOL=ON|OFF
  * BUILD_OpenCV:BOOL=ON|OFF

and so on. If BUILD_SUPERBUILD=OFF, and these variables are on, then CMake will just try finding
locally installed versions rather then downloading them.

To switch between static/dynamic linking, use CMake to set:

  * BUILD_SHARED_LIBS:BOOL=ON|OFF

To switch between Debug and Release mode, use CMake to set:

  * CMAKE_BUILD_TYPE:STRING=Debug|Release

Note: Only Debug and Release are supported. 

As mentioned in lectures, CMake will find 3rd party libraries using either
  1. a FindModule.cmake included within CMake's distribution, e.g. Boost
  2. a custom made FindModule.cmake, e.g. Eigen
  3. using CMAKE_PREFIX_PATH and 'config mode' e.g. OpenCV

(where Module is the name of your module, e.g. OpenCV, Boost).

Note: your host system is very likely to have a version of Boost that
is different to the one provided here. So if you want to turn Boost on,
you should probably try and use the one provided by this SuperBuild.


Windows Users
-------------

If you build the project with shared libraries (BUILD_SHARED_LIBS:BOOL=ON)
then after the SuperBuild has successfully completed, you should look for the batch file
StartVS_Debug.bat or StartVS_Release.bat in the GOICPZ-build folder.
This sets the path before launching Visual Studio, so that when you come to run your
application or unit tests within Visual Studio, the dynamically
loaded libraries are found at run time.


Preferred Branching Workflow
----------------------------

 1. Raise issue in this project's Github Issue Tracker.
 2. Create a feature branch called ```<issue-number>-<some-short-description>```
    replacing ```<issue-number>``` with the Github issue number
    and ```<some-short-description>``` with your description of the thing you are implementing.
 4. Code on that branch, making sure each commit contains "Issue #<issue-number>" replacing ```<issue-number>``` with the Github issue number you are working on.
 5. Push to your remote when ready.
 6. Create pull request to indicate that I should review it.
 7. We will review code, and merge to master when it looks ready.
