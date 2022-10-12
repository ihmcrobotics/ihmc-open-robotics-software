@echo off

pushd

pushd "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools"
call VsDevCmd.bat -host_arch=amd64 -arch=amd64
popd

:: Upgrade the current packages
winget upgrade --all

:: Install Visual Studio 2022 BuildTools
winget install -e --id Microsoft.VisualStudio.2022.BuildTools --override "--passive --wait --add Microsoft.VisualStudio.Workload.VCTools;includeRecommended"

:: Install Adoptium JDK 17
winget install -e --id EclipseAdoptium.Temurin.17.JDK

:: Move to user's downloads directory
cd %HOMEPATH%\Downloads

set EIGEN_VERSION=3.4.0

:: Download Eigen from Gitlab
curl https://gitlab.com/libeigen/eigen/-/archive/%EIGEN_VERSION%/eigen-%EIGEN_VERSION%.zip -o eigen-%EIGEN_VERSION%.zip

:: Extract Eigen build
tar -xf eigen-%EIGEN_VERSION%.zip

:: Create Eigen build dir and generate build files
cd eigen-%EIGEN_VERSION%
mkdir build
cd build
cmake ..

:: Run as administrator; gets installed to C:\Program Files (x86)\Eigen3
cmake --build . --target install

popd

pause