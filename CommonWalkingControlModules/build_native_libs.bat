@echo open VisualStudioTool\Nativex64 promprt with administrative right from startmenu.
@echo Please install Visual Studio 2013 Professional (Visual Studio Express can't compile to native x64)
@echo change JDK home in ..\CMakeLists.txt
rmdir /s /q  build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=MinSizeRel -G "Visual Studio 12 Win64" ..
REM msbuild CommonWalkingControlModules.sln /p:Configuration=MinSizeRel
REM msbuild CommonWalkingControlModules.sln /p:Configuration=Release
msbuild INSTALL.vcxproj /p:Configuration=Release
msbuild INSTALL.vcxproj /p:Configuration=MinSizeRel

cd ..