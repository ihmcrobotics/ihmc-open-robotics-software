@echo on

:: Requires: cmake, msvc2022, JDK 17

pushd "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools"
call VsDevCmd.bat -host_arch=amd64 -arch=amd64
popd

:: Most closely resembles "rm -rf" on *nix
rd /s /q build

mkdir build
cd build

cmake -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DBoost_USE_STATIC_LIBS=ON -DCMAKE_INSTALL_PREFIX=. .. || exit /b 1
cmake --build . --config Release || exit /b 1
cmake --install . || exit /b 1

copy /Y .\bin\slam-wrapper.dll ..\resources\slamWrapper\windows-x86_64

:: Use the latest release on GitHub
:: https://github.com/bytedeco/javacpp/releases
set JAVACPP_VERSION=1.5.8

:: Most closely resembles "cp -r" on *nix
mkdir .\java\us\ihmc\perception\slamWrapper\presets
copy /Y ..\java\us\ihmc\perception\slamWrapper\presets\SlamWrapperInfoMapper.java .\java\us\ihmc\perception\slamWrapper\presets

:: Move into the java directory; javacpp.jar needs to reside here
cd java

:: Windows 10 (as of recently) comes with cURL
curl -L https://github.com/bytedeco/javacpp/releases/download/%JAVACPP_VERSION%/javacpp-platform-%JAVACPP_VERSION%-bin.zip -o javacpp-platform-%JAVACPP_VERSION%-bin.zip
:: Windows 10 also comes with tar, although slightly iffy if you should use it with .zip. It seems to work fine, though.
tar -xf javacpp-platform-%JAVACPP_VERSION%-bin.zip --strip-components 1

java -jar javacpp.jar us\ihmc\perception\slamWrapper\presets\SlamWrapperInfoMapper.java || exit /b 1
:: This will generate the jni shared library and place it into the classpath resources dir
java -jar javacpp.jar -Dplatform.compiler.cpp17=/std:c++17 "us\ihmc\perception\slamWrapper\**.java" -d . || exit /b 1

copy /Y .\jniSlamWrapper.dll ..\..\resources\slamWrapper\windows-x86_64

:: Clean old generated Java code
rd /s /q ..\..\src\main\generated-java
mkdir ..\..\src\main\generated-java

:: Copy newly generated Java into generated-java
robocopy us ..\generated-java\us /e /xf *.class* SlamWrapperInfoMapper.java
:: I couldn't get robocopy to exclude a directory by name like this
:: So, just delete it afterwards
rd /s /q ..\generated-java\us\ihmc\perception\slamWrapper\presets

copy /Y "C:\Program Files\GTSAM\bin\gtsam.dll" ..\..\resources\slamWrapper\windows-x86_64

pause