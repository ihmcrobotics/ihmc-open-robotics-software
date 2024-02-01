@echo on

:: Requires: msvc2022 BuildTools, Eigen, JDK 17

pushd "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools"
call VsDevCmd.bat -host_arch=amd64 -arch=amd64
popd

:: Most closely resembles "rm -rf" on *nix
rd /s /q build

mkdir build
cd build

cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=. ..

nmake || exit /b 1
nmake install || exit /b 1

xcopy /Y promp.dll ..\src\main\resources\promp\windows-x86_64

:: msbuild promp.sln -t:promp -p:Configuration=Release || exit /b 1

:: Use the latest release on GitHub
:: https://github.com/bytedeco/javacpp/releases
set JAVACPP_VERSION=1.5.8

:: Most closely resembles "cp -r" on *nix
mkdir .\java\us\ihmc\promp\presets
xcopy /Y ..\src\main\java\us\ihmc\promp\presets\ProMPInfoMapper.java .\java\us\ihmc\promp\presets

:: Move into the java directory; javacpp.jar needs to reside here
cd java

:: Windows 10 (as of recently) comes with cURL
curl -L https://github.com/bytedeco/javacpp/releases/download/%JAVACPP_VERSION%/javacpp-platform-%JAVACPP_VERSION%-bin.zip -o javacpp-platform-%JAVACPP_VERSION%-bin.zip
:: Windows 10 also comes with tar, although slightly iffy if you should use it with .zip. It seems to work fine, though.
tar -xf javacpp-platform-%JAVACPP_VERSION%-bin.zip --strip-components 1

java -jar javacpp.jar us\ihmc\promp\presets\ProMPInfoMapper.java || exit /b 1
:: This will generate the jni shared library and place it into the classpath resources dir
java -jar javacpp.jar -Dplatform.compiler.cpp17=/std:c++17 "us\ihmc\promp\**.java" -d . || exit /b 1

xcopy /Y jnipromp.dll ..\..\src\main\resources\promp\windows-x86_64

:: Clean old generated Java code
rd /s /q ..\..\src\main\generated-java
mkdir ..\..\src\main\generated-java

:: Copy newly generated Java into generated-java
robocopy us ..\..\src\main\generated-java\us /e /xf *.class* ProMPInfoMapper.java
:: I couldn't get robocopy to exclude a directory by name like this
:: So, just delete it afterwards
rd /s /q ..\..\src\main\generated-java\us\ihmc\promp\presets

pause
