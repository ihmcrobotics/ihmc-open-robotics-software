@echo on

:: Requires: cmake, msvc2022, JDK 17
:: Assumes msbuild is in the Path. For VC2022, it's in: C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin

mkdir build
cd build

cmake -DCMAKE_INSTALL_PREFIX=. ..

msbuild promp.sln -t:promp -p:Configuration=Release

:: Use the latest release on GitHub
:: https://github.com/bytedeco/javacpp/releases
set JAVACPP_VERSION=1.5.7

:: Most closely resembles "cp -r" on *nix
xcopy /e /k /h /i ..\src\main\java\ .\java

:: Move into the java directory; javacpp.jar needs to reside here
cd java

:: Windows 10 (as of recently) comes with cURL
curl -L https://github.com/bytedeco/javacpp/releases/download/%JAVACPP_VERSION%/javacpp-platform-%JAVACPP_VERSION%-bin.zip -o javacpp-platform-%JAVACPP_VERSION%-bin.zip
:: Windows 10 also comes with tar, although slightly iffy if you should use it with .zip. It seems to work fine, though.
tar -xf javacpp-platform-1.5.7-bin.zip --strip-components 1

java -jar javacpp.jar us\ihmc\promp\presets\PrompInfoMapper.java
:: This will generate the jni shared library and place it into the classpath resources dir
java -jar javacpp.jar us\ihmc\promp\*.java us\ihmc\promp\presets\*.java us\ihmc\promp\global\*.java -d ..\..\src\main\resources

:: Clean old generated Java code
rm -rf ..\..\src\main\generated-java\*.*

:: Copy newly generated Java into generated-java
robocopy us ..\..\src\main\generated-java /e /xf *.class* presets
