@echo on

:: Requires: cmake, msvc2022, JDK 17
:: Assumes msbuild is in the Path. For VC2022, it's in: C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin
:: Assumes this is also in the Path: C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.33.31629\bin\Hostx64\x64

pushd "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools"
call VsDevCmd.bat
popd

:: Most closely resembles "rm -rf" on *nix
rd /s /q build

mkdir build
cd build

cmake -DCMAKE_INSTALL_PREFIX=. ..

msbuild promp.sln -t:promp -p:Configuration=Release || exit /b 1

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

java -jar javacpp.jar us\ihmc\promp\presets\PrompInfoMapper.java || exit /b 1
:: This will generate the jni shared library and place it into the classpath resources dir
java -jar javacpp.jar -Dplatform.compiler.cpp17=/std:c++17 "us\ihmc\promp\**.java" -d ..\..\src\main\resources || exit /b 1

:: Clean old generated Java code
rd /s /q ..\..\src\main\generated-java
mkdir ..\..\src\main\generated-java

:: Copy newly generated Java into generated-java
robocopy us ..\..\src\main\generated-java\us /e /xf *.class* PrompInfoMapper.java
:: I couldn't get robocopy to exclude a directory by name like this
:: So, just delete it afterwards
rd /s /q ..\..\src\main\generated-java\us\ihmc\promp\presets
